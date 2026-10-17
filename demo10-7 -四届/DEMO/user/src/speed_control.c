#include "zf_common_headfile.h"
#include "my_common.h"
#include "speed_control.h"
#include "get_data.h"

//控制电机速度，可取0-100，若太大可调低，若使用3S电池，占空比应限幅60%
// 这里保留 int8 类型和 duty 这个变量名，是为了和屏幕显示、以及原来那份开环代码的习惯保持一致。
int8 duty = (int8)SPEED_START_DUTY;

//速度环目标与参数（可以在屏幕上边看边改，也可以以后接按键/上位机在线调）
float speed_target = SPEED_TARGET_DEFAULT;
float speed_kp     = SPEED_PI_KP;
float speed_ki     = SPEED_PI_KI;

//速度环故障标志：1 = 自检发现反馈符号依然是反的，已经把输出锁死在 0
uint8 speed_fault = 0;

// 增量式 PI 的内部状态。用 static 关在文件里，外面改不到，避免被别的模块误写导致积分乱掉。
static float  speed_error_last  = 0.0f;                 // 上一次的偏差，增量式要拿它做差分
// 浮点占空比：增益很小的时候每次增量可能不到 1，如果直接对 int8 的 duty 累加，
// 每次都会被取整丢掉小数部分，积分就永远"走不动"了（所谓积分死区）。
// 所以内部用 float 累加，只在最后输出时取整一次。
static float  speed_out         = SPEED_START_DUTY;
static uint16 speed_fault_count = 0;                    // 连续"反馈为负"的周期数
static uint16 speed_zero_count  = 0;                    // 连续"给了油却没有反馈"的周期数

// 弯道速度规划的静态状态（见 speed_curve_plan 函数说明）
static float  speed_target_smooth = SPEED_TARGET_DEFAULT;   // 平滑后的目标速度（一阶低通输出）
static float  last_abs_error      = 0.0f;                   // 上一拍的 |adc_error|，用于算"偏差变化率"做入弯预判

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     弯道速度规划（直道提速 + 弯道连续降速 + 入弯提前减速 + 出弯提速）
// 参数说明     无
// 返回参数     无（直接修改全局 speed_target）
// 使用示例     在 set_speed_pwm 的闭环分支里、算偏差之前调用一次
// 备注信息     这是"弯道减速"的核心，原理分三层：
//
//   (1) 连续降速（越弯越慢，但有地板）：
//       期望目标 = SPEED_CURVE_MAX - SPEED_CURVE_GAIN * |adc_error|
//       |adc_error| 越大（弯越急），目标速度越低；但被夹在 [SPEED_CURVE_MIN, SPEED_CURVE_MAX] 之间，
//       所以不会"越弯越慢、慢到离谱"——最多降到弯道地板 SPEED_CURVE_MIN。
//
//   (2) 入弯提前减速（四个电感一排、没有前瞻时，用偏差变化率预判）：
//       |adc_error| 相比上一拍增大超过 SPEED_CURVE_ENTER_RATE，说明车正在入弯（偏差在快速拉开），
//       此时"提前"把目标压到 SPEED_CURVE_ENTER_SPEED，比等偏差已经很大再减更早、更平滑。
//
//   (3) 出弯提速：
//       |adc_error| 回落到 SPEED_CURVE_EXIT_ERROR 以下，说明已回到直道，恢复满速。
//
//   最后对目标速度做一阶低通平滑（SPEED_CURVE_SMOOTH），避免目标每拍跳变导致油门突变。
//   这一步的结果写进全局 speed_target，后面 speed_target - speed_now 走正常的增量式 PI。
//-------------------------------------------------------------------------------------------------------------------
static void speed_curve_plan(void)
{
    float abs_error;      // 当前偏差的绝对值
    float rate;           // 偏差变化率（本拍 - 上拍）
    float desired;        // 期望目标速度（平滑前）

    abs_error = adc_error;
    if (abs_error < 0.0f) abs_error = -abs_error;                 // 手写绝对值，避免 func_abs 对 float 截断

    // ---- (1) 连续降速 ----
    desired = SPEED_CURVE_MAX - SPEED_CURVE_GAIN * abs_error;

    // ---- (2) 入弯提前减速 ----
    rate = abs_error - last_abs_error;                            // 偏差在增大 = 正在入弯
    last_abs_error = abs_error;
    if (rate > SPEED_CURVE_ENTER_RATE)
    {
        if (desired > SPEED_CURVE_ENTER_SPEED)
        {
            desired = SPEED_CURVE_ENTER_SPEED;                    // 提前压到入弯速度（只降不升）
        }
    }

    // ---- (3) 出弯提速 ----
    if (abs_error < SPEED_CURVE_EXIT_ERROR)
    {
        desired = SPEED_CURVE_MAX;                                // 已回直道，恢复满速
    }

    // ---- 限幅到 [地板, 满速] ----
    if (desired < SPEED_CURVE_MIN) desired = SPEED_CURVE_MIN;
    if (desired > SPEED_CURVE_MAX) desired = SPEED_CURVE_MAX;

    // ---- 一阶低通平滑，避免目标跳变 ----
    speed_target_smooth = speed_target_smooth * (1.0f - SPEED_CURVE_SMOOTH) + desired * SPEED_CURVE_SMOOTH;

    speed_target = speed_target_smooth;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     复位速度环
// 参数说明     无
// 返回参数     无
// 使用示例     speed_loop_reset();
// 备注信息     清掉上一次的偏差记忆、累计输出和故障标志，把占空比拉回起始值。
//               什么时候需要它：重新起步、切换目标速度、从丢线状态恢复，或者排除完接线问题之后解故障锁。
//               不清的话，上次遗留的 speed_error_last 会让"第一次增量"算出一个很突兀的值；
//               而故障标志不清，输出会一直锁在 0，车怎么都不走。
//-------------------------------------------------------------------------------------------------------------------
void speed_loop_reset(void)
{
    speed_error_last  = 0.0f;
    speed_out         = SPEED_START_DUTY;
    speed_fault       = 0;
    speed_fault_count = 0;
    speed_zero_count  = 0;
    duty              = (int8)SPEED_START_DUTY;
    // 弯道速度规划的平滑状态也一起复位，避免上次遗留的平滑值干扰重新起步
    speed_target_smooth = SPEED_TARGET_DEFAULT;
    last_abs_error      = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机PWM（编码器速度闭环）
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     三种工作状态，按优先级从上到下判断：
//               ① 开环（SPEED_LOOP_ENABLE = 0）—— 占空比固定，行为和改造前的原例程一样。
//                  专门用来安全地验证编码器接线和方向符号，轮子只会慢转，符号配错也不会失控。
//               ② 故障锁停（speed_fault = 1）—— 自检判定反馈符号反了，输出锁 0。
//                  这是最后一道防线：宁可车不动，也不要它满速冲出去。
//               ③ 闭环 PI —— 偏差限幅 -> 增量式 PI -> 单周期输出变化限幅 -> 总限幅 -> 反馈自检。
//
//               用增量式而不是位置式的理由：增量式天然带着"上次输出"的记忆，不需要额外累加历史误差，
//               也就不会出现位置式那种积分饱和后要很久才退出来的问题；
//               速度环的目标值会频繁变化（直道加速、弯道减速），这点很重要。
//-------------------------------------------------------------------------------------------------------------------
void set_speed_pwm(void)
{
    float error;
    float delta;

    if (0 == SPEED_LOOP_ENABLE)
    {
        // ① 开环模式：不读反馈、不做运算，直接给固定占空比。
        //    这个分支的存在意义就是"让调车这件事变得安全"：即使编码器一根线都没接对，
        //    车也只是按 SPEED_START_DUTY 慢速转，你可以从容地看屏幕上的 SPD 去调符号。
        duty = (int8)SPEED_START_DUTY;
    }
    else if (speed_fault)
    {
        // ② 已判定反馈异常：清掉累加器并锁 0，绝不再给油。
        //    这里同时清 speed_out，是为了让 speed_loop_reset() 之后能从干净的状态重新起步。
        speed_out = 0.0f;
        duty      = 0;
    }
    else
    {
        // ③ 闭环
        // ---- 3.0 弯道速度规划：先根据当前偏差算出"这一拍该跑多快"，再交给下面的 PI 去追 ----
        speed_curve_plan();

        // ---- 3.1 算偏差：目标 - 实测 ----
        error = speed_target - speed_now;

        // ---- 3.2 单周期偏差限幅 ----
        // 电机换向尖峰、偶发丢脉冲都可能让某一拍的 speed_now 变成一个离谱的值；
        // 不限幅的话，增量式 PI 会按这个离谱偏差算出一个巨大的增量，一拍就把输出顶到上限
        // （你之前遇到的"一通电就飞快"就是这个机制）。限幅之后最坏情况也只是多走一小步。
        if (error >  SPEED_ERROR_LIMIT) error =  SPEED_ERROR_LIMIT;
        if (error < -SPEED_ERROR_LIMIT) error = -SPEED_ERROR_LIMIT;

        // ---- 3.3 增量式 PI：Δu = Kp*(e - e_last) + Ki*e ----
        delta = speed_kp * (error - speed_error_last) + speed_ki * error;
        speed_error_last = error;

        // ---- 3.4 单周期输出变化限幅（软启动 + 防弹射）----
        // 10ms 一个周期，SPEED_DUTY_SLEW = 3.0 意味着从 0 加到 50% 至少要 17 个周期（约 170ms），
        // 车不可能"一瞬间"从静止窜到满速，给人和机械都留出反应时间。
        if (delta >  SPEED_DUTY_SLEW) delta =  SPEED_DUTY_SLEW;
        if (delta < -SPEED_DUTY_SLEW) delta = -SPEED_DUTY_SLEW;

        // ---- 3.5 累加并限幅 ----
        speed_out += delta;
        if (speed_out > (float)MAX_PWM_DUTY) speed_out = (float)MAX_PWM_DUTY;   // 上限：保护电机与电池（3S 电池尤其注意）
        if (speed_out < 0.0f)                speed_out = 0.0f;                  // 下限：本车不做倒车

        duty = (int8)speed_out;                                                  // 取整只在这一处做

        // ---- 3.6 反馈自检（两道判据，任一条连续成立就锁停）----
        // (a) 反馈符号反了：占空比已经给到 SPEED_FAULT_DUTY_MIN 以上（确实在驱动电机），
        //     平均速度却明显是负的 —— 说明 ENCODER_DIR_x 配反了、或者 A/B 两相接反了。
        //     此刻继续给油就是正反馈失控：转得越快 -> 读数越负 -> 偏差越大 -> 占空比越大。
        // (b) 反馈丢失或被抵消：占空比给到 SPEED_FAULT_DUTY_HIGH 以上，平均速度却几乎为 0。
        //     要么编码器没接好，要么左右两路符号相反、被平均中和成了 0。后者很隐蔽：
        //     V 既不乱跳也不为负，就是稳稳的 0，速度环会以为车停着不动，于是一路加油到满速。
        // 两道都要求连续多个周期成立，是为了躲开转向时内侧轮瞬间反转、或过坎打滑造成的偶发值。
        if ((duty > (int8)SPEED_FAULT_DUTY_MIN) && (speed_now < SPEED_FAULT_NEG_LIMIT))
        {
            speed_fault_count ++;
        }
        else
        {
            speed_fault_count = 0;                                          // 中间只要有一拍正常就重新计数
        }

        // 注意这里不写 func_abs(speed_now)：func_abs 的形参是整型，传 float 会先把小数截掉，
        // 对"接近 0"这种判据来说截断反而容易出问题，所以干脆把绝对值展开成两次比较。
        if ((duty > (int8)SPEED_FAULT_DUTY_HIGH) &&
            (speed_now < SPEED_FAULT_ZERO_LIMIT) && (speed_now > -SPEED_FAULT_ZERO_LIMIT))
        {
            speed_zero_count ++;
        }
        else
        {
            speed_zero_count = 0;
        }

        if ((speed_fault_count >= SPEED_FAULT_COUNT) || (speed_zero_count >= SPEED_FAULT_ZERO_COUNT))
        {
            speed_fault = 1;                                                // 锁住，屏幕上的 F 会变成 1
            speed_out   = 0.0f;
            duty        = 0;
        }
    }

    // ④ 输出：DIR 定方向，PWM 定转速
    //    DIR 固定高电平，本车只前进不倒车；所以下限被夹在 0，PI 算出负数也不会真的反转。
    if (line_lost_flag_motor == 1||hall_flag==1)
    {
        //过赛道线|丢线停车：先复位速度环内部状态，防止丢线期间积分把 speed_out 顶满、恢复时猛冲；
        // 再把 duty 清 0，下面统一输出，四个电机全部停转。
        speed_error_last = 0.0f;
        speed_out        = 0.0f;
        duty             = 0;
    }

    gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                   // DIR输出高电平
    pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比

    gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                   // DIR输出高电平
    pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比

    gpio_set_level(MOTOR3_DIR, GPIO_HIGH);                                   // DIR输出高电平
    pwm_set_duty(MOTOR3_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比

    gpio_set_level(MOTOR4_DIR, GPIO_HIGH);                                   // DIR输出高电平
    pwm_set_duty(MOTOR4_PWM, duty * (PWM_DUTY_MAX / 100));                   // 计算占空比


}
