#include "zf_common_headfile.h"
#include "my_common.h"
#include "servo_control.h"
#include "get_data.h"
#include "roundabout.h"

//定义舵机打角值
float servo_motor_angle = SERVO_MOTOR_M;
//PID系数kp
// ★ 这个 kp 原样保留（0.008），没有跟着差比和一起改。
//   能沿用的原因是 my_common.h 里把 DIFF_SUM_SCALE 取成了 4000：
//   原来的 raw 差 ≈ 4095 × 偏差比例，新的差比和 ≈ 偏差比例，乘上各自的系数后量程和斜率都对齐，
//   所以 0.008 继续有效，最大打角依然在 ±15° 限幅附近收住。
//   换句话说：这里不动 kp，是因为误差的量纲被刻意对齐了，而不是碰巧没改。
float kp = 0.0055;

// 舵机微分系数 kd（PD 打角里的 D 项）。打角 = 中值 - kp*偏差 - kd*(偏差 - 上次偏差)。
// D 项对"偏差变化率"反应，相当于提前预判：弯道里偏差还在快速增大时，就提前多打一点角，
// 让打角更"跟手"，这是"弯道提速同时不偏离"在打角侧的保障。初值见 my_common.h 的 SERVO_KD。
float kd = SERVO_KD;

// 上一拍的偏差，D 项要用它做差分。static 关在文件里，避免被别的模块误写。
static float last_adc_error = 0.0f;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机PWM
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     舵机打角值 = 中值 - kp × 偏差 - kd × 偏差变化率，再做限幅，最后换算成 PWM 占空比输出。
//               符号约定：偏差为正 = 左侧电感信号更强 = 导线在车左前方 = 需要往左打角；
//               而 SERVO_MOTOR_L_MAX(75) < SERVO_MOTOR_M(90)，即"角度变小 = 往左"，
//               所以这里写的是"中值 - kp×偏差 - kd×偏差变化率"。
//               如果实车发现打角方向相反，先把舵盘拆下来按 90° 重装（main.c 说明第 1 条），
//               再不行就把这里的减号改成加号。
//-------------------------------------------------------------------------------------------------------------------
void set_servo_pwm(void)
{
	float error_rate;

	// 转向测试模式：把舵机钉死在 RB_STEER_TEST_DEG，其它逻辑一律不管。
	// 平时这个宏是 0，整段会被编译器优化掉，一个字节都不占。
	// 它的唯一用途是上车量"给定舵角能跑出多大的圆"，用来分清是代码时机不对还是车本身转不过来。
	if (0 != RB_STEER_TEST_DEG)
	{
		servo_motor_angle = (float)RB_STEER_TEST_DEG;

		pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
		pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
		pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
		return;
	}

	// 环岛阶段由 roundabout.c 直接接管：进入/绕行/出环这三段里打角是算好的，
	// 不再走下面的 PD。判断必须放在最前面，否则 PD 算完又被覆盖，等于白算一遍。
	if (roundabout_servo_takeover(&servo_motor_angle))
	{
		// 顺手同步一下 D 项的记忆值。不同步的话，环岛那几秒里偏差可能已经跑了几千，
		// 出环那一拍 error_rate 会变成"现在的偏差 - 几年前（夸张）的偏差"，
		// 舵机会被这一下猛地拽一把 —— 明明只是恢复循迹，车却像被绊了一脚。
		last_adc_error = adc_error;
	}
	else
	{
		// 偏差变化率 = 本拍偏差 - 上拍偏差。D 项就靠这个量提前感知"偏差正在变大（正在进弯）"。
		error_rate = adc_error - last_adc_error;
		last_adc_error = adc_error;

		// 根据偏差 + 偏差变化率计算舵机打角（PD）
		servo_motor_angle = SERVO_MOTOR_M - kp * adc_error - kd * error_rate;
	}

	// 环岛识别阶段（WAIT / HUMP）只封一个方向：不许往右打。
	// 为什么需要它：岔口那两根线（直道 + 圆环）会把差比和算成一个很强的"往右"偏差，
	// 而这两个阶段还没确认、舵机还在按 PD 走，车就会在交界处被猛地拽向右边冲出赛道。
	// ★ 方向是实测的：角度变大 = 往左，角度变小 = 往右（用 RB_STEER_TEST_DEG=76 架车看出来的）。
	//   所以"不许往右"就是"不许小于中值"，钳位方向和直觉相反，别写反了。
	// 只封一个方向，往左该打多少还是多少，正常循迹能力一点没丢。
	if (roundabout_forbid_right() && (servo_motor_angle < (float)SERVO_MOTOR_M))
	{
		servo_motor_angle = (float)SERVO_MOTOR_M;
	}

	//打角限幅：舵机机械行程有限，超出范围会堵转、发热，长期堵转会扫齿。
	// 这一步不是可选项，去掉限幅就等于把舵机往死里顶。
	if(servo_motor_angle > SERVO_MOTOR_R_MAX)servo_motor_angle = SERVO_MOTOR_R_MAX;
	if(servo_motor_angle < SERVO_MOTOR_L_MAX)servo_motor_angle = SERVO_MOTOR_L_MAX;

	//设置舵机打角。三路舵机写的是同一个值，只是为了适配"舵机到底插在哪个引脚"不确定的情况，
	// 实际接线只会用到其中一路，另外两路可以删掉（留着也不会有什么坏处）。
	pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
