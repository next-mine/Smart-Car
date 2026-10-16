#include "zf_common_headfile.h"
#include "my_common.h"
#include "get_data.h"

//定义存放编码器数值的变量
// 这几个量的含义是"本采样周期内的脉冲增量"，不是累计值：
// get_encoder() 每次都是先读走计数、再立即清零，所以增量 = 这一周期转过的角度。
// 周期由 main.c 的 pit_ms_init(PIT_CH, 10) 决定；改了周期，速度环的所有标定值都要重来。
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;
int32 hall_num = 0;
//定义电磁信号adc相关变量
// channel_list 只是"第 i 路电感挂在哪个 ADC 通道"的查表，内容全程不变，
// 所以加 static const 让编译器把它放进只读段；顺便把原来占着全局符号的 channel_index 收进函数内部。
static const adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] =
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};

uint16 adc_buffer[ADC_CHANNEL_NUMBER];                  // 滤波后的电感原始值
int16  adc_normal[ADC_CHANNEL_NUMBER];                  // 归一化后的电感值 0~100
int16  adc_min[ADC_CHANNEL_NUMBER];                     // 归一化下限（由 get_data_init 填默认值）
int16  adc_max[ADC_CHANNEL_NUMBER];                     // 归一化上限

//偏差与状态
float adc_error = 0.0f;                                 // 差比和得到的偏差，量程 ±DIFF_SUM_SCALE
uint8 line_lost_flag = 0;                               // 1 = 左右电感一起丢线
uint8 line_lost_flag_motor = 0;                         // 1 = 丢线严重需停电机（阈值 LINE_LOST_MOTOR，speed_control.c 使用）
float speed_now = 0.0f;                                 // 实测速度（脉冲/周期）
uint8 hall_flag=0;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化归一化标定值
// 参数说明     无
// 返回参数     无
// 使用示例     get_data_init();
// 备注信息     用循环填默认值，而不是在定义数组时写死 4 个初始值：
//               这样以后把 ADC_CHANNEL_NUMBER 改成 3 或 5，这里不用跟着改，也不会出现
//               "初始值个数和数组长度对不上"的编译错误。
//               标定好的 adc_min/adc_max 可以在这里直接改成实测值，或者以后加按键在运行时改。
//-------------------------------------------------------------------------------------------------------------------
void get_data_init(void)
{
    uint8 ch;

    for (ch = 0; ch < ADC_CHANNEL_NUMBER; ch ++)
    {
        adc_min[ch]    = ADC_NORM_MIN_DEFAULT;
        adc_max[ch]    = ADC_NORM_MAX_DEFAULT;
        adc_normal[ch] = 0;
    }

    adc_error      = 0.0f;
    line_lost_flag = 0;
    line_lost_flag_motor = 0;
    speed_now      = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器数值
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     编码器数值必须"先读计数，再清计数"，顺序不能颠倒，也不能漏掉清零：
//               硬件计数器是自由累加的，读出来的是"上次清零到现在"的增量；
//               先清零再读，读到的永远是 0；只读不清零，数值会一直累加直到 int16 溢出，
//               测出来的速度就完全错了。这一个函数一次性读了 4 路。
//-------------------------------------------------------------------------------------------------------------------
void get_encoder(void)
{
		encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                             // 清空编码器计数

    encoder_data_3 = encoder_get_count(ENCODER_3);                              // 获取编码器计数
    encoder_clear_count(ENCODER_3);                                             // 清空编码器计数

    encoder_data_4 = encoder_get_count(ENCODER_4);                              // 获取编码器计数
    encoder_clear_count(ENCODER_4);                                             // 清空编码器计数

		
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     去极值平均滤波
// 参数说明     buf             采样数据首地址
// 参数说明     count           采样点个数，必须 >= 1
// 返回参数     int16           去掉一个最大值和一个最小值之后的平均值
// 使用示例     median_average_filter(buffer, FILTER_N);
// 备注信息     ★ count 必须由调用者显式传进来，绝对不能在函数里用 sizeof(buf) 求个数：
//               C 语言里数组做函数形参会退化成指针，sizeof(buf) 得到的是指针大小（ARM 上是 4 字节），
//               而不是元素个数。4 字节刚好等于 4 个 int16，所以在"4 点去极值"这种写法下会碰巧算对，
//               一旦把点数改成 6 就悄悄算错 —— 这是抄别人代码时最容易一起抄进来的坑。
//               内部用 int32 累加：6 点 × 4095 = 24570 还没超 int16 的 32767，
//               但点数到 9 点（36855）就会溢出翻负，所以这里直接上 int32，别省这点空间。
//-------------------------------------------------------------------------------------------------------------------
static int16 median_average_filter(const int16 *buf, uint8 count)
{
    int32 sum = 0;
    int16 max;
    int16 min;
    uint8 i;

    if (0 == count)                                             // 空数组保护，否则下面的 buf[0] 会越界
    {
        return 0;
    }

    if (count < 3)                                              // 不足 3 个点时没有"极值"可去，只能直接求平均
    {
        for (i = 0; i < count; i ++)
        {
            sum += buf[i];
        }
        return (int16)(sum / count);
    }

    max = buf[0];
    min = buf[0];
    for (i = 0; i < count; i ++)
    {
        if (buf[i] > max) max = buf[i];                         // 找最大值
        if (buf[i] < min) min = buf[i];                         // 找最小值
        sum += buf[i];
    }

    return (int16)((sum - max - min) / (count - 2));            // 去掉一个最大、一个最小，剩下的求平均
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电感值归一化
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     把每个电感从"adc_min ~ adc_max"线性映射到"0 ~ ADC_NORM_MAX_VALUE"。
//               这一步是抗电池电压跌落、抗车高变化的关键：所有电感一起变小时，
//               归一化会把这个整体变化除掉，只留下"左右谁更强"这个真正有用的信息。
//-------------------------------------------------------------------------------------------------------------------
static void adc_normalize(void)
{
    uint8 ch;
    int32 span;
    int32 value;

    for (ch = 0; ch < ADC_CHANNEL_NUMBER; ch ++)
    {
        span = (int32)adc_max[ch] - (int32)adc_min[ch];
        if (span <= 0)                                          // 标定区间非法（max <= min），归一化会除零
        {
            adc_normal[ch] = 0;                                 // 记 0 而不是让程序崩掉；屏幕上一片 0 就说明标定没做
            continue;
        }

        // (adc - min) 可能为负，(adc - min) * 100 最大到 4095*100 = 409500，
        // 远超 int16 的 32767，所以这一步必须全程 int32，不能图省事用 int16 中间变量。
        value = ((int32)adc_buffer[ch] - (int32)adc_min[ch]) * ADC_NORM_MAX_VALUE / span;

        if (value < 0)                      value = 0;                      // 低于标定下限，压到 0
        if (value > ADC_NORM_MAX_VALUE)     value = ADC_NORM_MAX_VALUE;     // 高于标定上限，压到 100

        adc_normal[ch] = (int16)value;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和计算赛道偏差
// 参数说明     无
// 返回参数     float           偏差，量程 ±DIFF_SUM_SCALE，正 = 导线偏左，负 = 导线偏右
// 使用示例     无
// 备注信息     左侧取前两路电感、右侧取后两路。这个分组必须和探头实际安装位置对应：
//               main.c 的说明里写了"最左电感放在 adc_buffer[0]、最右放在 adc_buffer[3]"，
//               如果接反了，偏差符号会整个反过来，车会朝远离导线的方向打死。
//               分母 left+right 带着总信号强度，所以信号整体变强变弱时这个比值几乎不变，
//               这就是差比和比"直接相减"稳的原因。
//-------------------------------------------------------------------------------------------------------------------
static float adc_diff_sum(void)
{
    int32 left;
    int32 right;
    int32 total;
    float  error;

    // 丢线方向记忆：记住"上一次还有信号时"的偏差方向与大小。
    // 电磁组丢线几乎只发生在弯道，且丢线前最后一拍的偏差方向就是弯道方向。
    // 一旦丢线就回中（返回 0），车会在弯道最需要继续打角的时候走直线冲出赛道；
    // 改成"按记忆方向打死角"，就能在信号变弱的瞬间保持弯道打角，把车头拽回弯道内侧。
    static float last_error = 0.0f;

    left  = 1.3*(int32)adc_normal[0] + (int32)adc_normal[1];        // 左侧两路电感之和
    right = (int32)adc_normal[2] + 1.3*(int32)adc_normal[3];        // 右侧两路电感之和
    total = 1.3* (int32)adc_normal[0] + (int32)adc_normal[1] + (int32)adc_normal[2] + 1.3*(int32)adc_normal[3]; 

    // 丢线停车判定：total 低于 LINE_LOST_MOTOR 才置位，阈值比舵机丢线阈值更严，避免信号稍弱就把车刹死。
    if (total < LINE_LOST_MOTOR)
    {
        line_lost_flag_motor = 1;
    }
    else
    {
        line_lost_flag_motor = 0;
    }

    if (total < LINE_LOST_THRESHOLD)                            // 左右都没信号：分母趋零，差比和没有意义
    {
        line_lost_flag = 1;

        // 方案一（冻结打角）：丢线时不打死角、也不回中，而是返回丢线前最后一拍的偏差值，
        // 让舵机"冻结"在当前打角上。这样丢线瞬间车头平滑保持弯道姿态，重新拾到线时
        // 打角也不会从极限角猛弹回来，从而消除"左右来回摆动"。
        // 代价：如果丢线持续较久，车会一直保持这个中等打角，可能不足以把车头拉回弯道内侧，
        // 需要更猛的打角时再考虑配合速度降速或改回饱和打角。
        return last_error;
    }

    line_lost_flag = 0;

    // left/right 各是 0~200，total 是 0~400，所以 (left-right)/total 天然落在 ±1，
    // 乘上 DIFF_SUM_SCALE 之后量程就是 ±DIFF_SUM_SCALE。
    // 注意 (float) 强制转换要写在减法之后：left/right/total 都是 int32，
    // 不转的话先做整数除法会把小数部分全部丢光，偏差就变成只有 -1/0/1 三档了。
    error = (float)(left - right) * DIFF_SUM_SCALE / (float)total;

    // 正常算出偏差后更新记忆，供下一拍丢线时使用。
    last_error = error;

    return error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算平均轮速
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     用 SPEED_ENCODER_MASK 选出真正装了编码器的通道，乘上各自的方向符号之后再平均：
//               ① 掩码的意义：没装编码器的通道引脚是悬空的，正交解码器会把噪声边沿也当成脉冲数，
//                  如果一起平均进来，实测速度会被噪声污染（可能乱跳、也可能整体偏负），
//                  速度环就会朝一个错误的方向猛给油 —— 这是"闭环一开就冲出去"的常见原因。
//               ② 方向符号的意义（ENCODER_DIR_x）：左轮编码器向前转读数天生是负的，
//                  必须先翻正再参与运算。不然负反馈会变成正反馈，车会直接失控。
//               ③ 左右轮一起平均还有个好处：转弯时左右轮一快一慢，平均之后正好互相抵消，
//                  速度环就不会被转向动作带着一起抖。
//-------------------------------------------------------------------------------------------------------------------
static void speed_calc(void)
{
    // 每路编码器的前进方向符号表。static const 会被编译器放进只读段，不占 RAM，
    // 也不会每次进入函数重新构造。下标 i 对应 ENCODER_{i+1}。
    static const int8 encoder_dir[ENCODER_NUMBER] =
    {
        ENCODER_DIR_1, ENCODER_DIR_2, ENCODER_DIR_3, ENCODER_DIR_4,
    };

    int32 encoder_data[ENCODER_NUMBER];
    int32 sum = 0;
    uint8 count = 0;
    uint8 i;

    encoder_data[0] = encoder_data_1;
    encoder_data[1] = encoder_data_2;
    encoder_data[2] = encoder_data_3;
    encoder_data[3] = encoder_data_4;

    for (i = 0; i < ENCODER_NUMBER; i ++)
    {
        if (SPEED_ENCODER_MASK & (1 << i))                      // 这一路参与测速
        {
            // ★ 乘方向符号，把"前进时读数为负"的编码器翻正。
            //   注意 int8 乘 int16 会先整型提升成 int，这里显式转 int32 只是为了写明意图、
            //   顺便让累加全程都在 32 位上做，避免以后点数变多时溢出。
            sum += (int32)encoder_dir[i] * (int32)encoder_data[i];
            count ++;
        }
    }

    if (0 == count)                                             // 一个通道都没选（掩码写成 0），保护一下别除零
    {
        speed_now = 0.0f;
        return;
    }

    speed_now = (float)sum / (float)count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取ADC数值并算出偏差
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     三步走：① 采样 + 滤波  ② 归一化  ③ 差比和。
//               以前这里是 adc_error = adc_buffer[0] - adc_buffer[3] 直接相减，
//               现在换成了 4 路电感分左右做差比和，抗电池电压跌落和抗整体信号漂移都好得多。
//-------------------------------------------------------------------------------------------------------------------
void get_adc(void)
{
    int16 filter_buf[FILTER_N];                     // 单路电感的采样缓存：只要一份，12 字节，不占多少栈
    uint8 ch;
    uint8 i;

    // 第 1 步：逐路采样 + 去极值平均
    for (ch = 0; ch < ADC_CHANNEL_NUMBER; ch ++)
    {
        for (i = 0; i < FILTER_N; i ++)
        {
            // 内层均值滤波负责压高频噪声。adc_convert 是阻塞式"发起转换 + 死等完成"，
            // 这里会被调用 FILTER_N * ADC_MEAN_COUNT 次，周期时间要留够（开销估算见 my_common.h）
            filter_buf[i] = (int16)adc_mean_filter_convert(channel_list[ch], ADC_MEAN_COUNT);
        }

        adc_buffer[ch] = (uint16)median_average_filter(filter_buf, FILTER_N);
    }

    // 第 2 步：归一化
    adc_normalize();

    // 第 3 步：算差比和偏差
    adc_error = adc_diff_sum();
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     采集霍尔传感器示数
// 参数说明     无
// 返回参数     hall_raw  0或1
// 使用示例     无
// 备注信息    	
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     采集一次所有传感器数据
// 参数说明     无
// 返回参数     无
// 使用示例     无
// 备注信息     中断里调用的入口函数。顺序是"先读编码器，再读电感"，
//               这两件事互不依赖，顺序其实无所谓；但编码器必须尽早读走，
//               否则这一周期多花的 ADC 时间会让"采样周期"和"实际计时间隔"对不上，速度标定就会偏。
//-------------------------------------------------------------------------------------------------------------------
void get_data(void)
{
	get_encoder();
	speed_calc();
	get_adc();
}





