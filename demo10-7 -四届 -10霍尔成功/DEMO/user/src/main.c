/*********************************************************************************************************************
* 逐飞科技 RT1064DVL6A Opensource Library —— 基于逐飞 RT1064DVL6A 开源库
* 
* 版本修改记录
* 日期              修改人                备注
* 2025-09-30        wjm            first version
* 2026-02-14        ----           归一化标定值改为循环填充，与数组长度解耦，避免初始值个数不匹配
* 2026-02-15        ----           实现速度环闭环、编码器方向与掩码处理，完善丢线与故障标志
********************************************************************************************************************/

// *************************** 硬件接线说明 ***************************
/*
				车头四路电磁电感：
				最左电感 -> 电感通道 4（对应 ENCODER_4）
				最右电感 -> 电感通道 3（对应 ENCODER_3）
			  见 my_common.h 中 SPEED_ENCODER_MASK = 0x0C，只采两路编码器
			  见 my_common.h 中 ENCODER_DIR_4 = -1，右轮向前转为负，需先翻正
			  方向反了会导致速度环正反馈，车会失控，务必核对
*/
// *************************** 算法流程说明 ***************************
/*
			整体算法流程：
			1.采集四路电感原始值（去极值平均滤波 + 一阶低通），经 ADC 采入 MCU
			2.电感值归一化到 0~100，去除电池电压跌落与车高变化的影响
			3.四路电感做差比和，得到赛道偏差
			4.偏差按 kp 比例放大
			5.偏差控制舵机 + 编码器 PI 控制电机占空比
			
			
			注意！注意！注意！注意！
			注意！注意！注意！注意！
			注意！注意！注意！注意！

			1.电感归一化上下限在 my_common.h 里改，也可以运行时改。调试时要按实际测到的电感值来装
			2.编码器接线方向。方向反了速度环会变成正反馈，车会直接失控。要改方向请在 speed_control.c 里改 DIR 的电平
			3.缩小/放大电感间距，希望车更稳定/更灵，靠调整电感间距来实现，不要只调比例系数
			4.最左电感接 adc_buffer[0]、最右接 adc_buffer[3]，接反了偏差符号会整体反，车会向远离导线的方向打死
			
			★ 上电调试前请按顺序检查，否则车会不可预料地窜出去
			   a) 先标定每一路的 adc_min / adc_max 实测值
			      见 my_common.h 里的 ADC_NORM_*_DEFAULT 只是占位值
			      在屏幕 2~4 行看到 R0~R3 是原始值，N0~N3 是归一化值，据此确定
			
			   b) 保证编码器方向正确（★ 装好车后务必空转验证一次）
			      先把 my_common.h 里 SPEED_LOOP_ENABLE 设为 0，开环只转电机
			      空转时看屏幕上的 SPD：
			        手动正转时 SPD 应该为正（且大小合理）—— 若为负说明方向反了/没接好
			        手动反转时 SPD 应该为负
			      确定方向后再把 my_common.h 里 ENCODER_DIR_x 取反（A/B 相交换）
			      确认无误后再把 SPEED_LOOP_ENABLE 的值改为 1，开启闭环
			
			   c) 先定目标速度，用屏幕上的 SPD 和 SPEED_START_DUTY 的实测值来调
			      想跑多快就调 SPEED_TARGET_DEFAULT 到想要的值
			      注意：目标速度要和实测值对得上，占空比压到 0 时车应该是停的
			
			
*/

// **************************** 包含头文件 ****************************

#include "zf_common_headfile.h"
#include "my_common.h"
#include "get_data.h"
#include "servo_control.h"
#include "speed_control.h"

// 修改配置后需要重新编译才能生效
// 改完头文件先 project->clean 再重新编译，避免增量编译漏掉

void Init_All(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 初始化系统时钟
    debug_init();                   // 初始化串口调试

    system_delay_ms(300);           //上电延时等待外设稳定

	// 屏幕显示方向要在 tft180_init() 之前设置才会生效
	//   tft180_set_dir 是写屏驱动寄存器命令，要在屏幕初始化前写好方向寄存器(0x36)
	//   如果设置方向放在 tft180_init 之后，可能导致屏幕显示方向翻转
	//   原库默认写法是"先 init 后 set_dir"，这里提前到 init 之前更稳妥
	tft180_set_dir(TFT180_CROSSWISE);
	tft180_init();

	Init_All();						//初始化所有外设
	
	pit_ms_init(PIT_CH, 10);//开启10ms周期中断
	
	
	
	
	
	
	
	    // 初始化GPIO B9 为输出 默认输出低电平
    gpio_init(B9, GPO, 0, GPO_PUSH_PULL);

    // 初始化GPIO D0 D1 为输入引脚
    gpio_init(C4, GPI, 0, GPI_PULL_UP);
    gpio_init(D1, GPI, 0, GPI_PULL_UP);





	
	// 屏幕上的固定标签只写一次，循环里只刷数值，省得每帧重写字符串
	//
	// 按 8x16 字库（默认字库）来算，每个字符宽 8 高 16。屏是 160x128
	// tft180_show_int 实际占 num+1 个字符，tft180_show_float 占 num+pointnum+2 个字符
	//   （数字不够长的地方用空格补齐，所以占位宽度是固定的），因此必须满足
	//        x + (占位宽度 - 1) * 8 < 160
	//   否则 tft180_show_string 里的 zf_assert(x < tft180_x_max) 会断言失败，直接把程序停住
	//   改动下面任何坐标前，请照着这个式子算一遍
	//
	// 屏幕内容对照表（完整版）：
	//   DianCi_demo              程序名
	//   R0:1234   N:56           第0路电感：R=原始值(0~4095) / N=归一化值(0~100)
	//   R1:2345   N:60           R0~R3 依次对应 ADC_CHANNEL1~4（最左 -> 最右）
	//   R2:3456   N:40
	//   R3:4567   N:33
	//   E:-1234  TGT:100.0       E   = 差比和偏差（±4000，正 = 导线偏左）
	//                            TGT = 目标速度（脉冲/周期）
	//   SPD:100.0  Loop:0        SPD  = 实测速度（脉冲/周期）
	//                            Loop = 闭环开关（0 = 开环 / 1 = 闭环），编译期常量，只写一次
	//   DUTY:10  LOST:0  F:0     DUTY = 电机占空比(%)
	//                            LOST = 丢线标志（1 = 左右电感一起丢线）
	//                            F    = 速度环故障（1 = 检测到反馈符号异常，输出已锁 0）
	//														H    =霍尔原始电平(0/1)
	tft180_show_string(0,  16, "R0:");
	tft180_show_string(64, 16, "N:");
	tft180_show_string(0,  32, "R1:");
	tft180_show_string(64, 32, "N:");
	tft180_show_string(0,  48, "R2:");
	tft180_show_string(64, 48, "N:");
	tft180_show_string(0,  64, "R3:");
	tft180_show_string(64, 64, "N:");
	tft180_show_string(0,  80, "E:");
	tft180_show_string(72, 80, "TGT:");
	tft180_show_string(0,  96, "SPD:");
	tft180_show_string(88, 96, "Loop:");
	tft180_show_string(0, 112, "DUTY:");
	tft180_show_string(72,112, "LOST:");
	tft180_show_string(128,112,"F:");
	tft180_show_string(136, 32, "H:");

	// Loop 是编译期常量，运行期不会变，所以只写一次；这样"为什么车不进闭环"一眼就能看出来
	tft180_show_int(128, 96, SPEED_LOOP_ENABLE, 1);
	
    while(1)
    {
		hall_num++;
		if(hall_num>=200)hall_num=200;
			
        // 此处编写需要循环执行的代码
				// 4 路电感：左边是滤波后的原始值（用来标定 adc_min/adc_max），右边是归一化值 0~100
        tft180_show_int(24, 16, adc_buffer[0], 4);
        tft180_show_int(80, 16, adc_normal[0], 3);
		
        tft180_show_int(24, 32, adc_buffer[1], 4);
        tft180_show_int(80, 32, adc_normal[1], 3);
		
				tft180_show_int(24, 48, adc_buffer[2], 4);
        tft180_show_int(80, 48, adc_normal[2], 3);
		
				tft180_show_int(24, 64, adc_buffer[3], 4);
        tft180_show_int(80, 64, adc_normal[3], 3);
		
				tft180_show_float(16, 80, adc_error, 4, 1);					// E   = 差比和偏差，量程 ±4000
				tft180_show_float(104, 80, speed_target, 4, 1);				// TGT = 目标速度
				
				tft180_show_float(32, 96, speed_now, 4, 1);					// SPD = 实测速度（脉冲/周期）
				
				tft180_show_int(40, 112, duty, 3);							// DUTY = 当前占空比(%)
				tft180_show_int(112, 112, line_lost_flag, 1);				// LOST = 丢线标志
				tft180_show_int(144, 112, speed_fault, 1);					// F    = 速度环故障标志
				tft180_show_int(0, 0, hall_num, 3);						// H    = 霍尔原始电平(0/1)
				tft180_show_int(48, 0, round_true_in_flag, 3);	
    }
}

//此初始化函数共初始化4路电机接口、4路编码器接口、4路电感ADC、3路舵机接口，以及归一化标定值
void Init_All(void)
{
	//初始化电机驱动DRV8701
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

	//编码器初始化（正交解码模式）
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器正交解码模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器正交解码模式
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // 初始化编码器正交解码模式
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // 初始化编码器正交解码模式
	
	//编码器方向检测初始化
//    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器方向检测
//    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器方向检测
//    encoder_dir_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // 初始化编码器方向检测
//    encoder_dir_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // 初始化编码器方向检测
	
	//初始化adc采集
		adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL4, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
		
	//初始化舵机
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR3_PWM, SERVO_MOTOR_FREQ, 0);
	
	//初始化归一化标定值，在读取数据之前调用一次
	// 若标定没做好，归一化会得 0，屏幕上前几帧全是 0，因为 span<=0 走了保护分支
	get_data_init();
	
	
	// 以 D4 下降沿触发为例（比如霍尔压线从高变低）
	exti_init(D4, EXTI_TRIGGER_FALLING);

}

//中断服务函数，10ms 周期中断，每10ms执行一次
void pit_handler (void)
{
	
	get_data();//读取电磁信号，滤波 + 归一化 + 差比和偏差
	set_servo_pwm();//设置舵机
	set_speed_pwm();//设置电机速度环	
}


//EXTI外部中断函数
void GPIO3_Combined_0_15_IRQHandler(void)
{
    if (exti_flag_get(D4))          // 判断是不是 D4 触发的中断
    {
        exti_flag_clear(D4);        // 先清标志（必须清，否则反复进中断）
			if(hall_num>180)
			{
				hall_flag = 1;
			}
        // 在这里写你的处理：比如 hall_flag = 1;
    }
}
