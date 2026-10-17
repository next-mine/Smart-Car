#ifndef _get_data_h
#define _get_data_h

#include "zf_common_headfile.h"
#include "my_common.h"

//------------------------------------------------------------------------------------------------------------------
// 采集结果（别的模块要用这些量，所以在这里统一 extern 出来，
// 不再像以前那样在各个 .c 里散着手写 extern —— 手写 extern 一旦类型写错，编译器是不报错的）
//------------------------------------------------------------------------------------------------------------------
extern int16  encoder_data_1;                           // 编码器 1 本周期的脉冲增量
extern int16  encoder_data_2;
extern int16  encoder_data_3;
extern int16  encoder_data_4;
extern int32  hall_num;


extern uint16 adc_buffer[ADC_CHANNEL_NUMBER];           // 电感原始值（已滤波，未归一化，0~4095）
extern int16  adc_normal[ADC_CHANNEL_NUMBER];           // 电感归一化值（0~100）
extern int16  adc_min[ADC_CHANNEL_NUMBER];              // 归一化下限，可运行时标定
extern int16  adc_max[ADC_CHANNEL_NUMBER];              // 归一化上限，可运行时标定

extern float  adc_error;                                // 差比和偏差，量程 ±DIFF_SUM_SCALE
extern uint8  line_lost_flag;                           // 1 = 四路电感一起丢线（分母趋零，偏差已强制为 0）
extern uint8  line_lost_flag_motor;                     // 1 = 丢线严重需停电机（阈值 LINE_LOST_MOTOR，speed_control.c 使用）
extern float  speed_now;                                // 实测速度（脉冲/周期），供速度环使用
extern uint8  hall_flag;

//------------------------------------------------------------------------------------------------------------------
// 接口
//------------------------------------------------------------------------------------------------------------------
void get_data_init(void);                               // 初始化归一化标定值，必须在使用前调用一次
void get_encoder(void);                                 // 只读编码器
void get_adc(void);                                     // 只读电感：滤波 -> 归一化 -> 差比和
void get_data(void);                                    // 上面两个一起做（中断里调这个）
void get_hall(void);																		//只读霍尔传感器实数
#endif
