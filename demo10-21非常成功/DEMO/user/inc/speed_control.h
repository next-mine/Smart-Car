#ifndef _speed_control_h
#define _speed_control_h

#include "zf_common_headfile.h"
#include "my_common.h"

extern int8  duty;                                      // 当前输出占空比（0 ~ MAX_PWM_DUTY），供屏幕显示
extern float speed_target;                              // 目标速度（脉冲/周期）
extern float speed_kp;                                  // 速度环比例系数
extern float speed_ki;                                  // 速度环积分系数
extern uint8 speed_fault;                               // 速度环故障标志：1 = 反馈符号异常，已锁停输出

void set_speed_pwm(void);                               // 跑一次速度环并输出电机 PWM
void speed_loop_reset(void);                            // 复位速度环内部状态与故障标志

#endif
