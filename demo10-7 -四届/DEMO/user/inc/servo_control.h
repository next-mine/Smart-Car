#ifndef _servo_control_h
#define _servo_control_h

#include "zf_common_headfile.h"
#include "my_common.h"

extern float servo_motor_angle;                         // 当前舵机打角（角度，中值 90）
extern float kp;                                        // 舵机比例系数（P）
extern float kd;                                        // 舵机微分系数（D）

void set_servo_pwm(void);                               // 按当前偏差算打角并输出 PWM

#endif
