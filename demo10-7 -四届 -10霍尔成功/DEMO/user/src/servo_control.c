#include "zf_common_headfile.h"
#include "my_common.h"
#include "servo_control.h"
#include "get_data.h"

//定义舵机打角值
float servo_motor_angle = SERVO_MOTOR_M;
//PID系数kp
// ★ 这个 kp 原样保留（0.008），没有跟着差比和一起改。
//   能沿用的原因是 my_common.h 里把 DIFF_SUM_SCALE 取成了 4000：
//   原来的 raw 差 ≈ 4095 × 偏差比例，新的差比和 ≈ 偏差比例，乘上各自的系数后量程和斜率都对齐，
//   所以 0.008 继续有效，最大打角依然在 ±15° 限幅附近收住。
//   换句话说：这里不动 kp，是因为误差的量纲被刻意对齐了，而不是碰巧没改。
//4000*0.008=32°，早就已经打满了
float kp = Kp1;

// 舵机微分系数 kd（PD 打角里的 D 项）。打角 = 中值 - kp*偏差 - kd*(偏差 - 上次偏差)。
// D 项对"偏差变化率"反应，相当于提前预判：弯道里偏差还在快速增大时，就提前多打一点角，
// 让打角更"跟手"，这是"弯道提速同时不偏离"在打角侧的保障。初值见 my_common.h 的 SERVO_KD。
float kd = SERVO_KD;

// 上一拍的偏差，D 项要用它做差分。static 关在文件里，避免被别的模块误写。
static float last_adc_error = 0.0f;
uint8 round_pre_in_count=0;																				//用于滤波检测
uint8 round_true_in_count=0;																		
uint8 round_pre_in_flag=0;																			//置为1则进入环岛，强制打角
uint8 round_true_in_flag=0;
uint8 round_already_in_flag=0;

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
	if(adc_normal[0]>=adc_round_outer_pre_in && adc_normal[3]>=adc_round_outer_pre_in)//当两侧电感值同时增大至大于阈值时，开始预判断进环
	{
		round_pre_in_count++;
	}
	if(round_pre_in_count>=2)//两次大于阈值后再下一步（相当于滤波）
	{
		if(adc_normal[1]+adc_normal[2]>=adc_round_inner_pre_in)//再向前前进时，内测电感值也会同时增大
		{
			round_pre_in_flag=1;//预进环标志位置1
		}
		if(adc_normal[1]+adc_normal[2]<=adc_round_inner_true_in&&round_pre_in_flag==1)//继续向前，外侧电感减小值最低位值adc_normal[0]<=adc_round_outer_true_in && adc_normal[3]<=adc_round_outer_true_in
	  {
		round_true_in_count++;
	  }
		if(round_true_in_count>=2 && round_pre_in_flag==1)
		{
		//adc_error=DIFF_SUM_SCALE*(adc_normal[1]-adc_normal[2])/(adc_normal[1]+adc_normal[2]);
		round_true_in_flag=1;
		}
		//第一个问题：到底有没有判断入环成功  第二个：入环成功后到底要执行什么操作  第三个：IMU到底要不要用、该怎么用？
	}
	
	
	
	if(adc_normal[3]<50&&adc_normal[0]<30)
	{
		round_already_in_flag=1;
	}
	if(round_already_in_flag==1)
	{
		kp=Kp2;
	}
	
	float error_rate;

	// 偏差变化率 = 本拍偏差 - 上拍偏差。D 项就靠这个量提前感知"偏差正在变大（正在进弯）"。
	error_rate = adc_error - last_adc_error;
	last_adc_error = adc_error;

	// 根据偏差 + 偏差变化率计算舵机打角（PD）
	servo_motor_angle = SERVO_MOTOR_M - kp * adc_error - kd * error_rate;

	//打角限幅：舵机机械行程有限，超出范围会堵转、发热，长期堵转会扫齿。
	// 这一步不是可选项，去掉限幅就等于把舵机往死里顶。
	if(servo_motor_angle > SERVO_MOTOR_R_MAX)servo_motor_angle = SERVO_MOTOR_R_MAX;
	if(servo_motor_angle < SERVO_MOTOR_L_MAX)servo_motor_angle = SERVO_MOTOR_L_MAX;

	//正式进环后要进行的操作
	if(round_true_in_flag==1&&round_already_in_flag==0)
	{
		servo_motor_angle = 100;
	}

	
	//设置舵机打角。三路舵机写的是同一个值，只是为了适配"舵机到底插在哪个引脚"不确定的情况，
	// 实际接线只会用到其中一路，另外两路可以删掉（留着也不会有什么坏处）。
	pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
	pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
