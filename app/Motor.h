#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "ti_msp_dl_config.h"

/* 轮位: A=左前  B=右前  C=右后  D=左后
   PWM : A=PA12(TIMG0_CCP0) B=PA13(TIMG0_CCP1) C=PA17(TIMG7_CCP0) D=PA7(TIMG7_CCP1)
   方向: A=PB19/PB20  B=PB4/PB5  C=PB6/PB7  D=PB12/PB13    使能: STBY=PB0 (高=使能)
   PWM 0~1000 (period=1000), 符号只表示方向 (正=前进, 负=后退), 0=短路制动 */

void Motor_Front_A(int pwm);                /* 单路 (0 = 制动) */
void Motor_Front_B(int pwm);
void Motor_Front_C(int pwm);
void Motor_Front_D(int pwm);

void Motor_SetLeftPWM(int pwm);             /* 循迹用: 左 = A+D, 右 = B+C, 内部限幅 ±999 */
void Motor_SetRightPWM(int pwm);

void Motor_Stop(void);                      /* 四路停 + 方向脚清零 (自由滑行) */
void Motor_Brake(void);                     /* 短路制动: 停得住 (IMU 转到位后用) */
void Motor_Enable(void);                    /* STBY=1, 使能 TB6612 (先把 PWM/方向清零) */

#endif
