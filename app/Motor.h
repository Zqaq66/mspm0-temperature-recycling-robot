#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "ti_msp_dl_config.h"

/* 轮位: A=左前  B=右前  C=右后  D=左后
   PWM : A=PA12(TIMG0_CCP0) B=PA13(TIMG0_CCP1) C=PA17(TIMG7_CCP0) D=PA7(TIMG7_CCP1)
   方向: A=PB19/PB20  B=PB4/PB5  C=PB6/PB7  D=PB12/PB13
   使能: TB6612 STBY = PB0 (高电平使能)
   PWM 数值域 0~1000, 符号只表示方向(正=前进, 负=后退), 0=制动 */

void Motor_PWM_Front(int PWMA,int PWMB);    /* A / B 两路一起 */
void Motor_PWM_Back(int PWMC,int PWMD);     /* C / D 两路一起 */
void Motor_Front_A(int PWMA);               /* 单路 (0=制动) */
void Motor_Front_B(int PWMB);
void Motor_Front_C(int PWMC);
void Motor_Front_D(int PWMD);
void Motor_Stop(void);
void Motor_Brake(void);                     /* 短路制动: 停得住 (区别于 Stop 的滑行) */                      /* 四路停 + 方向脚清零 */
void Motor_Enable(void);                    /* STBY=1, 使能 TB6612 (先把 PWM/方向清零) */

/* 循迹用: 左右两侧一起给 (左 = A左前 + D左后, 右 = B右前 + C右后), 内部限幅 ±999
   —— middle/XunJi.c 的循迹层就调这两个; 单轮换向宏 MOTOR_x_REVERSE 在这里同样生效 */
void Motor_SetLeftPWM(int pwm);
void Motor_SetRightPWM(int pwm);
void Motor_Disable(void);                   /* STBY=0, 关闭输出(电机自由滑行) */
void Turn_left(void);                       /* 原地转, 开环 ±300 */
void Turn_right(void);

#endif
