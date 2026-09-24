#ifndef __PID_H
#define __PID_H

/*===========================================================================
 * PID 控制器 (电机测试模式的四路闭环速度环)
 *   Velocity_A/B/C/D: 增量式速度环, 输入 (目标计数, 实测计数) -> 输出 PWM
 *                     系数 Velocity_Kp/Ki/Kd 是 float, 在 middle/PID.c 里改
 *                     单次增量限幅 ±500, 总输出限幅 ±700; A=左前 B=右前 C=右后 D=左后
 *   PID_sure:         位置环 (当前未被调用, 保留备查)
 *   注意: 循迹模式不用这里的 PID —— 循迹的转向 PD 在 middle/XunJi.c (整数定点)
 *===========================================================================*/

int Velocity_A(int Targe, int Encoder);   /* Targe = 预期计数, Encoder = 实测计数 */
int Velocity_B(int Targe, int Encoder);
int Velocity_C(int Targe, int Encoder);
int Velocity_D(int Targe, int Encoder);
int PID_sure(int Err);

#endif
