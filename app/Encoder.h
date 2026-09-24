#ifndef __ENCODER_H
#define __ENCODER_H

#define ABS(a)      (a>0 ? a:(-a))

typedef struct{
    int Should_Get_Encoder_Count;   // 将要获得的编码器计数(中断里累加)
    int Obtained_Get_Encoder_Count; // 换出来的本周期计数(20ms 一次)
}Encoder;

void Encoder_Init(void);        /* 使能 GPIOA 编码器中断 */
void Encoder_Update(void);      /* 20ms 采样: 换出计数并清零 */
int Motor_Get_Encoder_A(void);  /* A 左前轮 本周期的编码器计数 */
int Motor_Get_Encoder_B(void);
int Motor_Get_Encoder_C(void);
int Motor_Get_Encoder_D(void);

#endif
