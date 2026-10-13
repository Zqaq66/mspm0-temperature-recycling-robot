/*===========================================================================
 * 电机执行层 (TB6612 x 2, 四轮)
 *   轮位: A 左前 / B 右前 / C 右后 / D 左后
 *   PWM 0~1000 (period=1000), 符号只表示方向 (正=前进, 负=后退), 0=短路制动
 *   换向宏: MOTOR_A~D_REVERSE —— 实测某轮转向反了就改 1, 不用改线
 *   循迹层只用 Motor_SetLeftPWM / Motor_SetRightPWM (左 = A+D, 右 = B+C)
 *===========================================================================*/

#include "ti_msp_dl_config.h"
#include "Motor.h"

#define MOTOR_A_REVERSE   0     /* A 左前 */
#define MOTOR_B_REVERSE   0     /* B 右前 */
#define MOTOR_C_REVERSE   1     /* C 右后 (实测接线相反) */
#define MOTOR_D_REVERSE   1     /* D 左后 (实测接线相反) */

/* 方向脚输出: >0 -> IN1 高, IN2 低 (前进) / ==0 -> 都低 (制动) / <0 -> IN2 高 (后退) */
static void dir_out(uint32_t i1, uint32_t i2, int v)
{
    if (v > 0)       { DL_GPIO_setPins(Motor_PORT, i1);   DL_GPIO_clearPins(Motor_PORT, i2); }
    else if (v == 0) { DL_GPIO_clearPins(Motor_PORT, i1); DL_GPIO_clearPins(Motor_PORT, i2); }
    else             { DL_GPIO_clearPins(Motor_PORT, i1); DL_GPIO_setPins(Motor_PORT, i2); }
}

void Motor_Front_A(int pwm)     /* A 左前 */
{
    if (MOTOR_A_REVERSE) pwm = -pwm;
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, (pwm > 0) ? pwm : -pwm,
                                     GPIO_Motor_PWM_Front_C0_IDX);
    dir_out(Motor_AIN1_PIN, Motor_AIN2_PIN, pwm);
}

void Motor_Front_B(int pwm)     /* B 右前 */
{
    if (MOTOR_B_REVERSE) pwm = -pwm;
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, (pwm > 0) ? pwm : -pwm,
                                     GPIO_Motor_PWM_Front_C1_IDX);
    dir_out(Motor_BIN1_PIN, Motor_BIN2_PIN, pwm);
}

void Motor_Front_C(int pwm)     /* C 右后 */
{
    if (MOTOR_C_REVERSE) pwm = -pwm;
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST, (pwm > 0) ? pwm : -pwm,
                                     GPIO_Motor_PWM_Back_C0_IDX);
    dir_out(Motor_CIN1_PIN, Motor_CIN2_PIN, pwm);
}

void Motor_Front_D(int pwm)     /* D 左后 */
{
    if (MOTOR_D_REVERSE) pwm = -pwm;
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST, (pwm > 0) ? pwm : -pwm,
                                     GPIO_Motor_PWM_Back_C1_IDX);
    dir_out(Motor_DIN1_PIN, Motor_DIN2_PIN, pwm);
}

/* TB6612 使能: STBY 拉高 (先把 PWM/方向清零, 防止上电抽动) */
void Motor_Enable(void)
{
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 0, GPIO_Motor_PWM_Front_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 0, GPIO_Motor_PWM_Front_C1_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,  0, GPIO_Motor_PWM_Back_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,  0, GPIO_Motor_PWM_Back_C1_IDX);
    DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN|Motor_AIN2_PIN|Motor_BIN1_PIN|Motor_BIN2_PIN
                                |Motor_CIN1_PIN|Motor_CIN2_PIN|Motor_DIN1_PIN|Motor_DIN2_PIN);
    DL_GPIO_setPins(Motor_PORT, Motor_STBY_PIN);
}

/* 全部输出关断: 方向脚清零 + PWM 0 (电机自由滑行) */
void Motor_Stop(void)
{
    DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN|Motor_AIN2_PIN|Motor_BIN1_PIN|Motor_BIN2_PIN
                                |Motor_CIN1_PIN|Motor_CIN2_PIN|Motor_DIN1_PIN|Motor_DIN2_PIN);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 0, GPIO_Motor_PWM_Front_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 0, GPIO_Motor_PWM_Front_C1_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,  0, GPIO_Motor_PWM_Back_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,  0, GPIO_Motor_PWM_Back_C1_IDX);
}

/* 短路制动: IN1=IN2=1 + PWM 满 -> 绕组被下桥短路, 电磁刹车 (IMU 转到位后立即停住用) */
void Motor_Brake(void)
{
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 1000, GPIO_Motor_PWM_Front_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 1000, GPIO_Motor_PWM_Front_C1_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,  1000, GPIO_Motor_PWM_Back_C0_IDX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,  1000, GPIO_Motor_PWM_Back_C1_IDX);
    DL_GPIO_setPins(Motor_PORT, Motor_AIN1_PIN|Motor_AIN2_PIN|Motor_BIN1_PIN|Motor_BIN2_PIN
                              |Motor_CIN1_PIN|Motor_CIN2_PIN|Motor_DIN1_PIN|Motor_DIN2_PIN);
}

void Motor_SetLeftPWM(int pwm)
{
    if (pwm >=  999) pwm =  999;
    if (pwm <= -999) pwm = -999;
    Motor_Front_A(pwm);
    Motor_Front_D(pwm);
}

void Motor_SetRightPWM(int pwm)
{
    if (pwm >=  999) pwm =  999;
    if (pwm <= -999) pwm = -999;
    Motor_Front_B(pwm);
    Motor_Front_C(pwm);
}
