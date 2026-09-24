/*===========================================================================
 * 电机执行层 (TB6612 x 2, 四轮)
 *   轮位: A 左前 / B 右前 / C 右后 / D 左后
 *   数值域: PWM 0~1000 (period=1000), 符号只表示方向 (正=前进, 负=后退), 0=制动
 *   接口: Motor_Front_A~D / Motor_PWM_Front/Back  -> 单路或两路一起
 *         Motor_SetLeftPWM / Motor_SetRightPWM    -> 循迹层用的左右两侧
 *         Motor_Enable/Disable                    -> TB6612 STBY (PB0)
 *   换向宏: MOTOR_A~D_REVERSE (见下) —— 实测某轮转向反了就改 1, 不用改线
 *===========================================================================*/

#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "PID.h"
#define ABS(a)      (a>0 ? a:(-a))

/* ---- 换向开关: 某个轮子的实际转向和指令相反时, 把对应项改成 1, 不用改线 ---- */
#define MOTOR_A_REVERSE   0     /* 1 = A 左前轮 反向 */
#define MOTOR_B_REVERSE   0     /* 1 = B 右前轮 反向 */
#define MOTOR_C_REVERSE   1     /* 1 = C 右后轮 反向 (实测接线相反, 置1) */
#define MOTOR_D_REVERSE   1     /* 1 = D 左后轮 反向 (实测接线相反, 置1) */

/* rev 是编译期常量, 这里会被编译器直接优化掉, 不影响速度 */
static int dir_fix(int val, int rev)
{
    return rev ? -val : val;
}

/* TB6612 使能: STBY 拉高 (先把方向脚和 PWM 清零, 防止上电抽动) */
void Motor_Enable(void)
{
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,0, GPIO_Motor_PWM_Front_C0_IDX);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,0, GPIO_Motor_PWM_Front_C1_IDX);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST, 0, GPIO_Motor_PWM_Back_C0_IDX);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST, 0, GPIO_Motor_PWM_Back_C1_IDX);
	DL_GPIO_clearPins(Motor_PORT, Motor_AIN1_PIN|Motor_AIN2_PIN|Motor_BIN1_PIN|Motor_BIN2_PIN
								|Motor_CIN1_PIN|Motor_CIN2_PIN|Motor_DIN1_PIN|Motor_DIN2_PIN);
	DL_GPIO_setPins(Motor_PORT, Motor_STBY_PIN);
}

/* TB6612 关闭: STBY 拉低, 输出高阻 -> 电机自由滑行 */
void Motor_Disable(void)
{
	DL_GPIO_clearPins(Motor_PORT, Motor_STBY_PIN);
}

void Motor_PWM_Front(int PWMA,int PWMB)
{
	PWMA = dir_fix(PWMA, MOTOR_A_REVERSE);
	PWMB = dir_fix(PWMB, MOTOR_B_REVERSE);
	 DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,ABS(PWMA), GPIO_Motor_PWM_Front_C0_IDX);
	 DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,ABS(PWMB), GPIO_Motor_PWM_Front_C1_IDX);
	 if(PWMA>0)//A 左前轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_AIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_AIN2_PIN);
	 }
	 else if(PWMA==0)//A 左前轮 停止(短路制动)
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_AIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_AIN2_PIN);
	 }
	 else //A 左前轮 后退
	 {
	    DL_GPIO_clearPins(Motor_PORT,Motor_AIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_AIN2_PIN);
	 }
	 if(PWMB>0)//B 右前轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_BIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_BIN2_PIN);
	 }
	 else if(PWMB==0)//B 右前轮 停止
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_BIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_BIN2_PIN);
	 }
	 else //B 右前轮 后退
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_BIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_BIN2_PIN);
	 }

}

void Motor_PWM_Back(int PWMC,int PWMD)
{
	PWMC = dir_fix(PWMC, MOTOR_C_REVERSE);
	PWMD = dir_fix(PWMD, MOTOR_D_REVERSE);
	 DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,ABS(PWMC), GPIO_Motor_PWM_Back_C0_IDX);
	 DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,ABS(PWMD), GPIO_Motor_PWM_Back_C1_IDX);
	 if(PWMC>0)//C 右后轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_CIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_CIN2_PIN);
	 }
	 else if(PWMC==0)//C 右后轮 停止
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_CIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_CIN2_PIN);
	 }
	 else //C 右后轮 后退
	 {
	    DL_GPIO_clearPins(Motor_PORT,Motor_CIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_CIN2_PIN);
	 }
	 if(PWMD>0)//D 左后轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_DIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_DIN2_PIN);
	 }
	 else if(PWMD==0)//D 左后轮 停止
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_DIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_DIN2_PIN);
	 }
	 else //D 左后轮 后退
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_DIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_DIN2_PIN);
	 }
}

void Motor_Front_A(int PWMA)
{
	PWMA = dir_fix(PWMA, MOTOR_A_REVERSE);
	 DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,ABS(PWMA), GPIO_Motor_PWM_Front_C0_IDX);
	 if(PWMA>0)//A 左前轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_AIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_AIN2_PIN);
	 }
	 else if(PWMA==0)//A 左前轮 制动
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_AIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_AIN2_PIN);
	 }
	 else //A 左前轮 后退
	 {
	    DL_GPIO_clearPins(Motor_PORT,Motor_AIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_AIN2_PIN);
	 }
}

void Motor_Front_B(int PWMB)
{
	PWMB = dir_fix(PWMB, MOTOR_B_REVERSE);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,ABS(PWMB), GPIO_Motor_PWM_Front_C1_IDX);
	 if(PWMB>0)//B 右前轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_BIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_BIN2_PIN);
	 }
	 else if(PWMB==0)//B 右前轮 制动
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_BIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_BIN2_PIN);
	 }
	 else //B 右前轮 后退
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_BIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_BIN2_PIN);
	 }
}

void Motor_Front_C(int PWMC)
{
	PWMC = dir_fix(PWMC, MOTOR_C_REVERSE);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,ABS(PWMC), GPIO_Motor_PWM_Back_C0_IDX);
	 if(PWMC>0)//C 右后轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_CIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_CIN2_PIN);
	 }
	 else if(PWMC==0)//C 右后轮 制动
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_CIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_CIN2_PIN);
	 }
	 else //C 右后轮 后退
	 {
	    DL_GPIO_clearPins(Motor_PORT,Motor_CIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_CIN2_PIN);
	 }
}

void Motor_Front_D(int PWMD)
{
	PWMD = dir_fix(PWMD, MOTOR_D_REVERSE);
	 DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,ABS(PWMD), GPIO_Motor_PWM_Back_C1_IDX);
	 if(PWMD>0)//D 左后轮 前进
	 {
	   DL_GPIO_setPins(Motor_PORT,Motor_DIN1_PIN);
       DL_GPIO_clearPins(Motor_PORT,Motor_DIN2_PIN);
	 }
	 else if(PWMD==0)//D 左后轮 制动
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_DIN1_PIN);
		DL_GPIO_clearPins(Motor_PORT,Motor_DIN2_PIN);
	 }
	 else //D 左后轮 后退
	 {
		DL_GPIO_clearPins(Motor_PORT,Motor_DIN1_PIN);
		DL_GPIO_setPins(Motor_PORT,Motor_DIN2_PIN);
	 }
}

void Motor_Stop(void)
{
	DL_GPIO_clearPins(Motor_PORT,Motor_AIN1_PIN);
	DL_GPIO_clearPins(Motor_PORT,Motor_AIN2_PIN);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,0, GPIO_Motor_PWM_Front_C0_IDX);
	DL_GPIO_clearPins(Motor_PORT,Motor_BIN1_PIN);
	DL_GPIO_clearPins(Motor_PORT,Motor_BIN2_PIN);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST,0, GPIO_Motor_PWM_Front_C1_IDX);
	DL_GPIO_clearPins(Motor_PORT,Motor_CIN1_PIN);
	DL_GPIO_clearPins(Motor_PORT,Motor_CIN2_PIN);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,0, GPIO_Motor_PWM_Back_C0_IDX);
	DL_GPIO_clearPins(Motor_PORT,Motor_DIN1_PIN);
	DL_GPIO_clearPins(Motor_PORT,Motor_DIN2_PIN);
	DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST,0, GPIO_Motor_PWM_Back_C1_IDX);
}

/* 原地左转: A/C 后退, B/D 前进 (走 MOTOR_x_REVERSE 开关) */
void Turn_left(void)
{
	Motor_PWM_Front(-300,300);
	Motor_PWM_Back(-300,300);
}
/* 原地右转 */
void Turn_right(void)
{
	Motor_PWM_Front(300,-300);
	Motor_PWM_Back(300,-300);
}

/*---------------------------------------------------------------------------
 * 循迹用的左右两侧接口 (移植自例程 Motor_setLeftPWM/Motor_setRightPWM)
 *   左 = A 左前 + D 左后 ; 右 = B 右前 + C 右后 ; 限幅 ±999
 *   内部走单路函数, 所以 MOTOR_x_REVERSE 换向开关依旧生效
 *-------------------------------------------------------------------------*/
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
