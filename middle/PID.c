#include "ti_msp_dl_config.h"
#define ABS(a)      (a>0 ? a:(-a))
/*****************************速度环************************/
/*相关速度PID参数*/
float Velocity_Kp=5,
      Velocity_Ki=1,//0.3
	  Velocity_Kd=0;//0.021
//float Velocity_Kp=18.5,Velocity_Ki=0.1,Velocity_Kd=0.020;
/*****************************位置环***********************/
float Kp_P=0.5,
	  Ki_P=0.1,
      Kd_P=0.3;
int Output=0;
/*角度环的PID参数*/
float Kp_angle =5.1, Ki_angle =0.05, Kd_angle =0.55; // PID参数//Ki_angle = 0.05
/*********************************************
函数功能:电机的速度PID闭环控制
参数:左右电机的编码器值
返回值:电机的PWM
*********************************************/
int Velocity_A(int Targe/*预期值*/,int Encoder/*读取的编码值*/)//A 左前轮
{
	 static int Output = 0;          // 最终输出PWM值
     static int Error[2] = {0, 0};   // 存储当前误差和上一次误差
    
    // 计算当前误差
    int CurrentError = Targe - Encoder;
    
    // 标准增量式PID公式实现
    int DeltaOutput = Velocity_Kp * (CurrentError - Error[0]) +  // 比例项
                      Velocity_Ki * CurrentError +                // 积分项
                      Velocity_Kd * (CurrentError - 2 * Error[0] + Error[1]);  // 微分项
    
    // 输出限幅 - 防止过大变化
    if (DeltaOutput > 500) DeltaOutput = 500;
    if (DeltaOutput < -500) DeltaOutput = -500;
    
    // 累加增量到输出
    Output += DeltaOutput;
    
    // 最终输出限幅 - 根据电机PWM范围调整
    if (Output > 700) Output = 700;
    if (Output < -700) Output = -700;
    
    // 更新误差历史
    Error[1] = Error[0];
    Error[0] = CurrentError;
    
    return Output;
}
/*********************************************
函数功能:电机的速度PID闭环控制
参数:左右电机的编码器值
返回值:电机的PWM
*********************************************/
int Velocity_B(int Targe/*预期值*/,int Encoder/*读取的编码值*/)//B 右前轮
{
	 static int Output = 0;          // 最终输出PWM值
     static int Error[2] = {0, 0};   // 存储当前误差和上一次误差
    
    // 计算当前误差
    int CurrentError = Targe - Encoder;
    
    // 标准增量式PID公式实现
    int DeltaOutput = Velocity_Kp * (CurrentError - Error[0]) +  // 比例项
                      Velocity_Ki * CurrentError +                // 积分项
                      Velocity_Kd * (CurrentError - 2 * Error[0] + Error[1]);  // 微分项
    
    // 输出限幅 - 防止过大变化
    if (DeltaOutput > 500) DeltaOutput = 500;
    if (DeltaOutput < -500) DeltaOutput = -500;
    
    // 累加增量到输出
    Output += DeltaOutput;
    
    // 最终输出限幅 - 根据电机PWM范围调整
    if (Output > 700) Output = 700;
    if (Output < -700) Output = -700;
    
    // 更新误差历史
    Error[1] = Error[0];
    Error[0] = CurrentError;
    
    return Output;
}
/*********************************************
函数功能:电机的速度PID闭环控制
参数:左右电机的编码器值
返回值:电机的PWM
*********************************************/
int Velocity_C(int Targe/*预期值*/,int Encoder/*读取的编码值*/)//C 右后轮
{
	 static int Output = 0;          // 最终输出PWM值
     static int Error[2] = {0, 0};   // 存储当前误差和上一次误差
    
    // 计算当前误差
    int CurrentError = Targe - Encoder;
    
    // 标准增量式PID公式实现
    int DeltaOutput = Velocity_Kp * (CurrentError - Error[0]) +  // 比例项
                      Velocity_Ki * CurrentError +                // 积分项
                      Velocity_Kd * (CurrentError - 2 * Error[0] + Error[1]);  // 微分项
    
    // 输出限幅 - 防止过大变化
    if (DeltaOutput > 500) DeltaOutput = 500;
    if (DeltaOutput < -500) DeltaOutput = -500;
    
    // 累加增量到输出
    Output += DeltaOutput;
    
    // 最终输出限幅 - 根据电机PWM范围调整
    if (Output > 700) Output = 700;
    if (Output < -700) Output = -700;
    
    // 更新误差历史
    Error[1] = Error[0];
    Error[0] = CurrentError;
    
    return Output;
}
/*********************************************
函数功能:电机的速度PID闭环控制
参数:左右电机的编码器值
返回值:电机的PWM
*********************************************/
int Velocity_D(int Targe/*预期值*/,int Encoder/*读取的编码值*/)//D 左后轮
{
	 static int Output = 0;          // 最终输出PWM值
     static int Error[2] = {0, 0};   // 存储当前误差和上一次误差
    
    // 计算当前误差
    int CurrentError = Targe - Encoder;
    
    // 标准增量式PID公式实现
    int DeltaOutput = Velocity_Kp * (CurrentError - Error[0]) +  // 比例项
                      Velocity_Ki * CurrentError +                // 积分项
                      Velocity_Kd * (CurrentError - 2 * Error[0] + Error[1]);  // 微分项
    
    // 输出限幅 - 防止过大变化
    if (DeltaOutput > 500) DeltaOutput = 500;
    if (DeltaOutput < -500) DeltaOutput = -500;
    
    // 累加增量到输出
    Output += DeltaOutput;
    
    // 最终输出限幅 - 根据电机PWM范围调整
    if (Output > 700) Output = 700;
    if (Output < -700) Output = -700;
    
    // 更新误差历史
    Error[1] = Error[0];
    Error[0] = CurrentError;
    
    return Output;
}
int PID_sure(int Err)
{
    static int error, prev_error = 0;  // 误差
    
	static int integral = 0, derivative = 0;
    
    // 保存上一次误差
    prev_error = error;
    
    // 计算当前误差
    error = Err;
    
    // 计算积分项
    if (ABS(error) < 10) {  
        integral += error;
        
        // 积分限幅，防止积分饱和
        if (integral > 24) integral = 24;
        if (integral < -24) integral = -24;
    } else {
        // 大误差时逐渐减小积分
        integral = (integral * 9) / 10;
    }
    
    // 计算微分项（误差变化率）
    derivative = error - prev_error;
    
    // 计算PID输出 - 加入微分项
    Output = Kp_P * error + 
             Ki_P * integral + 
             Kd_P * derivative;
    
    // 输出限幅
    if (Output > 32) Output = 32;
    if (Output < -32) Output = -32;
	return Output;
}
/*********************************************
函数功能:电机的角度PID闭环控制
参数:预期的角度值,实测的角度值，左右轮的PWM值
返回值:左右电机的angle_output值
*********************************************/
float Angle_PID_Update(float target_yaw,float Yaw_T)
{
    static float prev_error = 0, integral = 0;
//    static float angle_output = 0;     // 角度环输出;
    float error = target_yaw - Yaw_T;
    
    // 角度归一化 (-180° to 180°)
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
         // 当误差很小时清零积分项
        // 积分项（带限幅和抗饱和）
        integral += error;
        if (integral > 100) integral = 100;
        if (integral < -100) integral = -100;
    
    // 微分项（考虑时间变化）
    float derivative = (error - prev_error);
    prev_error = error;
    
    // PID计算
    float angle_output = Kp_angle * error+
                         Ki_angle * integral + 
                         Kd_angle * derivative;
    
    // 输出限幅（根据基础速度动态调整）
    float max_output = 215 * 0.5; // 最大转向输出为基础速度的80%
    if (angle_output > max_output) angle_output = max_output;
    if (angle_output < -max_output) angle_output = -max_output;
    return angle_output;
//    // 正确的输出应用：基于基础速度进行差速控制
//    left_duty_T = L_PWM - angle_output;
//    right_duty_T =R_PWM + angle_output;
//    
//    // 电机速度限幅（防止超速）
//    if (left_duty_T > 367) left_duty_T = 367;
//    if (left_duty_T < -367) left_duty_T = -367;
//    if (right_duty_T > 315) right_duty_T = 315;
//    if (right_duty_T < -315) right_duty_T = -315;
}