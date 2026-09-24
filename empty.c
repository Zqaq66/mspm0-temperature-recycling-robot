/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*===========================================================================
 * MSPM0G3507 四轮小车 —— 主程序 (main + 协作式调度器 + 两套应用层)
 *
 *   本文件: System_Init() / main() / 5·10·20·100·1000ms 任务 / 循迹 与 电机测试 两个模式
 *   两套固件二选一, 只改 [1] 里的 APP_MODE 一行; 调度器在 middle/Task.c, 时基在 middle/Timer.c
 *
 * ============================ [1] 模式与开关 ============================
 *   APP_MODE      = APP_MODE_TRACK (默认) | APP_MODE_MOTOR_TEST   ← 切固件只改这一行
 *   app_mode      = APP_MODE 的运行期副本 (各处都判它; 运行期没人改, 不是热切换)
 *   app_run       = 0 停 / 1 跑, 由板载 S2 单击切换 (上电默认 0 = 停)
 *   SKIP_OPEN_LOOP= 仅电机模式有效; 1 = 上电直接进闭环速度环 (跳过 26.5s 开环自检)
 *
 * ============================ [2] 任务划分 (middle/Task.c 调度) ============================
 *   周期     循迹模式 (TRACK)                      电机测试模式 (MOTOR_TEST)
 *   --------------------------------------------------------------------------------------
 *   5ms      Track_Run(): 读灰度+质心+PD+写PWM     —
 *   10ms     Key_Scan(): 单击启停 / 长按调速        同左 (调的是闭环目标)
 *   20ms     只采样编码器, 不写 PWM                 开环阶段表 / 四路闭环速度环, 写 PWM
 *   100ms    LCD_track()                           阶段推进 + LCD_status()
 *   1000ms   串口日志 (默认整段注释掉)              同左
 *   时基: TIMER_0 = 1ms 中断 -> Get_Time();  printf 出口 = LORA 口 UART3 (PB2/PB3, 9600)
 *
 * ============================ [3] 板载按键 (只用 S2 = PB21, 无需接线) ============================
 *   单击 (按住 <0.6s 松开)  ->  开始 / 停止           屏上: STOP press S2
 *   长按 (按住 >0.6s)       ->  加速一档, 之后每 0.1s 再加一档; 到上限自动回绕到下限
 *   循迹模式: 调基础速度 150~900 (默认 400)          屏上: RUN Bxxx
 *   电机模式: 调闭环目标   5~200 (默认 TARGET_SPEED) 屏上: TGT xxx CLOSED
 *   调默认值: middle/XunJi.h  TRACK_SPEED_DEFAULT / empty.c  TARGET_SPEED
 *   调手感:   middle/Key.c    KEY_LONG_TICKS(长按 60=600ms) / KEY_REPEAT_TICKS(连发 10=100ms)
 *   不想回绕: middle/XunJi.c  Track_StepSpeed() 里改成封顶 (注释里有写法)
 *
 * ============================ [4] 接线表 ============================
 *
 *   [4.1] 电源 (先接电源, 再接信号)
 *     12V 电池 +     ->  TB6612 #1 的 VM 和 #2 的 VM (两片 VM 并联)
 *     12V 电池 -     ->  MCU GND              (必须共地)
 *     LaunchPad 3V3  ->  两片 VCC             (逻辑电源必须 3.3V, 不能接 5V:
 *                                             TB6612 逻辑门限 = 0.7*VCC, 接5V时 3.5V > 3.3V 会识别不稳)
 *     LaunchPad GND  ->  两片 GND
 *     注意: 12V 绝不进 MCU / 3V3
 *
 *   [4.2] TB6612 #1 —— 前桥 (A 左前 / B 右前)
 *     TB6612 引脚        接 MCU                      接电机
 *     ------------------------------------------------------------------
 *     PWMA               PA12 (TIMG0_CCP0, 20kHz)
 *     AIN1               PB19
 *     AIN2               PB20
 *     AO1 / AO2                                     A 左前电机 + / -
 *     PWMB               PA13 (TIMG0_CCP1, 20kHz)
 *     BIN1               PB4
 *     BIN2               PB5
 *     BO1 / BO2                                     B 右前电机 + / -
 *     STBY               PB0  (两片 STBY 拧在一起接 PB0)
 *     VM / VCC / GND     见 [4.1]
 *     注: 芯片丝印的 A/B 通道 不是 轮位 A/B; #1 片的 A 通道驱动 A 轮, B 通道驱动 B 轮
 *
 *   [4.3] TB6612 #2 —— 后桥 (C 右后 / D 左后)
 *     PWMA               PA17 (TIMG7_CCP0, 20kHz)
 *     AIN1               PB6
 *     AIN2               PB7
 *     AO1 / AO2                                     C 右后电机 + / -
 *     PWMB               PA7  (TIMG7_CCP1, 20kHz)
 *     BIN1               PB12
 *     BIN2               PB13
 *     BO1 / BO2                                     D 左后电机 + / -
 *     STBY               PB0  (和 #1 的 STBY 并联)
 *     不想用 GPIO 控 STBY: 两片 STBY 直接接 3V3 (常使能), Motor_Enable() 就失效
 *
 *   [4.4] 编码器 (每路: A相 / B相 两根信号线, 另加 VCC / GND 供电)
 *     A 左前:  A相 -> PA21    B相 -> PA26      VCC -> 3V3, GND -> GND
 *     B 右前:  A相 -> PA31    B相 -> PA28      VCC -> 3V3, GND -> GND
 *     C 右后:  A相 -> PA24    B相 -> PA25      VCC -> 3V3, GND -> GND
 *     D 左后:  A相 -> PA14    B相 -> PA15      VCC -> 3V3, GND -> GND
 *     - 编码器电源用 3V3 (模块若必须 5V, A/B 相要分压或加电平转换)
 *     - 这 8 个脚是 RESISTOR_NONE (无内部上下拉); 编码器若为开漏输出, A/B 各加 10k 上拉到 3V3
 *     - A/B 相接反不用拆线: 改 app/Encoder.c 里的 ENC_x_REVERSE
 *     - 电机线 M+ / M- 接 TB6612 的 AO1/AO2 (或 BO1/BO2), 别接到编码器的信号线上
 *
 *   [4.5] 灰度传感器 (循迹模式用; 芯流 AuroraT8 八路)
 *     SDA -> PA0    SCL -> PA1    (硬件 I2C0, 400kHz, 模块地址 0x40)
 *     VCC -> 3V3    GND -> 共地    (用 3V3 可避免 I2C 上拉被抬到 5V)
 *     - 与 MLX90614 (0x5A) 共用同一条 I2C0 总线
 *     - 探头朝下, 8 路垂直于前进方向横排, 居中安装
 *
 *   [4.6] LCD (ST7735S 128x160, 硬件 SPI1)
 *     SCLK -> PB9     SDA/MOSI -> PB8     RES -> PB10
 *     DC   -> PB11    CS -> PB14          BLK -> PB26      VCC -> 3V3, GND -> 共地
 *
 *   [4.7] 上电前检查
 *     1) 万用表量 VM-GND / VCC-GND 不短路; VCC 实测 3.3V, VM 实测 12V
 *     2) 电池负 与 LaunchPad GND 导通 (共地)
 *     3) 四个轮子悬空再上电
 *
 * ============================ [5] 循迹模式 (APP_MODE_TRACK) ============================
 *   读法:   5ms 读 0x09 八路模拟量 (0~255, 除8数据, 越大越黑) —— 不读 0x0C 数字量
 *   算法:   平方加权质心  pos = Σ(v*v*i*100) / Σ(v*v)   -> 0~700, 中心 350 (亚路分辨率)
 *             error = pos - 350                          (负 = 线偏左)
 *           三区增益调度 PD: |err|<40 用直线软增益, >=160 用弯道激进增益, 中间线性过渡
 *             turn = (Kp*error + Kd*低通Δerror) / 1024   (Q10 定点)
 *           弯道自适应降速: 弯得越深速度越低 (最多降到 50%)
 *           左 = speed + turn, 右 = speed - turn          (turn>0 = 线偏右 -> 左轮快 = 右转)
 *           左 = A 左前 + D 左后 ; 右 = B 右前 + C 右后   (Motor_SetLeftPWM/SetRightPWM)
 *   丢线:   丢线瞬间按"丢线前最后一帧的 |error|"分流 (阈值 LOST_SIDE_ERR = 120):
 *             A 偏出 (|E| >= 120): 线被甩到一侧 -> 直接按偏向差速转弯找回, 不直行、不原地转
 *                                   (内侧轮慢 + 外侧轮快, 两轮同向);
 *                                   外侧 = LOST_TURN_SPEED(250), 差速 = LOST_TURN_DIFF(150);
 *                                   找超过 LOST_FIND_TIMEOUT_MS(2500ms) 仍未见到线 -> 停车
 *             B 中间消失 (|E| < 120): 线在中间突然没了 -> 保持直行 LOST_STRAIGHT_MS(500ms) -> 停车
 *             丢线头 60ms (LOST_KEEP_MS) 先保持上一拍输出 (防单帧误丢乱打);
 *             线一回来立刻自动恢复巡线 (不用按键)
 *   屏上:   第1行 8 路数字量 ('#'=压线, 左到右; 本地由阈值算)
 *           第2行 P质心(0~700) E误差(负=线偏左)
 *           第3行 L左轮PWM V弯道降速后的实际速度
 *           第4行 R右轮PWM
 *           第5行 STOP press S2 / LOST nnn(丢线时长 ms) / RUN Bxxx(按键设定的基础速度)
 *           第6行 I2CERR (必须恒为 0, 非 0 = I2C 没通)
 *   调参:   全部在 middle/XunJi.c 顶部 (质心阈值/极性/镜像/LR、PD 与过渡带、D 低通、
 *           降速比、丢线参数) —— 见 [7] 的顺序
 *   注:     循迹用的是开环 PWM; 想接编码器速度环, 把 Motor_SetLeft/RightPWM() 换成写
 *           Target_A/B/C/D 即可 (当前没接, 麦轮打滑会让速度环失真)
 *
 * ============================ [6] 电机测试模式 (APP_MODE_MOTOR_TEST) ============================
 *   单击 S2 开始后自动跑 (开环全程约 26.5 秒), 屏第 1 行 = 阶段 + 名字, 第 6 行 = OPEN/CLOSED:
 *     1 - 4    只有 A 左前: 正转 -> 停 -> 反转 -> 停            (1.2s / 0.8s)
 *     5 - 8    只有 B 右前: 同上
 *     9 - 12   只有 C 右后: 同上
 *     13 - 16  只有 D 左后: 同上
 *     17 - 20  四轮一起: 前进 -> 停 -> 后退 -> 停               (2s / 1s)
 *     21 - 22  四轮原地左转 / 右转                           (1.5s)
 *     23       全停                                         (1.5s)
 *     24+      闭环速度环: 四轮一直转, 第 6 行变 TGT 40 CLOSED
 *             (SKIP_OPEN_LOOP=1 时跳过 1~23 段, 一上电就是这个状态)
 *
 *   看现象定位问题:
 *     某个阶段动了别的轮子          -> 通道 / 接线错位
 *     标 FWD 却反转                 -> app/Motor.c 里 MOTOR_x_REVERSE 改 1
 *     闭环阶段 SPD 为负 / 一顿一顿  -> app/Encoder.c 里 ENC_x_REVERSE 改 1
 *     某轮 SPD 恒为 0               -> 该路编码器没接 / 相线接错
 *
 *   调参入口: empty.c 的 TARGET_SPEED 和 steps[] 表; middle/PID.c 的 Velocity_Kp/Ki/Kd
 *
 * ============================ [7] 调参顺序 (按这个顺序来, 一次只改一个) ============================
 *
 *   0) 先定方向 —— 还没到调 PID 这步:
 *        某轮转向与指令相反             -> app/Motor.c     MOTOR_A/B/C/D_REVERSE 改 1
 *        某轮实测速度为负 / 闭环抖       -> app/Encoder.c   ENC_A/B/C/D_REVERSE 改 1
 *        灰度探头左右装反(挡左边右边亮)   -> middle/XunJi.c  TRACK_SENSOR_FLIP 改 1
 *        越偏越往错方向打(纠正方向反)     -> middle/XunJi.c  TRACK_LR_SWAP 改 1
 *        白底数值反而比黑线大(极性反)     -> middle/XunJi.c  TRACK_DARK_IS_HIGH 改 0
 *
 *   1) 传感器自检 (车架起来, 单击 S2 先别让它跑): 屏第1行 8 个符号随黑线变 '#'
 *        拿黑线依次挡每个探头, 位置一一对应;  I2CERR 必须恒为 0 (非 0 = I2C 没通)
 *
 *   2) 基础速度 (单击 S2 开始 / 长按加速, 屏上 RUN Bxxx):
 *        先在 250 ~ 400 把循迹调稳, 稳了再往上提; 提速后 Kp 一般要同步降一点
 *        屏上 Vxxx 是"弯道降速后"的实际速度: 过弯时它自己掉下来 = 降速在工作
 *
 *   3) 转向 PD (Q10 定点, 1024 = 1.0) —— middle/XunJi.c 顶部
 *        TRACK_KP / TRACK_KD          弯道段增益 (默认 1000 / 1200)
 *        TRACK_STRAIGHT_KP / KD       直线段增益 (默认 500 / 600, 用来压直线微振)
 *        TRANSITION_LOW / HIGH        过渡带 (默认 40 / 160), 中间线性插值, 不分档跳变
 *        D_FILTER_STRENGTH            D 项低通强度 (默认 3 = 1/8)
 *        CURVE_REDUCE_PCT             弯道最大降速% (默认 50)
 *        直线画龙        -> TRACK_STRAIGHT_KP 调小 (500 -> 300)
 *        弯道切内线/出线 -> TRACK_KP 调大 (1000 -> 1300), 同时把基础速度降一档
 *        过冲/回摆大     -> TRACK_KD 调大 (1200 -> 1800), 或把 D_FILTER_STRENGTH 加到 4
 *        过弯太慢/太急   -> CURVE_REDUCE_PCT 调小 / 调大
 *        屏上 P 跳变、E 不连续 -> NOISE_THRESHOLD (默认 30) 太大, 或极性反了
 *
 *   4) 丢线参数 (middle/XunJi.c 顶部):
 *        LOST_SIDE_ERR(120)        判"偏出"还是"中间消失"的 |error| 分界
 *        LOST_TURN_SPEED(250)      找回线外侧轮 PWM (别超过正常巡线速度)
 *        LOST_TURN_DIFF(150)       找回线内外差速 (内侧 = 250-150 = 100, 同向不反转)
 *        LOST_FIND_TIMEOUT_MS(2500) 偏出找回多久还没找到就停车 (填 0 = 一直找)
 *        LOST_STRAIGHT_MS(500)     中间消失后直行多久停车
 *        LOST_KEEP_MS(60)          丢线头去抖窗口 (保持上一拍输出)
 *        屏上 LOST nnn 是丢线时长(ms): 一直涨 = 没找回; 停在 500 附近 = 中间消失已停车;
 *        停在 2500 附近 = 偏出找超时停车
 *
 *   5) 闭环速度环 (只跑电机测试模式; SKIP_OPEN_LOOP=1 上电直接进闭环)
 *        屏上 SPD 追不上 TGT   -> TARGET 调低, 或 middle/PID.c 的 Velocity_Kp 加大 (现 5)
 *        SPD 一顿一顿 / 嗡嗡响 -> Velocity_Kp 减小, 或 Velocity_Kd 加一点 (现 0)
 *        稳态总差一点          -> Velocity_Ki 加大 (现 1); 但输出顶到 ±700 就是目标定太高
 *        SPD 符号与 TGT 反     -> app/Encoder.c 的 ENC_x_REVERSE
 *
 *   6) 调参时的两个坑:
 *        所有参数都是编译期宏 —— 改完必须重新编译下载, 没有运行时可改的入口
 *        调试日志别用 printf: 9600 下发一行要 20~40ms, 会把 5ms 循迹节拍卡住 -> 看屏幕
 *===========================================================================*/

#include <stdio.h>
#include "ti_msp_dl_config.h"
#include "Task.h"
#include "Timer.h"
#include "Time.h"
#include "lcd.h"
#include "lcd_init.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "XunJi.h"          /* 循迹层: Track_Init/Track_Run */
#include "Key.h"            /* 板载按键: 速度在线可调 */

#define TARGET_SPEED   40       /* 闭环目标: 编码器计数 / 20ms (先给小的, 按实测调) */

/* ---- 调试开关 ----
 * 1 = 上电直接进闭环速度环 (跳过 26.5 秒开环自检, 专门调速度环用)
 * 0 = 正常流程: 先跑 23 段开环自检(验证通道/转向), 再进闭环 */
#define SKIP_OPEN_LOOP   1

/* ---- 应用模式 (二选一) ----
 * APP_MODE_MOTOR_TEST : 电机测试 (开环阶段表 -> 闭环速度环)
 * APP_MODE_TRACK      : 循迹 (5ms 读灰度 -> 加权质心 -> PD -> 左右 PWM) */
#define APP_MODE_MOTOR_TEST  0
#define APP_MODE_TRACK       1
#define APP_MODE             APP_MODE_TRACK

uint8_t app_mode = APP_MODE;

/* ---- 运行开关: S2(PB21) 单击切换 ----
 * 0 = 停 (上电默认): 电机输出全 0, 循迹不跑, 屏上显示 STOP
 * 1 = 跑: 循迹 / 电机测试按各自流程走 */
uint8_t app_run = 0;

/* 开环测试步骤: {A, B, C, D, 持续时间ms, 阶段名} */
typedef struct {
    int16_t  a, b, c, d;
    uint16_t ms;
    char     name[10];
} TestStep;

static const TestStep steps[] = {
    { 300,   0,   0,   0, 1200, "A FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {-300,   0,   0,   0, 1200, "A REV"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0, 300,   0,   0, 1200, "B FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,-300,   0,   0, 1200, "B REV"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0, 300,   0, 1200, "C FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0,-300,   0, 1200, "C REV"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0,   0, 300, 1200, "D FWD"},  {   0,   0,   0,   0, 800, "STOP"},
    {   0,   0,   0,-300, 1200, "D REV"},  {   0,   0,   0,   0, 800, "STOP"},
    { 400, 400, 400, 400, 2000, "ALL FWD"},
    {   0,   0,   0,   0, 1000, "STOP"},
    {-400,-400,-400,-400, 2000, "ALL REV"},
    {   0,   0,   0,   0, 1000, "STOP"},
    {-300, 300,-300, 300, 1500, "TURN L"},      /* 与 Motor.c 的 Turn_left() 一致 */
    { 300,-300, 300,-300, 1500, "TURN R"},      /* 与 Turn_right() 一致 */
    {   0,   0,   0,   0, 1500, "STOP"},
};
#define STEP_NUM   (sizeof(steps)/sizeof(TestStep))    /* 走完这些步就进闭环 */

 
static uint8_t  stage    = 0;      /* 当前阶段; >= STEP_NUM 表示已进闭环 */
static uint32_t stage_ms = 0;      /* 当前阶段已运行时长 */

int PWMA=0,PWMB=0,PWMC=0,PWMD=0;                  /* 4 路 PWM 指令 */
int A_speed=0,B_speed=0,C_speed=0,D_speed=0;      /* 4 路实测速度(计数/20ms) */
static int target_speed = TARGET_SPEED;           /* 闭环目标, 电机模式下按键可调 */
#define TARGET_SPEED_MIN      5
#define TARGET_SPEED_MAX    200
#define TARGET_SPEED_STEP     5

int Target_A=TARGET_SPEED,Target_B=TARGET_SPEED,Target_C=TARGET_SPEED,Target_D=TARGET_SPEED;

/*---------------------------------------------------------------------------
 * LCD 状态显示 (只刷 6 行文字, 不做整屏刷新, 免得拖慢 20ms 环)
 *-------------------------------------------------------------------------*/
static void LCD_status(void)
{
    char buf[24];

    if (!app_run)              sprintf(buf, "STOP  press S2 ");
    else if (stage >= STEP_NUM) sprintf(buf, "CLOSED LOOP    ");
    else                       sprintf(buf, "%2d %-11s", stage + 1, steps[stage].name);
    LCD_ShowString(0,   0, (const u8 *)buf, app_run ? YELLOW : RED, BLACK, 16, 0);

    sprintf(buf, "PWM A/B %4d %4d", PWMA, PWMB);
    LCD_ShowString(0,  24, (const u8 *)buf, WHITE, BLACK, 16, 0);

    sprintf(buf, "PWM C/D %4d %4d", PWMC, PWMD);
    LCD_ShowString(0,  40, (const u8 *)buf, WHITE, BLACK, 16, 0);

    sprintf(buf, "SPD A/B %4d %4d", A_speed, B_speed);
    LCD_ShowString(0,  64, (const u8 *)buf, GREEN, BLACK, 16, 0);

    sprintf(buf, "SPD C/D %4d %4d", C_speed, D_speed);
    LCD_ShowString(0,  80, (const u8 *)buf, GREEN, BLACK, 16, 0);
 
    if (stage >= STEP_NUM) sprintf(buf, "TGT %4d CLOSED", Target_A);
    else                   sprintf(buf, "TGT ---- OPEN  ");
    LCD_ShowString(0, 104, (const u8 *)buf, CYAN, BLACK, 16, 0);
}

/*---------------------------------------------------------------------------
 * 循迹模式: LCD 显示
 *   第1行 8 路数字量: '#'=压线 '.'=未压线 (bit0 最左, bit7 最右)
 *-------------------------------------------------------------------------*/
static void LCD_track(void)
{
    char buf[24];
    int  i;

    for (i = 0; i < 8; i++) buf[i] = ((track_bits >> i) & 0x01) ? '.' : '#';
    buf[8] = ' '; buf[9] = ' '; buf[10] = 0;
    LCD_ShowString(0,   0, (const u8 *)buf, GREEN, BLACK, 16, 0);

    /* P = 质心位置(0~700, 中心350), E = 误差(负=线偏左) */
    sprintf(buf, "P%3d E%+4d     ", track_pos, track_error);
    LCD_ShowString(0,  24, (const u8 *)buf, YELLOW, BLACK, 16, 0);

    /* L = 左轮 PWM, V = 弯道降速后的实际速度 */
    sprintf(buf, "L%+5d V%3d     ", track_left_pwm, track_speed);
    LCD_ShowString(0,  48, (const u8 *)buf, WHITE, BLACK, 16, 0);

    sprintf(buf, "R%+5d          ", track_right_pwm);
    LCD_ShowString(0,  64, (const u8 *)buf, WHITE, BLACK, 16, 0);

    /* B = 按键设定的基础速度 */
    if (!app_run)        sprintf(buf, "STOP  press S2");
    else if (track_lost) sprintf(buf, "LOST %-5d   ", track_lost);
    else                 sprintf(buf, "RUN   B%3d   ", Track_GetBaseSpeed());
    LCD_ShowString(0,  88, (const u8 *)buf, app_run ? CYAN : RED, BLACK, 16, 0);

    sprintf(buf, "I2CERR %-4d   ", track_i2c_err);
    LCD_ShowString(0, 112, (const u8 *)buf, track_i2c_err ? RED : GRAY, BLACK, 16, 0);
}

/*---------------------------------------------------------------------------
 * 5ms 任务: 循迹 (读灰度 + 控制)
 *-------------------------------------------------------------------------*/
void APP_Task_5ms(void)
{
    if ((app_mode == APP_MODE_TRACK) && app_run)
    {
        Track_Run();
    }
}

/*---------------------------------------------------------------------------
 * 10ms 任务: 板载按键 S2(PB21) —— 单击 开始/停止, 长按 加速
 *-------------------------------------------------------------------------*/
void APP_Task_10ms(void)
{
    int ev = Key_Scan();

    if (ev == 0) return;

    /* ---------------- 单击: 开始 / 停止 ---------------- */
    if (ev & KEY_EV_CLICK)
    {
        app_run = app_run ? 0 : 1;

        if (app_run)
        {
            if (app_mode == APP_MODE_TRACK) Track_Reset();  /* 清 PID / 丢线计时 */
            Motor_Enable();                                  /* STBY 使能 */

        }
        else
        {
            Motor_Stop();                                    /* 输出清零 + 方向脚复位 */
            PWMA = PWMB = PWMC = PWMD = 0;                    /* 屏上同步归零 */
        }
        /* 这里刻意不发串口: 9600 下发一行日志要 20~40ms, 会把 5ms 循迹节拍卡住 */
    }

    /* ---------------- 长按: 加速一档 ---------------- */
    if (ev & KEY_EV_ACCEL)
    {
        if (app_mode == APP_MODE_TRACK)
        {
            Track_StepSpeed(1);                              /* 150~900, 到顶回绕到 150 */
        }
        else
        {
            target_speed += TARGET_SPEED_STEP;
            if (target_speed > TARGET_SPEED_MAX) target_speed = TARGET_SPEED_MIN;
            Target_A = Target_B = Target_C = Target_D = target_speed;
        }
    }
}

/*---------------------------------------------------------------------------
 * 20ms 任务: 编码器采样 + 闭环速度环
 *-------------------------------------------------------------------------*/
void APP_Task_20ms(void)
{
    Encoder_Update();                        /* 换出 4 路 20ms 计数值 */

    A_speed = Motor_Get_Encoder_A();
    B_speed = Motor_Get_Encoder_B();
    C_speed = Motor_Get_Encoder_C();
    D_speed = Motor_Get_Encoder_D();

    if (!app_run)
    {
        return;                              /* 停止状态: 只采样编码器, 不写 PWM */
    }

    if (app_mode != APP_MODE_MOTOR_TEST)
    {
        return;                              /* 循迹模式: PWM 由 Track_Run() 直接写, 这里不插手 */
    }

    if (stage >= STEP_NUM)                   /* 闭环阶段: 4 路独立速度环 */
    {
        PWMA = Velocity_A(Target_A, A_speed);
        PWMB = Velocity_B(Target_B, B_speed);
        PWMC = Velocity_C(Target_C, C_speed);
        PWMD = Velocity_D(Target_D, D_speed);
    }
    /* 开环阶段 PWMA~PWMD 由 100ms 任务按步骤表写入 */

    Motor_PWM_Front(PWMA, PWMB);
    Motor_PWM_Back(PWMC, PWMD);
}

/*---------------------------------------------------------------------------
 * 100ms 任务: 测试阶段推进 + LCD 刷新
 *-------------------------------------------------------------------------*/
void APP_Task_100ms(void)
{
    if (app_mode == APP_MODE_TRACK)
    {
        LCD_track();
        return;
    }

    if (!app_run)
    {
        LCD_status();                        /* 停止状态: 不推进阶段, 只刷屏 */
        return;
    }

    if (stage < STEP_NUM)
    {
        PWMA = steps[stage].a;
        PWMB = steps[stage].b;
        PWMC = steps[stage].c;
        PWMD = steps[stage].d;

        stage_ms += 100;
        if (stage_ms >= steps[stage].ms)
        {
            stage_ms = 0;
            stage++;                          /* 进入下一步 */
            if (stage >= STEP_NUM)            /* 开环全部走完 -> 清 PID 状态, 进闭环 */
            {
                PWMA = PWMB = PWMC = PWMD = 0;
                Motor_Stop();
                printf("---- open loop done, closed loop ON (target=%d) ----\r\n", target_speed);
            }
        }
    }
    LCD_status();
}

/*---------------------------------------------------------------------------
 * 1000ms 任务: 串口日志
 *-------------------------------------------------------------------------*/
void APP_Task_1000ms(void)
{
//    if (stage >= STEP_NUM)
//        printf("CLOSED  tgt=%d  spd=%d/%d/%d/%d  pwm=%d/%d/%d/%d\r\n",
//               Target_A, A_speed, B_speed, C_speed, D_speed, PWMA, PWMB, PWMC, PWMD);
//    else
//        printf("STEP %2d %-8s pwm=%d/%d/%d/%d  spd=%d/%d/%d/%d\r\n",
//               stage + 1, steps[stage].name, PWMA, PWMB, PWMC, PWMD,
//               A_speed, B_speed, C_speed, D_speed);
}

/*---------------------------------------------------------------------------
 * 系统初始化
 *-------------------------------------------------------------------------*/
static void System_Init(void)
{
    SYSCFG_DL_init();                       /* 时钟/GPIO/PWM/SPI(LCD)/SysTick */

    Motor_Stop();
    Motor_Enable();                         /* TB6612 STBY = 1 使能 */

    Encoder_Init();                         /* 打开 8 个编码器脚的中断 */
    Key_Init();                             /* 板载按键 S1/S2 调速 */

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);  /* 1ms 时基 -> 调度器 */
    DL_TimerG_startCounter(TIMER_0_INST);

    LCD_Init();
    /* 上电自检: 刷 3 个纯色 + 清屏, 确认 LCD 在位 */
    LCD_Fill(0, 0, LCD_W, LCD_H, RED);   delay_ms(200);
    LCD_Fill(0, 0, LCD_W, LCD_H, GREEN); delay_ms(200);
    LCD_Fill(0, 0, LCD_W, LCD_H, BLUE);  delay_ms(200);
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_DrawRectangle(0, 0, LCD_W - 1, LCD_H - 1, WHITE);

    if (app_mode == APP_MODE_TRACK)
    {
        Track_Init();                       /* 循迹 PID 初始化 */
        printf("\r\n==== TRACK mode: AuroraT8 (I2C0 PA0/PA1), 5ms loop ====\r\n");
        printf("KEY S2/PB21: click = start/stop, long press = speed up\r\n");
    }
    else
    {
#if SKIP_OPEN_LOOP
        stage = STEP_NUM;                   /* 直接进闭环, 跳过开环自检 */
        printf("\r\n==== Motor test (DEBUG: skip open loop, CLOSED LOOP from boot) ====\r\n");
#else
        printf("\r\n==== Motor test (TB6612, 4 wheels, open loop -> closed loop) ====\r\n");
#endif
        printf("A=LF PA12/PB19/PB20  B=RF PA13/PB4/PB5  C=RB PA17/PB6/PB7  D=LB PA7/PB12/PB13\r\n");
        printf("STBY=PB0  encoder 20ms  target=%d\r\n", target_speed);
    }
}

int main(void)
{
    System_Init();

    while (1)
    {
        Scheduler_Run();
    }
}
