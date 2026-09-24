#ifndef __XUNJI_H
#define __XUNJI_H

#include "ti_msp_dl_config.h"
#include "stdint.h"

/* ============================ 循迹层 ============================

 *   [1] 平方加权质心:  w = v*v,  pos = Σ(w*i*100)/Σw    -> 0~700, 中心 350
 *       (v 低于 NOISE_THRESHOLD 的路当噪声丢掉; 分母为 0 视为丢线)
 *   [2] 位置误差:      error = pos - 350   (负 = 线偏左)
 *   [3] 三区增益调度 PD:  |err| 小 -> 直线软增益; 大 -> 弯道激进增益; 中间线性过渡
 *       turn = (Kp*error + Kd*低通后Δerror) / 1024     (Q10 定点)
 *   [4] 弯道自适应降速:  弯得越深速度越低 (CURVE_REDUCE_PCT)
 *   [5] 丢线分流 (本工程逻辑): 看丢线前最后一帧的 |error|
 *         偏出(|E|>=LOST_SIDE_ERR) -> 直接按偏向差速转弯找回 (内侧慢/外侧快, 同向),
 *                                     超时 LOST_FIND_TIMEOUT_MS 后停车
 *         中间消失(|E|< 阈值)      -> 保持直行 LOST_STRAIGHT_MS 后停车
 *         线一回来立刻自动恢复巡线
 * 没有搬的: 直角弯特判 / 宽线停车 / yaw 一周停车 / 只巡直线 / 单圈模式
 *
 * 传感器: 芯流 AuroraT8, 硬件 I2C0 (PA0=SDA/PA1=SCL, 地址 0x40), 读 0x09 八路模拟量
 * 调参:   全部在 middle/XunJi.c 顶部 (质心/极性/PD 增益/过渡带/降速/丢线)
 * ==============================================================*/

void Track_Init(void);      /* PID/状态初始化, 上电调一次 */
void Track_Run(void);       /* 5ms 调一次: 读灰度 -> 质心 -> PD -> 左右 PWM */
void Track_Reset(void);     /* 重新开始前清状态 (丢线计时/PD 历史) */

extern int     track_error;     /* 位置误差 (负数=线偏左) */
extern int     track_pos;       /* 质心位置 0~700 (中心 350) */
extern int     track_speed;     /* 弯道降速后的实际前进速度 */
extern int     track_left_pwm;  /* 最近一次左右 PWM 指令 */
extern int     track_right_pwm;
extern int     track_lost;      /* 丢线计数 (0=正常见线) */
extern uint8_t track_bits;      /* 本地判断的 8 路数字量 (bit=0 表示该路压线) */
extern int     track_i2c_err;   /* I2C 读失败次数 */
extern int     track_base_speed;/* 当前直行基础速度 (在线可调) */

/* ---------------- 速度在线可调 (配合 middle/Key.c 板载按键) ---------------- */
#define TRACK_SPEED_DEFAULT   250   /* 上电默认直行基础速度 (开环 PWM 0~999) */
#define TRACK_SPEED_MIN       150   /* 可调下限 */
#define TRACK_SPEED_MAX       900   /* 可调上限 */
#define TRACK_SPEED_STEP       25   /* 每次按键步进 */

void Track_SetBaseSpeed(int s);     /* 设定基础速度 (自动限幅) */
int  Track_GetBaseSpeed(void);      /* 读当前基础速度 */
void Track_StepSpeed(int dir);      /* dir>0 加速一档, dir<0 减速一档 */

/* ------- 旧实现(未使用, 保留备查): 权重 ±16/±12/±8/±4 ×5 + 低通 ------- */
int Track_Err(uint8_t Digtal);

#endif
