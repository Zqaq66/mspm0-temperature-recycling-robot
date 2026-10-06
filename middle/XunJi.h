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

/* ================= 路口 / 房间检测 (供任务状态机 Mission 用) =================
 * 判据按参考实现总结的(第四组报告 5.4 节):
 *   路口   = 最左(bit0) 或 最右(bit7) 有一路压线   <- 线在这里分叉/被横穿
 *   房间/工作区 = 大面积压线 (track_nblack >= ROOM_MIN_BLACK)  <- 房间门口那块黑区
 *   终点/断线   = 全白 -> 由 track_lost 反映 (本站不做特殊处理)
 * 三个标志都做了连续帧去抖, 单帧毛刺不会触发。
 *
 * 给状态机用的接口是 **边沿式** 的 (Take 系列): 读一次就清, 只有在"新出现"时返回 1。
 * 这样状态机不会因为"转弯完车还压在路口上"而连跳两步 (参考实现是用一个变量
 * 区分"到门口/出门口", 边沿式更省事)。track_cross/track_room 是电平, 只给屏看。
 *
 * 【标定办法】把车放在普通直线上, 看屏第 1 行 8 个点: 正常巡线应该只有 1~3 个 '#'、
 * 且 bit0/bit7 是 '.'。把车推到一个路口上, bit0 或 bit7 应该变 '#'。如果普通线上
 * 也经常有边缘位为 '#' -> 把 CROSS_EDGE_MASK 改成只看更靠外的路 (见宏注释)。 */
#define CROSS_CONFIRM_FRAMES   2      /* 连续这么多帧才算数 (去抖) */
#define ROOM_MIN_BLACK         6      /* >= 这么多路压线 = 房间/工作区黑块 (8=全黑) */
#define TRACK_STOP_PWM         0      /* 停车时的 PWM (0 + 短路制动) */

extern int  track_nblack;      /* 本帧压线路数 0~8 */
extern int  track_cross;       /* 电平: 1 = 现在压在路口上 (已去抖) */
extern int  track_cross_side;  /* -1=只在左边, +1=只在右边, 0=两边都有 */
extern int  track_room;        /* 电平: 1 = 现在压在房间/工作区黑块上 (已去抖) */
extern int  track_cross_cnt;   /* 上电以来检测到的路口次数 (调试/屏显) */

int  Track_TakeCross(void);    /* 边沿: 有新路口 -> 1 (读后自动清) */
int  Track_TakeRoom(void);     /* 边沿: 进新房间/工作区 -> 1 (读后自动清) */
void Track_Stop(void);         /* 停车: 左右 PWM 0 + 短路制动 (状态机用) */

/* ------- 旧实现(未使用, 保留备查): 权重 ±16/±12/±8/±4 ×5 + 低通 ------- */
int Track_Err(uint8_t Digtal);

#endif
