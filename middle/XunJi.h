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

void Track_Init(void);      /* 循迹状态初始化, 上电调一次 */
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
#define POINT_CONFIRM_FRAMES   4      /* 大面积黑要连续这么多帧才算 EV_BLACK (去抖, 5ms/帧) */
#define POINT_MIN_RUN          5      /* 且最长连续压线 >= 它: 停车线/横杠是整片黑, 污渍多半断续 */
#define TRACK_STOP_PWM         0      /* 停车时的 PWM (0 + 短路制动) */

/* ---- 事件层 (2026-10-13 简化) ---------------------------------------------
 * 传感器只报"原始事件", 不判语义 —— 是路口还是弯, 由路线表说了算:
 *   EV_BLACK  (大面积黑): nb >= ROOM_MIN_BLACK。丁字口横杠 / 停车线 / 房间黑块
 *                         在 8 路灰度眼里是同一个签名, 统一走 Track_TakeRoom();
 *                         状态机按当前步的"期望"去解释它 (该停就停, 该直行就穿, 该拐就拐)。
 *   EV_CORNER (普通弯)  : 外侧连续 CORNER_EDGE_BLACK_REQ 路压线 且 中心那几路全白
 *                         (线从阵列侧面整段离开) -> Track_TakeCorner()。
 *   EV_LOST   (丢线)    : 全白, 由 Track_Run 内部的丢线找回处理 (不是事件)。
 * ★ Track_Detect(0) 关掉判定时 (转向/找线全过程), 一切事件作废并清零。 */
#define CORNER_EDGE_BLACK_REQ  3      /* 普通弯: 外侧连续压线路数 (线贴到这个宽度才算) */
#define CORNER_CENTER_MASK  0x3C      /* bit2~5 = "中心那几路" (1=白, 0=压线) */
#define CORNER_WIN_FRAMES     12      /* 外侧展宽后, 中心持续没线这么多帧才算弯 (5ms/帧 -> 60ms) */
#define CORNER_LOST_MS       100      /* 或: 线整段丢了连续这么久, 也算弯 (方向取最后看到线的那侧) */
#define LINE_MAX_BLACK        3       /* "细线"判据: 压线路数 <= 它 (找线/回阵列正中都用) */
#ifndef TRACK_SUPPRESS_MS
#define TRACK_SUPPRESS_MS     400     /* 转向收工后屏蔽判定的时长 (0 = 不屏蔽) */
#endif

void Track_Detect(int on);             /* 0 = 关掉事件判定 (转向/找线过程中用) */
void Track_Suppress(uint32_t ms);      /* 屏蔽判定 ms 毫秒 (转向收工后用) */
int  Track_GetSuppressed(void);        /* 1 = 判定现在被关着 */

extern int  track_nblack;      /* 本帧压线路数 0~8 */
extern int  track_cross;       /* 电平: 1 = 现在压在路口上 (只给屏看) */
extern int  track_cross_side;  /* -1=只在左边, +1=只在右边, 0=两边都有 */
extern int  track_room;        /* 电平: 1 = 现在压在大面积黑上 (已去抖) */
extern int  track_cross_cnt;   /* 上电以来"最左/最右压线"的次数 (调试/屏显) */
extern int  track_corner;      /* 电平: 1 = 现在线从侧面离开了 (普通弯) */
extern int  track_corner_side; /* 普通弯往哪边: +1 右 / -1 左 */

int  Track_TakeCross(void);    /* 边沿: 最左/最右压线 (只给屏/调试用) */
int  Track_TakeRoom(void);     /* 边沿: 到大面积黑 -> 1 ★状态机的主事件 (到口子/停车线) */
int  Track_TakeCorner(void);   /* 边沿: 确认普通弯 -> 1 */
void Track_Stop(void);         /* 停车: 左右 PWM 0 + 短路制动 (状态机用) */
void Track_SetDrive(int on);   /* 0 = 观察模式: 照读传感器/刷屏, 但不驱动电机 (台架标定用) */
extern uint8_t track_raw[8];     /* 每路原始值 (0~255, 越大越黑) —— 标定阈值时看这个 */
extern uint8_t track_white[8];   /* 每路白底基线 (上电标定 + 在线慢速跟踪) */
extern uint8_t track_black[8];   /* 每路黑参考 (在线学习出来的实测黑线值) */
extern uint8_t track_thr[8];     /* 每路当前生效阈值 (= 滞回上沿; 屏显/找线都用它) */
void Track_CalibWhite(void);     /* 重新标定白底基线 (车停着且 8 路都在白底上时调) */

/* ---- 找线 (2026-10-06): 转弯落点不确定, 状态机转完要闭环找回线 ------------------
 *   Track_ReadOnce(): 读一帧灰度(更新 track_bits/raw/nblack), 不写电机; 返回压线路数
 *   Track_LineFound(): 1 = 线找回来了 (到阵列中间 或 >=2 路压线) */
int  Track_ReadOnce(void);
int  Track_LineFound(void);
int  Track_LineCentered(void); /* 1 = 细线正压在阵列正中 (bit3/bit4 压线 且 总压线 <= 3)
                                *   转弯的"传感器出口"判据: 成立 = 车已经转到位 + 线在车轴正下方 */

#endif
