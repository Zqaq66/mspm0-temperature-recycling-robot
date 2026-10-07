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

/* ---- 岔路 / 普通弯 判定 (2026-10-06 新增) ----------------------------------
 * 边侧压线(最左/最右)之后, 看"中心那几路"(bit2~5)有没有重新压上线:
 *     中心线回来   -> 岔路  : 主线还在, 车可直行也可拐进支线 -> 方向 = 边侧
 *     中心线不回来 -> 普通弯: 主线自己拐走了(最后全白)       -> 方向 = 边侧
 * 判定窗口 = BRANCH_WIN_FRAMES 帧 (5ms/帧)。★ 必须按实际岔口宽度/车速标定:
 *     太小 -> 主线还没回到中心就判成"普通弯", 该按表转/该直行的岔路被当成弯自己转了
 *     太大 -> 普通弯要等很久才转, 车已经冲过弯口
 *   标定办法: 慢慢推车过一个岔路口, 看屏第 6 行末尾是 'B+'/'B-'(岔路) 还是 'C+'/'C-'(弯)。
 * 另外: 黑路数 > BRANCH_MAX_BLACK 时不当岔路/弯判 (那是房间黑区或主线的横条)。 */
#define BRANCH_WIN_FRAMES     12      /* 判定窗口帧数 (5ms/帧 -> 默认 60ms) */

/* ---- 2d 误判抑制 (2026-10-06: 转弯时边缘蹭线被当成岔路, 转完立刻连转) --------
 * 四个机制配合, 分两层:
 *   [B] CROSS_EDGE_BLACK_REQ  触发要求"外侧【连续这么多路】同时压线"
 *         真岔口线在口子处是展宽的 (你给的模式 = 边侧 3 路黑 `*****###` / `###*****`);
 *         直道上/转弯中"边缘蹭一下"只有最外 1 路黑 -> 不算 -> 直接滤掉这类误判。
 *         ★ 标定: 推车过真岔路口, 屏第 6 行应看到边侧连着的 3 个 '#'。口子更窄就降到 2,
 *           更宽(4 路)就提到 4。
 *   [E] REARM_ERR / REARM_FRAMES  报告过一次后, 必须先连续 REARM_FRAMES 帧
 *         |error| < REARM_ERR (线回到中间) 才允许下一次判定 -> 防"转完立刻连转"。
 *   [D] Track_Detect(0/1)   转向(前移+原地转)全过程关掉判定 (状态机调)
 *   [A] Track_Suppress(ms)  收工后再屏蔽 ms (那段是"找线过程", 蹭到线很正常)
 * D/A 由 Mission 调 (turn_seq 起转/收工 + 每次步间停车), 本层只提供开关。 */
#ifndef BRANCH_LINE_MAX_BLACK
#define BRANCH_LINE_MAX_BLACK   3      /* "岔路"判据: 主线回来时必须还是【细线】(总压线 <= 3)
                                        *   ★ 为什么: 过直角弯时, 拐角那条新臂会整段压在阵列上
                                        *   (# 从一端连到中心, nb=4~5), 老判据"中心任一路压线"会把它
                                        *   当成"主线回来了" -> 直角弯被判成岔路。收紧成"细线"即可区分。 */
#endif
#ifndef CROSS_EDGE_BLACK_REQ
#define CROSS_EDGE_BLACK_REQ    3      /* 触发要求外侧连续压线路数 (3 = 边侧 3 路) */
#endif
#ifndef REARM_ERR
#define REARM_ERR              40      /* 重新武装要求 |error| 小于它 (线在中间) */
#endif
#ifndef REARM_FRAMES
#define REARM_FRAMES           20      /* 且连续这么多帧 (5ms/帧 -> 100ms) */
#endif
#ifndef TRACK_SUPPRESS_MS
#define TRACK_SUPPRESS_MS     400      /* 转向收工后屏蔽判定的时长 (0 = 不屏蔽) */
#endif

void Track_Detect(int on);             /* 0 = 关掉岔路/普通弯判定 (转向过程中用) */
void Track_Suppress(uint32_t ms);      /* 屏蔽判定 ms 毫秒 (转向收工后用, 里面的时间从调用的这一刻算) */
int  Track_GetSuppressed(void);        /* 1 = 判定现在被关着 (转向过程 / 收工后的屏蔽窗口) */
#define BRANCH_MAX_BLACK       5      /* 黑路数超过它 = 房间/横条, 不参与岔路判定 */
#define BRANCH_CENTER_MASK  0x3C      /* bit2~5 = "中心那几路" (1=白, 0=压线) */

extern int  track_nblack;      /* 本帧压线路数 0~8 */
extern int  track_cross;       /* 电平: 1 = 现在压在路口上 (已去抖) */
extern int  track_cross_side;  /* -1=只在左边, +1=只在右边, 0=两边都有 */
extern int  track_room;        /* 电平: 1 = 现在压在房间/工作区黑块上 (已去抖) */
extern int  track_cross_cnt;   /* 上电以来检测到的路口次数 (调试/屏显) */

int  Track_TakeCross(void);    /* 边沿: 有新路口 -> 1 (读后自动清) */
int  Track_TakeRoom(void);     /* 边沿: 进新房间/工作区 -> 1 (读后自动清) */

/* ---- 岔路 / 普通弯 (2026-10-06 新增, 供状态机决定"要不要转") ----------------
 *   ★ 路线表只说"要不要转", 往哪边转由传感器判: 事件成立时读下面两个 side 即可。
 *   track_branch/track_corner 是电平(锁存最近一次判定结果), 给屏显/调试看。 */
extern int  track_branch;       /* 1 = 最近一次判定: 岔路 (主线还在) */
extern int  track_branch_side;  /* 岔路分支在哪侧: +1 右 / -1 左 */
extern int  track_corner;       /* 1 = 最近一次判定: 普通弯 (主线拐走了) */
extern int  track_corner_side;  /* 普通弯往哪边拐: +1 右 / -1 左 */
int  Track_TakeBranch(void);    /* 边沿: 确认岔路 -> 1 (读后自动清) */
int  Track_TakeCorner(void);    /* 边沿: 确认普通弯 -> 1 (读后自动清) */
void Track_Stop(void);         /* 停车: 左右 PWM 0 + 短路制动 (状态机用) */
void Track_SetDrive(int on);   /* 0 = 观察模式: 照读传感器/刷屏, 但不驱动电机 (台架标定用) */
extern uint8_t track_raw[8];     /* 每路原始值 (0~255, 越大越黑) —— 标定阈值时看这个 */
extern uint8_t track_white[8];   /* 每路白底基线 (上电/手动标定出来的) */
extern uint8_t track_thr[8];     /* 每路当前生效阈值 (= 白底基线 + ADAPT_MARGIN) */
void Track_CalibWhite(void);     /* 重新标定白底基线 (车停着且 8 路都在白底上时调) */

/* ---- 找线 (2026-10-06): 转弯落点不确定, 状态机转完要闭环找回线 ------------------
 *   Track_ReadOnce(): 读一帧灰度(更新 track_bits/raw/nblack), 不写电机; 返回压线路数
 *   Track_LineFound(): 1 = 线找回来了 (到阵列中间 或 >=2 路压线) */
int  Track_ReadOnce(void);
int  Track_LineFound(void);
int  Track_LineCentered(void); /* 1 = 细线正压在阵列正中 (bit3/bit4 压线 且 总压线 <= 3)
                                *   转弯的"传感器出口"判据: 成立 = 车已经转到位 + 线在车轴正下方 */

/* ------- 旧实现(未使用, 保留备查): 权重 ±16/±12/±8/±4 ×5 + 低通 ------- */
int Track_Err(uint8_t Digtal);

#endif
