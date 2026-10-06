#ifndef __MISSION_H
#define __MISSION_H

#include "ti_msp_dl_config.h"
#include "stdint.h"

/*===========================================================================
 * 任务状态机 (移动控制层) —— 把"循迹 / 检测路口 / 判方向 / 转向 / 停车 / 恢复循迹"
 * 串成一条【可配置的步骤链】, 以后要加的测温/语音/LCD/LORA 就是表里的一个"工作步"。
 *
 * ---------------------------------------------------------------------------
 * 为什么是"表 + 状态机"而不是一堆 if:
 *   参考实现(第四组同题目同板子的 empty.c)是 819 行里一个大 switch(step), 每步写死
 *   动作和判据。那样能跑但加一步要动代码。这里把"路线"抽成一张表(route[]),
 *   改路线只改表, 不动逻辑; 每步之间统一有停车间隙(以后插功能就在这儿插)。
 *
 * ---------------------------------------------------------------------------
 * 每一步的执行过程 (三拍):
 *   ① 动作      MS_TRACK 循迹 / MS_BLIND 盲走直行 / MS_TURN IMU定角转 / MS_WAIT 停等
 *   ② 等退出条件 检测到路口 / 检测到房间黑块 / 计时到 / 转向完成 / 指定路重新压线
 *   ③ 停车停够 stop_ms 毫秒  ->  进入下一步
 *   ★ 步与步之间一定是"先停住" —— 以后加语音播报/测温/上位机通信, 就挂在
 *     MS_WORK 步上(它会调 Mission_OnWork()), 或者在 stop_ms 里延时。
 *
 * ---------------------------------------------------------------------------
 * 为什么路口后面常常要跟一步 MS_BLIND:
 *   丁字路口/十字路口上, 传感器会看到旁边的支线, 质心被带偏 -> 循迹会往支线拐。
 *   所以"直行通过路口"要用 MS_BLIND 强行走一段直线(参考实现也是这么干的: 它的
 *   qianjin() 就是定速直行)。要拐弯的口才用 MS_TURN。
 *
 * ---------------------------------------------------------------------------
 * 用法 (empty.c 里已经接好, 见 APP_Task_5ms):
 *   Mission_Init();                       // 上电初始化
 *   if (app_run) { Mission_Start(); }     // 单击 S2 开始 -> 启动任务
 *   Mission_Run();                        // 5ms 任务里调, 由它决定循迹/转向/停车
 *   Mission_Stop();                       // 单击 S2 停止 / 急停
 *   屏第 6 行显示 M<步号><步名><子状态>B<速度>; 子状态: R跑 S停 W工作 D完成
 *   自己要加的功(测温/语音/LCD/LORA)写在 Mission_OnWork() 里 (弱函数, 在 empty.c 里覆盖)
 *===========================================================================*/

/* ---------------- 动作 ---------------- */
#define MS_TRACK     0   /* 循迹前进 (调 Track_Run) */
#define MS_BLIND     1   /* 盲走直行: 定 PWM 直走, 不看线 (穿过路口/驶入房间用) */
#define MS_TURN      2   /* IMU 定角转向: arg = 角度 (+左 / -右, 90/180) */
#define MS_WAIT      3   /* 停车等待: arg = 毫秒 */
#define MS_WORK      4   /* 工作环节: 停车 + 调 Mission_OnWork() + 等 arg 毫秒 */
#define MS_END       5   /* 收工: 停车, 任务结束 */

/* ---------------- 退出条件 ---------------- */
#define MS_EX_NONE   0   /* 不会自己退出 (靠 timeout_ms / 只做动作) */
#define MS_EX_CROSS  1   /* 检测到路口 (最左/最右压线) —— 边沿式, 不会连触 */
#define MS_EX_ROOM   2   /* 检测到房间/工作区黑块 (大面积压线) */
#define MS_EX_TIME   3   /* 计时到 exit_arg 毫秒 */
#define MS_EX_TURN   4   /* IMU 转向完成 */
#define MS_EX_LINE   5   /* 指定通道重新压线: exit_arg = 位掩码 (bit=0 表示压线),
                          * 例: 0xC0 = bit6/bit7 (车右侧的线), 0x03 = bit0/bit1 (左侧) */

/* ---------------- 步骤 ---------------- */
typedef struct
{
    uint8_t  act;         /* MS_* 动作 */
    int16_t  arg;         /* TURN: 角度 | BLIND: PWM(0=默认) | WAIT/WORK: 毫秒 */
    uint8_t  exit;        /* MS_EX_* 退出条件 */
    int16_t  exit_arg;    /* TIME: 毫秒 | LINE: 位掩码 */
    uint16_t stop_ms;     /* 退出后先停车这么多毫秒 (步间停车, 0 = 不停直接进下一步) */
    uint16_t timeout_ms;  /* 本步兜底超时 (0 = 不限); 超时也往下走, 防止卡死 */
    uint8_t  room;        /* WORK 步: 房间号 (0 = 不是房间) */
    const char *name;     /* 短名 (<=4 字符, 屏显/串口用) */
} MissionStep;

void Mission_Init(void);
void Mission_Start(void);              /* 启动任务 (回到第 0 步) */
void Mission_Stop(void);               /* 放弃/急停 */
void Mission_Run(void);                /* 5ms 任务里调 */
int  Mission_IsActive(void);           /* 1 = 任务在跑 */
int  Mission_GetStep(void);            /* 当前步号 */
const char *Mission_GetStepName(void); /* 当前步名 */
char Mission_GetSub(void);             /* 'R'跑 / 'S'停 / 'W'工作 / 'D'完成 */
const char *Mission_GetStatus(void);   /* "M03 R90 S" 这种一行状态 (串口/屏显) */
int  Mission_GetCrossCnt(void);        /* 路上一共遇到几个路口 (调试) */

/* ---------------- 弱钩子 (在 empty.c 里覆盖即可, 这里是空实现) ----------------
 * ★ 别在里面长时间阻塞 (5ms 任务里): 要等就靠 MS_WORK 步的 arg 毫秒 */
void Mission_OnWork(int room);         /* 到房间/工作区的工作环节 (测温/语音/LCD/LORA) */
void Mission_OnStepEnter(int step);    /* 每一步开始时 (打串口日志用) */
void Mission_OnDone(void);             /* 任务全部完成 */

#endif
