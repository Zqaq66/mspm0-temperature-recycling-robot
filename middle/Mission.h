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
 *   ① 动作      MS_TRACK 循迹 / MS_BLIND 盲走直行 / MS_CROSS 岔路口 / MS_TURN IMU定角转
 *               / MS_WAIT 停等 / MS_WORK 工作环节
 *   ② 等退出条件 检测到岔路口 / 检测到房间黑块 / 计时到 / 转向完成 / 指定路重新压线
 *   ③ 停车停够 stop_ms 毫秒  ->  进入下一步
 *   ★ 步与步之间一定是"先停住" —— 以后加语音播报/测温/上位机通信, 就挂在
 *     MS_WORK 步上(它会调 Mission_OnWork()), 或者在 stop_ms 里延时。
 *
 * ---------------------------------------------------------------------------
 * 路口怎么处理 (2026-10-06 重新设计):
 *   传感器层 (XunJi.c) 把"边侧压线"分成两种 (看中心那几路回不回来):
 *     岔路  = 最左/最右压线之后, 中心那几路又压回线了 -> 主线还在: 可以直行, 也可以拐进支线
 *     普通弯= 最左/最右压线之后, 中心一直没线(最后全白)   -> 主线自己拐走了: 必须跟着转
 *
 *   ① 岔路口 —— 路线表只说"要不要转", 往哪边转由传感器自己判 (track_branch_side):
 *        {MS_CROSS, 1, ...} = 要转: 先盲走 MS_ADV_MS (补传感器前伸的距离) 再原地转 90
 *        {MS_CROSS, 0, ...} = 不转: 盲走直行穿过口子 (支线会把质心带偏, 所以这一段不看线)
 *   ② 普通弯 —— 路线表里【不用写】: 车在 MS_TRACK / MS_CROSS 的循迹段里自己
 *      "前移 + 原地转 90", 转完回到原来那一步继续循迹。
 *   ③ 为什么转弯前要先盲走一小段 (MS_ADV_MS):
 *      8 路灰度装在小车最前方, 传感器"看到路口"时车的中心还没到路口;
 *      马上原地转的话, 转完传感器就落到线的外面了 -> 车找不到线。
 *      先直走一小段把这段距离补回来, 转完传感器正好还在线的上方。
 *      ★ 这个值按"传感器前伸距离 / 车速"标定 (见 Mission.c 顶部 MS_ADV_MS)。
 *   ④ 误判抑制 (2026-10-06 用户实测: 转弯时传感器边缘蹭到线, 被当成岔路 -> 转完立刻连转):
 *      [D] 转向(前移 + 原地转)整个过程: Track_Detect(0) 关掉岔路/普通弯判定
 *      [A] 收工那一刻: Track_Detect(1) + Track_Suppress(400ms) 屏蔽判定 (那段是"找线过程")
 *          另外每次"步间停车"也重新屏蔽一次 (车多半还停在路口边上)
 *      [B]/[E] 是判定层自己的门槛 (XunJi.c 2d 段): 边侧要连续 N 路黑才算, 报告过要等
 *          线回到中间稳定 100ms 才允许再判。→ 调参看 XunJi.h 的 CROSS_EDGE_BLACK_REQ。
 *      ★ 结论: 转弯前后不会再被"边缘蹭线"骗到; 代价是转完后 ~500ms 内真路口会漏掉。
 *   ⑤ 转弯落点闭环 (2026-10-06 用户反馈"转完传感器不一定在线上, 甚至找不到线"):
 *      转弯动作 = 盲走 MS_ADV_MS(开环) -> 原地转 90 -> 【找线】慢速前进 / 慢速后退 / 原地左右扫
 *      ±MS_SCAN_DEG, 每步都读传感器, 线回到阵列中间就收工交回循迹; 全扫完还没有 -> 停车报 NOLINE
 *      (屏上第 1 行显示 NOLINE, 子状态 'F' 找线 / 'X' 扫描)。参数见 Mission.c 顶部 MS_FIND_xxx / MS_SCAN_xxx。
 *   ⑥ MS_TURN 步 (180 掉头 / 从房间支线回主路) 也走【同一套】转弯序列 (2026-10-07):
 *      以前它是"直接 ImuTurn_TurnDeg 原地转", 没有前移也没有找线 -> 转完的落点全交给下一步循迹
 *      的 PD 去纠 (偏多了就丢线)。现在: 前移 MS_ADV_MS -> 原地转 (角度由 arg 给) -> 传感器出口
 *      -> 找线兜底, 和岔路/普通弯完全一致。
 *      * 180 掉头前后挪都在线上, 所以没有单独的"前移开关", 统一用 MS_ADV_MS。
 *      * 序列总超时 / IMU 起不来 -> 停车报警, 屏上第 1 行显示 TURN fail (Mission_GetFail()==2)。
 *
 * ---------------------------------------------------------------------------
 * 用法 (empty.c 里已经接好, 见 APP_Task_5ms):
 *   Mission_Init();                       // 上电初始化
 *   if (app_run) { Mission_Start(); }     // 单击 S2 开始 -> 启动任务
 *   Mission_Run();                        // 5ms 任务里调, 由它决定循迹/转向/停车
 *   Mission_Stop();                       // 单击 S2 停止 / 急停
 *   屏第 1 行显示 M<步号><步名><子状态>B<速度>; 子状态: R跑 S停 W工作 D完成
 *   自己要加的功(测温/语音/LCD/LORA)写在 Mission_OnWork() 里 (弱函数, 在 empty.c 里覆盖)
 *
 * ★★ 怎么按场地填 route[] 表: 看 Mission.c 里 route[] 正上方那一段 (怎么填表 0~7 条) ——
 *    动作/退出条件怎么选、三种拐弯(普通弯/岔路口/掉头)分别怎么写、一个房间的 5 行、
 *    两列时间参数、分段调试 (MISSION_DEBUG_STEPS) 都写在那里。
 *===========================================================================*/

/* ---------------- 动作 ---------------- */
#define MS_TRACK     0   /* 循迹前进 (调 Track_Run) */
#define MS_BLIND     1   /* 盲走直行: 定 PWM 直走, 不看线 (穿过路口/驶入房间用) */
#define MS_TURN      2   /* IMU 定角转向: arg = 角度 (±90 / ±180; 按实测约定 正=右转, 同 MS_DEG_RIGHT)
                          *   也走统一转弯序列 (前移+转+找线), 见下面 ⑥
                          *   arg = MS_TURN_MEMO 时用"上次岔路方向"的 ±90 */
#define MS_WAIT      3   /* 停车等待: arg = 毫秒 */
#define MS_WORK      4   /* 工作环节: 停车 + 调 Mission_OnWork() + 等 arg 毫秒 */
#define MS_END       5   /* 收工: 停车, 任务结束 */
#define MS_CROSS     6   /* 岔路口: 循迹到岔路, 再按 arg 决定 要不要转
                          *   arg = 1: 要转 -> 盲走 MS_ADV_MS + 原地转 90 (方向由传感器判)
                          *   arg = 0: 不转 -> 盲走直行穿过口子 (MS_BLIND_THRU_MS 毫秒) */

/* MS_TURN 的 arg 特殊值: 用"上一次岔路的方向"转 90 (从房间支线回到主路那一步用 —
 * T 字口上传感器判不出左右, 只能照原路方向回去) */
#define MS_TURN_MEMO  (-2000)

/* ---------------- 退出条件 ---------------- */
#define MS_EX_NONE   0   /* 不会自己退出 (靠 timeout_ms / 只做动作) */
#define MS_EX_CROSS  1   /* 检测到路口 (最左/最右压线) —— 边沿式, 不会连触 */
#define MS_EX_ROOM   2   /* 检测到房间/工作区黑块 (大面积压线) */
#define MS_EX_TIME   3   /* 计时到 exit_arg 毫秒 */
#define MS_EX_TURN   4   /* IMU 转向完成 */
#define MS_EX_JUNC   6   /* 到 T 字口/主线路口 (和 MS_EX_ROOM 同一个判据: 大面积黑),
                          * 名字更贴切, 给"从房间支线回到主路"那一步用 */
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
int  Mission_GetFail(void);            /* 1 = 上一次任务因为【转完找不到线】停了 (屏显 NOLINE) */
/* ---- 上一次转弯的观测值 (标定/诊断用: 屏或串口看) ---- */
int  Mission_GetTurnDev(void);         /* 看到线时偏阵列中心多少 (x10 路, 正 = 偏右) */
int  Mission_GetTurnDegSeen(void);     /* 看到线时已经转过多少度 */
int  Mission_GetTurnFound(void);       /* 靠哪一步找到线: 0=转到位 1=直进 2=弧A 3=弧B 4=直退 5=扫描 9=失败 */
int  Mission_GetTurnAdvGone(void);     /* 1 = 前移时就发现"主线没了" -> 这个口其实是直角弯 */

/* ---------------- 弱钩子 (在 empty.c 里覆盖即可, 这里是空实现) ----------------
 * ★ 别在里面长时间阻塞 (5ms 任务里): 要等就靠 MS_WORK 步的 arg 毫秒 */
void Mission_OnWork(int room);         /* 到房间/工作区的工作环节 (测温/语音/LCD/LORA) */
void Mission_OnStepEnter(int step);    /* 每一步开始时 (打串口日志用) */
void Mission_OnDone(void);             /* 任务全部完成 */

#endif
