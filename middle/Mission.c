/*===========================================================================
 * 任务状态机 (移动控制层) —— 循迹 -> 检测路口 -> 判方向 -> 转向 -> 停车 -> 恢复循迹
 *
 * 逻辑全在 Mission_Run() (5ms 调一次); "路线"全在下面的 route[] 表里。
 * 改路线 = 改表; 加功能 = 在 MS_WORK 步里挂 Mission_OnWork()。
 *
 * ---------------------------------------------------------------------------
 * ★★★ 下表是【骨架示例】, 转弯方向/判据请按实际场地标定后再定稿 ★★★
 *      地图(项目7 图1)上的位置参考:
 *        左下 = 模拟停车场(出发)   左中 = 丁字口(往右 2m 是充电区支线)
 *        左上角 = 直角弯           顶部横线
 *        右上 = 1号房间口(竖线往下挂 2/3/4 号房间, 各一个口)
 *        右下角 = 直角弯           右侧 3m 竖线拐回顶部
 *      参考实现(第四组)走的是: 出发 -> 直行穿过左中口 -> 左上角右转 ->
 *        沿顶边直行到 1 号房间 -> 停车测温 -> 掉头 -> ... (他们的方向/角度按他们场地调的)
 * ---------------------------------------------------------------------------
 * 每行的列: 动作 | arg | 退出条件 | 条件参数 | 停车ms | 超时ms | 房间号 | 名字
 *   动作: MS_TRACK 循迹 / MS_BLIND 盲走直行 / MS_TURN IMU定角转 / MS_WAIT 停等
 *         MS_WORK 工作环节(停车+调钩子) / MS_END 收工
 *   退出: MS_EX_CROSS 路口 / MS_EX_ROOM 房间黑块 / MS_EX_TIME 计时 / MS_EX_TURN 转完
 *         / MS_EX_LINE 指定路重新压线 / MS_EX_NONE 只能靠超时
 * ============================================================================*/
#include "Mission.h"
#include "XunJi.h"
#include "ImuTurn.h"
#include "Motor.h"
#include "Timer.h"
#include "stdio.h"

#define MS_BLIND_SPEED   250    /* MS_BLIND 没给 PWM 时的直行速度 */
#define MS_BLIND_THRU_MS 600    /* "直行穿过路口"的建议时长 (按车速/口宽标定) */

static const MissionStep route[] =
{
/* 动作           arg   退出条件       条件参数 停车ms 超时ms 房间  名字   */
/* -- 0: 出发, 循迹直到第一个路口 (丁字口/直角弯都算) ---------------------- */
  { MS_TRACK,      0,   MS_EX_CROSS,     0,      400,  12000,  0,  "GO"   },
/* -- 1: 直行穿过这个口 (支线会把循迹质心带偏, 所以盲走一段) --------------- */
  { MS_BLIND,    250,   MS_EX_TIME,    MS_BLIND_THRU_MS, 200,  2000, 0, "THRU" },
/* -- 2: 循迹到下一个口 (左上角直角弯) ------------------------------------ */
  { MS_TRACK,      0,   MS_EX_CROSS,     0,      400,  12000,  0,  "T2"   },
/* -- 3: 右转 90 度 (IMU 定角, 到位自己停稳) ------------------------------ */
  { MS_TURN,     +90,   MS_EX_TURN,      0,      300,   6000,  0,  "R90"  },
/* -- 4: 循迹到 1 号房间口 (房间是一块大面积黑区) ------------------------- */
  { MS_TRACK,      0,   MS_EX_ROOM,      0,      400,  12000,  0,  "T3"   },
/* -- 5: 工作环节: 停车 + Mission_OnWork(1) + 等 1.5s --------------------
 *        以后测温/语音播报/LCD刷新/LORA上传都挂这里 (改 Mission_OnWork) */
  { MS_WORK,       0,   MS_EX_TIME,   1500,      200,   8000,  1,  "WORK1"},
/* -- 6: 掉头 180 度 (房间是伸出去的支线, 测完要转回来) ------------------- */
  { MS_TURN,    +180,   MS_EX_TURN,      0,      300,   9000,  0,  "U180" },
/* -- 7: 循迹回主路, 到下一个路口 ----------------------------------------- */
  { MS_TRACK,      0,   MS_EX_CROSS,     0,      400,  12000,  0,  "T4"   },
/* -- ★ 2/3/4 号房间照抄 2~7 这一段, 把 转弯方向/角度 和 房间号 按实际场地改 --
 * --   例如: {MS_TRACK,0,MS_EX_CROSS,0,400,12000,0,"T5"},
 * --         {MS_TURN,-90,MS_EX_TURN,0,300,6000,0,"L90"},
 * --         {MS_TRACK,0,MS_EX_ROOM,0,400,12000,0,"T6"},
 * --         {MS_WORK,0,MS_EX_TIME,1500,200,8000,2,"WORK2"},
 * --         {MS_TURN,+180,MS_EX_TURN,0,300,9000,0,"U180"}, ... */
/* -- 最后: 回充电区停 5s -> 回停车场 --------------------------------------- */
  { MS_TRACK,      0,   MS_EX_ROOM,      0,      400,  15000,  0,  "CHG"  },
  { MS_WAIT,       0,   MS_EX_TIME,   5000,      300,   8000,  0,  "CHG5S"},
  { MS_TRACK,      0,   MS_EX_ROOM,      0,      400,  15000,  0,  "PARK" },
/* -- 收工 ----------------------------------------------------------------- */
  { MS_END,        0,   MS_EX_NONE,      0,        0,      0,  0,  "END"  },
};

#define ROUTE_N   ((int)(sizeof(route) / sizeof(route[0])))

#define SUB_RUN   0
#define SUB_STOP  1
#define SUB_DONE  2

static int      ms_active       = 0;
static int      ms_step         = 0;
static uint8_t  ms_sub          = SUB_DONE;
static uint32_t ms_sub_t0       = 0;    /* 本子状态(停车段)开始时刻 */
static uint32_t ms_step_t0      = 0;    /* 本步开始时刻 (计时/超时用) */
static uint8_t  ms_turn_started = 0;    /* 本步的 IMU 转向已启动 */
static uint8_t  ms_work_entered = 0;    /* 本步的 MS_WORK 钩子已调过 */
static char     ms_status[24];

/*---------------------------------------------------------------------------
 * 弱钩子: 默认空实现; 在 empty.c 里同名再写一遍就覆盖了 (不用改本文件)
 *-------------------------------------------------------------------------*/
__attribute__((weak)) void Mission_OnWork(int room)      { (void)room; }
__attribute__((weak)) void Mission_OnStepEnter(int step) { (void)step; }
__attribute__((weak)) void Mission_OnDone(void)          { }

/*---------------------------------------------------------------------------
 * 内部
 *-------------------------------------------------------------------------*/
static void ms_sync_now(uint32_t now)
{
    ms_sub_t0  = now;
    ms_step_t0 = now;
}

static void ms_finish(void)
{
    ms_active = 0;
    ms_sub    = SUB_DONE;
    Track_Stop();
    Mission_OnDone();
}

/* 进入"步间停车" */
static void ms_enter_stop(uint32_t now)
{
    ms_sub    = SUB_STOP;
    ms_sub_t0 = now;
    Track_Stop();
}

/* 进入下一步 */
static void ms_next_step(uint32_t now)
{
    ms_step++;
    if (ms_step >= ROUTE_N) { ms_finish(); return; }

    ms_sub          = SUB_RUN;
    ms_turn_started = 0;
    ms_work_entered = 0;

    /* 停车期间(车没动)攒下的路口/房间边沿事件丢掉, 否则下一步一进 RUN 就立刻误触发 */
    (void)Track_TakeCross();
    (void)Track_TakeRoom();

    ms_sync_now(now);
    Mission_OnStepEnter(ms_step);
}

/* 退出条件成立? (每拍在 SUB_RUN 里问一次) */
static int ms_exit_met(const MissionStep *s, uint32_t now)
{
    switch (s->exit)
    {
    case MS_EX_CROSS: return Track_TakeCross();
    case MS_EX_ROOM:  return Track_TakeRoom();
    case MS_EX_TIME:  return ((now - ms_step_t0) >= (uint32_t)s->exit_arg);
    case MS_EX_TURN:  return (ms_turn_started && !ImuTurn_IsBusy());
    case MS_EX_LINE:  return (((~track_bits) & (uint8_t)s->exit_arg) != 0);  /* 掩码里任一路压线 */
    default:          return 0;
    }
}

/*---------------------------------------------------------------------------
 * 对外接口
 *-------------------------------------------------------------------------*/
void Mission_Init(void)
{
    ms_active       = 0;
    ms_step         = 0;
    ms_sub          = SUB_DONE;
    ms_turn_started = 0;
    ms_work_entered = 0;
    ms_sync_now(Get_Time());
}

void Mission_Start(void)
{
    ms_active       = 1;
    ms_step         = 0;
    ms_sub          = SUB_RUN;
    ms_turn_started = 0;
    ms_work_entered = 0;
    ms_sync_now(Get_Time());
    Track_Reset();                  /* 清循迹的丢线计时/PD 历史 */
    Mission_OnStepEnter(0);
}

void Mission_Stop(void)
{
    ImuTurn_Stop();                 /* 转向中就先停它 */
    ms_finish();
}

int Mission_IsActive(void) { return ms_active; }
int Mission_GetStep(void)  { return ms_step; }
char Mission_GetSub(void)
{
    if (!ms_active)             return 'D';
    if (ms_sub == SUB_STOP)     return 'S';
    return 'R';
}

const char *Mission_GetStepName(void)
{
    if (ms_step >= ROUTE_N) return "----";
    return route[ms_step].name;
}

const char *Mission_GetStatus(void)
{
    if (!ms_active) { sprintf(ms_status, "IDLE"); return ms_status; }
    sprintf(ms_status, "M%02d %-4s %c", ms_step, Mission_GetStepName(), Mission_GetSub());
    return ms_status;
}

int Mission_GetCrossCnt(void) { return track_cross_cnt; }

/*---------------------------------------------------------------------------
 * 5ms 推进
 *-------------------------------------------------------------------------*/
void Mission_Run(void)
{
    const MissionStep *s;
    uint32_t now;

    if (!ms_active) return;

    now = Get_Time();
    s   = &route[ms_step];

    /* ---- 收工步 ---- */
    if (s->act == MS_END) { ms_finish(); return; }

    switch (ms_sub)
    {
    /* ================= 执行本步 ================= */
    case SUB_RUN:
        /* 兜底超时: 条件一直不成立也往下走, 防止卡死在一步 */
        if ((s->timeout_ms > 0) && ((now - ms_step_t0) > (uint32_t)s->timeout_ms))
        {
            ms_enter_stop(now);
            return;
        }

        switch (s->act)
        {
        case MS_TRACK:
            Track_Run();                                  /* 循迹: 读灰度 + PD + 写 PWM */
            break;

        case MS_BLIND:                                    /* 盲走直行: 不看线 */
            {
                int p = (s->arg > 0) ? (int)s->arg : MS_BLIND_SPEED;
                Motor_SetLeftPWM(p);
                Motor_SetRightPWM(p);
            }
            break;

        case MS_TURN:                                     /* 原地定角转 (IMU) */
            if (!ms_turn_started)
            {
                if (!ImuTurn_IsBusy() && (ImuTurn_TurnDeg((int)s->arg, 0) == 0))
                {
                    ms_turn_started = 1;
                    Track_Reset();
                }
                /* 启动失败 (正忙 / IMU 没数据) -> 下一拍再试; 由 timeout_ms 兜底 */
            }
            break;

        case MS_WAIT:
            Track_Stop();
            break;

        case MS_WORK:                                     /* 工作环节: 停车 + 钩子 */
            Track_Stop();
            if (!ms_work_entered)
            {
                ms_work_entered = 1;
                ms_step_t0      = now;                    /* 工作计时从这里开始 */
                Mission_OnWork((int)s->room);
            }
            break;

        default:
            Track_Stop();
            break;
        }

        /* ---- 退出条件 ---- */
        if (ms_exit_met(s, now)) ms_enter_stop(now);
        return;

    /* ================= 步间停车 (以后插功能的地方) ================= */
    case SUB_STOP:
        Track_Stop();
        if (s->stop_ms == 0 || (now - ms_sub_t0) >= (uint32_t)s->stop_ms) ms_next_step(now);
        return;

    default:
        return;
    }
}
