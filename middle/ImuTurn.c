#include "ImuTurn.h"
#include "Motor.h"
#include "Timer.h"
#include "imu_uart_driver.h"

/*---------------------------------------------------------------------------
 * 内部状态
 *-------------------------------------------------------------------------*/
static int      turn_mode    = IMU_TURN_IDLE;
static float    target_yaw   = 0.0f;
static float    cur_yaw      = 0.0f;
static float    diff_deg     = 0.0f;
static int      turn_speed   = 0;
static uint32_t start_ms     = 0;
static int      arrive_cnt   = 0;
static int      just_finished= 0;
static int      done_reason  = IMU_TURN_DONE_ARRIVED;
static void   (*done_cb)(int reason) = 0;

/* 运行期可调 (初值 = 头文件宏) */
static int      kp_turn = IMU_TURN_KP;
static int      min_pwm = IMU_TURN_MIN_PWM;
static int      kp_hold = IMU_HOLD_KP;

/* ---- 按数据帧更新的量 (不是按 5ms 拍) ---- */
static float    last_yaw_sample = 0.0f;   /* 上一拍读到的 yaw: 值变了 = 来了新数据帧 */
static float    last_ad_frame   = 0.0f;   /* 上一帧的 |剩余角度| (TurnDeg 里会被设成初始剩余角) */
static float    v_app_f         = 0.0f;   /* 低通后的接近角速度 (度/秒), 正 = 在靠近目标 */
static uint32_t last_frame_ms   = 0;      /* 上一帧数据的时刻 (用真实帧间隔, 25Hz 标称 40ms) */

/* ---- 方向 / 阶段 ---- */
static int8_t   init_dir        = 1;      /* 启动时 diff 的方向: 用来判断"是否冲过了目标" */
static int      spot_phase      = 0;      /* 0 = 定速段; 1 = 补足段 (停早了, 换最小 PWM 单向补) */
static char     spot_st         = 'B';    /* 本拍相位: D=定速 C=低速 R=反向制动(仅BRAKE_PWM>0) B=滑行/保持 F=补足 */
static int      errway_cnt      = 0;      /* 驱动段连续"远离目标"的拍数 (判方向反了) */

/* IMU 在线检测: 看"收到的字节数"还在不在涨 */
static uint32_t last_rx_bytes = 0;
static uint32_t last_rx_ms    = 0;

/*---------------------------------------------------------------------------
 * 工具
 *-------------------------------------------------------------------------*/
static float _norm180(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* IMU 在线判据: "收到的字节数"还在涨 = 在线 (同时刷新时间戳) */
static int _imu_alive(void)
{
    uint32_t now = Get_Time();
    uint32_t rx  = IMU_GetRxByteCount();

    if (rx != last_rx_bytes)
    {
        last_rx_bytes = rx;
        last_rx_ms    = now;
    }
    return ((now - last_rx_ms) <= IMU_TURN_IMU_OFF_MS);
}

static void _finish(int reason)
{
    turn_mode = IMU_TURN_IDLE;
    Motor_Brake();                       /* 电磁制动: 停得住, 不是自由滑行 */
    just_finished = 1;
    done_reason   = reason;
    if (done_cb != 0) done_cb(reason);
}

/*---------------------------------------------------------------------------
 * 初始化 / 参数
 *-------------------------------------------------------------------------*/
void ImuTurn_Init(void)
{
    turn_mode     = IMU_TURN_IDLE;
    target_yaw    = 0.0f;
    cur_yaw       = 0.0f;
    diff_deg      = 0.0f;
    turn_speed    = 0;
    arrive_cnt    = 0;
    just_finished = 0;
    done_reason   = IMU_TURN_DONE_ARRIVED;
    done_cb       = 0;
    last_rx_bytes = IMU_GetRxByteCount();
    last_rx_ms    = Get_Time();
}

void ImuTurn_SetDoneCallback(void (*cb)(int reason)) { done_cb = cb; }

void ImuTurn_SetGains(int kp_t, int mp, int kp_h)
{
    if (kp_t >  0) kp_turn = kp_t;
    if (mp   >= 0) min_pwm = mp;
    if (kp_h >  0) kp_hold = kp_h;
}

/*---------------------------------------------------------------------------
 * 启动
 *-------------------------------------------------------------------------*/
int ImuTurn_TurnDeg(int delta_deg, int speed)
{
    float euler[3];

    if (turn_mode != IMU_TURN_IDLE) return -1;          /* 正忙 */
    if (IMU_GetEuler(euler) != 0)   return -1;          /* 取不到 yaw */
    if (!_imu_alive())              return -1;          /* IMU 没数据 (没接/没通电) */

    cur_yaw    = _norm180(euler[2] * (float)IMU_TURN_YAW_SIGN);   /* 归一化到 (-180,180] */
    target_yaw = _norm180(cur_yaw + (float)delta_deg);
    turn_speed = (speed > 0) ? speed : IMU_TURN_SPEED_MAX;
    if (turn_speed > IMU_TURN_SPEED_MAX) turn_speed = IMU_TURN_SPEED_MAX;  /* 惯性大, 统一压低上限 */

    arrive_cnt      = 0;
    just_finished   = 0;
    last_yaw_sample = cur_yaw;          /* 清"按帧"的状态 */
    /* last_ad_frame 的初值必须 = 初始 |剩余角度| (= |delta_deg|), 不能给 0:
     * 给 0 的话第一帧 dad = 90 - 0 = 90, 算出 -2250 度/秒的假角速度, 会把"越走越远"保护误触发 */
    last_ad_frame   = (delta_deg >= 0) ? (float)delta_deg : -(float)delta_deg;
    v_app_f         = 0.0f;
    spot_phase      = 0;                /* 从定速段开始 */
    spot_st         = 'D';
    errway_cnt      = 0;
    last_frame_ms   = Get_Time();
    init_dir        = (delta_deg >= 0) ? 1 : -1;   /* 记住出发方向, 用来判断是否冲过目标 */
    start_ms        = Get_Time();
    last_rx_bytes   = IMU_GetRxByteCount();
    last_rx_ms      = start_ms;
    turn_mode       = IMU_TURN_SPOT;

    Motor_Enable();                                     /* 保证 STBY 使能 (只在启动时清一次) */
    return 0;
}

int ImuTurn_HoldAbs(float abs_yaw_deg, int speed)
{
    if (turn_mode != IMU_TURN_IDLE) return -1;
    if (!_imu_alive())              return -1;          /* IMU 没数据 (没接/没通电) */

    target_yaw = _norm180(abs_yaw_deg);
    turn_speed = (speed < 0) ? 0 : speed;
    if (turn_speed > 999) turn_speed = 999;

    arrive_cnt    = 0;
    just_finished = 0;
    start_ms      = Get_Time();
    last_rx_bytes = IMU_GetRxByteCount();
    last_rx_ms    = start_ms;
    turn_mode     = IMU_TURN_HOLD;

    Motor_Enable();
    return 0;
}

void ImuTurn_Stop(void)
{
    if (turn_mode != IMU_TURN_IDLE) _finish(IMU_TURN_DONE_STOPPED);
}

/*---------------------------------------------------------------------------
 * 5ms 状态机 —— 由 empty.c 的 APP_Task_5ms() 调用
 *-------------------------------------------------------------------------*/
void ImuTurn_Update(void)
{
    float    euler[3];
    uint32_t now;
    int      z, lim;

    if (turn_mode == IMU_TURN_IDLE) return;

    /* ---- IMU 在线检测: 300ms 没有新字节 = 掉线, 立即停车 ---- */
    now = Get_Time();
    if (!_imu_alive())
    {
        _finish(IMU_TURN_DONE_IMU_OFF);
        return;
    }

    if (IMU_GetEuler(euler) != 0)
    {
        _finish(IMU_TURN_DONE_IMU_OFF);
        return;
    }

    cur_yaw  = _norm180(euler[2] * (float)IMU_TURN_YAW_SIGN);   /* 归一化: 从任意角度起都不会出现 -350 这种值 */
    diff_deg = _norm180(target_yaw - cur_yaw);          /* >0 = 还要往正方向(左)转 */

    /* ---- D 项: 只在"新数据帧"到来时更新 (25Hz -> 40ms 一帧) ----
     * yaw 值没变 = 还是上一帧的数据 -> 什么都不做, 否则会变成打脉冲 */
    if (cur_yaw != last_yaw_sample)
    {
        float    ad_now = (diff_deg >= 0.0f) ? diff_deg : -diff_deg;
        float    dad    = ad_now - last_ad_frame;       /* 每帧 |剩余角度| 的变化 (负 = 在靠近) */
        uint32_t dt     = now - last_frame_ms;
        float    v;

        if (dt <  5)  dt = 5;                           /* 帧间隔异常保护 */
        if (dt > 200) dt = 200;
        v = -dad * 1000.0f / (float)dt;                 /* 接近角速度 (度/秒) */

        last_yaw_sample = cur_yaw;
        last_ad_frame   = ad_now;
        last_frame_ms   = now;
        v_app_f        += (v   - v_app_f ) * IMU_TURN_RATE_LPF; /* 低通: 压 yaw 抖动 */
    }

    /* ================= 原地转到指定角度 (v5: 两级降速 + 提前角 + 反向制动) =================
     * 远段定速转 -> 近段低速接近 -> 到提前角 [反向出力制动] 把转速刹下来 -> 断电保持、停稳收工。
     * 冲过目标不回转, 停早了才用最小 PWM 单向补足。 */
    if (turn_mode == IMU_TURN_SPOT)
    {
        float ad   = (diff_deg >= 0.0f) ? diff_deg : -diff_deg;   /* 剩余角度 */
        float rate;                                               /* 角速度大小 (度/秒) */
        float v_signed;                                           /* 带符号的接近角速度: >0 = 在靠近目标 */
        float lead = (spot_phase == 0) ? IMU_TURN_LEAD_DEG : IMU_TURN_LEAD_MIN_DEG;
        int   crossed = (((diff_deg > 0.0f) && (init_dir < 0)) ||
                         ((diff_deg < 0.0f) && (init_dir > 0))) ? 1 : 0;
        int   dir  = (diff_deg >= 0.0f) ? 1 : -1;                 /* 往哪边转 */
        int   hold = 0;                                           /* 1 = 本拍断电保持 */

        /* IMU 静止时 yaw 值可能一模一样 -> 没有新帧 -> 角速度按 0 算。
         * 注意制动的判据也必须用这个"失效后归零"的值: 否则帧停了而 v_app_f 还停在旧的大值上,
         * 制动会一直给 -> 又把车反着推出去 (v5.0 那个 bug 的另一条触发路径)。 */
        v_signed = v_app_f;
        if ((now - last_frame_ms) > IMU_TURN_RATE_STALE_MS) v_signed = 0.0f;
        rate     = (v_signed >= 0.0f) ? v_signed : -v_signed;

        spot_st = 'B';

        if (crossed)
        {
            /* 冲过目标: 断电保持, 停稳就收工 —— 绝不往回拉 */
            hold = 1;
            if (rate < IMU_TURN_RATE_STOP)
            {
                if (++arrive_cnt >= IMU_TURN_ARRIVE_TICKS)
                {
                    _finish(IMU_TURN_DONE_ARRIVED);
                    return;
                }
            }
            else arrive_cnt = 0;
        }
        else if (ad > lead)
        {
            /* 还没到提前刹车角: 驱动。远段定速, 近段(<=SLOW_FROM_DEG)低速把惯性降下来 */
            arrive_cnt = 0;

            /* 保护: 在驱动段却一直"远离目标"(v_app_f 负 = |剩余角| 在变大) ->
             * 方向接反 / 车被顶住 / IMU 姿态异常, 立刻停车, 免得白转一大圈 */
            if (v_signed < -IMU_TURN_RATE_STOP)
            {
                if (++errway_cnt >= IMU_TURN_ERRWAY_TICKS)
                {
                    _finish(IMU_TURN_DONE_ERRWAY);
                    return;
                }
            }
            else errway_cnt = 0;
            if ((spot_phase == 0) && (ad > IMU_TURN_SLOW_FROM_DEG))
            {
                z = dir * turn_speed;
                spot_st = 'D';
            }
            else
            {
                z = dir * min_pwm;
                spot_st = 'C';
            }
        }
        else
        {
            /* 进了提前刹车角 ----
             * 麦轮刹不住自转, 光断电会一路滑几十度(实测), 所以靠[反向出力]把转速拉下来。
             * 判据必须用**带符号**的接近角速度 (v_signed):
             *   v_app_f >  RATE_STOP = 还在朝目标快速靠近 -> 反向出力制动
             *   v_app_f <= RATE_STOP = 已经刹住 / 已经开始往回走 -> 松手, 交回断电保持
             * 出力也**按转速比例**给 (v 越小出力越小) -> 转速一过零出力自然消失,
             * 不会像 v5.0 那样把车反着推出去。 */
            if ((IMU_TURN_BRAKE_PWM > 0) && (v_signed > IMU_TURN_RATE_STOP))
            {
                int bp = (int)(v_signed * (float)IMU_TURN_BRAKE_PWM / 100.0f);
                if (bp > IMU_TURN_BRAKE_MAX) bp = IMU_TURN_BRAKE_MAX;
                if (bp < 0) bp = 0;

                z       = -init_dir * bp;
                spot_st = 'R';
                arrive_cnt = 0;
            }
            else
            {
                hold = 1;                                   /* 转速已低: 断电保持 */
                if (rate < IMU_TURN_RATE_STOP)
                {
                    if (ad <= IMU_TURN_ARRIVE_DEG)          /* 落在窗口里 -> 到位 */
                    {
                        if (++arrive_cnt >= IMU_TURN_ARRIVE_TICKS)
                        {
                            _finish(IMU_TURN_DONE_ARRIVED);
                            return;
                        }
                    }
                    else if (spot_phase == 0)               /* 第一次停早了 -> 换低速补足 */
                    {
                        spot_phase = 1;
                        spot_st    = 'F';
                        arrive_cnt = 0;
                    }
                    else                                    /* 补足段还差一点 -> 就此收工 */
                    {
                        if (++arrive_cnt >= IMU_TURN_ARRIVE_TICKS)
                        {
                            _finish(IMU_TURN_DONE_ARRIVED);
                            return;
                        }
                    }
                }
                else arrive_cnt = 0;
            }
        }

        if (hold)
        {
            Motor_Brake();                                  /* 断电 + 短路制动 */
        }
        else
        {
            Motor_SetLeftPWM(z);                            /* z>0 = 左转: 左轮正转, 右轮反转 */
            Motor_SetRightPWM(-z);
        }

        if ((IMU_TURN_TIMEOUT_MS > 0) && ((now - start_ms) > IMU_TURN_TIMEOUT_MS))
            _finish(IMU_TURN_DONE_TIMEOUT);
        return;
    }

    /* ================= 保持指定航向 ================= */
    if (turn_mode == IMU_TURN_HOLD)
    {
        if ((diff_deg < IMU_HOLD_DEADBAND_DEG) && (diff_deg > -IMU_HOLD_DEADBAND_DEG))
        {
            z = 0;                                    /* 死区内不修, 防抖 */
        }
        else
        {
            z = (int)(diff_deg * (float)kp_hold);

            /* 差速上限: 边走边修时不超过前进速度的 80%; 原地锁角用固定上限 */
            lim = (turn_speed > 0) ? ((turn_speed * IMU_HOLD_ZLIM_PCT) / 100)
                                   : IMU_HOLD_ZMAX;
            if (z >  lim) z =  lim;
            if (z < -lim) z = -lim;

            /* 原地锁角时也要给得动 */
            if ((turn_speed == 0) && (z != 0))
            {
                if (z > 0) { if (z < min_pwm) z =  min_pwm; }
                else       { if (z > -min_pwm) z = -min_pwm; }
            }
        }

        Motor_SetLeftPWM (turn_speed - z);   /* z>0 = 需要左转 -> 左轮慢(甚至反转) */
        Motor_SetRightPWM(turn_speed + z);

        if ((IMU_HOLD_TIMEOUT_MS > 0) && ((now - start_ms) > IMU_HOLD_TIMEOUT_MS))
            _finish(IMU_TURN_DONE_TIMEOUT);
        return;
    }
}

/*---------------------------------------------------------------------------
 * 状态查询
 *-------------------------------------------------------------------------*/
int ImuTurn_IsBusy(void)        { return (turn_mode != IMU_TURN_IDLE); }

int ImuTurn_JustFinished(void)
{
    int r = just_finished;
    just_finished = 0;
    return r;
}

int ImuTurn_GetDoneReason(void) { return done_reason; }
float ImuTurn_GetYaw(void)      { return cur_yaw; }
float ImuTurn_GetDiff(void)     { return diff_deg; }
float ImuTurn_GetTarget(void)   { return target_yaw; }
int ImuTurn_GetMode(void)       { return turn_mode; }
float ImuTurn_GetRate(void)     { return v_app_f; }        /* 接近角速度 (度/秒) */
char  ImuTurn_GetPhase(void)    { return spot_st;  }        /* 本拍相位: D/C/R/B/F */
