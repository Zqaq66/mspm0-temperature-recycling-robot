#include "ImuTurn.h"
#include "Motor.h"
#include "Timer.h"
#include "imu_uart_driver.h"

/* 原地转的 4 个段 (单向, 只往前走, 不回头) */
#define SP_FAR      0   /* 远段: 定速 turn_speed */
#define SP_NEAR     1   /* 近段: 降到 min_pwm 低速接近 */
#define SP_COAST    2   /* 已切断驱动, 等它靠惯性停稳 */
#define SP_PULSE    3   /* 末段脉冲微调: 用 min_pwm 通一个很短的时间 */

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
static int      speed_max = IMU_TURN_SPEED_MAX;
static int      min_pwm   = IMU_TURN_MIN_PWM;
static int      kp_hold   = IMU_HOLD_KP;

/* ---- 按数据帧更新的量 (不是按 5ms 拍) ---- */
static float    last_yaw_sample = 0.0f;   /* 上一拍读到的 yaw: 值变了 = 来了新数据帧 */
static float    v_app_f         = 0.0f;   /* 低通后的接近角速度 (度/秒), 正 = 在靠近目标 */
static uint32_t last_frame_ms   = 0;      /* 上一帧数据的时刻 (用真实帧间隔, 25Hz 标称 40ms) */

/* ---- 方向 / 段 ---- */
static int8_t   init_dir        = 1;      /* 启动时 diff 的方向: 用来判断"是否冲过了目标" */
static int      spot_phase      = SP_FAR;
static char     spot_st         = 'D';    /* 本拍相位: D=远段定速 C=近段低速 B=断电等停稳 P=脉冲微调 */
static int      errway_cnt      = 0;      /* 驱动段连续"远离目标"的拍数 (判方向反了) */
static int      has_crossed     = 0;      /* 已经冲过目标了 -> 只收工, 绝不往回拉 */

/* ---- 自标定 (掉电不保存, 重新上电从种子值重新学) ---- */
static float    decel_est       = IMU_TURN_DECEL_DEF;       /* a: 减速能力 (度/秒^2) */
static float    pulse_gain      = IMU_TURN_PULSE_GAIN_DEF;  /* 脉冲增益 (度/毫秒) */
static float    coast_pred      = 0.0f;   /* 本拍预测的滑行角 (度), 显示用 */
static float    cut_rate        = 0.0f;   /* 切断那一刻的角速度 (标定 a 用) */
static float    cut_yaw         = 0.0f;   /* 切断那一刻的 yaw   (标定 a 用) */
static int      cal_pending     = 0;      /* 停稳后要标定 a ? */
static int      pulse_cnt       = 0;      /* 本次转向已用脉冲数 */
static uint32_t pulse_start     = 0;
static uint32_t pulse_ms        = 0;
static int      pulse_dir       = 1;      /* 脉冲方向 (启动时锁死, 中途不翻) */
static float    pulse_yaw0      = 0.0f;
static int      pulse_cal       = 0;      /* 停稳后要标定脉冲增益 ? */
static float    last_err        = 0.0f;   /* 上一次落点误差 (度, 绝对值) */

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

static float _fabs(float v) { return (v >= 0.0f) ? v : -v; }

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

void ImuTurn_SetTurnSpeed(int sp, int mp)
{
    if (sp > 0)   speed_max = (sp > 999) ? 999 : sp;
    if (mp > 0)   min_pwm   = (mp > 999) ? 999 : mp;
}

void ImuTurn_SetDecel(float a)
{
    if (a < IMU_TURN_DECEL_MIN) a = IMU_TURN_DECEL_MIN;
    if (a > IMU_TURN_DECEL_MAX) a = IMU_TURN_DECEL_MAX;
    decel_est = a;
}

void ImuTurn_SetHoldKp(int kp) { if (kp > 0) kp_hold = kp; }

void ImuTurn_ResetCalib(void)
{
    decel_est  = IMU_TURN_DECEL_DEF;
    pulse_gain = IMU_TURN_PULSE_GAIN_DEF;
    last_err   = 0.0f;
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
    turn_speed = (speed > 0) ? speed : speed_max;
    if (turn_speed > speed_max) turn_speed = speed_max;            /* 惯性大, 统一压上限 */

    arrive_cnt      = 0;
    just_finished   = 0;
    last_yaw_sample = cur_yaw;          /* 清"按帧"的状态 (角速度按实测 yaw 差算, 不需要 ad 初值) */
    v_app_f         = 0.0f;
    spot_phase      = SP_FAR;
    spot_st         = 'D';
    errway_cnt      = 0;
    has_crossed     = 0;
    cut_rate        = 0.0f;
    cal_pending     = 0;
    pulse_cnt       = 0;
    pulse_ms        = 0;
    pulse_cal       = 0;
    coast_pred      = 0.0f;
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
    uint32_t gap;
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

    /* ---- 角速度: 只在"新数据帧"到来时更新 (25Hz -> 40ms 一帧)
     * yaw 值没变 = 还是上一帧的数据 -> 什么都不做, 否则会变成按 5ms 打脉冲。
     *
     * ★★ 必须量【yaw 本身的带符号角速度】, 不能量 d(|剩余角度|)/dt:
     *   v6/v7 原来用 -(dad)/dt (ad = |目标-当前|)。车**冲过目标的那一刻** ad 由减变增,
     *   这个量会变号, 而低通滤波后的值要穿过零点 -> 瞬态掉到 0 附近并停留一两帧,
     *   于是"角速度 < RATE_STOP 连续 8 拍"成立 -> **误判"停稳了"**,
     *   而此时车其实还在 100 度/秒以上转 (实测就是这么"停稳"的, 结果多滑 29 度)。
     *   直接量 yaw 就完全没有这个问题: 它不随"是否过冲"变号。
     * 正 = yaw 在变大(往正方向转); 用 _norm180 处理跨 +-180 的情况。 */
    if (cur_yaw != last_yaw_sample)
    {
        uint32_t dt = now - last_frame_ms;
        float    v;

        if (dt <  5)  dt = 5;                           /* 帧间隔异常保护 */
        if (dt > 200) dt = 200;
        v = _norm180(cur_yaw - last_yaw_sample) * 1000.0f / (float)dt;   /* 带符号角速度 (度/秒) */

        last_yaw_sample = cur_yaw;
        last_frame_ms   = now;
        v_app_f        += (v - v_app_f) * IMU_TURN_RATE_LPF;   /* 低通: 压 yaw 抖动 */
    }

    /* ==================== 原地转到指定角度 (v7) ==================== */
    if (turn_mode == IMU_TURN_SPOT)
    {
        float ad   = _fabs(diff_deg);                     /* 剩余角度 */
        float rate;                                       /* 角速度大小 (度/秒) */
        float v_signed;
        float blind = 0.0f;                               /* 盲走补偿: 上一帧到现在已经多走的度数 */
        int   dir  = (diff_deg >= 0.0f) ? 1 : -1;         /* 往哪边转 */

        /* 超时保护 (放最前面, 任何一段都逃不掉) */
        if ((IMU_TURN_TIMEOUT_MS > 0) && ((now - start_ms) > IMU_TURN_TIMEOUT_MS))
        {
            last_err = ad;
            _finish(IMU_TURN_DONE_TIMEOUT);
            return;
        }

        /* 角速度: 低通值 + "过期归零" (IMU 静止时 yaw 一模一样, 没有新帧 -> 按 0 算) */
        v_signed = v_app_f;
        gap      = now - last_frame_ms;
        if (gap > IMU_TURN_RATE_STALE_MS) v_signed = 0.0f;

        rate = _fabs(v_signed);
        /* ★ 盲走补偿: IMU 只有 25Hz, 从上一帧到本拍车其实已经多走了 w*dt 度。
         *   全速(230 度/秒)时一帧 40ms 就是 9 度! 不算这一项必然每次冲过 9 度。 */
        if (gap < IMU_TURN_RATE_STALE_MS) blind = rate * (float)gap * 0.001f;

        if (rate > IMU_TURN_RATE_MAX) rate = IMU_TURN_RATE_MAX;   /* 夹野值 (只夹"预测用"的 rate) */

        /* 冲过目标 (滑过头 / 方向被外部因素翻掉): 记住, 只收工, 绝不往回拉 */
        if (((diff_deg > 0.0f) && (init_dir < 0)) ||
            ((diff_deg < 0.0f) && (init_dir > 0))) has_crossed = 1;

        /* 两处门限都由【本拍实测角速度】现算 —— "现在松手, 惯性正好滑到目标"的那一刻就松手。
         * ★ a 要【打折】(IMU_TURN_DECEL_TRUST): 过冲不可逆(冲过就收工), 差一点还能用脉冲补,
         *   所以宁可认为"它比标定值滑得更远", 提前切。地面/电量变了最多差几度, 不会冲过头。 */
        {
            float a_eff = decel_est * IMU_TURN_DECEL_TRUST;
            coast_pred  = rate * rate / (2.0f * a_eff) + blind + IMU_TURN_CUT_MARGIN;
        }

        /* ---------------- 已经切断驱动: 等停稳 -> 判落点 -> (可选)脉冲补 ---------------- */
        if ((spot_phase == SP_COAST) || (spot_phase == SP_PULSE))
        {
            if (spot_phase == SP_PULSE)
            {
                /* 脉冲通电中: 到点 或者 已经够近/冲过了 -> 立刻松手 */
                if (((now - pulse_start) < pulse_ms) && !has_crossed && (ad > 1.5f))
                {
                    Motor_SetLeftPWM (pulse_dir * min_pwm);
                    Motor_SetRightPWM(-pulse_dir * min_pwm);
                    spot_st = 'P';
                    return;
                }
                Motor_Brake();
                spot_phase = SP_COAST;
                pulse_cal  = 1;      /* 停稳后按实测标定脉冲增益 */
                spot_st    = 'B';
                arrive_cnt = 0;
                return;
            }

            /* SP_COAST: 等它停稳 (角速度连续 ARRIVE_TICKS 拍都低于 RATE_STOP) */
            if (rate < IMU_TURN_RATE_STOP)
            {
                if (++arrive_cnt < IMU_TURN_ARRIVE_TICKS) return;   /* 还没确认够, 继续等 */

                /* ======== 停稳了, 落点已定 ======== */
                {
                    float err = ad;

                    /* --- 标定脉冲增益: 本次脉冲(含其后滑行)走了多少度 / 通电多少毫秒 --- */
                    if (pulse_cal)
                    {
                        float moved = _fabs(_norm180(cur_yaw - pulse_yaw0));
                        if ((pulse_ms > 0) && (moved > 0.2f))
                        {
                            float g = moved / (float)pulse_ms;
                            if (g < IMU_TURN_PULSE_GAIN_MIN) g = IMU_TURN_PULSE_GAIN_MIN;
                            if (g > IMU_TURN_PULSE_GAIN_MAX) g = IMU_TURN_PULSE_GAIN_MAX;
                            /* ★ 不对称: 实际走过头了(实测增益更高)要赶紧上修, 否则下一拍还冲;
                             *   实际走得少(增益更低)就慢慢下修 (这个方向是安全的) */
                            pulse_gain += (g - pulse_gain) *
                                          ((g > pulse_gain) ? IMU_TURN_PULSE_GAIN_UP
                                                            : IMU_TURN_PULSE_GAIN_DOWN);
                        }
                        pulse_cal = 0;
                    }

                    /* --- 标定 a: a = w^2 / (2*实际滑行角) ---
                     * 这是个闭环: 老冲过头 -> 反推的 a 变小 -> 预测滑得更远 -> 下次自动切得更早 */
                    if (cal_pending)
                    {
                        float coast = _fabs(_norm180(cur_yaw - cut_yaw));
                        /* coast < 170: norm180 已经归一, 超过 170 说明这一下转了大半圈, 数据不可信 */
                        if ((cut_rate > 40.0f) && (coast > 0.5f) && (coast < 170.0f))
                        {
                            float a = cut_rate * cut_rate / (2.0f * coast);
                            if (a < IMU_TURN_DECEL_MIN) a = IMU_TURN_DECEL_MIN;
                            if (a > IMU_TURN_DECEL_MAX) a = IMU_TURN_DECEL_MAX;
                            /* ★ 不对称学习: 发现"比想的滑得更远"立刻采纳(过冲不可逆),
                             *   发现"比想的滑得更近"慢慢上修(防一次野测值把车带飞) */
                            if (a < decel_est) decel_est += (a - decel_est) * IMU_TURN_DECEL_LPF_DOWN;
                            else               decel_est += (a - decel_est) * IMU_TURN_DECEL_LPF_UP;
                        }
                        cal_pending = 0;
                    }

                    /* --- 到位 / 不该再补 -> 收工 --- */
                    if ((err <= IMU_TURN_ARRIVE_DEG) || has_crossed ||
                        (IMU_TURN_PULSE_ENABLE == 0) || (pulse_cnt >= IMU_TURN_PULSE_MAX))
                    {
                        last_err = err;
                        _finish(IMU_TURN_DONE_ARRIVED);
                        return;
                    }

                    /* --- 停早了: 用最小 PWM 打一个"短到转速还没建立起来"的脉冲 ---
                     * ★ 时长按【剩余误差的一半】算 (等比收敛): 
                     *   一次补满是脆的 —— 脉冲增益估偏一点就直接冲过目标(不可逆);
                     *   补一半则误差每拍减半, 增益估偏 2 倍也只是"刚好到位", 永远不会过冲。 */
                    {
                        float t;
                        if (pulse_gain < IMU_TURN_PULSE_GAIN_MIN) pulse_gain = IMU_TURN_PULSE_GAIN_MIN;
                        t = (IMU_TURN_PULSE_FRAC * err) / pulse_gain;        /* 毫秒 */
                        if (t < (float)IMU_TURN_PULSE_MIN_MS) t = (float)IMU_TURN_PULSE_MIN_MS;
                        if (t > (float)IMU_TURN_PULSE_MAX_MS) t = (float)IMU_TURN_PULSE_MAX_MS;
                        pulse_ms = (uint32_t)t;
                    }
                    pulse_cnt++;
                    pulse_dir   = dir;                        /* 方向在这一刻锁死, 脉冲中途不翻 */
                    pulse_start = now;
                    pulse_yaw0  = cur_yaw;
                    spot_phase  = SP_PULSE;
                    spot_st     = 'P';
                    arrive_cnt  = 0;
                    Motor_SetLeftPWM (pulse_dir * min_pwm);
                    Motor_SetRightPWM(-pulse_dir * min_pwm);
                    return;
                }
            }
            else
            {
                arrive_cnt = 0;                               /* 还在动, 重新数 */
            }
            return;
        }

        /* ================= 还在驱动 (FAR / NEAR) ================= */

        /* 保护: 驱动段却一直"远离目标" -> 方向接反 / 车被顶住 / IMU 姿态异常, 立刻停车。
         * ★ 判据要和方向无关: 旋转方向和"需要的方向"相反就是远离 (原来只写 v_signed<-RATE_STOP,
         *   那只对正方向转向成立, 负方向一进去就会误报) */
        if (((v_signed >  IMU_TURN_RATE_STOP) && (diff_deg < 0.0f)) ||
            ((v_signed < -IMU_TURN_RATE_STOP) && (diff_deg > 0.0f)))
        {
            if (++errway_cnt >= IMU_TURN_ERRWAY_TICKS)
            {
                last_err = ad;
                _finish(IMU_TURN_DONE_ERRWAY);
                return;
            }
        }
        else errway_cnt = 0;

        /* 该切断驱动了? (冲过目标 或 剩余角 <= 预测滑行角)
         * ★★ 必须同时是 NEAR 段: FAR(高速)段即使算出来满足也【不许切】,
         *    否则一进 FAR 就立刻满足 "剩余角 <= coast_pred(182)" = 全速切断 = 滑 180 度。
         *    冲过目标(has_crossed)是例外: 那时必须马上刹住。 */
        if (((has_crossed || ((spot_phase == SP_NEAR) && (ad <= coast_pred))))
            && ((now - start_ms) >= IMU_TURN_STARTUP_MS))
        {
            cut_rate    = rate;              /* 记下 w 和 yaw, 停稳后用它们标定 a */
            cut_yaw     = cur_yaw;
            cal_pending = 1;
            spot_phase  = SP_COAST;
            spot_st     = 'B';
            arrive_cnt  = 0;
            Motor_Brake();
            return;
        }

        /* 该减速了? (单向闩锁: 进了 NEAR 就不再回 FAR)
         * ★ FAR 段必须用【满速估计】而不是本拍实测角速度: 本车 a=145 时满速 coast
         *   = 230^2/(2*145) = 182 度; 拿实测角速度算要等转速真的升上来才成立,
         *   那时车已经转过半个弯, 剩下的角度不够减速 -> 必然冲过去(过冲 25 度以上)。
         *   按满速估计算的结果: 90/180 度转向一开始就直接进 NEAR 低速段。 */
        {
            float a_eff    = decel_est * IMU_TURN_DECEL_TRUST;
            float rate_far = (spot_phase == SP_FAR) ? IMU_TURN_RATE_FAST : rate;
            if ((spot_phase == SP_FAR) &&
                (ad <= (rate_far * rate_far / (2.0f * a_eff) + blind + IMU_TURN_SLOW_MARGIN)))
                spot_phase = SP_NEAR;
        }

        if (spot_phase == SP_NEAR)
        {
            z = dir * min_pwm;              /* 低速接近, 把惯性先降下来 */
            spot_st = 'C';
        }
        else
        {
            z = dir * turn_speed;           /* 远段定速 */
            spot_st = 'D';
        }
        arrive_cnt = 0;

        Motor_SetLeftPWM (z);               /* z>0 = 左转: 左轮正转, 右轮反转 */
        Motor_SetRightPWM(-z);
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

int   ImuTurn_GetDoneReason(void)  { return done_reason; }
float ImuTurn_GetYaw(void)         { return cur_yaw; }
float ImuTurn_GetDiff(void)        { return diff_deg; }
float ImuTurn_GetTarget(void)      { return target_yaw; }
int   ImuTurn_GetMode(void)        { return turn_mode; }
float ImuTurn_GetRate(void)        { return v_app_f; }      /* 接近角速度 (度/秒) */
char  ImuTurn_GetPhase(void)       { return spot_st;  }      /* 本拍相位: D/C/B/P */
float ImuTurn_GetCoast(void)       { return coast_pred; }    /* 预测滑行角 (度) */
float ImuTurn_GetDecel(void)       { return decel_est;   }   /* 标定出的 a (度/秒^2) */
float ImuTurn_GetLastErr(void)     { return last_err;    }   /* 上次落点误差 (度) */
int   ImuTurn_GetPulseCount(void)  { return pulse_cnt;   }   /* 上次用了几拍脉冲 */

