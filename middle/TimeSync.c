/*===========================================================================
 * middle/TimeSync.c —— I2C 授时查询端实现 (接口/接线/调参入口见 TimeSync.h)
 *---------------------------------------------------------------------------
 * 数据流:
 *   TimeSync_Poll()  (100ms 任务, 到点才真读)
 *        -> ts_i2c_read()   纯读 15 字节 (I2C_TIME_INST = I2C1, PA30=SDA/PA29=SCL, 400kHz)
 *        -> ts_parse()      校验 + "YYYYMMDDhhmmss" -> 真 UTC 秒 (减 8h)
 *        -> 存基准 (ts_base_utc, ts_sync_ms)
 *   任何取时间的接口都是 "基准 + (Get_Time()-基准时刻)/1000" 现算, 不用打点累加。
 *===========================================================================*/

#include "TimeSync.h"
#include "Timer.h"      /* Get_Time(): 1ms 系统时基 (TIMER_0 中断累加) */

/*-------------------------------- 内部状态 --------------------------------*/
static uint8_t  ts_raw[TIMESYNC_FRAME_LEN];     /* 最近一次读到的原始帧 */
static uint8_t  ts_valid    = 0;                /* 1 = 有走时基准 (曾经同步成功过) */
static uint8_t  ts_synced   = 0;                /* 1 = 最近一次读回的就是有效时间 */
static uint8_t  ts_fail     = 0;                /* 连续读失败次数 */
static uint8_t  ts_force    = 1;                /* 1 = 下次 Poll 立刻读 (Init 后第一次) */
static uint32_t ts_sync_ms  = 0;                /* 建基准那一刻的 Get_Time() */
static uint32_t ts_base_utc = 0;                /* 建基准那一刻的真 UTC 秒 */
static uint32_t ts_last_ms  = 0;                /* 上次尝试读的时刻 */
static uint32_t ts_ok_cnt   = 0;
static uint32_t ts_err_cnt  = 0;

/*-------------------------------- 日期换算 --------------------------------*/
/* Howard Hinnant 的 civil<->days 算法 (microlib 没有 time.h, 自己算)
 * 天数基准 1970-01-01; 只做加减和格式化, 和时区无关 */
static int32_t ts_days_from_civil(int32_t y, int32_t m, int32_t d)
{
    int32_t era, yoe, doy, doe;

    y  -= (m <= 2);
    era = (y >= 0 ? y : y - 399) / 400;
    yoe = y - era * 400;                                        /* [0,399]   */
    doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;       /* [0,365]   */
    doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;                /* [0,146096]*/

    return era * 146097 + doe - 719468;
}

static void ts_civil_from_days(int32_t z, int32_t *py, int32_t *pm, int32_t *pd)
{
    int32_t era, doe, yoe, y, doy, mp, d, m;

    z  += 719468;
    era = (z >= 0 ? z : z - 146096) / 146097;
    doe = z - era * 146097;                                     /* [0,146096]*/
    yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;/* [0,399]   */
    y   = yoe + era * 400;
    doy = doe - (365 * yoe + yoe / 4 - yoe / 100);              /* [0,365]   */
    mp  = (5 * doy + 2) / 153;                                  /* [0,11]    */
    d   = doy - (153 * mp + 2) / 5 + 1;                         /* [1,31]    */
    m   = mp + (mp < 10 ? 3 : -9);                              /* [1,12]    */

    *py = y + (m <= 2);
    *pm = m;
    *pd = d;
}

static uint32_t ts_atoi(const uint8_t *s, int n)
{
    uint32_t v = 0;
    int      i;

    for (i = 0; i < n; i++) v = v * 10u + (uint32_t)(s[i] - '0');
    return v;
}

/*------------------------------ I2C 15 字节纯读 ----------------------------*/
/* 返回: 0 成功 / -1 等空闲超时 / -2 从机 NACK(器件不在) / -3 收数据超时 / -4 STOP 超时
 * 注意: 本函数是阻塞的, 正常约 0.6ms (400kHz), 最坏 TIMESYNC_TIMEOUT_MS */
static int ts_i2c_read(uint8_t *buf, uint8_t len)
{
    uint32_t t0 = Get_Time();
    uint32_t i  = 0;

    /* 1) 等控制器空闲 (上一次事务没做完就别插队) */
    while (!(DL_I2C_getControllerStatus(I2C_TIME_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        if ((uint32_t)(Get_Time() - t0) > TIMESYNC_TIMEOUT_MS) return -1;
    }

    /* 2) 发起纯读事务: 驱动会自动产生 START + (0x42<<1|1) + ... + STOP
     *    ★ 必须先写寄存器/命令再读 —— 协议明确不支持, 这里就是"一读到底" */
    DL_I2C_startControllerTransfer(I2C_TIME_INST, TIMESYNC_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    /* 3) 逐字节收满 */
    while (i < len)
    {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_TIME_INST))
        {
            buf[i++] = (uint8_t)DL_I2C_receiveControllerData(I2C_TIME_INST);
            continue;
        }

        /* 从机没应答 (器件不在 / 线没接 / 地址不对): 立刻退出, 别干等满超时 */
        if (DL_I2C_getControllerStatus(I2C_TIME_INST) & DL_I2C_CONTROLLER_STATUS_ERROR)
        {
            DL_I2C_flushControllerRXFIFO(I2C_TIME_INST);
            DL_I2C_resetControllerTransfer(I2C_TIME_INST);
            return -2;
        }

        if ((uint32_t)(Get_Time() - t0) > TIMESYNC_TIMEOUT_MS)
        {
            DL_I2C_flushControllerRXFIFO(I2C_TIME_INST);
            DL_I2C_resetControllerTransfer(I2C_TIME_INST);
            return -3;
        }
    }

    /* 4) 等 STOP 发完 */
    while (DL_I2C_getControllerStatus(I2C_TIME_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
    {
        if ((uint32_t)(Get_Time() - t0) > TIMESYNC_TIMEOUT_MS)
        {
            DL_I2C_resetControllerTransfer(I2C_TIME_INST);
            return -4;
        }
    }

    return 0;
}

/*-------------------------------- 帧解析 ----------------------------------*/
/* 返回: 0 = 有效时间, 已更新走时基准
 *       1 = 帧合法但模块还没联网 (Status=0x00), 沿用旧基准继续走
 *      -1 = 帧非法 (Status 不是 0/1, 或时间字段不是纯数字/超范围) */
static int ts_parse(const uint8_t *b, uint32_t now_ms)
{
    uint32_t y, mo, d, h, mi, s;
    uint32_t i;
    int32_t  days;

    if (b[0] != 0x00 && b[0] != 0x01) return -1;

    for (i = 1; i < TIMESYNC_FRAME_LEN; i++)
    {
        if (b[i] < '0' || b[i] > '9') return -1;      /* 定长纯数字, 有一个不对就是脏帧 */
    }

    if (b[0] == 0x00) return 1;                        /* 总线通, 但模块没联上网 */

    y  = ts_atoi(&b[1],  4);
    mo = ts_atoi(&b[5],  2);
    d  = ts_atoi(&b[7],  2);
    h  = ts_atoi(&b[9],  2);
    mi = ts_atoi(&b[11], 2);
    s  = ts_atoi(&b[13], 2);

    /* 范围校验: 挡掉"数字但不可能"的帧 */
    if (y < 2020u || y > 2099u)               return -1;
    if (mo < 1u || mo > 12u || d < 1u || d > 31u) return -1;
    if (h > 23u || mi > 59u || s > 60u)       return -1;

    /* 东八区墙钟 -> 真 UTC 秒。
     * 全部用 32 位无符号算: 秒数上限 = 2106-02-07 07:28:15, 覆盖 2020~2099 的校验区间绰绰有余,
     * 又不用拖 64 位乘除的库函数 (省 ROM)。 */
    days = ts_days_from_civil((int32_t)y, (int32_t)mo, (int32_t)d);

    ts_base_utc = (uint32_t)days * 86400u + h * 3600u + mi * 60u + s
                - TIMESYNC_TZ_OFFSET_SEC;
    ts_sync_ms  = now_ms;

    return 0;
}

/*--------------------------------- 对外接口 -------------------------------*/
void TimeSync_Init(void)
{
    uint32_t i;

    for (i = 0; i < TIMESYNC_FRAME_LEN; i++) ts_raw[i] = 0;

    ts_valid    = 0;
    ts_synced   = 0;
    ts_fail     = 0;
    ts_force    = 1;
    ts_sync_ms  = 0;
    ts_base_utc = 0;
    ts_last_ms  = Get_Time();
    ts_ok_cnt   = 0;
    ts_err_cnt  = 0;
}

void TimeSync_PollNow(void)
{
    ts_force = 1;
}

void TimeSync_Poll(void)
{
    uint32_t now = Get_Time();
    uint32_t period;
    int      r, p;

    /* 什么时候该去读:
     *   已经有时间 -> 每 TIMESYNC_CALIB_MS 校准一次
     *   还没有时间 -> 每 TIMESYNC_RETRY_MS 试一次
     *   总线故障   -> 退避到 TIMESYNC_BACKOFF_MS (别反复卡住调度器) */
    if (ts_valid)          period = TIMESYNC_CALIB_MS;
    else if (ts_fail >= TIMESYNC_FAIL_MAX) period = TIMESYNC_BACKOFF_MS;
    else                   period = TIMESYNC_RETRY_MS;

    if (!ts_force && (uint32_t)(now - ts_last_ms) < period) return;

    ts_force  = 0;
    ts_last_ms = now;

    r = ts_i2c_read(ts_raw, TIMESYNC_FRAME_LEN);

    if (r != 0)
    {
        /* 读失败: 总线/器件问题 (线没接、上拉不对、模块没电、地址不对) */
        if (ts_fail < 255u) ts_fail++;
        ts_err_cnt++;
        ts_synced = 0;
        return;
    }

    ts_fail = 0;          /* 器件有应答 -> 总线是通的 */

    p = ts_parse(ts_raw, now);

    if (p == 0)
    {
        ts_valid  = 1;
        ts_synced = 1;
        ts_ok_cnt++;
    }
    else if (p == 1)
    {
        /* 模块在, 但还没联上网/没对到时: 不算错误, 沿用上次基准继续走时 */
        ts_synced = 0;
    }
    else
    {
        ts_err_cnt++;     /* 帧非法: 当一次错误, 基准不动 */
        ts_synced = 0;
    }
}

int TimeSync_Status(void)
{
    if (ts_fail >= TIMESYNC_FAIL_MAX) return -1;   /* 总线故障 */
    if (ts_synced)                    return  1;   /* 时间可用 */
    return 0;                                      /* 总线通, 模块还没同步 */
}

int TimeSync_IsValid(void)
{
    return ts_valid ? 1 : 0;
}

uint32_t TimeSync_GetEpochUtc(void)
{
    if (!ts_valid) return 0u;

    return ts_base_utc + (uint32_t)((Get_Time() - ts_sync_ms) / 1000u);
}

int TimeSync_Get(TimeSync_DateTime *out)
{
    uint32_t wall, rem;
    int32_t  y, mo, d;

    if (!ts_valid || out == 0) return 0;

    /* 本板走时基准是真 UTC; 显示/记录都要东八区墙钟, 这里加回来 */
    wall = TimeSync_GetEpochUtc() + (uint32_t)TIMESYNC_TZ_OFFSET_SEC;
    rem  = wall % 86400u;

    ts_civil_from_days((int32_t)(wall / 86400u), &y, &mo, &d);

    out->year = (uint16_t)y;
    out->mon  = (uint8_t)mo;
    out->day  = (uint8_t)d;
    out->hour = (uint8_t)(rem / 3600u);
    out->min  = (uint8_t)((rem % 3600u) / 60u);
    out->sec  = (uint8_t)(rem % 60u);

    return 1;
}

/* 字符串输出: 都是静态缓冲, 每次调用覆盖上一次的内容 */
static void ts_put2(char *p, uint32_t v) { p[0] = (char)('0' + (v / 10u) % 10u); p[1] = (char)('0' + v % 10u); }
static void ts_put4(char *p, uint32_t v) { ts_put2(p, v / 100u); ts_put2(p + 2, v % 100u); }

const char *TimeSync_GetStr(void)
{
    static char s[24];
    TimeSync_DateTime dt;

    if (!TimeSync_Get(&dt))
    {
        return "0000-00-00 00:00:00";           /* 未同步: 定长占位, 调用方用 Status() 判断 */
    }

    ts_put4(s,     dt.year);
    s[4] = '-';
    ts_put2(s + 5, dt.mon);
    s[7] = '-';
    ts_put2(s + 8, dt.day);
    s[10] = ' ';
    ts_put2(s + 11, dt.hour);
    s[13] = ':';
    ts_put2(s + 14, dt.min);
    s[16] = ':';
    ts_put2(s + 17, dt.sec);
    s[19] = 0;

    return s;
}

const char *TimeSync_GetCompact(void)
{
    static char s[16];
    TimeSync_DateTime dt;

    if (!TimeSync_Get(&dt)) return "00000000000000";

    ts_put4(s,     dt.year);
    ts_put2(s + 4, dt.mon);
    ts_put2(s + 6, dt.day);
    ts_put2(s + 8, dt.hour);
    ts_put2(s + 10, dt.min);
    ts_put2(s + 12, dt.sec);
    s[14] = 0;

    return s;
}

const char *TimeSync_GetClock(void)
{
    static char s[16];
    TimeSync_DateTime dt;

    if (!TimeSync_Get(&dt)) return "-- --:--:--";

    ts_put2(s,     dt.mon);
    s[2] = '-';
    ts_put2(s + 3, dt.day);
    s[5] = ' ';
    ts_put2(s + 6, dt.hour);
    s[8] = ':';
    ts_put2(s + 9, dt.min);
    s[11] = ':';
    ts_put2(s + 12, dt.sec);
    s[14] = 0;

    return s;
}

uint32_t TimeSync_GetAgeSec(void)
{
    if (!ts_valid) return 0xFFFFFFFFu;      /* 从没成功过 */
    return (uint32_t)((Get_Time() - ts_sync_ms) / 1000u);
}

uint32_t TimeSync_GetOkCount(void)  { return ts_ok_cnt;  }
uint32_t TimeSync_GetErrCount(void) { return ts_err_cnt; }
const uint8_t *TimeSync_GetRaw(void) { return ts_raw; }
