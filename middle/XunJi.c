/*===========================================================================
 * 循迹控制层 (普通巡线)
 *
 *   控制流水线 (5ms 一拍):
 *     data[8] (0x09 模拟量 0~255)
 *       -> 平方加权质心  w=v*v, pos=Σ(w*i*100)/Σw   -> 0~700, 中心 350
 *       -> error = pos - 350                    (负 = 线偏左)
 *       -> 三区增益调度 PD (直线软增益 <-> 弯道激进增益, 中间线性过渡)
 *       -> turn = (Kp*error + Kd*Δerror低通) / 1024   (Q10)
 *       -> 弯道自适应降速 speed = base*(1 - 降速比*|error|/350)
 *       -> 左 = speed + turn, 右 = speed - turn  (turn>0 = 线偏右 = 左轮快)
 *       -> Motor_SetLeftPWM / Motor_SetRightPWM
 *
 *   二进制约定: bit=0 表示该路压线 (track_bits, 供 LCD 显示/丢线判断)
 *   传感器: 芯流 AuroraT8, I2C0(PA0=SDA/PA1=SCL), 读 0x09 八路模拟量(0~255)
 *===========================================================================*/

#include "ti_msp_dl_config.h"
#include "XunJi.h"
#include "Xinliu_AuroraT8_IIC.h"
#include "Motor.h"
#include "Timer.h"              /* Get_Time() 1ms 时基 */
#include "stdio.h"

/* ============================ 调参入口 (都在这个文件顶部) ============================
 * ---- 传感器 ----
 *   NOISE_THRESHOLD : 低于此值的路当噪声丢掉 (同时就是"有没有压线"的判断)
 *                     白底反而偏大 -> 说明极性反了, 把 TRACK_DARK_IS_HIGH 改 0
 *   TRACK_DARK_IS_HIGH : 1 = 数值越大越黑 (AuroraT8 0x09 默认口径); 0 = 反相
 *   TRACK_SENSOR_FLIP  : 1 = 8 路左右镜像 (探头物理装反时用)
 *   TRACK_LR_SWAP      : 1 = 左右输出对调 (实测"越偏越往错方向打"时用)
 *
 * ---- 转向 PD (Q10 定点, 1024 = 1.0) ----
 *   TRACK_KP / KD          弯道段增益 (|err| 大时用)
 *   TRACK_STRAIGHT_KP / KD 直线段增益 (|err| 小时用, 软一点抑制直线微振)
 *   TRANSITION_LOW/HIGH    过渡带: |err|<LOW 纯直线增益, >=HIGH 纯弯道增益, 中间线性插值
 *   D_FILTER_STRENGTH      D 项一阶低通强度 (0=不滤波, 1~8 越大约平滑, 默认 3)
 *   TRACK_TURN_MAX         转向量限幅 (PWM 差值上限)
 *   CURVE_REDUCE_PCT       弯道最大降速百分比 (0=不降速, 100=最急弯停住)
 *
 *   调法: 先只改 TRACK_KP (直线画龙 -> 调小; 弯道切内线/出线 -> 调大),
 *         直线段还在抖再降 TRACK_STRAIGHT_KP, 过冲大再把 TRACK_KD 加大
 * ===============================================================================*/
#define NOISE_THRESHOLD         30      /* 固定阈值: 只在自适应标定完成前兜底用 (见下面) */

/* ---- 自适应阈值 (解决"白底上最外侧概率性误判成压线") ----
 * 为什么: 8 路探头的白底基线天生不一样 (探头个体差异 + 离地高度/倾斜 + 最外侧两路在模块边缘
 *         会漏光/反光)。用一个固定阈值卡 8 路 -> 白底偏高那几路 (常见就是最外侧) 概率性跨阈值,
 *         屏上 '#' 闪; 而且路口判据是"最左或最右压线", 会连带把路口判错。
 * 做法: 上电 (或调 Track_CalibWhite()) 采 CALIB_FRAMES 帧, 每路取平均 = 白底基线 track_white[i],
 *       判定改成  v > white[i] + ADAPT_MARGIN  才算压线, 阈值不低于 ADAPT_MIN。
 *       标定没完成时 track_thr[i] = NOISE_THRESHOLD (旧行为), 不会一脚踩空。
 * ★ 标定时车必须停着、且 8 路都在白底上 (上电后的默认位置通常就是白底)。 */
#ifndef TRACK_CALIB_ON_BOOT          /* 也可以用 Keil Misc Controls 加 -D 覆盖 */
#define TRACK_CALIB_ON_BOOT     1       /* 1 = 上电自动标一次白底; 0 = 不自动, 只靠 Track_CalibWhite() */
#endif
#ifndef ADAPT_MARGIN
#define ADAPT_MARGIN            60      /* 高于白底基线这么多就算压线 (黑线越黑这个可以越大) */
#endif
#ifndef ADAPT_MIN
#define ADAPT_MIN               25      /* 阈值绝对下限 (防基线异常低导致过敏) */
#endif
#ifndef CALIB_FRAMES
#define CALIB_FRAMES            20      /* 标定采样帧数 (5ms 一拍 -> 20 帧 = 100ms) */
#endif
#define TRACK_DARK_IS_HIGH      1       /* 1 = 越大越黑; 0 = 越小越黑 */
#define TRACK_SENSOR_FLIP       0       /* 1 = 8 路左右镜像 */
#define TRACK_LR_SWAP           0       /* 1 = 左右输出对调 */

#define CENTER_POS              350     /* 质心中心 (8 路 -> 3.5 路) */
#define Q_SCALE                 1024    /* Q10 定点标度 */

#define TRACK_KP                400    /* 弯道段 Kp 1000 (≈0.98) */
#define TRACK_KD                1200    /* 弯道段 Kd 1200 (≈1.17, 作用在低通后的 Δerror 上) */
#define TRACK_STRAIGHT_KP       300     /* 直线段 Kp 500  (≈0.49) */
#define TRACK_STRAIGHT_KD       600     /* 直线段 Kd 600  (≈0.59) */
#define TRANSITION_LOW          40      /* |err| < 40  -> 纯直线增益 */
#define TRANSITION_HIGH         160     /* |err| >= 160 -> 纯弯道增益 */
#define D_FILTER_STRENGTH       3       /* D 项低通强度 (1/2^3 = 1/8) */
#define TRACK_TURN_MAX          500     /* turn 限幅 */
#define CURVE_REDUCE_PCT        50      /* 弯道最大降速 50% */

/* ---- 丢线处理参数 (判别 + 找回) ----
 * 丢线瞬间先看"丢线前最后一帧的 |error|", 分成两种情况:
 *   A. 偏出 (|error| >= LOST_SIDE_ERR): 线被甩到一侧了
 *        -> 直接按偏向差速转弯找回 (内侧轮慢, 外侧轮快, 两轮同向不反转), 不直行
 *   B. 中间消失 (|error| < LOST_SIDE_ERR): 线在中间突然没了 (断线/到终点)
 *        -> 保持直行 LOST_STRAIGHT_MS, 然后停车
 * 找回速度刻意压低: LOST_TURN_SPEED 是外侧轮 PWM, 默认 250 (别超过正常巡线速度) */
#define LOST_KEEP_MS            60      /* 丢线后先去抖保持这么久 (避免单帧误丢就乱打) */
#define LOST_SIDE_ERR          120      /* 判定"偏出"的 |error| 阈值 (0~350) */
#define LOST_TURN_SPEED        250      /* 找回线: 外侧轮 PWM */
#define LOST_TURN_DIFF         150      /* 找回线: 内外差速 (内侧 = 250-150 = 100, 同向) */
#define LOST_FIND_TIMEOUT_MS  2500      /* 偏出找回超时 -> 停车; 填 0 = 不超时一直找 */
#define LOST_STRAIGHT_MS       500      /* 中间消失: 直行这么久后停车 */

/* 对外状态量 (供 LCD / 串口观察) */
int     track_error     = 0;
int     track_pos       = CENTER_POS;
int     track_speed     = 0;
int     track_left_pwm  = 0;
int     track_right_pwm = 0;
int     track_lost      = 0;
uint8_t track_bits      = 0xFF;
int     track_i2c_err   = 0;

/* ---- 路口 / 房间检测的对外状态 (见 XunJi.h) ---- */
int     track_nblack    = 0;
int     track_cross     = 0;
int     track_cross_side= 0;
int     track_room      = 0;
int     track_cross_cnt = 0;

/* ---- 岔路 / 普通弯 (见 XunJi.h): 电平锁存给屏看 + 边沿事件给状态机 ---- */
int     track_branch       = 0;
int     track_branch_side  = 0;
int     track_corner       = 0;
int     track_corner_side  = 0;

/* ---- 自适应阈值的对外量 (给屏 / 串口看) ---- */
uint8_t track_raw[8];               /* 每路原始值 (0~255, 已按极性校正: 越大越黑) */
uint8_t track_white[8];             /* 每路白底基线 (标定出来的) */
uint8_t track_thr[8] = { NOISE_THRESHOLD, NOISE_THRESHOLD, NOISE_THRESHOLD, NOISE_THRESHOLD,
                         NOISE_THRESHOLD, NOISE_THRESHOLD, NOISE_THRESHOLD, NOISE_THRESHOLD };
                                    /* 每路当前生效阈值: 标定完成前 = NOISE_THRESHOLD */

/* 检测内部: 去抖计数 / 本帧已报标志 / 待取边沿事件 (只在 Track_Run 里更新) */
static uint8_t det_cross_cnt = 0, det_cross_rep = 0, det_cross_ev = 0;
static uint8_t det_room_cnt  = 0, det_room_rep  = 0, det_room_ev  = 0;

/* 岔路/普通弯 判定状态 (2026-10-06) */
static uint8_t br_win    = 0;    /* >0 = 判定窗口还剩几帧 */
static int8_t  br_side   = 0;    /* 窗口期间记下的边侧: +1 右 / -1 左 */
static uint8_t br_armed  = 1;    /* 离开路口区域后才允许再开窗口 (防同一个口连触) */
static uint8_t br_ev     = 0, cn_ev = 0;   /* 边沿事件 */
static uint8_t cn_hold   = 0;    /* 已确认普通弯: 丢线找回让位给"前移+原地转", 直到重新看见线 */
/* ---- 误判抑制 (2026-10-06: 转向时边缘蹭线被当成岔路/普通弯) ---- */
static uint8_t  det_en    = 1;      /* [D] 1 = 允许岔路/普通弯判定 (转向全过程由状态机关掉) */
static uint32_t sup_until = 0;      /* [A] 屏蔽判定到什么时候 (ms); 0 = 不屏蔽 */
static uint8_t  rearm_cnt = 0;      /* [E] "线在中间"的连续帧数 (够 REARM_FRAMES 才重新武装) */
/* 白底标定内部量 */
static uint16_t calib_sum[8];
static uint8_t  calib_cnt   = 0;    /* 已采帧数 */
static uint8_t  calibrating = 0;    /* 1 = 正在标定 (★必须独立于 calib_cnt: 光看 cnt<FRAMES
                                     *   会把"从没标定过"也当成标定中, 那就永远不判线了) */
static uint8_t  calib_req   = 0;    /* 1 = 下一拍重新开始标定 */

static int base_speed = TRACK_SPEED_DEFAULT;    /* 直行基础速度(开环 PWM), 在线可调 */
int track_base_speed  = TRACK_SPEED_DEFAULT;    /* 对外只读副本 */

static uint8_t reset_req = 0;                   /* 1 = 下一拍清历史状态 */
static uint8_t drive_en  = 1;                   /* 0 = 观察模式: 照跑算法但不驱动电机 (Track_SetDrive) */

/* 左右输出统一出口 (TRACK_LR_SWAP 在这里起作用) */
static void track_output(int left, int right)
{
    track_left_pwm  = left;
    track_right_pwm = right;

    if (!drive_en) return;      /* 观察模式: 屏上照旧显示左右 PWM, 但不真的写电机 */
#if TRACK_LR_SWAP
    Motor_SetLeftPWM(right);
    Motor_SetRightPWM(left);
#else
    Motor_SetLeftPWM(left);
    Motor_SetRightPWM(right);
#endif
}

/*---------------------------------------------------------------------------
 * 速度在线可调: 由 10ms 按键任务调用 (见 middle/Key.c)
 *-------------------------------------------------------------------------*/
void Track_SetBaseSpeed(int s)
{
    if (s < TRACK_SPEED_MIN) s = TRACK_SPEED_MIN;
    if (s > TRACK_SPEED_MAX) s = TRACK_SPEED_MAX;
    base_speed = s;
    track_base_speed = s;
}

int Track_GetBaseSpeed(void)
{
    return base_speed;
}

void Track_StepSpeed(int dir)
{
    if (dir == 0) return;

    base_speed += TRACK_SPEED_STEP * dir;

    /* 到顶/到底回绕 —— 只有一个按键, 靠回绕把速度调回来
     * 不想要回绕就把下面两行改成封顶:  if (base_speed > MAX) base_speed = MAX; */
    if (base_speed > TRACK_SPEED_MAX) base_speed = TRACK_SPEED_MIN;
    if (base_speed < TRACK_SPEED_MIN) base_speed = TRACK_SPEED_MAX;

    track_base_speed = base_speed;
}

/*---------------------------------------------------------------------------
 * 单击"开始"时调一次: 清掉上一轮的丢线与 PD 历史
 *-------------------------------------------------------------------------*/
void Track_Reset(void)
{
    reset_req = 1;
}

void Track_Init(void)
{
    int i;

    Track_SetBaseSpeed(TRACK_SPEED_DEFAULT);

    for (i = 0; i < 8; i++) track_thr[i] = NOISE_THRESHOLD;   /* 标定完成前用固定阈值兜底 */

    reset_req = 1;

#if TRACK_CALIB_ON_BOOT
    calib_req = 1;      /* 上电自动标一次白底 (此时车停着、通常在白底上) */
#endif
}

/*---------------------------------------------------------------------------
 * 重新标定白底基线 (给每路算一个阈值)
 *   调用后下一拍开始采 CALIB_FRAMES 帧, 采完自动生效 (串口会打一行基线+阈值)。
 *   ★ 只能在车停着、8 路都在白底上时调。标定期间这一拍不做控制 (不驱动/不动丢线状态)。
 *   以后想挂按键或串口命令重标, 调这个函数即可 (上电自动那次由 TRACK_CALIB_ON_BOOT 管)。
 *-------------------------------------------------------------------------*/
void Track_CalibWhite(void)
{
    calib_req = 1;
}

/*---------------------------------------------------------------------------
 * 5ms 调用一次: 读灰度 -> 质心 -> PD -> 左右 PWM
 *-------------------------------------------------------------------------*/
void Track_Run(void)
{
    static int8_t   last_side  = 0;     /* -1 丢线前线偏左, +1 偏右 */
    static uint32_t last_time  = 0;     /* 最后一次"看见线"的时刻 */
    static uint8_t  lost_seen  = 0;     /* 本轮的丢线第一拍是否已处理 */
    static uint8_t  lost_big   = 0;     /* 1 = 偏出(按偏向找回), 0 = 中间消失(直行后停车) */
    static int16_t  last_error = 0;
    static int32_t  d_filt     = 0;

    uint32_t num = 0, den = 0;
    uint32_t now;
    int16_t  pos, error, abs_err;
    int32_t  use_kp, use_kd, turn, delta;
    int      speed, left, right, i;
    uint8_t  v, bits = 0xFF;

    /* ------ 单击"开始"之后的第一拍: 清掉上一轮遗留状态 ------ */
    if (reset_req)
    {
        reset_req  = 0;
        lost_seen  = 0;
        lost_big   = 0;
        last_side  = 0;
        last_error = 0;
        d_filt     = 0;
        last_time  = Get_Time();
        det_cross_cnt = det_cross_rep = det_cross_ev = 0;
        det_room_cnt  = det_room_rep  = det_room_ev  = 0;
        track_cross = track_room = 0;
        track_nblack = 0;
        track_cross_cnt = 0;
        br_win = 0; br_side = 0; cn_hold = 0;     /* 岔路/普通弯 判定也从头开始 */
        br_armed = 0; rearm_cnt = 0;               /* ★ 重新武装要等"线回到中间"稳定 (见 2d 的 [E]) */
        br_ev = 0; cn_ev = 0;
        track_branch = track_branch_side = 0;
        track_corner = track_corner_side = 0;
    }

    /* ------ 0. 标定请求: 重新采白底基线 ------ */
    if (calib_req)
    {
        calib_req   = 0;
        calibrating = 1;
        calib_cnt   = 0;
        for (i = 0; i < 8; i++)
        {
            calib_sum[i]  = 0;
            track_thr[i]  = NOISE_THRESHOLD;    /* 采完前先兜底 */
        }
    }

    /* ------ 1. 读 8 路模拟量 (0x09, 除8 数据 0~255) ------ */
    if (Xinliu_AuroraT8_Sensor_Read(GRAY_CMD_ALL_DIV8, &data) != 0)
    {
        track_i2c_err++;                 /* 读失败: 本轮不动作, 保持上次 PWM */
        return;
    }

    /* ------ 2. 读值 -> (标定) / 判定 -> 平方加权质心 ------
     * 阈值是自适应的: v > track_thr[i] 才算压线 (track_thr = 白底基线 + ADAPT_MARGIN)。
     * 标定没完成时 track_thr[i] = NOISE_THRESHOLD, 就是原来那套固定阈值。 */
    for (i = 0; i < 8; i++)
    {
#if TRACK_SENSOR_FLIP
        v = data.all_div8[7 - i];
#else
        v = data.all_div8[i];
#endif
#if !TRACK_DARK_IS_HIGH
        v = (uint8_t)(255 - v);
#endif
        track_raw[i] = v;                       /* 原始值: 给屏上"8 路原始值"那两行看 */

        /* 标定中: 只累加, 不参与质心/压线判定 */
        if (calibrating)
        {
            calib_sum[i] += v;
            continue;
        }

        if (v < track_thr[i]) continue;         /* 白底/噪声 (自适应阈值) */

        bits &= ~(uint8_t)(1u << i);            /* bit=0 表示该路压线 */

        {
            uint32_t w = (uint32_t)v * (uint32_t)v;
            num += w * (uint32_t)(i * 100);
            den += w;
        }
    }
    track_bits = bits;
    now = Get_Time();

    /* ------ 2c. 标定收尾: 采够 CALIB_FRAMES 帧 -> 算白底基线 + 每路阈值 ------ */
    if (calibrating)
    {
        calib_cnt++;

        if (calib_cnt >= CALIB_FRAMES)
        {
            calibrating = 0;
            for (i = 0; i < 8; i++)
            {
                track_white[i] = (uint8_t)(calib_sum[i] / CALIB_FRAMES);
                track_thr[i]   = (uint8_t)(track_white[i] + ADAPT_MARGIN);
                if (track_thr[i] < ADAPT_MIN) track_thr[i] = ADAPT_MIN;
            }

            printf("TRACK calib white=");
            for (i = 0; i < 8; i++) printf("%3u ", (unsigned)track_white[i]);
            printf("thr=");
            for (i = 0; i < 8; i++) printf("%3u ", (unsigned)track_thr[i]);
            printf("\r\n");
        }

        return;         /* 标定期间这一拍不做控制 (只在车停着时标定) */
    }

    /* ------ 2b. 路口 / 房间检测 (连续帧去抖 + 边沿缓存) ------
     * 只在这里算; 后面丢线分支会 return, 但丢线时 bits 全 1 -> 不会误报, 所以放这里安全 */
    {
        int nb = 0, i2;
        int left_black, right_black;

        for (i2 = 0; i2 < 8; i2++) if (!((bits >> i2) & 0x01)) nb++;
        track_nblack = nb;

        left_black  = !((bits >> 0) & 0x01);     /* bit0 = 最左 */
        right_black = !((bits >> 7) & 0x01);     /* bit7 = 最右 */

        /* 路口: 最左 或 最右 压线 */
        if (left_black || right_black)
        {
            if (det_cross_cnt < 250) det_cross_cnt++;
            if ((det_cross_cnt >= CROSS_CONFIRM_FRAMES) && !det_cross_rep)
            {
                det_cross_rep = 1;
                det_cross_ev  = 1;               /* 边沿事件, 等 Track_TakeCross() 取走 */
                track_cross_cnt++;
            }
        }
        else { det_cross_cnt = 0; det_cross_rep = 0; }

        track_cross      = (det_cross_cnt >= CROSS_CONFIRM_FRAMES) ? 1 : 0;
        track_cross_side = (left_black && right_black) ? 0
                         : (left_black ? -1 : (right_black ? 1 : 0));

        /* 房间/工作区: 大面积压线 */
        if (nb >= ROOM_MIN_BLACK)
        {
            if (det_room_cnt < 250) det_room_cnt++;
            if ((det_room_cnt >= CROSS_CONFIRM_FRAMES) && !det_room_rep)
            {
                det_room_rep = 1;
                det_room_ev  = 1;
            }
        }
        else { det_room_cnt = 0; det_room_rep = 0; }

        track_room = (det_room_cnt >= CROSS_CONFIRM_FRAMES) ? 1 : 0;

        /* ------ 2c. 岔路 / 普通弯 判定 (2026-10-06 新增; 2d 误判抑制) ------
         * 边侧压线之后, 看"中心那几路"(bit2~5)回不回来:
         *   回来   -> 岔路  (主线还在)   侧向 = 边侧
         *   不回来 -> 普通弯(主线拐走了) 侧向 = 边侧
         * 这一层只判"是什么/往哪边", 转弯动作由状态机做 (只有它知道这一步要不要转)。
         *
         * ★ 三层误判防护 (2026-10-06 用户实测: 转弯时传感器边缘蹭到线, 被当成岔路连转):
         *   [D] det_en=0: 转向(前移+原地转)全过程不判 —— 状态机用 Track_Detect(0) 关
         *   [A] sup_until: 收工后再屏蔽 TRACK_SUPPRESS_MS —— 状态机用 Track_Suppress() 开
         *   [B] 触发要求"外侧连续 CROSS_EDGE_BLACK_REQ 路同时压线":
         *       真岔口线在口子处是【展宽】的 (你给的模式 = 边侧 3 路黑);
         *       直道上/转弯中"边缘蹭一下"只有最外 1 路黑 -> 不算, 直接滤掉。
         *   [E] 报告过一次后, 要先连续 REARM_FRAMES 帧 |error| < REARM_ERR (线回到中间)
         *       才允许下一次判定 -> 防"转完立刻连转"。
         */
        {
            int center_white = ((bits & BRANCH_CENTER_MASK) == BRANCH_CENTER_MASK);  /* 中心那几路全白 */
            int lrun = 0, rrun = 0, edge_side = 0, i3;
            int32_t e_now = den ? ((int32_t)(num / den) - CENTER_POS) : 9999;   /* 本帧误差 (线不在视野里 = 大数) */
            int suppressed = (!det_en) || ((int32_t)(sup_until - now) > 0);

            /* [B] 外侧连续压线路数: 左从 bit0 起数, 右从 bit7 起数 */
            for (i3 = 0; i3 < 8; i3++) { if (!((bits >> i3) & 1)) lrun++; else break; }
            for (i3 = 0; i3 < 8; i3++) { if (!((bits >> (7 - i3)) & 1)) rrun++; else break; }
            if      (lrun >= CROSS_EDGE_BLACK_REQ && rrun >= CROSS_EDGE_BLACK_REQ) edge_side = 0;   /* 两侧都展宽 = 横穿黑块, 不判 */
            else if (lrun >= CROSS_EDGE_BLACK_REQ) edge_side = -1;
            else if (rrun >= CROSS_EDGE_BLACK_REQ) edge_side = +1;

            if (suppressed)
            {
                /* [D]/[A] 转向中 或 刚收工: 作废一切判定, 连锁存的电平也清掉 (状态机别读到过期结果) */
                br_win = 0; br_side = 0; br_armed = 0; rearm_cnt = 0;
                br_ev = 0; cn_ev = 0;
                track_branch = 0; track_corner = 0;
            }
            else
            {
                /* [E] 重新武装: 线回到中间(且不在路口上)并稳定 REARM_FRAMES 帧 */
                if (!track_cross && (e_now < REARM_ERR) && (e_now > -REARM_ERR))
                {
                    if (rearm_cnt < 250) rearm_cnt++;
                    if (rearm_cnt >= REARM_FRAMES) br_armed = 1;
                }
                else rearm_cnt = 0;

                if (br_win > 0)                                 /* 正在判定窗口里 */
                {
                    /* 触发那侧的 3 路是否已经清空 (bit=1 白): 清空了才可能是"主线回来了"。
                     * 右侧窗口看 bit5~7, 左侧窗口看 bit0~2 (与 [B] 触发用的是同 3 路)。 */
                    int edge_clear = (br_side > 0) ? (((bits >> 5) & 0x07) == 0x07)
                                                   : ((bits & 0x07) == 0x07);

                    if (nb > BRANCH_MAX_BLACK)                  /* 大面积黑: 是房间/主线横条, 撤销判定 */
                    {
                        br_win = 0; br_side = 0;
                    }
                    else if ((!BRANCH_REQ_EDGE_CLEAR || edge_clear) &&
                         !center_white && (nb <= BRANCH_LINE_MAX_BLACK))
                                                                /* 主线回到阵列正中 + 整体细线 + 【触发那侧已经清空】
                                                                 * -> 岔路。
                                                                 * (边侧清空这条是 2026-10-07 加的: 见 XunJi.h 的
                                                                 *  BRANCH_REQ_EDGE_CLEAR —— 否则三路宽贴端的直角弯会被
                                                                 *  当场判成岔路, 因为它和中心掩码重叠) */
                    {
                        if (br_side != 0)
                        {
                            track_branch      = 1;
                            track_corner      = 0;
                            track_branch_side = (int)br_side;
                            br_ev             = 1;
                            br_armed          = 0;              /* 同一个口只报一次 */
                        }
                        br_win = 0; br_side = 0;
                    }
                    else if (--br_win == 0)                     /* 窗口用完中心线还没回来 -> 普通弯 */
                    {
                        track_corner      = 1;
                        track_branch      = 0;
                        track_corner_side = (int)br_side;
                        cn_ev             = 1;
                        cn_hold           = 1;                  /* 丢线找回让位 (见下面 den==0 分支) */
                        br_armed          = 0;
                    }
                }
                else if (track_cross && br_armed && !cn_hold &&
                         (nb <= BRANCH_MAX_BLACK) && (edge_side != 0))
                {
                    /* 刚压到边侧(外侧连续 N 路): 开窗口, 记住哪一侧 (本拍不看中心线) */
                    br_win  = BRANCH_WIN_FRAMES;
                    br_side = (int8_t)edge_side;
                }
            }
        }
    }

    /* ------ 3. 丢线 (全部低于阈值) ------ */
    if (den == 0)
    {
        uint32_t lost_ms = now - last_time;

        /* ★ 岔路/普通弯 判定窗口里, 或已确认是普通弯(主线拐走了): 控制权交给状态机的
         *   "前移 + 原地转", 这里不要自己差速找回 —— 否则两套逻辑打架, 车会在弯口来回蹭。
         *   cn_hold 在重新看见线时(下面第 4 段)自动清掉。 */
        if (br_win > 0 || cn_hold)
        {
            track_lost  = (int)lost_ms;
            track_speed = 0;
            track_output(0, 0);
            return;
        }     /* 距"最后一次看见线"的时间 */

        if (!lost_seen)                          /* 丢线第一拍: 判定是"偏出"还是"中间消失" */
        {
            lost_seen = 1;
            lost_big  = ((last_error >= LOST_SIDE_ERR) ||
                         (last_error <= -LOST_SIDE_ERR)) ? 1 : 0;
        }
        track_lost = (int)lost_ms;               /* 屏显 LOST nnn = 丢线时长(ms) */

        if (lost_ms < LOST_KEEP_MS)              /* 去抖: 短暂丢失先保持上一拍输出 */
        {
            return;
        }

        /* ---- A. 偏出: 直接按偏向差速转弯找回 (不直行, 不原地转) ---- */
        if (lost_big)
        {
#if LOST_FIND_TIMEOUT_MS > 0
            if (lost_ms >= LOST_FIND_TIMEOUT_MS) /* 找太久 -> 停车 (防一直转圈) */
            {
                track_speed = 0;
                track_output(0, 0);
                return;
            }
#endif
            int inner = LOST_TURN_SPEED - LOST_TURN_DIFF;
            int outer = LOST_TURN_SPEED;
            if (inner < 0) inner = 0;

            track_speed = outer;
            if (last_side <= 0) track_output(inner, outer);   /* 丢线前线偏左 -> 左转找回 */
            else                track_output(outer, inner);   /* 偏右 -> 右转找回 */
            return;
        }

        /* ---- B. 中间消失: 先保持直行, 一会儿后停车 ---- */
        if (lost_ms < LOST_STRAIGHT_MS)
        {
            track_speed = base_speed;
            track_output(base_speed, base_speed);
        }
        else
        {
            track_speed = 0;
            track_output(0, 0);                              /* 停车 */
        }
        return;
    }

    /* ------ 4. 线找到了: 位置 + 误差 ------ */
    lost_seen  = 0;                          /* 线回来了: 丢线状态复位, 自动恢复巡线 */
    lost_big   = 0;
    track_lost = 0;
    cn_hold    = 0;                          /* 普通弯已经转过来看见线了, 交还丢线找回 */

    pos   = (int16_t)(num / den);            /* 0~700, 中心 350 */
    error = pos - CENTER_POS;                /* 负 = 线偏左 */
    if (error >  350) error =  350;
    if (error < -350) error = -350;
    abs_err = (error < 0) ? (int16_t)(-error) : error;

    track_pos    = pos;
    track_error  = error;
    last_side    = (error < 0) ? -1 : 1;     /* 丢线后往哪边找 */
    last_time    = now;

    /* ------ 5. D 项: 一阶 IIR 低通 (平滑帧间抖动) ------ */
    delta = (int32_t)error - (int32_t)last_error;
    if (D_FILTER_STRENGTH > 0) d_filt += (delta - d_filt) >> D_FILTER_STRENGTH;
    else                       d_filt  = delta;

    /* ------ 6. 三区增益调度 (线性过渡, 消除硬切换跳变) ------ */
    if (abs_err < TRANSITION_LOW)
    {
        use_kp = TRACK_STRAIGHT_KP;
        use_kd = TRACK_STRAIGHT_KD;
    }
    else if (abs_err >= TRANSITION_HIGH)
    {
        use_kp = TRACK_KP;
        use_kd = TRACK_KD;
    }
    else
    {
        int32_t t    = (int32_t)abs_err - TRANSITION_LOW;
        int32_t band = TRANSITION_HIGH - TRANSITION_LOW;
        use_kp = TRACK_STRAIGHT_KP + ((TRACK_KP - TRACK_STRAIGHT_KP) * t) / band;
        use_kd = TRACK_STRAIGHT_KD + ((TRACK_KD - TRACK_STRAIGHT_KD) * t) / band;
    }

    /* ------ 7. PD -> 转向量 ------ */
    turn = (use_kp * (int32_t)error + use_kd * d_filt) / Q_SCALE;
    if (turn >  TRACK_TURN_MAX) turn =  TRACK_TURN_MAX;
    if (turn < -TRACK_TURN_MAX) turn = -TRACK_TURN_MAX;

    /* ------ 8. 弯道自适应降速 ------ */
    speed = base_speed - (base_speed * CURVE_REDUCE_PCT * abs_err) / (100 * CENTER_POS);
    if (speed < 0) speed = 0;

    /* ------ 9. 输出 ------ */
    left  = speed + (int)turn;               /* turn>0 (线偏右) -> 左轮快 = 右转 */
    right = speed - (int)turn;

    track_speed = speed;
    track_output(left, right);

    last_error = error;
}

/*---------------------------------------------------------------------------
 * 路口/房间事件的"边沿式"读取 (状态机用: 读一次就清)
 *-------------------------------------------------------------------------*/
int Track_TakeCross(void)
{
    int r = (int)det_cross_ev;
    det_cross_ev = 0;
    return r;
}

int Track_TakeRoom(void)
{
    int r = (int)det_room_ev;
    det_room_ev = 0;
    return r;
}

/*---------------------------------------------------------------------------
 * 岔路 / 普通弯 的边沿式读取 (2026-10-06)
 *   事件成立时, 侧向在 track_branch_side / track_corner_side 里 (+1 右 / -1 左)
 *---------------------------------------------------------------------------*/
int Track_TakeBranch(void)
{
    int r = (int)br_ev;
    br_ev = 0;
    return r;
}

int Track_TakeCorner(void)
{
    int r = (int)cn_ev;
    cn_ev = 0;
    return r;
}

/*---------------------------------------------------------------------------
 * 找线用 (2026-10-06): 转弯落点不确定 -> 状态机"转完找线"阶段要自己读线, 但不走 PD
 *   Track_ReadOnce(): 只读一帧灰度, 更新 track_bits/track_raw/track_nblack, 不写电机。
 *                     返回本帧压线路数 (0 = 全白)。
 *   Track_LineFound(): 找线判据 —— "线回到阵列中间(bit2~5 任一路压线)" 或 "同时压了 >=2 路"。
 *                     单路压在阵列最外侧 = 还没对准 (继续找/继续扫), 不认。
 *-------------------------------------------------------------------------*/
int Track_ReadOnce(void)
{
    GrayData d;
    uint8_t  bits = 0xFF;
    int      i, nb = 0;

    if (Xinliu_AuroraT8_Sensor_Read(GRAY_CMD_ALL_DIV8, &d) != 0)
    {
        track_i2c_err++;
        return 0;
    }

    for (i = 0; i < 8; i++)
    {
        uint8_t v = d.all_div8[i];
        track_raw[i] = v;
        if (v >= track_thr[i])
        {
            bits &= (uint8_t)~(1u << i);        /* bit=0 表示压线 */
            nb++;
        }
    }
    track_bits   = bits;
    track_nblack = nb;
    return nb;
}

int Track_LineFound(void)
{
    if (((~track_bits) & BRANCH_CENTER_MASK) != 0) return 1;   /* 线到了中间那几路 */
    if (track_nblack >= 2) return 1;                           /* 至少有 2 路压线 = 真线 */
    return 0;
}

/* 细线正压在阵列正中 = 车已经转到位、线在车轴正下方 (转弯的"传感器出口"判据) */
int Track_LineCentered(void)
{
    if ((track_bits & 0x18) != 0) return 0;      /* bit3/bit4 必须都压线 */
    if (track_nblack > BRANCH_LINE_MAX_BLACK) return 0;   /* 宽黑块 (口子/拐角) 不算 */
    return 1;
}

/*---------------------------------------------------------------------------
 * 停车 (任务状态机的"步间停车"用): 左右 PWM 给 0 + 短路制动
 *   注意: 循迹任务还在 5ms 跑的话下一拍会把 PWM 又写回去 —— 所以状态机在停车
 *         阶段不要再调 Track_Run(), 直接调本函数。
 *-------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 * 台架"观察模式": 车停着也能看传感器 (标定探头 / 路口 / 房间用)
 *   on = 0: Track_Run() 照跑 (读灰度 -> 质心/误差 -> 路口/房间判定 -> 更新给屏的量),
 *           但 track_output() 不写电机 —— 手推车或拿黑线挡探头时, 屏上 8 路点位 / X / H / N
 *           都实时更新, 车不会动。
 *   on = 1: 正常驱动 (单击 S2 开始时会自动切回 1)
 * ★ 只有 track_output() 一条写电机通路, 所以这里一挡就够; Track_Stop() 是"停", 不受影响。
 *-------------------------------------------------------------------------*/
void Track_SetDrive(int on)
{
    drive_en = on ? 1 : 0;

    if (!drive_en)
    {
        Motor_SetLeftPWM(0);
        Motor_SetRightPWM(0);
    }
}

/*---------------------------------------------------------------------------
 * 误判抑制 (2026-10-06): 转向过程中 / 收工之后, 关掉"岔路 / 普通弯"判定
 *   [D] Track_Detect(0): 状态机在自己做动作(前移+原地转)时关掉; 收工 Track_Detect(1) 打开
 *   [A] Track_Suppress(ms): 收工后再屏蔽 ms —— 那段是"找线过程", 传感器扫过线/蹭到线
 *       都很正常, 不该被判成新的岔路或弯 (用户实测的误判就是这个时刻)
 *   两个都会把"正在判定中"的窗口作废, 并清掉锁存的事件, 防止状态机读到过期结果。
 *-------------------------------------------------------------------------*/
static void track_det_clear(void)
{
    br_win = 0; br_side = 0; br_armed = 0; rearm_cnt = 0;
    br_ev = 0; cn_ev = 0;
    track_branch = 0; track_corner = 0;
}

void Track_Detect(int on)
{
    det_en = on ? 1 : 0;
    if (!det_en) track_det_clear();
}

void Track_Suppress(uint32_t ms)
{
    sup_until = Get_Time() + ms;
    track_det_clear();
}

/* 给屏/串口看: 1 = 现在判定被关着 (转向过程 或 收工后的屏蔽窗口里) */
int Track_GetSuppressed(void)
{
    if (!det_en) return 1;
    if ((int32_t)(sup_until - Get_Time()) > 0) return 1;
    return 0;
}

void Track_Stop(void)
{
    track_speed     = 0;
    track_left_pwm  = 0;
    track_right_pwm = 0;
    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);
    Motor_Brake();                 /* 短路制动: 停得住, 不滑 */
}

/*---------------------------------------------------------------------------
 * 旧实现(本工程原有, 未被调用, 保留备查)
 *-------------------------------------------------------------------------*/
int Track_Err(uint8_t Digital)
{
    int weight[8] = {-16, -12, -8, -4, 4, 8, 12, 16};
    int sum = 0;
    int activeSensors = 0;
    static int lastErr = 0;

    for (int i = 0; i < 8; i++)
    {
        if (!((Digital >> i) & 0x01))
        {
            sum += weight[i];
            activeSensors++;
        }
    }
    if (activeSensors == 0) return lastErr;

    {
        int err = (sum / activeSensors) * 5;
        int maxErr = (36 + (8 - activeSensors) * 2) * 5;
        if (err >  maxErr) err =  maxErr;
        if (err < -maxErr) err = -maxErr;
        err = (err * 3 + lastErr) / 4;
        lastErr = err;
        return err;
    }
}
