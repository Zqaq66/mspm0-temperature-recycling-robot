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
#define NOISE_THRESHOLD         30      /* 噪声/白底阈值 (0~255) */
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

static int base_speed = TRACK_SPEED_DEFAULT;    /* 直行基础速度(开环 PWM), 在线可调 */
int track_base_speed  = TRACK_SPEED_DEFAULT;    /* 对外只读副本 */

static uint8_t reset_req = 0;                   /* 1 = 下一拍清历史状态 */

/* 左右输出统一出口 (TRACK_LR_SWAP 在这里起作用) */
static void track_output(int left, int right)
{
    track_left_pwm  = left;
    track_right_pwm = right;
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
    Track_SetBaseSpeed(TRACK_SPEED_DEFAULT);
    reset_req = 1;
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
    }

    /* ------ 1. 读 8 路模拟量 (0x09, 除8 数据 0~255) ------ */
    if (Xinliu_AuroraT8_Sensor_Read(GRAY_CMD_ALL_DIV8, &data) != 0)
    {
        track_i2c_err++;                 /* 读失败: 本轮不动作, 保持上次 PWM */
        return;
    }

    /* ------ 2. 平方加权质心 ------ */
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
        if (v < NOISE_THRESHOLD) continue;      /* 白底/噪声 */

        bits &= ~(uint8_t)(1u << i);            /* bit=0 表示该路压线 */

        {
            uint32_t w = (uint32_t)v * (uint32_t)v;
            num += w * (uint32_t)(i * 100);
            den += w;
        }
    }
    track_bits = bits;
    now = Get_Time();

    /* ------ 3. 丢线 (全部低于阈值) ------ */
    if (den == 0)
    {
        uint32_t lost_ms = now - last_time;     /* 距"最后一次看见线"的时间 */

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
