/*===========================================================================
 * 板载按键驱动 (单键: S2 = PB21, 按下 = 低电平, MCU 内部上拉)
 *
 *   10ms 扫描一拍:
 *     - 连续 2 拍电平一致才算稳定 (20ms 去抖)
 *     - 按下未满 600ms 就松开 -> 单击事件 KEY_EV_CLICK  (开始/停止)
 *     - 按住满 600ms          -> KEY_EV_ACCEL, 并进入长按;
 *                                之后每 100ms 再发一次 KEY_EV_ACCEL
 *     - 长按松开时不再产生单击事件
 *===========================================================================*/

#include "Key.h"

#define KEY_DEBOUNCE_CNT    2       /* 连续 2 拍 (20ms) 电平一致 -> 稳定 */
#define KEY_LONG_TICKS      60      /* 按住 600ms 判定为长按 */
#define KEY_REPEAT_TICKS    10      /* 长按后每 100ms 加速一档 */

static uint8_t  stable  = 0;        /* 去抖后状态: 1 = 按住 */
static uint8_t  last    = 0;        /* 上一拍原始电平 */
static uint8_t  cnt     = 0;        /* 去抖计数 */
static uint16_t hold    = 0;        /* 按住拍数 */
static uint8_t  long_on = 0;        /* 是否已进入长按 */

void Key_Init(void)
{
    stable  = 0;
    last    = 0;
    cnt     = 0;
    hold    = 0;
    long_on = 0;
}

static uint8_t read_raw(void)
{
    return (DL_GPIO_readPins(KEY1_PORT, KEY1_UP_PIN) == 0) ? 1 : 0;   /* 按下 = 低 */
}

int Key_Pressed(void) { return stable; }

int Key_Scan(void)
{
    int     ev = 0;
    uint8_t r  = read_raw();

    /* ---- 去抖 ---- */
    if (r == last) { if (cnt < KEY_DEBOUNCE_CNT) cnt++; }
    else           { last = r; cnt = 0; }

    if ((cnt >= KEY_DEBOUNCE_CNT) && (stable != r))
    {
        stable = r;                       /* 状态翻转 */
        if (stable)
        {
            hold    = 0;                  /* 刚按下: 重新计时 */
            long_on = 0;
        }
        else if (!long_on)
        {
            ev |= KEY_EV_CLICK;           /* 短按松开 = 单击 */
        }
    }
    else if (stable)                      /* 按住中 */
    {
        hold++;
        if (!long_on && (hold >= KEY_LONG_TICKS))
        {
            long_on = 1;                  /* 首次进入长按, 立即加速一档 */
            ev |= KEY_EV_ACCEL;
        }
        else if (long_on && (((hold - KEY_LONG_TICKS) % KEY_REPEAT_TICKS) == 0))
        {
            ev |= KEY_EV_ACCEL;           /* 长按连发 */
        }
    }

    return ev;
}
