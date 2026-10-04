/*===========================================================================
 * 板载按键驱动 (单键: S2 = PB21, 按下 = 低电平, MCU 内部上拉)
 *
 *   ★ 由 1ms 中断钩子 APP_SysTick_Hook() 调用 (见 Key.h 顶部说明为什么不在 10ms 任务里扫)
 *
 *   每 1 拍:
 *     - 电平要连续 KEY_DEBOUNCE_MS 拍一致才算"稳定"(按键抖动只有几百 us~几 ms)
 *     - 按下到松开不满 KEY_LONG_MS -> KEY_EV_CLICK   (开始/停止)
 *     - 按住满 KEY_LONG_MS         -> KEY_EV_ACCEL, 并进入长按;
 *                                     之后每 KEY_REPEAT_MS 再发一次 KEY_EV_ACCEL
 *     - 长按松开时不再产生单击事件
 *===========================================================================*/

#include "Key.h"

#define KEY_DEBOUNCE_CNT    (KEY_DEBOUNCE_MS / KEY_SCAN_MS)     /* 去抖拍数 */
#define KEY_LONG_TICKS      (KEY_LONG_MS     / KEY_SCAN_MS)     /* 长按拍数 */
#define KEY_REPEAT_TICKS    (KEY_REPEAT_MS   / KEY_SCAN_MS)     /* 连发拍数 */

static uint8_t  stable  = 0;        /* 去抖后状态: 1 = 按住 */
static uint8_t  last    = 0;        /* 上一拍原始电平 */
static uint16_t cnt     = 0;        /* 去抖计数 */
static uint32_t hold    = 0;        /* 按住拍数 */
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

    /* ---- 去抖: 电平连续 KEY_DEBOUNCE_CNT 拍不变才算稳定 ---- */
    if (r != last)
    {
        last = r;                       /* 电平变了: 重新开始计数 */
        cnt  = 0;
    }
    else if (cnt < KEY_DEBOUNCE_CNT)
    {
        cnt++;
    }

    if ((cnt >= KEY_DEBOUNCE_CNT) && (stable != last))
    {
        stable = last;                  /* 状态翻转 */
        if (stable)
        {
            hold    = 0;                /* 刚按下: 重新计时 */
            long_on = 0;
        }
        else if (!long_on)
        {
            ev |= KEY_EV_CLICK;         /* 短按松开 = 单击 */
        }
    }
    else if (stable)                    /* 按住中 */
    {
        hold++;
        if (!long_on && (hold >= KEY_LONG_TICKS))
        {
            long_on = 1;                /* 首次进入长按, 立即加速一档 */
            ev |= KEY_EV_ACCEL;
        }
        else if (long_on && (((hold - KEY_LONG_TICKS) % KEY_REPEAT_TICKS) == 0))
        {
            ev |= KEY_EV_ACCEL;         /* 长按连发 */
        }
    }

    return ev;
}
