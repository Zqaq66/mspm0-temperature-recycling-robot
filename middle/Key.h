#ifndef __KEY_H__
#define __KEY_H__

#include "ti_msp_dl_config.h"
#include "stdint.h"

/*===========================================================================
 * 板载按键 (只用 LaunchPad 的 S2 = PB21, 按下接 GND, MCU 内部上拉)
 *
 *   单击 (按住 < KEY_LONG_MS 松开)  ->  开始 / 停止 (切换)
 *   长按 (按住 >= KEY_LONG_MS)      ->  加速一档, 之后每 KEY_REPEAT_MS 再加一档
 *                                       加到上限自动回绕到下限 (单键也能往回调)
 *
 * ★ 调用方式 (重要, 别改回去):
 *     Key_Scan() 由 **1ms 中断钩子 APP_SysTick_Hook()** 每 1ms 调一次
 *     (empty.c 里重写了这个弱函数), 事件缓存起来, 10ms 任务再去处理。
 *
 *     为什么不在 10ms 任务里扫:
 *       主循环是"协作式"的, 任何一个任务卡住 (LCD 一次刷新要扫几万个 SPI 字节,
 *       实测 ~30ms; 串口 printf 一行 9600 波特 ~70ms) 都会把 10ms 任务拖到几十 ms 一次。
 *       原来的写法要求"电平连续 3 拍不变"才算按下, 按 20~60ms 才扫一次摊下来就得按住
 *       60~180ms —— 快速单击根本扫不到 (实测现象: 轻点没反应, 按住一会儿再松开才动)。
 *       放到 1ms 中断里扫, 主循环再怎么卡也不会漏按键, 去抖也只要几 ms。
 *
 *   本文件用"拍数"计数, 单位 = KEY_SCAN_MS 毫秒 (见下面的换算宏)。
 *   如果哪天改成 10ms 调一次, 只要把 KEY_SCAN_MS 改成 10 即可, 参数不用动。
 *===========================================================================*/

#define KEY_SCAN_MS     1       /* Key_Scan 的调用周期(ms): 1 = 由 1ms 中断调用 */

#define KEY_DEBOUNCE_MS 14       /* 去抖: 电平连续稳定这么久才算数 */
#define KEY_LONG_MS     600     /* 按住这么久 -> 判为长按 */
#define KEY_REPEAT_MS   100     /* 长按后每隔这么久加速一档 */

#define KEY_EV_CLICK    0x01    /* 单击: 开始 / 停止 */
#define KEY_EV_ACCEL    0x02    /* 加速一档 */

void Key_Init(void);
int  Key_Scan(void);                /* 建议 1ms 一次; 返回 KEY_EV_x 的或, 0 = 无事件 */
int  Key_Pressed(void);             /* 当前是否按住 (调试用) */

#endif
