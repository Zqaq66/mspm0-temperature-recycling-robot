#ifndef __KEY_H
#define __KEY_H

#include "ti_msp_dl_config.h"
#include "stdint.h"

/*===========================================================================
 * 板载按键 (只用 LaunchPad 的 S2 = PB21, 按下接 GND, MCU 内部上拉)
 *
 *   单击 (按住 < 0.6s 松开)  ->  开始 / 停止 (切换)
 *   长按 (按住 >= 0.6s)      ->  加速一档, 之后每 0.1s 再加一档
 *                               加到上限自动回绕到下限 (单键也能往回调)
 *
 *   调用: 10ms 一次 Key_Scan(), 返回事件位; 0 = 无事件
 *         单击不产生 ACCEL 事件, 长按不产生 CLICK 事件 (松开时不会误触发)
 *===========================================================================*/

#define KEY_EV_CLICK    0x01        /* 单击: 开始 / 停止 */
#define KEY_EV_ACCEL    0x02        /* 加速一档 */

void Key_Init(void);
int  Key_Scan(void);                /* 10ms 一次; 返回 KEY_EV_x 的或, 0 = 无事件 */
int  Key_Pressed(void);             /* 当前是否按住 (调试用) */

#endif
