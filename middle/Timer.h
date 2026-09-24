#ifndef __TIMER_H__
#define __TIMER_H__

#include "ti_msp_dl_config.h"

/*===========================================================================
 * 1ms 系统时基 (SysConfig 里的 TIMER_0 = TIMG6, 周期 1ms, ZERO 中断)
 *   中断里 systick_counter++ -> Get_Time() 返回毫秒时间戳, 调度器/循迹都用它
 *   同时每 1ms 调一次弱钩子 APP_SysTick_Hook() (空实现, 需要时在别处重写)
 *===========================================================================*/

void Timer_Init(void);
void Timer_Start(void);
void Timer_Stop(void);

/* 系统时间戳(ms): 由 TIMER_0 中断每 1ms 累加, 供调度器使用 */
uint32_t Get_Time(void);

#endif
