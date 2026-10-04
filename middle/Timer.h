#ifndef __TIMER_H__
#define __TIMER_H__

#include "ti_msp_dl_config.h"

/*===========================================================================
 * 1ms 系统时基 (SysConfig 里的 TIMER_0 = TIMG6, PERIODIC + ZERO 中断)
 *   中断里 systick_counter++ -> Get_Time() 返回毫秒时间戳, 调度器/循迹/IMU 转向都用它
 *   同时每 1ms 调一次弱钩子 APP_SysTick_Hook() (定时器中断上下文, 里面只能放很短的活)
 *
 *   ★ 本工程重写了 APP_SysTick_Hook() (在 empty.c) 来做 1ms 按键扫描 ——
 *     按键放中断里扫, 主循环被 LCD/串口卡住也不会漏按。
 *
 *   【核实过的时基, 别再怀疑它】
 *     TIMG6 时钟 = BUSCLK / 8 = 80MHz / 8 = 10MHz
 *     LOAD = TIMER_0_INST_LOAD_VALUE = 9999  ->  周期 = 10000 / 10MHz = 1.000 ms  (正确)
 *   依据: SysConfig 里 PWM1 的注释就是 "80MHz/4/1 = 20MHz -> 20kHz", 说明 BUSCLK = 80MHz
 *         (ULPCLK 分频 2 只管 ULP 域, 不影响 BUSCLK); SysConfig 也正是按 10MHz 算出
 *         LOAD=9999 的, 与 TIMG6 分频 8 完全吻合。
 *   注意: ti_msp_dl_config.h 里 Motor_PWM_Front_INST_CLK_FREQ = 10000000 是旧值,
 *         实际和 Motor_PWM_Back 一样是 20MHz —— 别拿它去反推 BUSCLK, 会算错。
 *   验证: 屏上诊断行的 L (5ms 任务实际间隔) 应该稳定在 5。
 *===========================================================================*/

void Timer_Init(void);      /* 清标志 + 开中断 (1ms 周期由 SysConfig 生成, 正确) */
void Timer_Start(void);
void Timer_Stop(void);

/* 系统时间戳(ms): 由 TIMER_0 中断每 1ms 累加, 供调度器使用 */
uint32_t Get_Time(void);

/* 1ms 中断里的用户钩子 (弱函数, 默认空; 本工程在 empty.c 里重写做按键扫描) */
void APP_SysTick_Hook(void);

#endif
