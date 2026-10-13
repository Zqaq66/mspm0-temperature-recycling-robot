#ifndef __TASK_H__
#define __TASK_H__

#include "ti_msp_dl_config.h"
#include "Timer.h"

/* 任务结构体 */
typedef struct {
    uint32_t interval;    /* 任务执行间隔(ms) */
    uint32_t last_call;   /* 上次执行时间戳 */
    void (*task)(void);   /* 任务函数指针 */
} Task;

/* 调度器主函数: 放在 while(1) 里循环调用 */
void Scheduler_Run(void);

/* 用户任务函数 (弱函数, 实现在 empty.c) */
void APP_Task_5ms(void);      /* 循迹/任务状态机推进 */
void APP_Task_10ms(void);     /* 按键事件: 单击启停 / 长按调速 */
void APP_Task_20ms(void);     /* IMU 姿态帧解析 */
void APP_Task_100ms(void);    /* LCD 文字影子 */
void APP_Task_1000ms(void);   /* 授时轮询 */

#endif
