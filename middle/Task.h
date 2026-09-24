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

/* 用户任务函数 (弱函数, 实现在 empty.c)
 * 具体每个周期干什么由 APP_MODE 决定 (见 empty.c 头部 [2] 任务划分表) */
void APP_Task_5ms(void);      /* 5ms    循迹: 读灰度 + 质心 + PD + 写 PWM */
void APP_Task_10ms(void);     /* 10ms   按键: 单击启停 / 长按调速 */
void APP_Task_20ms(void);     /* 20ms   编码器采样; 电机模式再跑闭环速度环 */
void APP_Task_100ms(void);    /* 100ms  LCD 刷新 (循迹/电机两套不同屏) */
void APP_Task_1000ms(void);   /* 1000ms 串口日志 (默认注释掉) */

#endif
