/*===========================================================================
 * 协作式任务调度器 / Cooperative Task Scheduler
 *
 * 使用方法:
 *   1. 在 empty.c 中实现 APP_Task_10ms/20ms/100ms/1000ms (覆盖弱函数)
 *   2. 要加任务就往下面的 tasks[] 数组里加一行
 *   3. 主循环里调用 Scheduler_Run()
 *===========================================================================*/

#include "Task.h"

/* 默认弱任务——用户可在 empty.c 中重写 */
__attribute__((weak)) void APP_Task_5ms(void)    {}
__attribute__((weak)) void APP_Task_10ms(void)   {}
__attribute__((weak)) void APP_Task_20ms(void)   {}
__attribute__((weak)) void APP_Task_100ms(void)  {}
__attribute__((weak)) void APP_Task_1000ms(void) {}

/* 任务注册表: {间隔ms, 上次调用时间戳(初始0), 任务函数}
   高频任务排前面, 减少阻塞延迟 */
Task tasks[] = {
    {5,    0, APP_Task_5ms},      /* 5ms    循迹: 读灰度 + 质心 + PD + 写 PWM */
    {10,   0, APP_Task_10ms},     /* 10ms   按键扫描 (单击启停 / 长按加速) */
    {20,   0, APP_Task_20ms},     /* 20ms   编码器采样 (+ 电机模式的闭环速度环) */
    {100,  0, APP_Task_100ms},    /* 100ms  LCD 刷新 */
    {1000, 0, APP_Task_1000ms},   /* 1000ms 串口日志 (默认注释掉) */
};

void Scheduler_Run(void)
{
    uint32_t now = Get_Time();          /* 当前时间戳(ms) */
    uint32_t i;

    for (i = 0; i < sizeof(tasks) / sizeof(Task); i++)
    {
        /* now - last_call 是无符号减法, 自动处理 32 位溢出 */
        if (now - tasks[i].last_call >= tasks[i].interval)
        {
            tasks[i].task();                  /* 执行任务 */
            tasks[i].last_call = now;         /* 以实际执行时刻为基准, 消除累积漂移 */
        }
    }
}
