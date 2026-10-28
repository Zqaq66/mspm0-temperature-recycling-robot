/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*===========================================================================
 * MSPM0G3507 四轮麦克纳姆轮小车 —— 主程序 (main + 协作式调度器)
 *
 * 功能: 沿黑线循迹, 按 Mission.c 的 route[] 表走路线; 到房间停下做
 *       语音提示 + 测温 + 读电量 + 屏显结果 + LORA 上报。
 * 控件: 板载 S2(PB21) 单击 = 开始/停止, 长按 = 加减速档。
 *
 * ---- 任务划分 (middle/Task.c 调度) ----
 *   1ms(中断) 按键扫描 + 时基 Get_Time()
 *   5ms       循迹/任务状态机推进 (ImuTurn 忙时让出电机)
 *   10ms      处理按键事件 (单击启停 / 长按调速)
 *   20ms      IMU 姿态帧解析
 *   100ms     LCD 文字影子 (推屏在主循环 LCD_RowFlush 里分摊)
 *   1000ms    授时轮询
 *
 * ---- 模块 ----
 *   middle/XunJi.c    循迹: 平方加权质心 + 三区 PD + 路口/房间/岔路判定
 *   middle/Mission.c  任务状态机: route[] 表驱动 (动作 / 退出条件 / 步间停车)
 *   middle/ImuTurn.c  IMU 原地定角转 (四段 + 自标定 + 脉冲补)
 *   middle/TimeSync.c I2C1 授时查询端 (ESP32-S2 0x42)
 *   middle/UART.c     printf 出口 = LORA 口 UART3; UART0 = 语音
 *   app/Motor.c       TB6612 x2 四轮 PWM/方向/短路制动
 *   app/MLX90614.c    红外测温 (I2C0 0x5A)   app/TW.c 语音播报 (UART0)
 *   hardware/lcd.*    ST7735S 128x160 + 批量刷屏 + 文字影子层
 *   grayscale/       芯流 AuroraT8 八路灰度 (I2C0 0x40)
 *
 * ---- 接线速查 (详见各驱动头文件) ----
 *   电机 PWM A=PA12 B=PA13 C=PA17 D=PA7   方向 A=PB19/20 B=PB4/5 C=PB6/7 D=PB12/13
 *   STBY=PB0                              灰度 SDA=PA0 SCL=PA1
 *   LCD SCLK=PB9 MOSI=PB8 RES=PB10 DC=PB11 CS=PB14 BLK=PB26
 *   IMU TX=PA9 RX=PA8 (115200)            LORA TX=PB2 RX=PB3 (9600, printf 出口)
 *   授时 SDA=PA30 SCL=PA29 (400k, 0x42)    语音 TX=PA10 RX=PA11
 *   电池电压 ADC0 = PA27 (板上 1/4 分压)
 *
 * ---- 主要开关 (都能用 Keil Misc Controls 加 -Dxxx=1 覆盖) ----
 *   MISSION_ENABLE       1 = 按路线表跑 / 0 = 纯循迹
 *   WORK_FEATURES        1 = 房间环节 (测温/电量/LORA/语音)
 *   LCD_RESULTS          1 = 屏第 6~10 行显示房间结果区 (0 = 标定/诊断行, 调阈值时用)
 *   LCD_DEBUG            1 = 允许显示标定/诊断行 (要 LCD_RESULTS=0 才看得到)
 *   MISSION_DEBUG_STEPS  (Mission.c) N>0 只跑前 N 步, 台架分段验路线
 *   调参入口都在各模块 .c/.h 顶部
 *===========================================================================*/
#include <stdio.h>
#include "ti_msp_dl_config.h"
#include "Task.h"
#include "Timer.h"
#include "Time.h"
#include "lcd.h"
#include "lcd_init.h"
#include "Motor.h"
#include "XunJi.h"          /* 循迹层: Track_Init/Track_Run */
#include "Key.h"            /* 板载按键: 速度在线可调 */
#include "imu_uart_driver.h"/* IMU: YbIMU UART 驱动 (UART1 PA8=TX/PA9=RX) */
#include "ImuTurn.h"        /* IMU 转向/保向: 转到指定角度 / 保持指定角度 */
#include "Mission.h"        /* 任务状态机: 循迹->路口->转向->工作->恢复循迹 (路线表在 Mission.c) */
#include "MLX90614.h"       /* 红外测温 MLX90614 (I2C0 0x5A, 与灰度共用总线) */
#include "TW.h"             /* 语音播报: Voice_0/Voice_1 -> UART0 */
#include "UART.h"           /* UART_Init / UART_send_char -> UART0 (语音) */
#include "TimeSync.h"        /* I2C 授时查询端: ESP32-S2(0x42) -> 本地走时 (接线/调参见 TimeSync.h) */

/* ---- 任务状态机总开关 ----
 * 1 = 单击 S2 后按 Mission.c 的路线表跑 (循迹+路口检测+定角转弯+步间停车 + 工作钩子)
 * 0 = 退回"纯循迹" (原来的行为, 台架调循迹/转弯参数时用)
 * ★ 台架一段一段验: 不用改这里, 改 Mission.c 顶部的 MISSION_DEBUG_STEPS = N
 *    (只跑路线表前 N 步就收工停车; 1=只验"循迹到第一个路口", 2=再验"盲走穿口", ...)
 * ★ 观察模式: 没开始时也读灰度刷屏, 只是不驱动电机 (XunJi 的 Track_SetDrive(0)),
 *    所以标定探头/路口/房间时不用按键, 手推车或拿黑线挡探头屏上就实时变 */
#define MISSION_ENABLE   1

/* ---- 题目功能: 房间测温环节 (语音 / 测温 / 电量 / LORA 上报) ----
 * 1 = 开 (默认): Mission_OnWork() 做完整流程; 0 = 关 (只打一行串口日志) */
#ifndef WORK_FEATURES
#define WORK_FEATURES   1
#endif

/* ---- 屏第 6~10 行显示哪一套 (两套都要这片屏, 只能二选一) ----
 * 1 = 房间结果区: 电量 + 4 个房间的温度/时间 (题目要求)
 * 0 = 标定/诊断行: 8 路点位 + 判定 / X H N T F / V RX E / 8 路原始值 (调阈值/路口用) */
#ifndef LCD_RESULTS
#define LCD_RESULTS     0
#endif
#define LCD_SHOW_RESULT (WORK_FEATURES && LCD_RESULTS)

/* ---- 语音播报时序 (2026-10-14 加 ASRPRO2.0 语音模块) ----
 * 到房间: 播报"请N号房间人员配合测温" -> 等 WORK_PROMPT_MS -> 测温 -> 播报"N号房间测温完成"。
 * ★ 必须 >= 那句 TTS 的时长 (天问"播报"块阻塞, 播放期间收不到新命令), 再加队员把手掌放上去的反应。
 *   标定: 拿秒表量"听到播报 -> 手掌放好"最多几秒, 再留一点余量。
 * ★ 这是往返 60s 预算的大头, 别随便加大; 播报词稿也要短。 */
#ifndef WORK_PROMPT_MS
#define WORK_PROMPT_MS  3500
#endif

/* 电池电压: ADC0/PA27, 板上 1/4 分压 -> Vbat = Vpin * BAT_DIV_RATIO
 * 电量% = (Vbat - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100 (题目给的参考公式) */
#define BAT_DIV_RATIO   4.0f
#define BAT_EMPTY_V     10.5f
#define BAT_FULL_V      12.6f

/* 测温: TEMP_SAMPLES 次取中位, 两次间隔 TEMP_GAP_MS (MLX90614 内部约 10Hz 刷新) */
/* 测温采样: TEMP_SAMPLES 次取中位 (越多越稳但越占时间);
 * TEMP_BUDGET_MS = 整段测温的时间预算, 超了就用手上的数据, 不让 5ms 任务被占太久 */
#define TEMP_SAMPLES    3
#define TEMP_GAP_MS     20
#define TEMP_BUDGET_MS  120

/* ---- 运行开关: S2(PB21) 单击切换 ----
 * 0 = 停 (上电默认): 电机输出全 0, 循迹不跑, 屏上显示 STOP
 * 1 = 跑: 循迹 / 任务状态机 */
uint8_t app_run = 0;

/* ---- 屏幕调试行开关 ----
 * 0 = 工作 (默认): 只显示 5 行工作信息, 屏干净
 *                   第1行 状态+基速 / 第2行 左右PWM / 第3行 质心误差 / 第4行 IMU航向 / 第5行 时间
 * 1 = 台架调试  : 额外多出 3 行 (8路点位 / X H L K / V RX ERR)
 *                   标定传感器、查路口检测、查实时性时打开; 平时关掉 */
#ifndef LCD_DEBUG          /* 也可以用 Keil 的 Misc Controls 加 -DLCD_DEBUG=1 覆盖 */
#define LCD_DEBUG       1
#endif

/*---------------------------------------------------------------------------
 * 1ms 中断钩子: 按键扫描放这里
 *   主循环是协作式的, LCD 刷一屏要几十 ms、串口 printf 一行要几十 ms,
 *   这些都会把 10ms 任务拖到几十 ms 一次 —— 按键放在任务里扫就会漏掉快速单击。
 *   放 1ms 定时器中断里扫, 主循环再怎么卡也不会漏按。
 *   事件缓存到 key_ev_pending, 由 APP_Task_10ms() 取走处理。
 *-------------------------------------------------------------------------*/
volatile int      key_ev_pending = 0;   /* 按键事件缓存 (中断里或, 10ms 任务里清零) */
volatile uint16_t key_scan_cnt   = 0;   /* 按键扫描次数累计 (诊断用: 每100ms应≈100) */
volatile uint16_t app_loop_ms    = 0;   /* 5ms 任务实际间隔 (诊断用: 正常=5, 变大=主循环被卡) */

void APP_SysTick_Hook(void)
{
    key_ev_pending |= Key_Scan();       /* 1ms 一次, 返回 0 或事件位 */
    key_scan_cnt++;
}



/*---------------------------------------------------------------------------
 * LCD 行输出 (统一入口, 防止印到屏外)
 *   屏: 128 x 160 (USE_HORIZONTAL=1), 16 号字 = 8 px 宽 -> 一行顶多 16 个字符。
 *   规则: 不足 16 字符补空格(盖掉上一帧的残字), 超过 16 字符截断(不然会印出屏外看不到)。
 *   所有 LCD 文字行都走这个函数, 别直接 LCD_ShowString。
 *   ★ 这里只把文字写进"影子"(LCD_RowSet, 纯 RAM, 几 us 返回), 一个字节 SPI 都不发;
 *     真正推屏在主循环的 LCD_RowFlush() 里 1ms 最多一行地做掉 —— 所以本函数不会卡主循环。
 *     内容/颜色没变的话 LCD_RowSet 里就判掉了, 连脏行都不标 (静止画面 0 流量)。
 *     (补空格/截断也搬进 LCD_RowSet 了; 参数 y 仍是像素, 内部 /16 换成行号)
 *-------------------------------------------------------------------------*/
#define LCD_COLS   16
static void lcd_row(int y, char *buf, unsigned fc)
{
    LCD_RowSet((u8)(y / 16), buf, (u16)fc);
}

/*---------------------------------------------------------------------------
 * 行布局 —— y 只能是 16 的倍数 (0,16,32,...,144)
 *   屏 128x160, 16 号字 = 8px 宽 x 16px 高 -> 一行 16 字符, 竖着正好 10 行。
 *   ★ 别再引入非 16 倍数的 y: 以前用过 y=136, 字高 16 -> 占到 151, 和 y=144 那行
 *     叠了 8 个像素, 屏上就是两行糊在一起 (2026-10-06 修掉)。
 *   工作模式 (LCD_DEBUG=0) 只用 R_STEP..R_TIME 这 5 行; 调试模式再加 R_BITS..R_IO。
 *-------------------------------------------------------------------------*/
#define LCD_Y(n)     ((n) * 16)          /* 行号 -> y 像素, n = 0..9 */
#define R_STEP       0                   /* 步骤/状态 + 基速 (或 STOP / LOST) */
#define R_PWM        1                   /* 左右轮 PWM */
#define R_TRACK      2                   /* 质心 + 误差 */
#define R_IMU        3                   /* IMU 航向 (转向时 + 剩余角) */
#define R_TIME       4                   /* 授时时间 (未同步时整行空) */
#define R_BITS       5                   /* [仅调试] 8 路点位 */
#define R_TEMP       6                   /* 倒数第 4 行: 测温 (关 WORK_FEATURES 时 = 路口/转弯诊断) */
#define R_IO         7                   /* [仅调试] 降速后速度 + IMU 字节 + 灰度I2C错误 */
#define R_RAW0       8                   /* [仅调试] 8 路原始值 ch0~3 (标定阈值时看这个) */
#define R_RAW1       9                   /* [仅调试] 8 路原始值 ch4~7 */


/*---------------------------------------------------------------------------
 * IMU 行 (工作模式占 R_IMU = 第4行)
 *   平时:   "YAW   +12"      当前航向(度), 已按 IMU_TURN_YAW_SIGN 校正并归一化
 *   转向中: "Y +12 E +78"    当前航向 + 距目标的剩余角(度)
 *   颜色:   绿 = 有数据, 灰 = IMU 没数据; 转向中黄色
 *   ★ 调转向参数时要看相位(D远段/C近段/B等停稳/P脉冲)和角速度 -> 把 LCD_DEBUG 改成 1,
 *     转向中就会显示成 "Y +12 E +78 D 113" (正好 16 字符)
 *-------------------------------------------------------------------------*/
static void LCD_imu(int row)
{
    char  buf[24];
    float euler[3];
    int   yaw = 0;
    int   ok  = (IMU_GetEuler(euler) == 0);

    if (ok)
    {
        /* 按 IMU_TURN_YAW_SIGN 校正并归一化到 (-180,180]: 屏上的 yaw 和命令同向 (+ = 左转),
         * 而且从任意角度起都不会显示成 -350 这种值。
         * 注意 IMU 原始报值的符号是相反的 (罗盘式顺时针为正), 别拿屏上的数去和上位机对 */
        float y = euler[2] * (float)IMU_TURN_YAW_SIGN;
        while (y >  180.0f) y -= 360.0f;
        while (y < -180.0f) y += 360.0f;
        yaw = (int)((y >= 0.0f) ? (y + 0.5f) : (y - 0.5f));
    }

    if (ImuTurn_IsBusy())                        /* 转向/保向中: 当前角 + 剩余角 */
    {
#if LCD_DEBUG
        int w = (int)ImuTurn_GetRate();          /* 调试时补上相位和角速度 (调转向参数用) */
        if (w >  99) w =  99;
        if (w < -99) w = -99;
        sprintf(buf, "Y%+4d E%+4d %c%3d", yaw, (int)ImuTurn_GetDiff(), ImuTurn_GetPhase(), w);
#else
        sprintf(buf, "Y%+4d E%+4d", yaw, (int)ImuTurn_GetDiff());
#endif
        lcd_row(LCD_Y(row), buf, YELLOW);
    }
    else                                         /* 平时: 只显示当前航向 */
    {
        sprintf(buf, "YAW   %+5d", yaw);
        lcd_row(LCD_Y(row), buf, ok ? GREEN : GRAY);   /* 灰 = IMU 没数据 */
    }
}


/*---------------------------------------------------------------------------
 * 授时时间行 (工作模式占 R_TIME = 第5行)
 *   st ==  1: 绿字 "10-06 14:05:23" (14 字符) —— 秒是本板自己走出来的, 不用每秒读总线
 *   st ==  0: 整行空 (模块还没联上网; 上电 3~5s 内属正常, 不报警)
 *   st == -1: 红字 "TIME I2C ERR" (总线故障: 查线/上拉/供电)
 *   ts_last_status: 给 1000ms 任务判断"状态变了没", 只在变化时打一行串口
 *-------------------------------------------------------------------------*/
static int ts_last_status = 99;     /* 授时状态上报用: 1000ms 任务里看它变没变 */

static void LCD_clock(int row)
{
    char buf[24];
    int  st = TimeSync_Status();

    if (st == 1)
    {
        sprintf(buf, "%s", TimeSync_GetClock());
        lcd_row(LCD_Y(row), buf, GREEN);
    }
    else if (st == 0)
    {
        buf[0] = 0;                                  /* 按要求: 未同步不显示 */
        lcd_row(LCD_Y(row), buf, GRAY);
    }
    else
    {
        sprintf(buf, "TIME I2C ERR");                /* 总线故障: 查线/上拉/供电 */
        lcd_row(LCD_Y(row), buf, RED);
    }
}


/*===========================================================================
 * 题目功能: 房间测温环节 (MS_WORK 步里由 Mission_OnWork() 调)
 *   流程: 语音提示 -> 测温 -> 读电量 -> 记结果(屏+时间戳) -> LORA 上报 -> 语音完成
 *   结果存 g_room[]: 中途不清, 下一次任务开始(Results_Clear)才清 (题目要求)
 *   ★ 本工程用 microlib, printf/sprintf 不支持 %f -> 浮点一律走 fmt_temp/fmt_volt 整数化
 *===========================================================================*/
#if WORK_FEATURES
typedef struct
{
    uint8_t valid;          /* 1 = 这个房间已经测过 */
    uint8_t time_ok;        /* 1 = 测量时刻有效 (I2C 授时已同步) */
    float   temp_c;         /* 体温 ℃ */
    float   bat_v;          /* 测量时的电池电压 V */
    uint8_t bat_pct;        /* 测量时的电量 0~100 */
    uint8_t hh, mm, ss;     /* 测量时刻 时:分:秒 */
} RoomResult;

#if LCD_SHOW_RESULT
static RoomResult g_room[5];          /* 只用 1..4 (下标 = 房间号) */
#endif
static float      g_bat_v   = 0.0f;   /* 最近一次读到的电池电压 (结果区第 1 行显示) */
static uint8_t    g_bat_pct = 0;
static uint8_t    g_task_done = 0;    /* 1 = 整表跑完 (屏第 1 行提示"任务完成") */

/* 房间工作环节的非阻塞状态机 (Mission_OnWork / Mission_OnWorkTick 两个钩子共用) */
static uint8_t    ws_phase = 0;       /* 0 = 已播报, 正在等队员放手掌; 1 = 本房间已测完 */

/* 屏第 6 行 (R_TEMP) 显示的温度: 上电 System_Init 里先测一次, 之后每次房间测温再刷新 */
static float      g_temp_c  = 0.0f;   /* 最近一次测得的物体温度 ℃ */
static uint8_t    g_temp_ok = 0;      /* 1 = 有效; 0 = 屏上显示 "--.-C" */
static unsigned char g_i2c_list[8];   /* 上电扫描 I2C0 扫到的地址 (给屏/串口看) */
static int           g_i2c_n = 0;     /* 扫到几个 */

/* 把扫到的地址排成 "5A" / "5A 00" / "none" 这种短串 (屏上最多放 3 个) */
static void fmt_i2c(char *out)
{
    int i, k = 0;

    if (g_i2c_n <= 0) { sprintf(out, "none"); return; }
    for (i = 0; (i < g_i2c_n) && (i < 3); i++)
        k += sprintf(out + k, "%s%02X", (i ? " " : ""), (unsigned)g_i2c_list[i]);
}

/* 清空结果 —— 在下一次任务开始时调 (满足"数据中途不得清除") */
static void Results_Clear(void)
{
#if LCD_SHOW_RESULT
    int i;
    for (i = 0; i < 5; i++) { g_room[i].valid = 0; g_room[i].time_ok = 0; }
#endif
    g_bat_v     = 0.0f;
    g_bat_pct   = 0;
    g_task_done = 0;
    ws_phase    = 0;
}

/* 读一次电池电压: ADC0/PA27 (SysConfig 已配: 12bit / 重复采样 / 软件触发)
 *   Vpin = raw/4095*3.3V;  板上 1/4 分压 -> Vbat = Vpin * BAT_DIV_RATIO */
static float Bat_ReadVoltage(void)
{
    uint16_t raw;
    uint32_t t0;

    DL_ADC12_startConversion(ADC_VOLTAGE_INST);
    t0 = Get_Time();
    while (!(DL_ADC12_getRawInterruptStatus(ADC_VOLTAGE_INST,
                                            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)))
    {
        if ((Get_Time() - t0) > 20u) break;      /* 兜底: 硬件没接也别把 5ms 任务卡死 */
    }
    DL_ADC12_clearInterruptStatus(ADC_VOLTAGE_INST,
                                  DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    raw = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_ADC_CH0);
    return ((float)raw * 3.3f / 4095.0f) * BAT_DIV_RATIO;
}

/* 连续读 4 次求平均 (去抖) */
static float Bat_ReadAvg(void)
{
    float sum = 0.0f;
    int   k;
    for (k = 0; k < 4; k++) sum += Bat_ReadVoltage();
    return sum * 0.25f;
}

static uint8_t Bat_ToPercent(float v)
{
    float pct = (v - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(pct + 0.5f);
}

/* 读体温: MLX90614 物体温度 (0x07), 采 TEMP_SAMPLES 次取中位数。
 * 用 MLX90614_ReadTemp() —— 它会沿用它上电自动探测到的地址(0x5A/0x00)。
 *
 * ★ 关于开销 (软件 I2C 是手动打时序, 会占 5ms 任务的 CPU):
 *   - 单次读约 1~2ms (几十个位, 每位几个 us 的忙等);
 *   - 这里有总时间预算 TEMP_BUDGET_MS: 总线异常时也不会一直占着任务;
 *   - ★★ 所以这个函数【只能在车停着的时候调】: 上电初始化, 或 MS_WORK 步(已停车)。
 *       千万别在跑动中调, 那会把 5ms 循迹节拍顶掉。
 *   (1ms 定时器中断不受影响: 按键扫描/走时照常。) */
static float Temp_ReadBody(uint8_t *ok)
{
    float    s[TEMP_SAMPLES];
    int      n = 0, i, j;
    float    out;
    uint32_t t0 = Get_Time();

    for (i = 0; i < TEMP_SAMPLES; i++)
    {
        float t = MLX90614_ReadTemp();
        if ((t > -10.0f) && (t < 80.0f)) s[n++] = t;     /* 失败(-1000)和超范围的都丢掉 */
        if ((Get_Time() - t0) >= (uint32_t)TEMP_BUDGET_MS) break;   /* 预算到了就不再补采 */
        delay_ms(TEMP_GAP_MS);
    }
    if (n == 0)                                  /* 一次都没读到: 屏上保持 "--.-C" */
    {
        *ok = 0;
        g_temp_ok = 0;
        return 0.0f;
    }

    for (i = 1; i < n; i++)                      /* 插入排序 (n <= TEMP_SAMPLES) */
    {
        float key = s[i];
        for (j = i - 1; (j >= 0) && (s[j] > key); j--) s[j + 1] = s[j];
        s[j + 1] = key;
    }
    out = (n & 1) ? s[n / 2] : (s[n / 2 - 1] + s[n / 2]) * 0.5f;

    *ok       = 1;
    g_temp_c  = out;                             /* 顺便刷新屏第 6 行 (R_TEMP) */
    g_temp_ok = 1;
    return out;
}

/* 浮点 -> 整数格式化 (microlib 无 %f): 温度 "36.5" / 电压 "11.85V" */
static void fmt_temp(char *out, float t)
{
    int t10 = (int)(t * 10.0f + ((t >= 0.0f) ? 0.5f : -0.5f));
    if (t10 < 0) sprintf(out, "-%d.%d", (-t10) / 10, (-t10) % 10);
    else         sprintf(out, "%d.%d", t10 / 10, t10 % 10);
}
static void fmt_volt(char *out, float v)
{
    int mv = (int)(v * 100.0f + 0.5f);
    if (mv < 0) mv = 0;
    sprintf(out, "%d.%02dV", mv / 100, mv % 100);
}

#if LCD_SHOW_RESULT
/* 房间结果区: 第 6 行 = 电量; 第 7~10 行 = 4 个房间 (题目要求: 中途不清) */
static void LCD_results(void)
{
    char buf[24];
    char tmp[10];
    int  r;

    if (g_bat_v > 1.0f) { fmt_volt(tmp, g_bat_v); sprintf(buf, "BAT %s %d%%", tmp, (int)g_bat_pct); }
    else                sprintf(buf, "BAT  --.--V ---");
    lcd_row(LCD_Y(5), buf, (g_bat_pct && (g_bat_pct < 20)) ? RED : GREEN);

    for (r = 1; r <= 4; r++)
    {
        if (g_room[r].valid)
        {
            fmt_temp(tmp, g_room[r].temp_c);
            if (g_room[r].time_ok)
                sprintf(buf, "%d %sC %02d:%02d:%02d", r, tmp,
                        (int)g_room[r].hh, (int)g_room[r].mm, (int)g_room[r].ss);
            else
                sprintf(buf, "%d %sC --:--:--", r, tmp);
            lcd_row(LCD_Y(5 + r), buf, YELLOW);
        }
        else
        {
            sprintf(buf, "%d --.-C --:--:--", r);
            lcd_row(LCD_Y(5 + r), buf, GRAY);
        }
    }
}
#endif /* LCD_SHOW_RESULT */
#endif /* WORK_FEATURES */

/*---------------------------------------------------------------------------
 * 循迹 LCD —— 工作模式 5 行, 不重叠/不超屏
 *   R_STEP  第1行 状态 + 基速: STOP press S2 / LOST nnn(丢线) / Mxx 步名 子状态 Bxxx
 *   R_PWM   第2行 左右轮 PWM
 *   R_TRACK 第3行 P 质心(0~700, 中心350) E 误差(负 = 线偏左)
 *   R_IMU   第4行 IMU 航向 (转向中 = 当前角 + 剩余角)
 *   R_TIME  第5行 授时时间 (未同步时不显示)
 *   LCD_DEBUG=1 再补 3 行: 8路点位 / X H L K / V RX ERR (标定和查问题时用)
 *-------------------------------------------------------------------------*/
static void LCD_track(void)
{
    char buf[24];

    /* 第1行: 状态 —— 停 / 丢线 / 状态机步号+子状态 / 纯循迹, 后两种都带基速 B */
#if MISSION_ENABLE
    /* 任务停下来报警: fail=1 找线失败(NOLINE) / fail=2 转弯序列超时或 IMU 起不来 (参见 Mission.c) */
    if (!app_run && Mission_GetFail() == 2)     sprintf(buf, "TURN  fail  ");
    else if (!app_run && Mission_GetFail())     sprintf(buf, "NOLINE  stop  ");
#if WORK_FEATURES
    else if (g_task_done)                       sprintf(buf, "TASK  DONE  ");
#endif
    else
#endif
    if (!app_run)                sprintf(buf, "STOP  press S2  ");
    else if (track_lost)         sprintf(buf, "LOST %-5d     ", track_lost);
#if MISSION_ENABLE
    else if (Mission_IsActive()) sprintf(buf, "%s B%3d", Mission_GetStatus(), Track_GetBaseSpeed());
#endif
    else                         sprintf(buf, "RUN    B%3d  ", Track_GetBaseSpeed());
    lcd_row(LCD_Y(R_STEP), buf, app_run ? CYAN : RED);

    /* 第2行: 左右轮 PWM */
    sprintf(buf, "L%+4d R%+4d", track_left_pwm, track_right_pwm);
    lcd_row(LCD_Y(R_PWM), buf, WHITE);

    /* 第3行: 质心 + 误差 */
    sprintf(buf, "P%3d E%+4d", track_pos, track_error);
    lcd_row(LCD_Y(R_TRACK), buf, YELLOW);

    LCD_imu(R_IMU);                          /* 第4行: 航向 (转向中 + 剩余角) */
    LCD_clock(R_TIME);                       /* 第5行: 授时时间 (未同步不显示) */

#if LCD_SHOW_RESULT
    LCD_results();                           /* 第 6~10 行: 电量 + 4 个房间结果 (题目要求) */
#else
    /* ★ 倒数第 4 行 (R_TEMP): 常显温度 —— 不跟着 LCD_DEBUG 走, 关掉调试时也看得到。
     *   上电测一次, 之后每个房间测完刷新 (数据来自软件 I2C 读的 MLX90614)。 */
#if WORK_FEATURES
    {
        char ts[10];
        if (g_temp_ok) { fmt_temp(ts, g_temp_c); sprintf(buf, "TEMP %sC", ts); }
        else
        {
            /* 读不到就改显示上电扫到的地址: I2C:none = 这条线上没器件; I2C:5A = 模块在但没读成 */
            char ls[12];
            fmt_i2c(ls);
            sprintf(buf, "I2C:%s", ls);
        }
    }
    lcd_row(LCD_Y(R_TEMP), buf, g_temp_ok ? GREEN : GRAY);
#endif /* WORK_FEATURES */

#if LCD_DEBUG
    {   /* 以下调试行只在 LCD_DEBUG=1 时显示: 标定传感器 / 查路口检测 / 查实时性 */
        static uint16_t loop_max = 0;
        static uint32_t last_rx  = 0;
        uint32_t rx = IMU_GetRxByteCount();
        unsigned rx_delta = (unsigned)(rx - last_rx);
        int      i;

        last_rx = rx;

        for (i = 0; i < 8; i++) buf[i] = ((track_bits >> i) & 0x01) ? '.' : '#';
        /* 后 4 格 = 事件层状态 (对路线表用):
         *   Pnt = 现在是大面积黑 (丁字口横杠 / 停车线 / 房间黑块)
         *   C+  = 现在线从侧面离开, 且往右拐    C- = 往左拐 (普通弯)
         *   SUP = 判定被关着 (转向/找线中)      --- = 什么都没发生 */
        if (Track_GetSuppressed()) sprintf(buf + 8, " SUP");
        else if (track_room)       sprintf(buf + 8, " Pnt");
        else if (track_branch)     sprintf(buf + 8, " Br%c", (track_cross_side > 0) ? '+' : '-');
        else if (track_corner)     sprintf(buf + 8, " C%c", (track_corner_side > 0) ? '+' : '-');
        else                       sprintf(buf + 8, " ---");
        lcd_row(LCD_Y(R_BITS), buf, GREEN);

        if (app_loop_ms > loop_max) loop_max = app_loop_ms;    /* 5ms 任务最坏间隔 (正常 5) */
        loop_max = 0;

        if (rx_delta > 999) rx_delta = 999;
        sprintf(buf, "V%3d RX%3u E%2d", track_speed, rx_delta, track_i2c_err);
        lcd_row(LCD_Y(R_IO), buf, track_i2c_err ? RED : GRAY);

        /* 8 路原始值 (0~255, 越大越黑) —— 标定阈值就看这两行:
         * 白底上哪几路本来就高 (常见是最外侧) 一眼能看出来, 阈值 = 白底值 + ADAPT_MARGIN(60)。
         * 对照上面 R_BITS 那行: 数值明显高的那几路应该正好是 '#'。上电标定完串口会打一行基线+阈值。 */
        sprintf(buf, "%3u %3u %3u %3u", (unsigned)track_raw[0], (unsigned)track_raw[1],
                                        (unsigned)track_raw[2], (unsigned)track_raw[3]);
        lcd_row(LCD_Y(R_RAW0), buf, (track_raw[0] > track_thr[0] || track_raw[3] > track_thr[3]) ? YELLOW : GRAY);

        sprintf(buf, "%3u %3u %3u %3u", (unsigned)track_raw[4], (unsigned)track_raw[5],
                                        (unsigned)track_raw[6], (unsigned)track_raw[7]);
        lcd_row(LCD_Y(R_RAW1), buf, (track_raw[4] > track_thr[4] || track_raw[7] > track_thr[7]) ? YELLOW : GRAY);
    }
#endif /* LCD_DEBUG */
#endif /* LCD_SHOW_RESULT */
}


/*---------------------------------------------------------------------------
 * 5ms 任务: 循迹 (读灰度 + 控制)
 *-------------------------------------------------------------------------*/
void APP_Task_5ms(void)
{
    /* 诊断: 本任务的实际调用间隔 (= 主循环一圈的时间)。正常 = 5ms */
    {
        static uint32_t last_ms = 0;
        uint32_t now_ms = Get_Time();
        app_loop_ms = (uint16_t)(now_ms - last_ms);
        last_ms     = now_ms;
    }

    /* IMU 转向/保向期间独占电机: 循迹让出, 由 ImuTurn 推进 */
    if (ImuTurn_IsBusy())
    {
        ImuTurn_Update();

        if (ImuTurn_JustFinished())          /* 刚结束: 循迹清状态, 下一拍自然接管 */
        {
            Track_Reset();
        }
        return;
    }

    /* 驱动闸门: 没按 S2 时也照读灰度刷屏, 只是不驱动电机 (台架手推车标定用)。
     * ★ 必须在 Mission 分支前面: 否则状态机接管后判定被闸门挡掉 -> 按 S2 后屏在跳、车不走 */
    {
        static int drive_last = -1;
        if (drive_last != app_run)              /* 开关状态变了才切, 不每拍写寄存器 */
        {
            Track_SetDrive(app_run ? 1 : 0);
            drive_last = app_run;
        }
    }

#if MISSION_ENABLE
    /* 任务状态机接管: 由它决定这一拍是 循迹 / 盲走 / 定向转 / 停车 */
    if (Mission_IsActive())
    {
        Mission_Run();
        return;
    }
#endif

    Track_Run();                                    /* 没开任务状态机 = 纯循迹 */
}

/*---------------------------------------------------------------------------
 * 10ms 任务: 板载按键 S2(PB21) —— 单击 开始/停止, 长按 加速
 *-------------------------------------------------------------------------*/
void APP_Task_10ms(void)
{
    int ev;

    if (key_ev_pending == 0) return;

    /* 取走中断里缓存的事件 (短暂关中断, 免得取+清之间又来了新事件被丢掉) */
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    ev = key_ev_pending;
    key_ev_pending = 0;
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    if (ev == 0) return;

    /* ---------------- 单击: 开始 / 停止 ---------------- */
    if (ev & KEY_EV_CLICK)
    {
        app_run = app_run ? 0 : 1;

        if (app_run)
        {
            Track_Reset();                                   /* 清丢线计时 / PD 历史 */
            Motor_Enable();                                  /* STBY 使能 */
#if MISSION_ENABLE
            Mission_Start();                                 /* 按路线表开跑 */
#endif
#if WORK_FEATURES
            Results_Clear();                                 /* 新任务开始: 清上一次的房间结果 */
#endif
        }
        else
        {
#if MISSION_ENABLE
            Mission_Stop();                                  /* 状态机一起停 (含急停转向) */
#endif
            ImuTurn_Stop();                                  /* 急停: 转向/保向一起停 */
            Motor_Stop();                                    /* 输出清零 + 方向脚复位 */
        }
        /* 这里刻意不发串口: 9600 下发一行日志要 20~40ms, 会把 5ms 循迹节拍卡住 */
    }

    /* ---------------- 长按: 加速一档 ---------------- */
    if (ev & KEY_EV_ACCEL)
    {
        Track_StepSpeed(1);                                  /* 150~900, 到顶回绕到 150 */
    }
}

/*---------------------------------------------------------------------------
 * 20ms 任务: IMU 姿态帧解析 (消费环形缓冲, 解析 0x7E 0x23 帧)
 *-------------------------------------------------------------------------*/
void APP_Task_20ms(void)
{
    IMU_Process();
}

/*---------------------------------------------------------------------------
 * 100ms 任务: 写 LCD 文字影子 (真正推屏在主循环 LCD_RowFlush())
 *-------------------------------------------------------------------------*/
void APP_Task_100ms(void)
{
    LCD_track();
}

/*---------------------------------------------------------------------------
 * 1000ms 任务: 串口日志
 *-------------------------------------------------------------------------*/
void APP_Task_1000ms(void)
{
    /* 授时模块: 1s 叫一次, 内部按 TIMESYNC_CALIB_MS(已同步=30s) / TIMESYNC_RETRY_MS(未同步=3s)
     * 自己判断这次要不要真读总线 —— 平时空跑不占时间。两次读数之间靠本板晶振走时, 秒不会停。
     * 放这个任务而不是 5ms 控制回路: 读事务是阻塞的 (正常 ~0.6ms, 断线时最坏 50ms 超时)。
     * 从机只在被读时应答、不主动上报, 所以必须自己轮询。 */
    {
        int st;

        TimeSync_Poll();

        st = TimeSync_Status();
        if (st != ts_last_status)      /* 只在状态变化时打串口 (9600 下发一行要几十 ms) */
        {
            ts_last_status = st;

            if (st == 1)
                printf("TIME ok: %s  (utc=%u ok=%u)\r\n", TimeSync_GetStr(),
                       (unsigned int)TimeSync_GetEpochUtc(), (unsigned int)TimeSync_GetOkCount());
            else if (st == 0)
                printf("TIME wait: module not synced yet (bus ok, ok=%u err=%u)\r\n",
                       (unsigned int)TimeSync_GetOkCount(), (unsigned int)TimeSync_GetErrCount());
            else
                printf("TIME FAULT: no ACK on I2C1 PA29/PA30 -> check wire/pullup/power (err=%u)\r\n",
                       (unsigned int)TimeSync_GetErrCount());
        }
    }

}

/*---------------------------------------------------------------------------
 * 系统初始化
 *-------------------------------------------------------------------------*/
static void System_Init(void)
{
    SYSCFG_DL_init();                       /* 时钟/GPIO/PWM/SPI(LCD)/SysTick */

    Motor_Stop();
    Motor_Enable();                         /* TB6612 STBY = 1 使能 */

    Key_Init();                             /* 板载按键 S1/S2 调速 */
    IMU_Init();                             /* IMU: UART1(PA8/PA9) RX 中断 + RX 上拉 */
    ImuTurn_Init();                         /* IMU 转向/保向层复位 */
    Mission_Init();                         /* 任务状态机复位 (路线表在 middle/Mission.c) */

#if WORK_FEATURES
    TW_Init();                              /* 语音: 空实现 (只提醒别碰 PB17/18, 见 app/TW.c) */
    UART_Init();                            /* UART0 开中断: 语音 ASRPRO2.0 (PA10) 命令出口 */
#endif

    Timer_Init();                           /* 1ms 时基: 清标志 + 开中断 */
    DL_TimerG_startCounter(TIMER_0_INST);   /* 启动 1ms 计数 -> 调度器时基 */

    TimeSync_Init();                        /* 授时查询端复位: 等 1000ms 任务去读第一次 */

    LCD_Init();
    /* 上电自检: 刷 3 个纯色 + 清屏, 确认 LCD 在位
     * 用批量版: 整屏 8.2ms/次 (老 LCD_Fill 逐字节要 0.28s/次, 4 次就是 1.1s) */
    LCD_FillFast(0, 0, LCD_W, LCD_H, RED);   delay_ms(200);
    LCD_FillFast(0, 0, LCD_W, LCD_H, GREEN); delay_ms(200);
    LCD_FillFast(0, 0, LCD_W, LCD_H, BLUE);  delay_ms(200);
    LCD_FillFast(0, 0, LCD_W, LCD_H, BLACK);
    LCD_DrawRectangle(0, 0, LCD_W - 1, LCD_H - 1, WHITE);

    Track_Init();                           /* 循迹 PD 初始化 */
#if WORK_FEATURES
    g_bat_v   = Bat_ReadAvg();              /* 上电先读一次电量 (结果区电量行) */
    g_bat_pct = Bat_ToPercent(g_bat_v);
    {
        uint8_t to;                         /* 上电先测一次温度 -> 屏第 6 行一上电就有数 */
        MLX90614_Init();                    /* 把测温的 PB17(SCL)/PB18(SDA) 配成 GPIO (PA0/PA1 留给循迹) */
        /* 上电先在测温那条线(PB17/PB18)上扫一遍: 看哪些地址有应答 */
        {
            char ls[24];
            g_i2c_n = MLX90614_BusScan(g_i2c_list, 8);
            fmt_i2c(ls);
            printf("MLX (PB17/PB18) scan: %s  (temp addr=0x%02X)\r\n", ls, (unsigned)MLX90614_I2C_ADDR);
        }
        MLX90614_AutoDetect();              /* 自动试 0x5A / 0x00 */
        printf("MLX90614 autodetect: addr=0x%02X\r\n", (unsigned)MLX90614_GetAddr());
        (void)Temp_ReadBody(&to);
    }
#endif
    printf("\r\n==== TRACK mode: AuroraT8 (I2C0 PA0/PA1), 5ms loop ====\r\n");
    printf("KEY S2/PB21: click = start/stop, long press = speed up\r\n");
    printf("TIME: I2C1 PA30(SDA)/PA29(SCL) -> ESP32-S2 @0x42 400kHz, calib %us\r\n",
           (unsigned int)(TIMESYNC_CALIB_MS / 1000u));
}

int main(void)
{
    System_Init();

    while (1)
    {
        Scheduler_Run();
        LCD_RowFlush();     /* 刷 1 行脏行 (<=0.9ms): 100ms 任务只写影子, 推屏在这里分摊 */
    }
}

/*===========================================================================
 * 任务状态机的钩子 (弱函数覆盖 middle/Mission.c 里的空实现)
 *   Mission_OnWork(room): 到达房间/工作区, 已经停车 —— 以后测温/语音/LCD/LORA 挂这里
 *   Mission_OnStepEnter:  每一步开始打一行串口 (台架调路线时很有用)
 *   Mission_OnDone:       全程收工
 * ★ 这些都在 5ms 任务里被调用, 别写长时间阻塞的代码 (要等就用 MS_WORK 步的毫秒数)
 *===========================================================================*/
#if MISSION_ENABLE
void Mission_OnStepEnter(int step)
{
    printf("MSTEP %d %s\r\n", step, Mission_GetStepName());
}

void Mission_OnWork(int room)
{
    /* 到达房间/工作区 (车已停稳) —— 这一步只"开始播报", 真正的测温放到 OnWorkTick 里:
     *   OnWork    : 播报"请 N 号房间人员配合测温"
     *   OnWorkTick: 等够 WORK_PROMPT_MS -> 测温/电量/记录/LORA -> 播报"N 号房间测温完成"
     * ★ 为什么拆成两个钩子: 天问的"播报"是阻塞的, 真人放手掌也要反应时间 —— 用一个非阻塞
     *   状态机在两次回调之间等, 而不是在 5ms 任务里 delay (那会把调度和急停一起卡住)。 */
#if WORK_FEATURES
    ws_phase = 0;
    Voice_RoomStart(room);                          /* 语音: 提示"请 N 号房间人员配合测温" */
#else
    printf("MWORK room=%d t=%s\r\n", room, TimeSync_GetStr());
#endif
}

void Mission_OnWorkTick(int room, uint32_t elapsed_ms)
{
    /* MS_WORK 步每 5ms 回调一次 (由 Mission.c 调); 本函数必须非阻塞 */
#if WORK_FEATURES
    uint8_t tok = 0;
    float   temp, bv;
    uint8_t pct;
    char    ts[10], vs[10];

    if (ws_phase != 0) return;                          /* 本房间已经做完了 */
    if (elapsed_ms < (uint32_t)WORK_PROMPT_MS) return;  /* 还没到点: 让 TTS 播完 + 队员放手掌 */

    ws_phase = 1;                                       /* 只做一次 */

    temp = Temp_ReadBody(&tok);                         /* 体温: 多次采样取中位 */
    bv   = Bat_ReadAvg();                               /* 电池电压/电量 (顺手刷新) */
    pct  = Bat_ToPercent(bv);
    g_bat_v   = bv;
    g_bat_pct = pct;
    g_temp_c  = temp;                                   /* 屏第 6 行 (R_TEMP) 实时温度 */
    g_temp_ok = tok;

#if LCD_SHOW_RESULT
    if ((room >= 1) && (room <= 4))                     /* 记录到结果区 (中途不清) */
    {
        TimeSync_DateTime dt;
        g_room[room].valid   = 1;
        g_room[room].temp_c  = temp;
        g_room[room].bat_v   = bv;
        g_room[room].bat_pct = pct;
        g_room[room].time_ok = (TimeSync_Get(&dt) == 1) ? 1 : 0;
        if (g_room[room].time_ok)
        {
            g_room[room].hh = dt.hour;
            g_room[room].mm = dt.min;
            g_room[room].ss = dt.sec;
        }
    }
#endif

    /* LORA 上报 (printf 出口 = LORA 口 UART3/PB2, 9600); 浮点用整数格式化 */
    fmt_temp(ts, temp);
    fmt_volt(vs, bv);
    printf("TEMP,room=%d,temp=%sC,ok=%d,bat=%s,%d%%,time=%s,utc=%u\r\n",
           room, tok ? ts : "--.-", (int)tok, vs, (int)pct,
           TimeSync_GetStr(), (unsigned)TimeSync_GetEpochUtc());

    Voice_RoomDone(room);                               /* 语音: 提示"N 号房间测温完成" */
#else
    (void)room; (void)elapsed_ms;
#endif
}

void Mission_OnDone(void)
{
    /* 收工即停: 跑完最后一步后不要再落回"纯循迹"自己往前跑 (MISSION_DEBUG_STEPS 限流时同理) */
    app_run = 0;
    Motor_Stop();
#if WORK_FEATURES
    g_task_done = 1;                                /* 屏第 1 行提示"任务完成" (结果不清) */
    Voice_AllDone();                                /* 语音: 播报"本次任务完成" */
    {
        char vs[10];
        fmt_volt(vs, g_bat_v);
        printf("DONE,task=complete,rooms=4,bat=%s,%d%%,time=%s,utc=%u\r\n",
               vs, (int)g_bat_pct, TimeSync_GetStr(), (unsigned)TimeSync_GetEpochUtc());
    }
#else
    printf("MDONE done, stopped. t=%s\r\n", TimeSync_GetStr());
#endif
}
#endif
