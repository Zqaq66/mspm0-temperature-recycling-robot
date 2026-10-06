/*===========================================================================
 * middle/TimeSync.h —— I2C 授时查询端 (本板作 I2C 主机) + 本地走时
 *---------------------------------------------------------------------------
 * 【这是什么】
 *   从 ESP32-S2 Mini 授时模块 (I2C 从机, 7位地址 0x42) 读回 15 字节时间帧,
 *   然后把时间"存下来自己走" —— 不用每秒去读总线, 每隔一段时间校准一次即可。
 *
 * 【硬件接线】(独立 I2C1 总线, 和循迹那条 PA0/PA1 完全分开)
 *   MSPM0G3507 (查询端/主机)      ESP32-S2 Mini (模块/从机)
 *     PA30 SDA1  ---------------  GPIO5 SDA
 *     PA29 SCL1  ---------------  GPIO6 SCL
 *     GND        ---------------  GND        (必须共地!)
 *     (模块单独供电 3V3, 峰值 300~500mA, 别从板子 3V3 取电)
 *   上拉: 整条总线只挂一组, 400kHz + 多器件时取 2.2k~3.3k 到 3V3 (不是 4.7k!)
 *   走线: 400kHz 下 <=10cm; 模块天线上方别盖金属
 *   ★ 引脚定义在 empty.syscfg 的 I2C_TIME 实例里 (peripheral.sdaPin/sclPin);
 *     要改脚就在那里改 + 重新生成, 并同步改这张表 (曾从 PA3/PA4 改成 PA29/PA30)
 *
 * 【协议摘要】(详见 docs/I2C授时协议.md)
 *   一次查询 = 一次纯读事务, 固定读 15 字节:
 *     buf[0]    : 0x01 = 时间有效 / 0x00 = 模块还没联网同步
 *     buf[1..14]: "YYYYMMDDhhmmss" 14 个 ASCII 数字, 东八区墙钟
 *   ★ 只读, 不写 (不要发"写命令再重复 START"), 不要读变长
 *
 * 【怎么用】
 *   1) System_Init() 里, 在 Timer_Init() 之后调一次:  TimeSync_Init();
 *   2) 每个 100ms 任务里调一次 (到点才真去读, 平时是空跑):
 *          TimeSync_Poll();
 *   3) 要用时间的地方:
 *          TimeSync_GetStr()      -> "2026-10-05 20:31:07"   (测温记录 / LORA 上报)
 *          TimeSync_GetCompact()  -> "20261005203107"
 *          TimeSync_GetClock()    -> "10-05 20:31:07"        (屏上一行, 14 字符)
 *          TimeSync_GetEpochUtc() -> 真 Unix 秒 (UTC, 上报给上位机用)
 *          TimeSync_Get(&dt)      -> 拆好的年月日时分秒
 *   4) 屏上怎么显示 (未同步就不显示, 按用户要求):
 *          int st = TimeSync_Status();
 *          st ==  1 -> 显示 TimeSync_GetClock() (青/绿)
 *          st ==  0 -> 整行清空 (模块还没联网, 正常, 不用报警)
 *          st == -1 -> 整行清空 + 诊断行可以红一下 (总线故障: 线/上拉/供电)
 *
 * 【调参入口】都在下面这堆宏里, .c 里没有别的常数
 *
 * 【坑】
 *   - 地址是 7 位: DL_I2C_startControllerTransfer() 文档明确要 7 位, 直接传 0x42,
 *     ★ 别左移成 0x84 (那是 STM32 HAL 的写法)。
 *   - 超时必须 >=50ms: 从机会拉低 SCL 拖时钟, 严格的几毫秒超时会偶发读失败。
 *     本模块用 Get_Time() (1ms 时基) 计时, 不是数循环圈数。
 *   - 上电竞态: 上电后 3~5s (最坏 20s) 内读到 0x00 是正常的, 不是故障。
 *   - 从机不保存时间, 每次上电都要重新联网 -> 这个等待窗口每次都会出现。
 *   - 总线卡死 (SCL 被拉低不放) 恢复: 主机发 9 个 SCL 脉冲 + STOP (本模块未自动做,
 *     连续失败会退避重试, 现场遇到就断电重启)。
 *   - Poll() 是阻塞的 (正常 ~0.6ms, 最坏超时 50ms), 别放高优先级中断里。
 *     本工程放在 100ms 任务里; 连续失败后会退避到 30s 才再试, 免得反复卡循环。
 *===========================================================================*/
#ifndef __TIMESYNC_H__
#define __TIMESYNC_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*--------------------------- 1) 协议 / 硬件参数 ---------------------------*/
#define TIMESYNC_ADDR            0x42u   /* 从机 7 位地址 (模块侧 config.h 可改, 改了两边同步) */
#define TIMESYNC_FRAME_LEN       15u     /* 协议固定 15 字节, 不能多不能少 */
#define TIMESYNC_TIMEOUT_MS      50u     /* 单次事务超时 (协议要求 >=50ms, 时钟拉伸) */
#define TIMESYNC_TZ_OFFSET_SEC   (8*3600)/* 模块给的是东八区墙钟 -> 减掉它才是真 UTC */

/*--------------------------- 2) 查询节奏 (可调) ---------------------------*/
/* 已同步后的校准间隔: 模块自己每 60s 重新对一次时, 我们 30s 校一次就够,
 * 两次校准之间靠本板晶振走时 (30s 误差 <2ms, 秒级应用完全够)。想更省总线就加大。 */
#define TIMESYNC_CALIB_MS        30000u
/* 还没拿到有效时间时的重试间隔 (上电 3~5s 内通常就有了) */
#define TIMESYNC_RETRY_MS        3000u
/* 连续读失败 -> 判总线故障, 退避到这个间隔再试 (避免反复卡住调度器) */
#define TIMESYNC_BACKOFF_MS      30000u
#define TIMESYNC_FAIL_MAX        3u      /* 连续失败几次算总线故障 */

/*--------------------------- 3) 对外结构 / 接口 ---------------------------*/
typedef struct
{
    uint16_t year;      /* 2026 */
    uint8_t  mon;       /* 1~12 */
    uint8_t  day;       /* 1~31 */
    uint8_t  hour;      /* 0~23 */
    uint8_t  min;       /* 0~59 */
    uint8_t  sec;       /* 0~59 */
} TimeSync_DateTime;

/* 复位: 清掉已同步状态 / 计数 / 时间 (System_Init 里调, Timer_Init 之后) */
void TimeSync_Init(void);

/* 周期调用 (100ms 任务), 未到点直接返回, 到点才真读总线 */
void TimeSync_Poll(void);

/* 让下一次 Poll 立刻去读 (比如按键想马上刷新一下) */
void TimeSync_PollNow(void);

/* 状态:  1 = 时间可用   0 = 总线通但模块还没同步   -1 = 总线故障(连续读失败) */
int TimeSync_Status(void);

/* 1 = 有可用时间 (等价于 TimeSync_Status()==1) */
int TimeSync_IsValid(void);

/* 取当前时间 (本地走时, 精度秒级), 1 = 成功, 0 = 还没有可用时间 */
int TimeSync_Get(TimeSync_DateTime *out);

/* 真 Unix 时间戳 (UTC 秒); 未同步返回 0。用于上报给上位机 / 打时间戳 */
uint32_t TimeSync_GetEpochUtc(void);

/* 字符串 (静态缓冲, 每次调用覆盖; 未同步时是占位符) */
const char *TimeSync_GetStr(void);      /* "2026-10-05 20:31:07" (19 字符) */
const char *TimeSync_GetCompact(void);  /* "20261005203107"      (14 字符, 同协议格式) */
const char *TimeSync_GetClock(void);    /* "10-05 20:31:07"      (14 字符, 给屏用) */

/* 诊断: 距上次成功校准的秒数 / 成功次数 / 失败次数 */
uint32_t TimeSync_GetAgeSec(void);
uint32_t TimeSync_GetOkCount(void);
uint32_t TimeSync_GetErrCount(void);

/* 最近一次读的原始 15 字节 (调试用, 读失败时内容是上一次的) */
const uint8_t *TimeSync_GetRaw(void);

#endif /* __TIMESYNC_H__ */
