/*===========================================================================
 * ST7735S 128x160 SPI LCD 驱动 — MSPM0G3507 适配版
 *
 * 引脚全部来自 SysConfig (ti_msp_dl_config.h 里的宏), 不自带引脚定义:
 *   数据线 = 硬件 SPI1 的 LCD_DATA_SCLK / LCD_DATA_PICO  (SCLK=PB9, MOSI=SDA=PB8)
 *   控制线 = GPIO 组的 LCD_LCD_RES / LCD_LCD_DC / LCD_LCD_CS / LCD_BLK
 *            (RES=PB10, DC=PB11, CS=PB14, BLK=PB26)
 *
 * 在 SysConfig 中修改 LCD 引脚后, 重新编译即可生效, 无需手动修改本文件。
 *===========================================================================*/

#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "ti_msp_dl_config.h"
#include "Time.h"   /* DZZH: delay_ms() 在 BSP/Time.c */

/* 屏幕方向: 0/1=竖屏 2/3=横屏 */
#define USE_HORIZONTAL 1

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 128
#endif

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

/* ---- 引脚操作 (RES/DC/CS/BLK = GPIO, SCLK/PICO = 硬件 SPI1) ----
 *
 *   SysConfig 命名                   → 本驱动命名
 *   LCD_DATA (SPI1, PB9=SCLK, PB8=PICO)
 *   LCD_RES  (Reset, PB10)          → RES
 *   LCD_DC   (Data/Command, PB11)   → DC
 *   LCD_CS   (Chip Select, PB14)    → CS
 *   BLK      (Backlight, PB26)      → BLK
 *----------------------------------------------------------------------------*/
#define LCD_RES_PORT    LCD_PORT
#define LCD_RES_PIN     LCD_LCD_RES_PIN
#define LCD_DC_PORT     LCD_PORT
#define LCD_DC_PIN      LCD_LCD_DC_PIN
#define LCD_CS_PORT     LCD_PORT
#define LCD_CS_PIN      LCD_LCD_CS_PIN
#define LCD_BLK_PORT    LCD_PORT
/* LCD_BLK_PIN: SysConfig 已定义, 无需重定义 */

/* 硬件 SPI 实例 (SysConfig 生成) */
#define LCD_SPI_INST    LCD_DATA_INST   /* SPI1 */

/* GPIO 操作宏 (仅 RES/DC/CS/BLK, SCLK/PICO 由硬件 SPI 管理) */
#define LCD_RES_Clr()   DL_GPIO_clearPins(LCD_RES_PORT, LCD_RES_PIN)
#define LCD_RES_Set()   DL_GPIO_setPins(LCD_RES_PORT, LCD_RES_PIN)
#define LCD_DC_Clr()    DL_GPIO_clearPins(LCD_DC_PORT, LCD_DC_PIN)
#define LCD_DC_Set()    DL_GPIO_setPins(LCD_DC_PORT, LCD_DC_PIN)
#define LCD_CS_Clr()    DL_GPIO_clearPins(LCD_CS_PORT, LCD_CS_PIN)
#define LCD_CS_Set()    DL_GPIO_setPins(LCD_CS_PORT, LCD_CS_PIN)
#define LCD_BLK_Clr()   DL_GPIO_clearPins(LCD_BLK_PORT, LCD_BLK_PIN)
#define LCD_BLK_Set()   DL_GPIO_setPins(LCD_BLK_PORT, LCD_BLK_PIN)

void LCD_GPIO_Init(void);
void LCD_Writ_Bus(u8 dat);
void LCD_WR_DATA8(u8 dat);
void LCD_WR_DATA(u16 dat);
void LCD_WR_REG(u8 dat);
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);
void LCD_Init(void);

/*---------------------------------------------------------------------------
 * 批量(流式)写 —— 2026-10-06 新增, 上面的例程函数一字未动
 *
 *   例程 LCD_Writ_Bus() 是"每字节一次事务": 每个字节都 CS 拉低 -> 发 -> 等忙 -> CS 拉高。
 *   单字节开销实测 3.4us, 而线上真正传输只要 0.2us(40MHz) —— 96% 的时间耗在翻 CS 和忙等上。
 *   下面这组是"CS 只翻一次, 连着推 N 个字节", 走同一个硬件 SPI1:
 *       4096 字节 (一整行 16 个 16 号字)   0.82ms        (老路径 4208 次事务 约 14ms)
 *       整屏填充 40960 字节               8.2ms         (老路径 约 0.28s)
 *
 *   用法 (DC 要自己保证: 发命令前 LCD_DC_Clr(), 发完 LCD_DC_Set(); LCD_Address_Set()
 *   结尾就是 0x2C 命令, 出来时 DC 已经是数据态, 可以直接接 Begin):
 *       LCD_Writ_BusBegin();                       // CS 拉低一次
 *       while (len--) LCD_Writ_BusStream(*p++);    // 字节之间不空等, 靠 TX FIFO 节流
 *       LCD_Writ_BusEnd();                         // 等最后一字节移出 + CS 拉高
 *   ★ LCD_Writ_BusStream 是 static inline, 循环里没有函数调用开销。
 *---------------------------------------------------------------------------*/
void LCD_Writ_BusBegin(void);                       /* CS 拉低一次 */
static inline void LCD_Writ_BusStream(u8 dat)       /* 推 1 字节: TX FIFO 满就等 */
{
    while (DL_SPI_isTXFIFOFull(LCD_SPI_INST)) { }
    DL_SPI_transmitData8(LCD_SPI_INST, dat);
}
void LCD_Writ_BusEnd(void);                         /* 等移出完 + CS 拉高 */
void LCD_Writ_BusBulk(const u8 *buf, u32 len);      /* Begin + 循环 Stream + End (整块缓冲) */

#endif
