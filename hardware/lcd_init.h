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

#endif
