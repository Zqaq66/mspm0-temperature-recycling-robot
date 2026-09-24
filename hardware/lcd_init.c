/*===========================================================================
 * ST7735S LCD 底层驱动 — MSPM0G3507 适配版
 * 硬件 SPI1 (PB9=SCLK, PB8=PICO), CS/DC/RES/BLK 仍为 GPIO 手动控制
 *===========================================================================*/

#include "lcd_init.h"
#include "lcd.h"


void LCD_GPIO_Init(void)
{
    /* RES/DC/CS/BLK 配置为 GPIO 输出 (PB8/PB9 已由 SYSCFG_DL_LCD_DATA_init 配置为 SPI1) */
    DL_GPIO_initDigitalOutput(LCD_LCD_RES_IOMUX);  /* PB10 = RES */
    DL_GPIO_initDigitalOutput(LCD_LCD_DC_IOMUX);   /* PB11 = DC  */
    DL_GPIO_initDigitalOutput(LCD_LCD_CS_IOMUX);   /* PB14 = CS  */
    DL_GPIO_initDigitalOutput(LCD_BLK_IOMUX);      /* PB26 = BLK */

    DL_GPIO_enableOutput(GPIOB,
        LCD_LCD_RES_PIN | LCD_LCD_DC_PIN |
        LCD_LCD_CS_PIN  | LCD_BLK_PIN);

    /* 初始电平: 全部拉高 */
    DL_GPIO_setPins(GPIOB,
        LCD_LCD_RES_PIN | LCD_LCD_DC_PIN |
        LCD_LCD_CS_PIN  | LCD_BLK_PIN);
}

/* 硬件 SPI 写入一个字节 (PB9=SCLK, PB8=PICO, CS 手动控制保持不变) */
void LCD_Writ_Bus(u8 dat)
{
    LCD_CS_Clr();
    DL_SPI_transmitDataBlocking8(LCD_SPI_INST, dat);
    while (DL_SPI_isBusy(LCD_SPI_INST)) {}  /* 等待字节完全移出后再释放 CS */
    LCD_CS_Set();
}

void LCD_WR_DATA8(u8 dat)   { LCD_Writ_Bus(dat); }
void LCD_WR_DATA(u16 dat)   { LCD_Writ_Bus(dat>>8); LCD_Writ_Bus(dat); }

void LCD_WR_REG(u8 dat)
{
    LCD_DC_Clr();
    LCD_Writ_Bus(dat);
    LCD_DC_Set();
}

void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
    if(USE_HORIZONTAL==0)
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+2); LCD_WR_DATA(x2+2);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+1); LCD_WR_DATA(y2+1);
        LCD_WR_REG(0x2c);
    }
    else if(USE_HORIZONTAL==1)
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+2); LCD_WR_DATA(x2+2);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+1); LCD_WR_DATA(y2+1);
        LCD_WR_REG(0x2c);
    }
    else if(USE_HORIZONTAL==2)
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+1); LCD_WR_DATA(x2+1);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+2); LCD_WR_DATA(y2+2);
        LCD_WR_REG(0x2c);
    }
    else
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+1); LCD_WR_DATA(x2+1);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+2); LCD_WR_DATA(y2+2);
        LCD_WR_REG(0x2c);
    }
}

void LCD_Init(void)
{
    LCD_GPIO_Init();

    LCD_RES_Clr(); delay_ms(100);
    LCD_RES_Set(); delay_ms(100);

    LCD_BLK_Set();
    delay_ms(100);

    /* ST7735S 初始化序列 */
    LCD_WR_REG(0x11); delay_ms(120);  /* Sleep out */

    /* Frame Rate */
    LCD_WR_REG(0xB1); LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB2); LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB3); LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
                      LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);

    LCD_WR_REG(0xB4); LCD_WR_DATA8(0x03);  /* Dot inversion */

    /* Power Sequence */
    LCD_WR_REG(0xC0); LCD_WR_DATA8(0x28); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x04);
    LCD_WR_REG(0xC1); LCD_WR_DATA8(0XC0);
    LCD_WR_REG(0xC2); LCD_WR_DATA8(0x0D); LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC3); LCD_WR_DATA8(0x8D); LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4); LCD_WR_DATA8(0x8D); LCD_WR_DATA8(0xEE);

    LCD_WR_REG(0xC5); LCD_WR_DATA8(0x1A);  /* VCOM */

    /* MX, MY, RGB mode */
    LCD_WR_REG(0x36);
    if(USE_HORIZONTAL==0)      LCD_WR_DATA8(0x00);
    else if(USE_HORIZONTAL==1) LCD_WR_DATA8(0xC0);
    else if(USE_HORIZONTAL==2) LCD_WR_DATA8(0x70);
    else                        LCD_WR_DATA8(0xA0);

    /* Gamma Sequence */
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x22); LCD_WR_DATA8(0x07); LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x2E); LCD_WR_DATA8(0x30); LCD_WR_DATA8(0x25); LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x28); LCD_WR_DATA8(0x26); LCD_WR_DATA8(0x2E); LCD_WR_DATA8(0x3A);
    LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x03); LCD_WR_DATA8(0x13);
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x16); LCD_WR_DATA8(0x06); LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x2D); LCD_WR_DATA8(0x26); LCD_WR_DATA8(0x23); LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x27); LCD_WR_DATA8(0x25); LCD_WR_DATA8(0x2D); LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x13);

    LCD_WR_REG(0x3A); LCD_WR_DATA8(0x05);  /* 65k mode */
    LCD_WR_REG(0x29);                       /* Display on */
}
