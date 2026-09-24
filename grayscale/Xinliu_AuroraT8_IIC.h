#ifndef __XINLIU_AURORAT8_IIC_H__
#define __XINLIU_AURORAT8_IIC_H__

#include "ti_msp_dl_config.h"
#include "stdint.h"             // 数据类型定义
#include "Time.h"               // 硬延时 delay_us()

// 定义设备的IIC地址，不同设备的IIC地址请修改这个宏定义
#define XINLIU_AURORA_T8_IIC_ADDR   0x40

// 定义IIC通信超时时间
#define I2C_TIMEOUT                 10000


// 单路除8数据
#define GRAY_CMD_SINGLE1_DIV8   0x01
#define GRAY_CMD_SINGLE2_DIV8   0x02
#define GRAY_CMD_SINGLE3_DIV8   0x03
#define GRAY_CMD_SINGLE4_DIV8   0x04
#define GRAY_CMD_SINGLE5_DIV8   0x05
#define GRAY_CMD_SINGLE6_DIV8   0x06
#define GRAY_CMD_SINGLE7_DIV8   0x07
#define GRAY_CMD_SINGLE8_DIV8   0x08

// 多路除8数据
#define GRAY_CMD_ALL_DIV8       0x09
#define GRAY_CMD_BLACK_THRESH_DIV8  0x0A   // 8路黑色阈值除8
#define GRAY_CMD_WHITE_THRESH_DIV8  0x0B   // 8路白色阈值除8

// 数字量直出(常用)
#define GRAY_CMD_DIGITAL_CALIB  0x0C

// 单路 16位 ADC(小端)
#define GRAY_CMD_SINGLE1_ADC    0x11
#define GRAY_CMD_SINGLE2_ADC    0x12
#define GRAY_CMD_SINGLE3_ADC    0x13
#define GRAY_CMD_SINGLE4_ADC    0x14
#define GRAY_CMD_SINGLE5_ADC    0x15
#define GRAY_CMD_SINGLE6_ADC    0x16
#define GRAY_CMD_SINGLE7_ADC    0x17
#define GRAY_CMD_SINGLE8_ADC    0x18

// 多路 16位 ADC / 阈值
#define GRAY_CMD_ALL_ADC        0x19         // 8路ADC，16字节数据
#define GRAY_CMD_BLACK_THRESH_16 0x1A        // 8路黑色阈值，16字节数据
#define GRAY_CMD_WHITE_THRESH_16 0x1B        // 8路白色阈值，16字节数据

// 错误状态
#define GRAY_CMD_ERROR          0xFF


// 解析结果联合体,谁最大，这块内存就是谁的大小
typedef union {
    uint8_t  single_div8;       // 单路除8数据
    uint8_t  all_div8[8];       // 8路除8数据
    uint8_t  black_thresh[8];   // 黑色阈值除8
    uint8_t  white_thresh[8];   // 白色阈值除8
    uint8_t  digital_bits;      // 数字量按位或
    uint16_t single_adc;        // 单路16位ADC(小端)
    uint16_t all_adc[8];        // 8路16位ADC
    uint16_t black_thresh16[8]; // 16位黑色阈值
    uint16_t white_thresh16[8]; // 16位白色阈值
} GrayData;

extern GrayData data;                   // 创建传感器接收数据联合体全局变量

int Xinliu_AuroraT8_IIC_Send_Request(uint8_t cmd);
int Xinliu_AuroraT8_IIC_Read_Data(uint8_t *data, uint8_t len);
int Xinliu_AuroraT8_Sensor_Read(uint8_t cmd, GrayData *out);

#endif
