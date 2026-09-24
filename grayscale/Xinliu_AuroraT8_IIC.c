/*
    该文件为Xinliu_AuroraT8传感器的IIC读取实例
    202605171250@Xinliu_Tech
    移植入本工程：仅把 Delay.h 换成 Time.h（delay_us），其余逻辑不动
*/
#include "Xinliu_AuroraT8_IIC.h"


GrayData data;              // 创建传感器接收数据联合体全局变量

/**
 * @brief 使用硬件IIC向Xinliu灰度传感器发送查询指令
 * @param cmd 发送的命令
 * @return int 执行结果
 *        0  成功
 *       -1  启动传输前等待I2C控制器空闲超时
 *       -2  等待传输完成超时
 *       -3  传输完成等待I2C控制器空闲超时
 */
int Xinliu_AuroraT8_IIC_Send_Request(uint8_t cmd)
{
    volatile uint32_t timeout = 0;
    uint32_t index = 0;         // 发送索引

    // 等待I2C总线空闲
    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout++ > I2C_TIMEOUT) {
            return -1;
        }
    }

    // 启动物理通道传输
    DL_I2C_startControllerTransfer(I2C_0_INST, XINLIU_AURORA_T8_IIC_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    timeout = 0;
    while (index < 1){
        if (!DL_I2C_isControllerTXFIFOFull(I2C_0_INST)){
            DL_I2C_transmitControllerData(I2C_0_INST, cmd);
            index++;
        }
        if (timeout++ > I2C_TIMEOUT *50) {
            return -4;
        }
        // FIFO满时等待
    }

    // 等待传输完成
    timeout = 0;
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout++ > I2C_TIMEOUT) {
            return -2; // 超时
        }
    }
    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)){
        if (timeout++ > I2C_TIMEOUT) {
            return -3; // 超时
        }
    }
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);           // 离开强制清理FIFO
    return 0;
}


/**
 * @brief 使用硬件IIC读取Xinliu灰度传感器返回的数据
 * @param *data 数组指针
 * @param len 数组长度
 * @return int 执行结果
 *        0  成功
 *       -1  启动传输前等待I2C控制器空闲超时
 *       -2  等待传输完成超时
 *       -3  传输完成等待I2C控制器空闲超时
 */
int Xinliu_AuroraT8_IIC_Read_Data(uint8_t *data, uint8_t len)
{
    uint32_t index = 0;     // 定义接收索引长度
    volatile uint32_t timeout = 0;


    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)){
        if (timeout++ > I2C_TIMEOUT) {
            return -1;
        }
    }

    // 读取数据
    DL_I2C_startControllerTransfer(I2C_0_INST, XINLIU_AURORA_T8_IIC_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    timeout = 0;
    while(index < len){
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)){
            data[index++] = DL_I2C_receiveControllerData(I2C_0_INST);
        }
        if (timeout++ > I2C_TIMEOUT *50) {
            return -4;
        }
        // FIFO满时等待
    }

    // 等待接收完成
    timeout = 0;
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout++ > I2C_TIMEOUT) {
            return -2;
        }
    }

    // 检查错误状态
    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)){
        if (timeout++ > I2C_TIMEOUT) {
            return -3;
        }
     }

    return 0;
}

/**
 * @brief 根据命令码获取数据长度查表
 * @param cmd 要查询返回长度的命令
 * @return uint8_t 长度
 */
static uint8_t get_cmd_data_len(uint8_t cmd) {
    switch (cmd) {
        case 0x01:case 0x02:case 0x03:case 0x04:
        case 0x05:case 0x06:case 0x07:case 0x08:
            return 1;
        case 0x09:case 0x0A:case 0x0B:
            return 8;
        case 0x0C:
            return 1;
        case 0x11:case 0x12:case 0x13:case 0x14:
        case 0x15:case 0x16:case 0x17:case 0x18:
            return 2;
        case 0x19:case 0x1A:case 0x1B:
            return 16;
        default:   return 0;    // 未知命令
    }
}

/**
 * @brief 内部解析函数,将接收到的原始数据(不含回显、长度、校验)填充到GrayData联合体
 * @param cmd 命令
 * @param *data_buf 接收的有效数据数组指针
 * @param data_len  接收的有效数据长度
 * @param *out      存储结果的联合体
 * @return void     空
 */
static void parse_data(uint8_t cmd, const uint8_t *data_buf, uint8_t data_len, GrayData *out) {
    if (!out) {     // 空指针检测
        return;
    }

    switch (cmd) {
        case 0x01:case 0x02:case 0x03:case 0x04:
        case 0x05:case 0x06:case 0x07:case 0x08: // 单路 ÷8 数据
            if (data_len >= 1) {
                out->single_div8 = data_buf[0];
            }
            break;

        case 0x09:                              // 全部 ÷8
        case 0x0A:                              // 黑色阈值 ÷8
        case 0x0B:                              // 白色阈值 ÷8
            if (data_len >= 8) {
                // 循环赋值
                for (int i = 0; i < 8; i++) {
                    out->all_div8[i] = data_buf[i];
                }
            }
            break;

        case 0x0C:                              // 校准数字量
            if (data_len >= 1) {
                out->digital_bits = data_buf[0];
            }
            break;

        case 0x11:case 0x12:case 0x13:case 0x14:
        case 0x15:case 0x16:case 0x17:case 0x18: // 单路 16位 ADC
            if (data_len >= 2) {
                // printf("Raw%d,%d\n", data_buf[0], data_buf[1]);
                out->single_adc = (uint16_t)data_buf[0] | ((uint16_t)data_buf[1] << 8);
            }
            break;

        case 0x19:                              // 8路 16位 ADC
            if (data_len >= 16) {
                for (int i = 0; i < 8; i++) {
                    out->all_adc[i] = (uint16_t)data_buf[i*2] | ((uint16_t)data_buf[i*2+1] << 8);
                }
            }
            break;

        case 0x1A:                              // 8路黑色阈值 (16位)
            if (data_len >= 16) {
                for (int i = 0; i < 8; i++) {
                    out->black_thresh16[i] = (uint16_t)data_buf[i*2] | ((uint16_t)data_buf[i*2+1] << 8);
                }
            }
            break;

        case 0x1B:                              // 8路白色阈值 (16位)
            if (data_len >= 16) {
                for (int i = 0; i < 8; i++) {
                    out->white_thresh16[i] = (uint16_t)data_buf[i*2] | ((uint16_t)data_buf[i*2+1] << 8);
                }
            }
            break;

        default:
            // 未知命令
            break;
    }
}

/**
 * @brief 发送命令至灰度传感器，读取并校验完整帧，解析数据
 * @param cmd 命令码（参见宏定义）
 * @param out 解析后的数据存储指针
 * @return int 执行结果
 *        0  成功
 *       -1  发送请求失败（I2C 发送命令超时或错误）
 *       -2  读取帧头失败（I2C 读取前2字节失败）
 *       -3  回显命令不匹配（传感器返回的命令码与发送不一致）
 *       -4  数据长度异常（长度预告 > 20，超出安全范围）
 *       -5  读取剩余数据失败（I2C 读取数据部分+校验和失败）
 *       -6  校验和不匹配（计算出的校验和与接收的不一致）
 */
int Xinliu_AuroraT8_Sensor_Read(uint8_t cmd, GrayData *out)
{
    // 根据命令码获取预期数据长度
    uint8_t expected_data_len = get_cmd_data_len(cmd);
    if (expected_data_len == 0) {
        return -4;   // 未知命令，可视为长度异常
    }

    uint8_t total_len = 1 + 1 + expected_data_len + 1; // 回显+长度+数据+校验

    // 发送查询命令
    if (Xinliu_AuroraT8_IIC_Send_Request(cmd) != 0){
        return -1;
    }

    // 等待传感器准备数据
    delay_us(200);  // 对于短命令，延时20-50us足够，长命令(长度16的命令)，需要等待200us才能正常读取，这几个命令不是很常用，这里演示为了兼容

    // 读取完整帧
    uint8_t frame[32];  // 最大 1+1+16+1=19
    if (Xinliu_AuroraT8_IIC_Read_Data(frame, total_len) != 0) {
        return -2;
    }

    // 校验回显命令是否与发送一致
    if (frame[0] != cmd){
        return -3;
    }

    // 校验长度预告
    if (frame[1] != expected_data_len) {
        return -4;
    }

    // 校验和计算（回显+长度+数据）
    uint8_t checksum = frame[0] + frame[1];
    for (uint8_t i = 0; i < expected_data_len; i++) {
        checksum += frame[2 + i];
    }
    if (checksum != frame[2 + expected_data_len]) {
        return -5;
    }

    // 解析数据
    if (out) {
        parse_data(cmd, &frame[2], expected_data_len, out);
    }

    return 0;
}
