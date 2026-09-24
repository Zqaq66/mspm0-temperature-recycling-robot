#include "MLX90614.h"

/******************************************************************
 * MLX90614 非接触红外测温
 *
 * 底层已从 GPIO 模拟时序改为硬件 I2C0：
 *   PA0 = SDA, PA1 = SCL, 400kHz（与 Xinliu_AuroraT8 循迹传感器共用总线）
 * 时序：SMBus Read Word
 *   S | addr+W | cmd | Sr | addr+R | dataL | dataH | P
 ******************************************************************/
#define MLX_I2C_TIMEOUT     10000

/******************************************************************
 * 函 数 名 称：MLX_WriteRegAddr
 * 函 数 说 明：发设备写地址+寄存器地址，**不发 STOP**，为后面 repeated start 做准备
 * 函 数 形 参：devAddr 7位从机地址, regAddr 寄存器地址
 * 函 数 返 回：0 成功，负值 失败
 ******************************************************************/
static int MLX_WriteRegAddr(unsigned char devAddr, unsigned char regAddr)
{
    volatile uint32_t timeout = 0;
    uint32_t index = 0;

    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout++ > MLX_I2C_TIMEOUT) {
            return -1;
        }
    }

    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, devAddr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    timeout = 0;
    while (index < 1) {
        if (!DL_I2C_isControllerTXFIFOFull(I2C_0_INST)) {
            DL_I2C_transmitControllerData(I2C_0_INST, regAddr);
            index++;
        }
        if (timeout++ > MLX_I2C_TIMEOUT * 50) {
            return -4;
        }
    }

    timeout = 0;
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout++ > MLX_I2C_TIMEOUT) {
            return -2;
        }
    }
    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout++ > MLX_I2C_TIMEOUT) {
            return -3;
        }
    }
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    return 0;
}

/******************************************************************
 * 函 数 名 称：MLX_ReadBytes
 * 函 数 说 明：repeated start 读取 N 字节
 * 函 数 形 参：devAddr 7位从机地址, buf 接收缓冲, len 长度
 * 函 数 返 回：0 成功，负值 失败
 ******************************************************************/
static int MLX_ReadBytes(unsigned char devAddr, unsigned char *buf, uint8_t len)
{
    uint32_t index = 0;
    volatile uint32_t timeout = 0;

    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout++ > MLX_I2C_TIMEOUT) {
            return -1;
        }
    }

    DL_I2C_startControllerTransfer(I2C_0_INST, devAddr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    timeout = 0;
    while (index < len) {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            buf[index++] = DL_I2C_receiveControllerData(I2C_0_INST);
        }
        if (timeout++ > MLX_I2C_TIMEOUT * 50) {
            return -4;
        }
    }

    timeout = 0;
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout++ > MLX_I2C_TIMEOUT) {
            return -2;
        }
    }
    timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout++ > MLX_I2C_TIMEOUT) {
            return -3;
        }
    }
    return 0;
}

/******************************************************************
 * 函 数 名 称：MLX90614_Read
 * 函 数 说 明：读 MLX90614 内(0x07)/外(0x06)温度，返回摄氏度
 * 函 数 形 参：SlaveAddr = 0x5A(7位地址)   RegAddr = 0x07
 * 函 数 返 回：温度(℃)，读取失败返回 0.0
 ******************************************************************/
float MLX90614_Read(unsigned char SlaveAddr, unsigned char RegAddr)
{
    uint8_t rx[2] = {0, 0};
    uint16_t tempRaw;

    if (MLX_WriteRegAddr(SlaveAddr, RegAddr) != 0) {
        return 0.0f;
    }
    if (MLX_ReadBytes(SlaveAddr, rx, 2) != 0) {
        return 0.0f;
    }

    tempRaw = ((uint16_t)rx[1] << 8) | rx[0];
    return (tempRaw * 0.02f) - 273.15f;
}
