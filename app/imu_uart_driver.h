/**
 * @file  imu_uart_driver.h
 * @brief YbIMU 六轴陀螺仪 UART 驱动 —— 公共接口与硬件定义
 *
 * @note  通信参数: 115200-8-N-1, 25 Hz 自动上报
 *        硬件:    UART1, PA8=TX, PA9=RX（SysConfig 实例名 IMU）
 *        协议帧:  0x7E 0x23 LEN FUNC DATA[0..N-2] CHECKSUM
 */

#ifndef IMU_UART_DRIVER_H
#define IMU_UART_DRIVER_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 硬件常量 / Hardware Constants
 *
 * 引脚定义完全依赖 SysConfig (ti_msp_dl_config.h 中的 IMU_* / GPIO_IMU_* 定义)
 * 在 SysConfig 中修改 IMU 引脚或波特率后, 重新编译即可生效。
 *===========================================================================*/

/* ---- UART 实例 (来自 SysConfig IMU_* 定义) ---- */
#define IMU_UART_INST                   IMU_INST
#define IMU_UART_IRQHandler             IMU_INST_IRQHandler
#define IMU_UART_INT_IRQN               IMU_INST_INT_IRQN
#define IMU_UART_IRQ_PRIORITY           (2U)    /* 本工程: 编码器 GROUP1=0, 其余默认 */

/* ---- 引脚 (来自 SysConfig GPIO_IMU_* 定义: PA8=TX, PA9=RX) ---- */
#define IMU_UART_TX_PORT                GPIO_IMU_TX_PORT
#define IMU_UART_TX_PIN                 GPIO_IMU_TX_PIN
#define IMU_UART_TX_IOMUX               GPIO_IMU_IOMUX_TX
#define IMU_UART_TX_IOMUX_FUNC          GPIO_IMU_IOMUX_TX_FUNC

#define IMU_UART_RX_PORT                GPIO_IMU_RX_PORT
#define IMU_UART_RX_PIN                 GPIO_IMU_RX_PIN
#define IMU_UART_RX_IOMUX               GPIO_IMU_IOMUX_RX
#define IMU_UART_RX_IOMUX_FUNC          GPIO_IMU_IOMUX_RX_FUNC

/* ---- 波特率 (来自 SysConfig, 默认 115200 bps) ---- */
#define IMU_UART_CLOCK_FREQ_HZ          IMU_INST_FREQUENCY
#define IMU_UART_BAUD_RATE              IMU_BAUD_RATE
#define IMU_UART_IBRD                   IMU_IBRD_8_MHZ_115200_BAUD
#define IMU_UART_FBRD                   IMU_FBRD_8_MHZ_115200_BAUD

/* ---- 环形缓冲区大小 ---- */
#define IMU_UART_RX_BUF_SIZE            (256U)

/*===========================================================================
 * 协议常量 / Protocol Constants
 *===========================================================================*/

#define FRAME_HEAD1                     (0x7EU)
#define FRAME_HEAD2                     (0x23U)

/* ---- 功能码 / Function Codes ---- */
#define IMU_FUNC_VERSION                (0x01U)  /* 固件版本 */
#define IMU_FUNC_RAW_ACCEL              (0x04U)  /* 原始加速度+陀螺仪+磁力计 */
#define IMU_FUNC_RAW_GYRO               (0x0AU)  /* 原始陀螺仪 */
#define IMU_FUNC_RAW_MAG                (0x10U)  /* 原始磁力计 */
#define IMU_FUNC_QUAT                   (0x16U)  /* 四元数 */
#define IMU_FUNC_EULER                  (0x26U)  /* 欧拉角 */
#define IMU_FUNC_BARO                   (0x32U)  /* 气压计(10轴) */
#define IMU_FUNC_SET_FREQ               (0x60U)  /* 设置输出频率 */
#define IMU_FUNC_SET_ALGO               (0x61U)  /* 设置算法类型 */
#define IMU_FUNC_CALIB_IMU              (0x70U)  /* 校准IMU */
#define IMU_FUNC_CALIB_MAG              (0x71U)  /* 校准磁力计 */
#define IMU_FUNC_CALIB_BARO             (0x72U)  /* 校准气压计 */
#define IMU_FUNC_CALIB_TEMP             (0x73U)  /* 校准温度 */
#define IMU_FUNC_REQUEST_DATA           (0x80U)  /* 请求数据 */
#define IMU_FUNC_RETURN_STATE           (0x81U)  /* 校准返回状态 */
#define IMU_FUNC_RESET_FLASH            (0xA0U)  /* 复位用户数据 */

/*===========================================================================
 * 数据结构 / Data Structures
 *===========================================================================*/

/** 一次性获取所有传感器数据 */
typedef struct {
    float accel[3];   /**< 加速度 [x,y,z] 单位: g     */
    float gyro[3];    /**< 角速度 [x,y,z] 单位: rad/s */
    float mag[3];     /**< 磁力计 [x,y,z] 单位: uT    */
    float quat[4];    /**< 四元数 [w,x,y,z]           */
    float euler[3];   /**< 欧拉角 [roll,pitch,yaw] 单位: 度 */
    float baro[4];    /**< 气压计 [高度(m),温度(C),气压(Pa),参考气压(Pa)] */
    char  version[8]; /**< 固件版本字符串 "X.Y.Z"     */
} imu_measurement_t;

/*===========================================================================
 * 公共 API / Public API
 *===========================================================================*/

/**
 * @brief IMU 硬件初始化 (UART0 + 引脚复用 + NVIC)
 * @note  必须在 SYSCFG_DL_init() 之后调用
 *        会覆盖 PA8/PA9 的 GPIO 配置为 UART 功能
 */
void IMU_Init(void);

/**
 * @brief IMU 数据轮询处理 (消费环形缓冲区，解析帧)
 * @note  应在调度器任务中周期性调用 (推荐 20 ms)
 *        传感器以 25 Hz 自动上报，20ms 周期确保不丢帧
 */
void IMU_Process(void);

/**
 * @brief 向 IMU 发送命令帧
 * @param function  功能码
 * @param params    参数数组 (可为 NULL)
 * @param param_len 参数长度 (0~3)
 * @return 0=成功, -1=参数错误
 * @note  阻塞发送 (轮询 TX)，单帧 < 1 ms
 */
int  IMU_SendCommand(uint8_t function, const uint8_t *params, uint8_t param_len);

/* ---- 传感器数据读取 (非阻塞，读取最近一次自动上报的缓存值) ---- */
int  IMU_GetAccelerometer(float out[3]);
int  IMU_GetGyroscope(float out[3]);
int  IMU_GetMagnetometer(float out[3]);
int  IMU_GetQuaternion(float out[4]);
int  IMU_GetEuler(float out[3]);       /**< 返回角度制 (度)，非弧度 */
int  IMU_GetBarometer(float out[4]);
int  IMU_GetAll(imu_measurement_t *out);

/** 调试: 获取 ISR 已接收的总字节数 */
uint32_t IMU_GetRxByteCount(void);

/** 调试: 抓取环形缓冲区中最近的原始字节 (用于诊断帧格式) */
uint16_t IMU_DumpRawBytes(uint8_t *buf, uint16_t max_len);

/** 清除自动上报缓存数据 (全部置零) */
void IMU_ClearData(void);

/* ---- 校准 API (阻塞，不可在调度器任务中调用) ---- */
int  IMU_CalibrateImu(void);           /**< 校准IMU (最多等待7秒) */
int  IMU_CalibrateMag(void);           /**< 校准磁力计 */
int  IMU_CalibrateTemp(float current_temperature); /**< 校准温度 */
int  IMU_ResetUserData(void);          /**< 复位用户校准数据 */

#ifdef __cplusplus
}
#endif

#endif /* IMU_UART_DRIVER_H */
