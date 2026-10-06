/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for Motor_PWM_Front */
#define Motor_PWM_Front_INST                                               TIMG0
#define Motor_PWM_Front_INST_IRQHandler                         TIMG0_IRQHandler
#define Motor_PWM_Front_INST_INT_IRQN                           (TIMG0_INT_IRQn)
#define Motor_PWM_Front_INST_CLK_FREQ                                   10000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_PWM_Front_C0_PORT                                       GPIOA
#define GPIO_Motor_PWM_Front_C0_PIN                               DL_GPIO_PIN_12
#define GPIO_Motor_PWM_Front_C0_IOMUX                            (IOMUX_PINCM34)
#define GPIO_Motor_PWM_Front_C0_IOMUX_FUNC             IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_Motor_PWM_Front_C0_IDX                          DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_PWM_Front_C1_PORT                                       GPIOA
#define GPIO_Motor_PWM_Front_C1_PIN                               DL_GPIO_PIN_13
#define GPIO_Motor_PWM_Front_C1_IOMUX                            (IOMUX_PINCM35)
#define GPIO_Motor_PWM_Front_C1_IOMUX_FUNC             IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_Motor_PWM_Front_C1_IDX                          DL_TIMER_CC_1_INDEX

/* Defines for Motor_PWM_Back */
#define Motor_PWM_Back_INST                                                TIMG7
#define Motor_PWM_Back_INST_IRQHandler                          TIMG7_IRQHandler
#define Motor_PWM_Back_INST_INT_IRQN                            (TIMG7_INT_IRQn)
#define Motor_PWM_Back_INST_CLK_FREQ                                    20000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_PWM_Back_C0_PORT                                        GPIOA
#define GPIO_Motor_PWM_Back_C0_PIN                                DL_GPIO_PIN_17
#define GPIO_Motor_PWM_Back_C0_IOMUX                             (IOMUX_PINCM39)
#define GPIO_Motor_PWM_Back_C0_IOMUX_FUNC             IOMUX_PINCM39_PF_TIMG7_CCP0
#define GPIO_Motor_PWM_Back_C0_IDX                           DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_PWM_Back_C1_PORT                                        GPIOA
#define GPIO_Motor_PWM_Back_C1_PIN                                 DL_GPIO_PIN_7
#define GPIO_Motor_PWM_Back_C1_IOMUX                             (IOMUX_PINCM14)
#define GPIO_Motor_PWM_Back_C1_IOMUX_FUNC             IOMUX_PINCM14_PF_TIMG7_CCP1
#define GPIO_Motor_PWM_Back_C1_IDX                           DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG6)
#define TIMER_0_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (9999U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for I2C_TIME */
#define I2C_TIME_INST                                                       I2C1
#define I2C_TIME_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_TIME_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_TIME_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_TIME_SDA_PORT                                             GPIOA
#define GPIO_I2C_TIME_SDA_PIN                                     DL_GPIO_PIN_30
#define GPIO_I2C_TIME_IOMUX_SDA                                   (IOMUX_PINCM5)
#define GPIO_I2C_TIME_IOMUX_SDA_FUNC                    IOMUX_PINCM5_PF_I2C1_SDA
#define GPIO_I2C_TIME_SCL_PORT                                             GPIOA
#define GPIO_I2C_TIME_SCL_PIN                                     DL_GPIO_PIN_29
#define GPIO_I2C_TIME_IOMUX_SCL                                   (IOMUX_PINCM4)
#define GPIO_I2C_TIME_IOMUX_SCL_FUNC                    IOMUX_PINCM4_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                            8000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_8_MHZ_115200_BAUD                                        (4)
#define UART_0_FBRD_8_MHZ_115200_BAUD                                       (22)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                            8000000
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOB
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_16
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_15
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM33)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM32)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM33_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM32_PF_UART2_TX
#define UART_2_BAUD_RATE                                                (115200)
#define UART_2_IBRD_8_MHZ_115200_BAUD                                        (4)
#define UART_2_FBRD_8_MHZ_115200_BAUD                                       (22)
/* Defines for LORA */
#define LORA_INST                                                          UART3
#define LORA_INST_FREQUENCY                                             16000000
#define LORA_INST_IRQHandler                                    UART3_IRQHandler
#define LORA_INST_INT_IRQN                                        UART3_INT_IRQn
#define GPIO_LORA_RX_PORT                                                  GPIOB
#define GPIO_LORA_TX_PORT                                                  GPIOB
#define GPIO_LORA_RX_PIN                                           DL_GPIO_PIN_3
#define GPIO_LORA_TX_PIN                                           DL_GPIO_PIN_2
#define GPIO_LORA_IOMUX_RX                                       (IOMUX_PINCM16)
#define GPIO_LORA_IOMUX_TX                                       (IOMUX_PINCM15)
#define GPIO_LORA_IOMUX_RX_FUNC                        IOMUX_PINCM16_PF_UART3_RX
#define GPIO_LORA_IOMUX_TX_FUNC                        IOMUX_PINCM15_PF_UART3_TX
#define LORA_BAUD_RATE                                                    (9600)
#define LORA_IBRD_16_MHZ_9600_BAUD                                         (104)
#define LORA_FBRD_16_MHZ_9600_BAUD                                          (11)
/* Defines for IMU */
#define IMU_INST                                                           UART1
#define IMU_INST_FREQUENCY                                               8000000
#define IMU_INST_IRQHandler                                     UART1_IRQHandler
#define IMU_INST_INT_IRQN                                         UART1_INT_IRQn
#define GPIO_IMU_RX_PORT                                                   GPIOA
#define GPIO_IMU_TX_PORT                                                   GPIOA
#define GPIO_IMU_RX_PIN                                            DL_GPIO_PIN_9
#define GPIO_IMU_TX_PIN                                            DL_GPIO_PIN_8
#define GPIO_IMU_IOMUX_RX                                        (IOMUX_PINCM20)
#define GPIO_IMU_IOMUX_TX                                        (IOMUX_PINCM19)
#define GPIO_IMU_IOMUX_RX_FUNC                         IOMUX_PINCM20_PF_UART1_RX
#define GPIO_IMU_IOMUX_TX_FUNC                         IOMUX_PINCM19_PF_UART1_TX
#define IMU_BAUD_RATE                                                   (115200)
#define IMU_IBRD_8_MHZ_115200_BAUD                                           (4)
#define IMU_FBRD_8_MHZ_115200_BAUD                                          (22)




/* Defines for LCD_DATA */
#define LCD_DATA_INST                                                      SPI1
#define LCD_DATA_INST_IRQHandler                                SPI1_IRQHandler
#define LCD_DATA_INST_INT_IRQN                                    SPI1_INT_IRQn
#define GPIO_LCD_DATA_PICO_PORT                                           GPIOB
#define GPIO_LCD_DATA_PICO_PIN                                    DL_GPIO_PIN_8
#define GPIO_LCD_DATA_IOMUX_PICO                                (IOMUX_PINCM25)
#define GPIO_LCD_DATA_IOMUX_PICO_FUNC                IOMUX_PINCM25_PF_SPI1_PICO
/* GPIO configuration for LCD_DATA */
#define GPIO_LCD_DATA_SCLK_PORT                                           GPIOB
#define GPIO_LCD_DATA_SCLK_PIN                                    DL_GPIO_PIN_9
#define GPIO_LCD_DATA_IOMUX_SCLK                                (IOMUX_PINCM26)
#define GPIO_LCD_DATA_IOMUX_SCLK_FUNC                IOMUX_PINCM26_PF_SPI1_SCLK



/* Defines for ADC_VOLTAGE */
#define ADC_VOLTAGE_INST                                                    ADC0
#define ADC_VOLTAGE_INST_IRQHandler                              ADC0_IRQHandler
#define ADC_VOLTAGE_INST_INT_IRQN                                (ADC0_INT_IRQn)
#define ADC_VOLTAGE_ADCMEM_ADC_CH0                            DL_ADC12_MEM_IDX_0
#define ADC_VOLTAGE_ADCMEM_ADC_CH0_REF           DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_VOLTAGE_ADCMEM_ADC_CH0_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC_VOLTAGE_C0_PORT                                           GPIOA
#define GPIO_ADC_VOLTAGE_C0_PIN                                   DL_GPIO_PIN_27



/* Port definition for Pin Group HCO5 */
#define HCO5_PORT                                                        (GPIOA)

/* Defines for STATE: GPIOA.16 with pinCMx 38 on package pin 9 */
#define HCO5_STATE_PIN                                          (DL_GPIO_PIN_16)
#define HCO5_STATE_IOMUX                                         (IOMUX_PINCM38)
/* Port definition for Pin Group KEY1 */
#define KEY1_PORT                                                        (GPIOB)

/* Defines for UP: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY1_UP_PIN                                             (DL_GPIO_PIN_21)
#define KEY1_UP_IOMUX                                            (IOMUX_PINCM49)
/* Port definition for Pin Group LCD */
#define LCD_PORT                                                         (GPIOB)

/* Defines for LCD_RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LCD_LCD_RES_PIN                                         (DL_GPIO_PIN_10)
#define LCD_LCD_RES_IOMUX                                        (IOMUX_PINCM27)
/* Defines for LCD_DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define LCD_LCD_DC_PIN                                          (DL_GPIO_PIN_11)
#define LCD_LCD_DC_IOMUX                                         (IOMUX_PINCM28)
/* Defines for LCD_CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LCD_LCD_CS_PIN                                          (DL_GPIO_PIN_14)
#define LCD_LCD_CS_IOMUX                                         (IOMUX_PINCM31)
/* Defines for BLK: GPIOB.26 with pinCMx 57 on package pin 28 */
#define LCD_BLK_PIN                                             (DL_GPIO_PIN_26)
#define LCD_BLK_IOMUX                                            (IOMUX_PINCM57)
/* Port definition for Pin Group Motor */
#define Motor_PORT                                                       (GPIOB)

/* Defines for AIN1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define Motor_AIN1_PIN                                          (DL_GPIO_PIN_19)
#define Motor_AIN1_IOMUX                                         (IOMUX_PINCM45)
/* Defines for AIN2: GPIOB.20 with pinCMx 48 on package pin 19 */
#define Motor_AIN2_PIN                                          (DL_GPIO_PIN_20)
#define Motor_AIN2_IOMUX                                         (IOMUX_PINCM48)
/* Defines for BIN1: GPIOB.4 with pinCMx 17 on package pin 52 */
#define Motor_BIN1_PIN                                           (DL_GPIO_PIN_4)
#define Motor_BIN1_IOMUX                                         (IOMUX_PINCM17)
/* Defines for BIN2: GPIOB.5 with pinCMx 18 on package pin 53 */
#define Motor_BIN2_PIN                                           (DL_GPIO_PIN_5)
#define Motor_BIN2_IOMUX                                         (IOMUX_PINCM18)
/* Defines for CIN1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define Motor_CIN1_PIN                                           (DL_GPIO_PIN_6)
#define Motor_CIN1_IOMUX                                         (IOMUX_PINCM23)
/* Defines for CIN2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define Motor_CIN2_PIN                                           (DL_GPIO_PIN_7)
#define Motor_CIN2_IOMUX                                         (IOMUX_PINCM24)
/* Defines for DIN1: GPIOB.12 with pinCMx 29 on package pin 64 */
#define Motor_DIN1_PIN                                          (DL_GPIO_PIN_12)
#define Motor_DIN1_IOMUX                                         (IOMUX_PINCM29)
/* Defines for DIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define Motor_DIN2_PIN                                          (DL_GPIO_PIN_13)
#define Motor_DIN2_IOMUX                                         (IOMUX_PINCM30)
/* Defines for STBY: GPIOB.0 with pinCMx 12 on package pin 47 */
#define Motor_STBY_PIN                                           (DL_GPIO_PIN_0)
#define Motor_STBY_IOMUX                                         (IOMUX_PINCM12)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOA)

/* Defines for E1B: GPIOA.26 with pinCMx 59 on package pin 30 */
// pins affected by this interrupt request:["E1B","E2A","E2B","E3A","E3B","E4A","E4B","E1A"]
#define ENCODER_INT_IRQN                                        (GPIOA_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODER_E1B_IIDX                                    (DL_GPIO_IIDX_DIO26)
#define ENCODER_E1B_PIN                                         (DL_GPIO_PIN_26)
#define ENCODER_E1B_IOMUX                                        (IOMUX_PINCM59)
/* Defines for E2A: GPIOA.31 with pinCMx 6 on package pin 39 */
#define ENCODER_E2A_IIDX                                    (DL_GPIO_IIDX_DIO31)
#define ENCODER_E2A_PIN                                         (DL_GPIO_PIN_31)
#define ENCODER_E2A_IOMUX                                         (IOMUX_PINCM6)
/* Defines for E2B: GPIOA.28 with pinCMx 3 on package pin 35 */
#define ENCODER_E2B_IIDX                                    (DL_GPIO_IIDX_DIO28)
#define ENCODER_E2B_PIN                                         (DL_GPIO_PIN_28)
#define ENCODER_E2B_IOMUX                                         (IOMUX_PINCM3)
/* Defines for E3A: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ENCODER_E3A_IIDX                                    (DL_GPIO_IIDX_DIO24)
#define ENCODER_E3A_PIN                                         (DL_GPIO_PIN_24)
#define ENCODER_E3A_IOMUX                                        (IOMUX_PINCM54)
/* Defines for E3B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODER_E3B_IIDX                                    (DL_GPIO_IIDX_DIO25)
#define ENCODER_E3B_PIN                                         (DL_GPIO_PIN_25)
#define ENCODER_E3B_IOMUX                                        (IOMUX_PINCM55)
/* Defines for E4A: GPIOA.14 with pinCMx 36 on package pin 7 */
#define ENCODER_E4A_IIDX                                    (DL_GPIO_IIDX_DIO14)
#define ENCODER_E4A_PIN                                         (DL_GPIO_PIN_14)
#define ENCODER_E4A_IOMUX                                        (IOMUX_PINCM36)
/* Defines for E4B: GPIOA.15 with pinCMx 37 on package pin 8 */
#define ENCODER_E4B_IIDX                                    (DL_GPIO_IIDX_DIO15)
#define ENCODER_E4B_PIN                                         (DL_GPIO_PIN_15)
#define ENCODER_E4B_IOMUX                                        (IOMUX_PINCM37)
/* Defines for E1A: GPIOA.21 with pinCMx 46 on package pin 17 */
#define ENCODER_E1A_IIDX                                    (DL_GPIO_IIDX_DIO21)
#define ENCODER_E1A_PIN                                         (DL_GPIO_PIN_21)
#define ENCODER_E1A_IOMUX                                        (IOMUX_PINCM46)
/* Port definition for Pin Group TW */
#define TW_PORT                                                          (GPIOB)

/* Defines for TW_0: GPIOB.17 with pinCMx 43 on package pin 14 */
#define TW_TW_0_PIN                                             (DL_GPIO_PIN_17)
#define TW_TW_0_IOMUX                                            (IOMUX_PINCM43)
/* Defines for TW_1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define TW_TW_1_PIN                                             (DL_GPIO_PIN_18)
#define TW_TW_1_IOMUX                                            (IOMUX_PINCM44)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_Motor_PWM_Front_init(void);
void SYSCFG_DL_Motor_PWM_Back_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_I2C_TIME_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_LORA_init(void);
void SYSCFG_DL_IMU_init(void);
void SYSCFG_DL_LCD_DATA_init(void);
void SYSCFG_DL_ADC_VOLTAGE_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
