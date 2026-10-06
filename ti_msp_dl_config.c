/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gMotor_PWM_BackBackup;
DL_TimerG_backupConfig gTIMER_0Backup;
DL_UART_Main_backupConfig gLORABackup;
DL_SPI_backupConfig gLCD_DATABackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_Motor_PWM_Front_init();
    SYSCFG_DL_Motor_PWM_Back_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_I2C_0_init();
    SYSCFG_DL_I2C_TIME_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_UART_2_init();
    SYSCFG_DL_LORA_init();
    SYSCFG_DL_IMU_init();
    SYSCFG_DL_LCD_DATA_init();
    SYSCFG_DL_ADC_VOLTAGE_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gMotor_PWM_BackBackup.backupRdy 	= false;
	gTIMER_0Backup.backupRdy 	= false;
	gLORABackup.backupRdy 	= false;
	gLCD_DATABackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(Motor_PWM_Back_INST, &gMotor_PWM_BackBackup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_0_INST, &gTIMER_0Backup);
	retStatus &= DL_UART_Main_saveConfiguration(LORA_INST, &gLORABackup);
	retStatus &= DL_SPI_saveConfiguration(LCD_DATA_INST, &gLCD_DATABackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(Motor_PWM_Back_INST, &gMotor_PWM_BackBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_0_INST, &gTIMER_0Backup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(LORA_INST, &gLORABackup);
	retStatus &= DL_SPI_restoreConfiguration(LCD_DATA_INST, &gLCD_DATABackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(Motor_PWM_Front_INST);
    DL_TimerG_reset(Motor_PWM_Back_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_I2C_reset(I2C_0_INST);
    DL_I2C_reset(I2C_TIME_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_reset(UART_2_INST);
    DL_UART_Main_reset(LORA_INST);
    DL_UART_Main_reset(IMU_INST);
    DL_SPI_reset(LCD_DATA_INST);
    DL_ADC12_reset(ADC_VOLTAGE_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(Motor_PWM_Front_INST);
    DL_TimerG_enablePower(Motor_PWM_Back_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_I2C_enablePower(I2C_0_INST);
    DL_I2C_enablePower(I2C_TIME_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_UART_Main_enablePower(UART_2_INST);
    DL_UART_Main_enablePower(LORA_INST);
    DL_UART_Main_enablePower(IMU_INST);
    DL_SPI_enablePower(LCD_DATA_INST);
    DL_ADC12_enablePower(ADC_VOLTAGE_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_PWM_Front_C0_IOMUX,GPIO_Motor_PWM_Front_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_PWM_Front_C0_PORT, GPIO_Motor_PWM_Front_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_PWM_Front_C1_IOMUX,GPIO_Motor_PWM_Front_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_PWM_Front_C1_PORT, GPIO_Motor_PWM_Front_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_PWM_Back_C0_IOMUX,GPIO_Motor_PWM_Back_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_PWM_Back_C0_PORT, GPIO_Motor_PWM_Back_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_PWM_Back_C1_IOMUX,GPIO_Motor_PWM_Back_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_PWM_Back_C1_PORT, GPIO_Motor_PWM_Back_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_TIME_IOMUX_SDA,
        GPIO_I2C_TIME_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_TIME_IOMUX_SCL,
        GPIO_I2C_TIME_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_TIME_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_TIME_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_2_IOMUX_TX, GPIO_UART_2_IOMUX_TX_FUNC);
    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_UART_2_IOMUX_RX, GPIO_UART_2_IOMUX_RX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_LORA_IOMUX_TX, GPIO_LORA_IOMUX_TX_FUNC);
    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_LORA_IOMUX_RX, GPIO_LORA_IOMUX_RX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_IMU_IOMUX_TX, GPIO_IMU_IOMUX_TX_FUNC);
    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_IMU_IOMUX_RX, GPIO_IMU_IOMUX_RX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_LCD_DATA_IOMUX_SCLK, GPIO_LCD_DATA_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_LCD_DATA_IOMUX_PICO, GPIO_LCD_DATA_IOMUX_PICO_FUNC);

    DL_GPIO_initDigitalInputFeatures(HCO5_STATE_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(KEY1_UP_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(LCD_LCD_RES_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_LCD_DC_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_LCD_CS_IOMUX);

    DL_GPIO_initDigitalOutput(LCD_BLK_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_BIN2_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_CIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_CIN2_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_DIN1_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_DIN2_IOMUX);

    DL_GPIO_initDigitalOutput(Motor_STBY_IOMUX);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E1B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E2A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E2B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E3A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E3B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E4A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E4B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_E1A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(TW_TW_0_IOMUX);

    DL_GPIO_initDigitalOutput(TW_TW_1_IOMUX);

    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_14_EDGE_RISE |
		DL_GPIO_PIN_15_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_26_EDGE_RISE |
		DL_GPIO_PIN_31_EDGE_RISE |
		DL_GPIO_PIN_28_EDGE_RISE |
		DL_GPIO_PIN_24_EDGE_RISE |
		DL_GPIO_PIN_25_EDGE_RISE |
		DL_GPIO_PIN_21_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_E1B_PIN |
		ENCODER_E2A_PIN |
		ENCODER_E2B_PIN |
		ENCODER_E3A_PIN |
		ENCODER_E3B_PIN |
		ENCODER_E4A_PIN |
		ENCODER_E4B_PIN |
		ENCODER_E1A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER_E1B_PIN |
		ENCODER_E2A_PIN |
		ENCODER_E2B_PIN |
		ENCODER_E3A_PIN |
		ENCODER_E3B_PIN |
		ENCODER_E4A_PIN |
		ENCODER_E4B_PIN |
		ENCODER_E1A_PIN);
    DL_GPIO_clearPins(GPIOB, LCD_LCD_RES_PIN |
		LCD_LCD_DC_PIN |
		LCD_LCD_CS_PIN |
		LCD_BLK_PIN |
		Motor_AIN1_PIN |
		Motor_AIN2_PIN |
		Motor_BIN1_PIN |
		Motor_BIN2_PIN |
		Motor_CIN1_PIN |
		Motor_CIN2_PIN |
		Motor_DIN1_PIN |
		Motor_DIN2_PIN |
		Motor_STBY_PIN |
		TW_TW_0_PIN |
		TW_TW_1_PIN);
    DL_GPIO_enableOutput(GPIOB, LCD_LCD_RES_PIN |
		LCD_LCD_DC_PIN |
		LCD_LCD_CS_PIN |
		LCD_BLK_PIN |
		Motor_AIN1_PIN |
		Motor_AIN2_PIN |
		Motor_BIN1_PIN |
		Motor_BIN2_PIN |
		Motor_CIN1_PIN |
		Motor_CIN2_PIN |
		Motor_DIN1_PIN |
		Motor_DIN2_PIN |
		Motor_STBY_PIN |
		TW_TW_0_PIN |
		TW_TW_1_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 4,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);

}


/*
 * Timer clock configuration to be sourced by  / 4 (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 10000000 Hz / (4 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gMotor_PWM_FrontClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gMotor_PWM_FrontConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_PWM_Front_init(void) {

    DL_TimerG_setClockConfig(
        Motor_PWM_Front_INST, (DL_TimerG_ClockConfig *) &gMotor_PWM_FrontClockConfig);

    DL_TimerG_initPWMMode(
        Motor_PWM_Front_INST, (DL_TimerG_PWMConfig *) &gMotor_PWM_FrontConfig);

    DL_TimerG_setCaptureCompareOutCtl(Motor_PWM_Front_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_ENABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(Motor_PWM_Front_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(Motor_PWM_Front_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_ENABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(Motor_PWM_Front_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Front_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(Motor_PWM_Front_INST);


    
    DL_TimerG_setCCPDirection(Motor_PWM_Front_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 4 (20000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   20000000 Hz = 20000000 Hz / (4 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gMotor_PWM_BackClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gMotor_PWM_BackConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_PWM_Back_init(void) {

    DL_TimerG_setClockConfig(
        Motor_PWM_Back_INST, (DL_TimerG_ClockConfig *) &gMotor_PWM_BackClockConfig);

    DL_TimerG_initPWMMode(
        Motor_PWM_Back_INST, (DL_TimerG_PWMConfig *) &gMotor_PWM_BackConfig);

    DL_TimerG_setCaptureCompareOutCtl(Motor_PWM_Back_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_ENABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(Motor_PWM_Back_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(Motor_PWM_Back_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_ENABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(Motor_PWM_Back_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(Motor_PWM_Back_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(Motor_PWM_Back_INST);


    
    DL_TimerG_setCCPDirection(Motor_PWM_Back_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   10000000 Hz = 10000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (1 ms * 10000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMER_0_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMER_0_INST);





}


static const DL_I2C_ClockConfig gI2C_0ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_0_init(void) {

    DL_I2C_setClockConfig(I2C_0_INST,
        (DL_I2C_ClockConfig *) &gI2C_0ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_0_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_0_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(I2C_0_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_0_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_0_INST);


}
static const DL_I2C_ClockConfig gI2C_TIMEClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_TIME_init(void) {

    DL_I2C_setClockConfig(I2C_TIME_INST,
        (DL_I2C_ClockConfig *) &gI2C_TIMEClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_TIME_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_TIME_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_TIME_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(I2C_TIME_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(I2C_TIME_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_TIME_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_TIME_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_TIME_INST);


}


static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_5
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115107.91
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_8_MHZ_115200_BAUD, UART_0_FBRD_8_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_0_INST);
}

static const DL_UART_Main_ClockConfig gUART_2ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_5
};

static const DL_UART_Main_Config gUART_2Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_2_init(void)
{
    DL_UART_Main_setClockConfig(UART_2_INST, (DL_UART_Main_ClockConfig *) &gUART_2ClockConfig);

    DL_UART_Main_init(UART_2_INST, (DL_UART_Main_Config *) &gUART_2Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115107.91
     */
    DL_UART_Main_setOversampling(UART_2_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_2_INST, UART_2_IBRD_8_MHZ_115200_BAUD, UART_2_FBRD_8_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_2_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_2_INST);
}

static const DL_UART_Main_ClockConfig gLORAClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_5
};

static const DL_UART_Main_Config gLORAConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_LORA_init(void)
{
    DL_UART_Main_setClockConfig(LORA_INST, (DL_UART_Main_ClockConfig *) &gLORAClockConfig);

    DL_UART_Main_init(LORA_INST, (DL_UART_Main_Config *) &gLORAConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9599.52
     */
    DL_UART_Main_setOversampling(LORA_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(LORA_INST, LORA_IBRD_16_MHZ_9600_BAUD, LORA_FBRD_16_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(LORA_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(LORA_INST);
}

static const DL_UART_Main_ClockConfig gIMUClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_5
};

static const DL_UART_Main_Config gIMUConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_IMU_init(void)
{
    DL_UART_Main_setClockConfig(IMU_INST, (DL_UART_Main_ClockConfig *) &gIMUClockConfig);

    DL_UART_Main_init(IMU_INST, (DL_UART_Main_Config *) &gIMUConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115107.91
     */
    DL_UART_Main_setOversampling(IMU_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(IMU_INST, IMU_IBRD_8_MHZ_115200_BAUD, IMU_FBRD_8_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(IMU_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(IMU_INST);
}

static const DL_SPI_Config gLCD_DATA_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gLCD_DATA_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_LCD_DATA_init(void) {
    DL_SPI_setClockConfig(LCD_DATA_INST, (DL_SPI_ClockConfig *) &gLCD_DATA_clockConfig);

    DL_SPI_init(LCD_DATA_INST, (DL_SPI_Config *) &gLCD_DATA_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     40000000 = (80000000)/((1 + 0) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(LCD_DATA_INST, 0);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(LCD_DATA_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(LCD_DATA_INST);
}

/* ADC_VOLTAGE Initialization */
static const DL_ADC12_ClockConfig gADC_VOLTAGEClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC_VOLTAGE_init(void)
{
    DL_ADC12_setClockConfig(ADC_VOLTAGE_INST, (DL_ADC12_ClockConfig *) &gADC_VOLTAGEClockConfig);
    DL_ADC12_initSingleSample(ADC_VOLTAGE_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_ADC_CH0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC_VOLTAGE_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC_VOLTAGE_INST,40000);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC_VOLTAGE_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableInterrupt(ADC_VOLTAGE_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableConversions(ADC_VOLTAGE_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /* Initialize the period to 1.00 μs */
    DL_SYSTICK_init(80);
    /* Enable the SysTick and start counting */
    DL_SYSTICK_enable();
}

