#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "string.h"
#include "Time.h"
volatile bool gCheckADC;
void ADC_Init(void)
{
	NVIC_ClearPendingIRQ(ADC_VOLTAGE_INST_INT_IRQN);
	NVIC_EnableIRQ(ADC_VOLTAGE_INST_INT_IRQN);
}
//读取ADC的数据
unsigned int adc_getValue(void)
{
	unsigned int gAdcResult = 0;

	//软件触发ADC开始转换
	DL_ADC12_startConversion(ADC_VOLTAGE_INST);
	//如果当前状态为正在转换中则等待转换结束
	while (false == gCheckADC) {
			__WFE();
	}
	//获取数据
	gAdcResult = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_ADC_CH0);

	//清除标志位
	gCheckADC = false;

	return gAdcResult;
}
//测量电池电压
float Get_Voltage(void)
{
	 uint16_t adc_value = 0;
	 float voltage_value=0;
	 adc_value=adc_getValue();
     voltage_value=adc_value/4095.0*3.3;
	 return  voltage_value;
}
//测量电池电量
int Get_Battery(void)
{
	 float voltage_value_tem=0;
	 int  Bat=0;
	 int i=0;
	 for(i=0;i<9;i++)
	{
	 voltage_value_tem +=Get_Voltage()*4*100;
	}
	voltage_value_tem/=9;
	 Bat=(voltage_value_tem-950)/(1200-950)*100;
	if(Bat>100){Bat=100;}
	return Bat;
}

//ADC中断服务函数
void ADC_VOLTAGE_INST_IRQHandler(void)
{
	//查询并清除ADC中断
	switch (DL_ADC12_getPendingInterrupt(ADC_VOLTAGE_INST))
	{
		//检查是否完成数据采集
		case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
			gCheckADC = true;//将标志位置1
			break;
		default:
			break;
	}
}