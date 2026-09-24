/*===========================================================================
 * 编码器采集层 (四路 AB 相, GPIO 中断正交计数)
 *   E1A:PA21  E1B:PA26  (A 左前)      E2A:PA31  E2B:PA28  (B 右前)
 *   E3A:PA24  E3B:PA25  (C 右后)      E4A:PA14  E4B:PA15  (D 左后)
 *   A/B 两相都计入中断 -> 2 倍频; 计数在 GROUP1_IRQHandler 里累加
 *   用法: 20ms 调一次 Encoder_Update() 换出本周期计数, 再读 Motor_Get_Encoder_A~D()
 *   符号宏: ENC_A~D_REVERSE (见下) —— 实测速度符号与 PWM 相反时改 1, 不用拆线
 *===========================================================================*/

#include "ti_msp_dl_config.h"
#include "Encoder.h"

/* ---- 计数方向取反: 某一路实测速度符号与 PWM 符号相反时, 把对应项改成 1 ---- */
#define ENC_A_REVERSE   0
#define ENC_B_REVERSE   1     /* 右前轮 B: 实测计数方向与 PWM 相反, 置1 */
#define ENC_C_REVERSE   1     /* 右后轮 C: 实测计数方向与 PWM 相反, 置1 */
#define ENC_D_REVERSE   0

static volatile Encoder Encoder_A;
static volatile Encoder Encoder_B;
static volatile Encoder Encoder_C;
static volatile Encoder Encoder_D;

void Encoder_Init(void)
{
	NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
	NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

/* 8 个编码器脚的公共中断: 正交计数, 用另一相电平判方向 (照搬原工程的做法) */
void GROUP1_IRQHandler(void)
{
    uint32_t flags;

    flags = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT,
        ENCODER_E1A_PIN|ENCODER_E1B_PIN|ENCODER_E2A_PIN|ENCODER_E2B_PIN|
        ENCODER_E3A_PIN|ENCODER_E3B_PIN|ENCODER_E4A_PIN|ENCODER_E4B_PIN);
    /* 必须清标志, 否则会反复进中断 */
    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_E1A_PIN|ENCODER_E1B_PIN|ENCODER_E2A_PIN|ENCODER_E2B_PIN|
        ENCODER_E3A_PIN|ENCODER_E3B_PIN|ENCODER_E4A_PIN|ENCODER_E4B_PIN);

    /* A 左前轮 */
    if((flags & ENCODER_E1A_PIN)==ENCODER_E1A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E1B_PIN)) Encoder_A.Should_Get_Encoder_Count++;
        else                                               Encoder_A.Should_Get_Encoder_Count--;
    }
    else if((flags & ENCODER_E1B_PIN)==ENCODER_E1B_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E1A_PIN)) Encoder_A.Should_Get_Encoder_Count--;
        else                                               Encoder_A.Should_Get_Encoder_Count++;
    }

    /* B 右前轮 */
    if((flags & ENCODER_E2A_PIN)==ENCODER_E2A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E2B_PIN)) Encoder_B.Should_Get_Encoder_Count++;
        else                                               Encoder_B.Should_Get_Encoder_Count--;
    }
    else if((flags & ENCODER_E2B_PIN)==ENCODER_E2B_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E2A_PIN)) Encoder_B.Should_Get_Encoder_Count--;
        else                                               Encoder_B.Should_Get_Encoder_Count++;
    }

    /* C 右后轮 */
    if((flags & ENCODER_E3A_PIN)==ENCODER_E3A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E3B_PIN)) Encoder_C.Should_Get_Encoder_Count++;
        else                                               Encoder_C.Should_Get_Encoder_Count--;
    }
    else if((flags & ENCODER_E3B_PIN)==ENCODER_E3B_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E3A_PIN)) Encoder_C.Should_Get_Encoder_Count--;
        else                                               Encoder_C.Should_Get_Encoder_Count++;
    }

    /* D 左后轮 */
    if((flags & ENCODER_E4A_PIN)==ENCODER_E4A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E4B_PIN)) Encoder_D.Should_Get_Encoder_Count++;
        else                                               Encoder_D.Should_Get_Encoder_Count--;
    }
    else if((flags & ENCODER_E4B_PIN)==ENCODER_E4B_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_PORT,ENCODER_E4A_PIN)) Encoder_D.Should_Get_Encoder_Count--;
        else                                               Encoder_D.Should_Get_Encoder_Count++;
    }
}

/* 20ms 采样: 换出计数并清零 (在调度器的 20ms 任务里调) */
void Encoder_Update(void)
{
    int a = Encoder_A.Should_Get_Encoder_Count;
    int b = Encoder_B.Should_Get_Encoder_Count;
    int c = Encoder_C.Should_Get_Encoder_Count;
    int d = Encoder_D.Should_Get_Encoder_Count;

    Encoder_A.Obtained_Get_Encoder_Count = ENC_A_REVERSE ? -a : a;
    Encoder_B.Obtained_Get_Encoder_Count = ENC_B_REVERSE ? -b : b;
    Encoder_C.Obtained_Get_Encoder_Count = ENC_C_REVERSE ? -c : c;
    Encoder_D.Obtained_Get_Encoder_Count = ENC_D_REVERSE ? -d : d;

    Encoder_A.Should_Get_Encoder_Count = 0;
    Encoder_B.Should_Get_Encoder_Count = 0;
    Encoder_C.Should_Get_Encoder_Count = 0;
    Encoder_D.Should_Get_Encoder_Count = 0;
}

int Motor_Get_Encoder_A(void) { return Encoder_A.Obtained_Get_Encoder_Count; }
int Motor_Get_Encoder_B(void) { return Encoder_B.Obtained_Get_Encoder_Count; }
int Motor_Get_Encoder_C(void) { return Encoder_C.Obtained_Get_Encoder_Count; }
int Motor_Get_Encoder_D(void) { return Encoder_D.Obtained_Get_Encoder_Count; }
