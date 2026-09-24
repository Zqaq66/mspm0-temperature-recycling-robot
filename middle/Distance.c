#include "ti_msp_dl_config.h"
#include "Distance.h"
//编码器分辨率,线数
#define RESOLUTION 13
//经过倍频之后的分辨率
#define RESOLUTION_TOTAL 13*4
//减速电机的减速比
#define REDUCTION_RATIO 30
//车轮半径(cm)
#define R 5.25/2
//PI
#define PI 3.14
//获取的脉冲数/(总分辨率*减速比)*周长
int Distance_get(int pulse_increment)
{
	float rotations = (float)pulse_increment/ (RESOLUTION_TOTAL * REDUCTION_RATIO);
    float distance = rotations * 2 * PI * R;  // 单位：cm
    return distance;
}