#ifndef CIRCUIT_ID_H
#define CIRCUIT_ID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc_sampler.h"

/*
 * 黑盒电路频响辨识（ADC 采样版）。
 * 频率由 FPGA 经 SPI 给出，Ui、Uo 由两路 ADC 采集，在该频率上做单频 DFT。
 * 前端是跨阻 I/V 放大器，运放反相端近似虚地，反馈电阻为 R：
 *   I = Ui * Y       Uo = -I * R
 *   H = Uo / Ui      Y  = -H / R
 *   Z = 1 / Y = -R / H
 *
 * 注意：两路前端的增益不同会使 |H| 有固定偏差，目前尚未标定。
 */

/* 1：计算阻抗和导纳；0：只计算传递函数。 */
#define CIR_DUT_TOPOLOGY            1u

/* 跨阻反馈电阻 R，单位 Ω。 */
#define CIR_FEEDBACK_RESISTOR_OHM   1000.0f

/* 单频幅值下限，单位：ADC 码值（2 * |S| / N）。低于此值视为噪声，待实测确定。 */
#define CIR_INPUT_MIN_AMP           4.0f
#define CIR_OUTPUT_MIN_AMP          4.0f

/* |H| 下限：低于此值阻抗趋于无穷，结果不可靠。 */
#define CIR_H_MIN_MAG               1e-6f

typedef struct
{
    float frequency_hz;
    float h_magnitude_dB;
    float h_phase_deg;

#if CIR_DUT_TOPOLOGY == 1u
    float z_mag_ohm;
    float z_mag_dB;
    float z_phase_deg;
    float y_mag_siemens;
    float y_phase_deg;
#endif

    uint8_t valid;
} FRF_Point_t;

/* 初始化频响辨识模块。当前无状态需要初始化。 */
void CircuitID_Init(void);

/* 由一个窗口的 Ui、Uo 计算 H、Z、Y；成功且结果有效时返回 1，否则返回 0。 */
uint8_t CircuitID_Compute(const ADC_Window_t *input, FRF_Point_t *output);

#ifdef __cplusplus
}
#endif

#endif /* CIRCUIT_ID_H */
