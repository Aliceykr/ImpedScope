#ifndef CIRCUIT_ID_H
#define CIRCUIT_ID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi_receiver.h"

/*
 * 黑盒电路频响辨识。
 *
 * 电路原理：
 *   Ui 加在被测电路（DUT）两端，流过 DUT 的电流为 I = Ui * Y。
 *   运放反相端接近 0 V（虚地），电流全部流过反馈电阻 R，
 *   所以输出 Uo = -I * R。
 *
 * 由此推出：
 *   H = Uo / Ui      传递函数（复数）
 *   Y = -H / R       导纳
 *   Z = 1 / Y = -R / H   阻抗
 */

/* 1：freq 是实际频率；0：freq 是 FFT bin。 */
#define CIR_FREQ_IS_RAW             1u
#define CIR_FREQ_UNIT               0u  /* 0：Hz，1：kHz，2：MHz */
#define CIR_SAMPLE_RATE_HZ          1000000.0f
#define CIR_FFT_POINTS              1024u

/* 1：计算阻抗和导纳；0：只计算传递函数。 */
#define CIR_DUT_TOPOLOGY            1u
#define CIR_FEEDBACK_RESISTOR_OHM   1000.0f

/* 预留给多帧平均功能；当前按单帧扫频处理。 */
#define CIR_EXCITATION              0u
#define CIR_AVERAGE_FRAMES          1u

/* 输入和输出幅值下限，低于下限的频点标记为无效。 */
#define CIR_INPUT_MIN_MAG           1e-4f
#define CIR_OUTPUT_MIN_MAG          1e-6f

/* |H| 下限：低于此值阻抗趋于无穷、结果不可靠。与输出幅值门限独立调参。 */
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

/* 计算一个频点；成功且结果有效时返回 1，否则返回 0。 */
uint8_t CircuitID_Compute(const FFT_Point_t *input, FRF_Point_t *output);

#ifdef __cplusplus
}
#endif

#endif /* CIRCUIT_ID_H */
