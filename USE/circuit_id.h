#ifndef CIRCUIT_ID_H
#define CIRCUIT_ID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi_receiver.h"

/*
 * 黑盒电路频响辨识。
 * 前端是跨阻 I/V 放大器，运放反相端近似虚地，反馈电阻为 R：
 *   I = Ui * Y       Uo = -I * R
 *   H = Uo / Ui      Y  = -H / R
 *   Z = 1 / Y = -R / H
 */

/* FPGA FFT 参数：f = bin * fs / N。
 * fs 按 adc0_clk = clk_40m 推断为 40 MSPS，待硬件实测确认。 */
#define CIR_SAMPLE_RATE_HZ          40000000.0f
#define CIR_FFT_POINTS              1024u

/* 12 位定点 FFT 结果到电压的比例系数，待结合 FPGA scaled 配置校准。
 * 算 H 时两路同缩放会约掉；算 |Z| 绝对值时必须已知。 */
#define CIR_FFT_SCALE               1.0f

/* 1：计算阻抗和导纳；0：只计算传递函数。 */
#define CIR_DUT_TOPOLOGY            1u
#define CIR_FEEDBACK_RESISTOR_OHM   1000.0f

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

/* 计算一条 FPGA 记录；成功且结果有效时返回 1，否则返回 0。 */
uint8_t CircuitID_Compute(const FPGA_Record_t *input, FRF_Point_t *output);

#ifdef __cplusplus
}
#endif

#endif /* CIRCUIT_ID_H */
