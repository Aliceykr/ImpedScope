#include "circuit_id.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "screen.h"

#define CIR_PI              3.14159265358979323846f
#define CIR_LOG_GUARD       1e-20f

static uint8_t CircuitID_IsFinite(float value);

void CircuitID_Init(void)
{
    /* 当前为单帧扫频模式，暂时没有运行时状态。 */
}

uint8_t CircuitID_Compute(const FPGA_Record_t *input, FRF_Point_t *output)
{
    float ui_re;
    float ui_im;
    float uo_re;
    float uo_im;
    float ui_magnitude_squared;
    float uo_magnitude_squared;
    float h_re;
    float h_im;
    float h_magnitude_squared;
    float h_magnitude;

    if ((input == (const FPGA_Record_t *)0) ||
        (output == (FRF_Point_t *)0))
    {
        return 0u;
    }

    (void)memset(output, 0, sizeof(*output));

    if (CIR_FEEDBACK_RESISTOR_OHM <= 0.0f)
    {
        Screen_ReportError(SCREEN_E07_CIR_CFG);
        return 0u;
    }

    /* 两通道峰值 bin 必须一致，否则 Ui、Uo 不在同一频点（PLAN 决策 A）。 */
    if (input->index0 != input->index1)
    {
        Screen_ReportError(SCREEN_E17_BIN_MISMATCH);
        return 0u;
    }

    /* 实信号 FFT：bin N/2+1 … N-1 是 1 … N/2-1 的镜像，镜像 bin 分量
     * 虚部反号会使 H 相位反号，故直接拒绝（bin N/2 奈奎斯特保留）。 */
    if (input->index0 > (CIR_FFT_POINTS / 2u))
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        return 0u;
    }

    if (input->index0 == 0u)
    {
        Screen_ReportError(SCREEN_E07_CIR_CFG);
        return 0u;
    }

    /* CH0 = Ui，CH1 = Uo；定点值乘比例系数还原电压。 */
    ui_re = (float)input->ch0_re * CIR_FFT_SCALE;
    ui_im = (float)input->ch0_im * CIR_FFT_SCALE;
    uo_re = (float)input->ch1_re * CIR_FFT_SCALE;
    uo_im = (float)input->ch1_im * CIR_FFT_SCALE;

    if (!CircuitID_IsFinite(ui_re) || !CircuitID_IsFinite(ui_im) ||
        !CircuitID_IsFinite(uo_re) || !CircuitID_IsFinite(uo_im))
    {
        Screen_ReportError(SCREEN_E08_CIR_NAN_IN);
        return 0u;
    }

    ui_magnitude_squared = (ui_re * ui_re) + (ui_im * ui_im);
    if (!CircuitID_IsFinite(ui_magnitude_squared) ||
        ui_magnitude_squared <= (CIR_INPUT_MIN_MAG * CIR_INPUT_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E09_CIR_UI_LOW);
        return 0u;
    }

    uo_magnitude_squared = (uo_re * uo_re) + (uo_im * uo_im);
    if (!CircuitID_IsFinite(uo_magnitude_squared) ||
        uo_magnitude_squared <= (CIR_OUTPUT_MIN_MAG * CIR_OUTPUT_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E10_CIR_UO_LOW);
        return 0u;
    }

    /* H = Uo * conj(Ui) / |Ui|^2。 */
    h_re = ((uo_re * ui_re) + (uo_im * ui_im)) /
           ui_magnitude_squared;
    h_im = ((uo_im * ui_re) - (uo_re * ui_im)) /
           ui_magnitude_squared;

    h_magnitude_squared = (h_re * h_re) + (h_im * h_im);
    if (!CircuitID_IsFinite(h_re) || !CircuitID_IsFinite(h_im) ||
        !CircuitID_IsFinite(h_magnitude_squared))
    {
        Screen_ReportError(SCREEN_E11_CIR_H_NAN);
        return 0u;
    }

    h_magnitude = sqrtf(h_magnitude_squared);

    output->frequency_hz = ((float)input->index0 * CIR_SAMPLE_RATE_HZ) /
                           (float)CIR_FFT_POINTS;
    output->h_magnitude_dB = 20.0f * log10f(h_magnitude + CIR_LOG_GUARD);
    output->h_phase_deg = atan2f(h_im, h_re) * 180.0f / CIR_PI;

    if (!CircuitID_IsFinite(output->frequency_hz) ||
        !CircuitID_IsFinite(output->h_magnitude_dB) ||
        !CircuitID_IsFinite(output->h_phase_deg))
    {
        Screen_ReportError(SCREEN_E12_CIR_OUT_NAN);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

#if CIR_DUT_TOPOLOGY == 1u
    /* H 过小表示阻抗趋于无穷，结果不可靠。 */
    if (h_magnitude_squared <= (CIR_H_MIN_MAG * CIR_H_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E13_CIR_H_LOW);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

    {
        float z_re;
        float z_im;
        float z_magnitude;

        /* Z = -R / H = -R * conj(H) / |H|^2
         *     = (-R*h_re + j*R*h_im) / |H|^2。 */
        z_re = -(CIR_FEEDBACK_RESISTOR_OHM * h_re) /
               h_magnitude_squared;
        z_im = (CIR_FEEDBACK_RESISTOR_OHM * h_im) /
               h_magnitude_squared;
        z_magnitude = CIR_FEEDBACK_RESISTOR_OHM / h_magnitude;

        output->z_mag_ohm = z_magnitude;
        output->z_mag_dB = 20.0f * log10f(z_magnitude + CIR_LOG_GUARD);
        output->z_phase_deg = atan2f(z_im, z_re) * 180.0f / CIR_PI;
    }

    {
        float y_re;
        float y_im;
        float y_magnitude;

        /* Y = -H / R。 */
        y_re = -h_re / CIR_FEEDBACK_RESISTOR_OHM;
        y_im = -h_im / CIR_FEEDBACK_RESISTOR_OHM;
        y_magnitude = h_magnitude / CIR_FEEDBACK_RESISTOR_OHM;

        output->y_mag_siemens = y_magnitude;
        output->y_phase_deg = atan2f(y_im, y_re) * 180.0f / CIR_PI;
    }

    if (!CircuitID_IsFinite(output->z_mag_ohm) ||
        !CircuitID_IsFinite(output->z_mag_dB) ||
        !CircuitID_IsFinite(output->z_phase_deg) ||
        !CircuitID_IsFinite(output->y_mag_siemens) ||
        !CircuitID_IsFinite(output->y_phase_deg))
    {
        Screen_ReportError(SCREEN_E14_CIR_ZY_NAN);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }
#endif

    output->valid = 1u;
    return 1u;
}

static uint8_t CircuitID_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}
