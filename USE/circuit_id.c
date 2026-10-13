#include "circuit_id.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "calib.h"
#include "screen.h"

#define CIR_PI              3.14159265358979323846
#define CIR_LOG_GUARD       1e-20

static uint8_t CircuitID_IsFinite(double value);

void CircuitID_Init(void)
{
    /* 当前为单频窗口模式，暂时没有运行时状态。 */
}

uint8_t CircuitID_Compute(const ADC_Window_t *input, FRF_Point_t *output)
{
    double n;
    double ui_mag;
    double uo_mag;
    double ui_mag2;
    double h_re;
    double h_im;
    double h_magnitude_squared;
    double h_magnitude;

    if ((input == (const ADC_Window_t *)0) ||
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

    if ((input->frequency_hz == 0u) || (input->sample_count == 0u))
    {
        Screen_ReportError(SCREEN_E07_CIR_CFG);
        return 0u;
    }

    if (!CircuitID_IsFinite(input->ui_re) || !CircuitID_IsFinite(input->ui_im) ||
        !CircuitID_IsFinite(input->uo_re) || !CircuitID_IsFinite(input->uo_im))
    {
        Screen_ReportError(SCREEN_E08_CIR_NAN_IN);
        return 0u;
    }

    /* 单频幅值 = 2 |S| / N，单位为 ADC 码值。 */
    n = (double)input->sample_count;
    ui_mag = 2.0 * sqrt((input->ui_re * input->ui_re) + (input->ui_im * input->ui_im)) / n;
    uo_mag = 2.0 * sqrt((input->uo_re * input->uo_re) + (input->uo_im * input->uo_im)) / n;

    if (ui_mag < (double)CIR_INPUT_MIN_AMP)
    {
        Screen_ReportError(SCREEN_E09_CIR_UI_LOW);
        return 0u;
    }

    if (uo_mag < (double)CIR_OUTPUT_MIN_AMP)
    {
        Screen_ReportError(SCREEN_E10_CIR_UO_LOW);
        return 0u;
    }

    /* H = Uo * conj(Ui) / |Ui|^2。两路 sample_count 相同，归一化约掉。 */
    ui_mag2 = (input->ui_re * input->ui_re) + (input->ui_im * input->ui_im);
    h_re = ((input->uo_re * input->ui_re) + (input->uo_im * input->ui_im)) / ui_mag2;
    h_im = ((input->uo_im * input->ui_re) - (input->uo_re * input->ui_im)) / ui_mag2;

    /* 标定修正:扣掉两路增益差与固定相位差。放在这里,后面的 |H|、angle(H)、Z、Y
     * 全部基于修正后的 H。表内当前为恒等值(1.0, 0.0),此调用不改变结果。 */
    (void)Calib_Apply((float)input->frequency_hz, &h_re, &h_im);

    h_magnitude_squared = (h_re * h_re) + (h_im * h_im);
    if (!CircuitID_IsFinite(h_re) || !CircuitID_IsFinite(h_im) ||
        !CircuitID_IsFinite(h_magnitude_squared))
    {
        Screen_ReportError(SCREEN_E11_CIR_H_NAN);
        return 0u;
    }

    h_magnitude = sqrt(h_magnitude_squared);

    output->frequency_hz = (float)input->frequency_hz;
    output->h_magnitude_dB = (float)(20.0 * log10(h_magnitude + CIR_LOG_GUARD));
    output->h_phase_deg = (float)(atan2(h_im, h_re) * 180.0 / CIR_PI);

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
    if (h_magnitude_squared <= ((double)CIR_H_MIN_MAG * (double)CIR_H_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E13_CIR_H_LOW);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

    {
        double r = (double)CIR_FEEDBACK_RESISTOR_OHM;
        double z_re;
        double z_im;
        double z_magnitude;

        /* Z = -R / H = -R * conj(H) / |H|^2。 */
        z_re = -(r * h_re) / h_magnitude_squared;
        z_im = (r * h_im) / h_magnitude_squared;
        z_magnitude = r / h_magnitude;

        output->z_mag_ohm = (float)z_magnitude;
        output->z_mag_dB = (float)(20.0 * log10(z_magnitude + CIR_LOG_GUARD));
        output->z_phase_deg = (float)(atan2(z_im, z_re) * 180.0 / CIR_PI);
    }

    {
        double r = (double)CIR_FEEDBACK_RESISTOR_OHM;
        double y_re;
        double y_im;
        double y_magnitude;

        /* Y = -H / R。 */
        y_re = -h_re / r;
        y_im = -h_im / r;
        y_magnitude = h_magnitude / r;

        output->y_mag_siemens = (float)y_magnitude;
        output->y_phase_deg = (float)(atan2(y_im, y_re) * 180.0 / CIR_PI);
        output->y_conductance_siemens = (float)y_re;   /* G = Re[Y] */
        output->y_susceptance_siemens = (float)y_im;   /* B = Im[Y] */
    }

    if (!CircuitID_IsFinite(output->z_mag_ohm) ||
        !CircuitID_IsFinite(output->z_mag_dB) ||
        !CircuitID_IsFinite(output->z_phase_deg) ||
        !CircuitID_IsFinite(output->y_mag_siemens) ||
        !CircuitID_IsFinite(output->y_phase_deg) ||
        !CircuitID_IsFinite(output->y_conductance_siemens) ||
        !CircuitID_IsFinite(output->y_susceptance_siemens))
    {
        Screen_ReportError(SCREEN_E14_CIR_ZY_NAN);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }
#endif

    output->valid = 1u;
    return 1u;
}

static uint8_t CircuitID_IsFinite(double value)
{
    return (uint8_t)((value == value) &&
                     (value <= DBL_MAX) &&
                     (value >= -DBL_MAX));
}
