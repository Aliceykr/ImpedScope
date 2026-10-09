#include "circuit_id.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define CIR_PI              3.14159265358979323846f
#define CIR_LOG_GUARD       1e-20f

static float CircuitID_FrequencyToHz(uint32_t raw_frequency);
static uint8_t CircuitID_IsFinite(float value);

void CircuitID_Init(void)
{
    /* 当前为单帧扫频模式，暂时没有运行时状态。 */
}

uint8_t CircuitID_Compute(const FFT_Point_t *input, FRF_Point_t *output)
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

    if ((input == (const FFT_Point_t *)0) ||
        (output == (FRF_Point_t *)0))
    {
        return 0u;
    }

    (void)memset(output, 0, sizeof(*output));

    if ((CIR_FEEDBACK_RESISTOR_OHM <= 0.0f) ||
        (input->freq == 0u))
    {
        return 0u;
    }

    ui_re = input->ui_re;
    ui_im = input->ui_im;
    uo_re = input->uo_re;
    uo_im = input->uo_im;

    if (!CircuitID_IsFinite(ui_re) || !CircuitID_IsFinite(ui_im) ||
        !CircuitID_IsFinite(uo_re) || !CircuitID_IsFinite(uo_im))
    {
        return 0u;
    }

    ui_magnitude_squared = (ui_re * ui_re) + (ui_im * ui_im);
    if (!CircuitID_IsFinite(ui_magnitude_squared) ||
        ui_magnitude_squared <= (CIR_INPUT_MIN_MAG * CIR_INPUT_MIN_MAG))
    {
        return 0u;
    }

    uo_magnitude_squared = (uo_re * uo_re) + (uo_im * uo_im);
    if (!CircuitID_IsFinite(uo_magnitude_squared) ||
        uo_magnitude_squared <= (CIR_OUTPUT_MIN_MAG * CIR_OUTPUT_MIN_MAG))
    {
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
        return 0u;
    }

    h_magnitude = sqrtf(h_magnitude_squared);

    output->frequency_hz = CircuitID_FrequencyToHz(input->freq);
    output->h_magnitude_dB = 20.0f * log10f(h_magnitude + CIR_LOG_GUARD);
    output->h_phase_deg = atan2f(h_im, h_re) * 180.0f / CIR_PI;

    if (!CircuitID_IsFinite(output->frequency_hz) ||
        !CircuitID_IsFinite(output->h_magnitude_dB) ||
        !CircuitID_IsFinite(output->h_phase_deg))
    {
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

#if CIR_DUT_TOPOLOGY == 1u
    /* H 过小表示阻抗趋于无穷，结果不可靠。 */
    if (h_magnitude_squared <= (CIR_OUTPUT_MIN_MAG * CIR_OUTPUT_MIN_MAG))
    {
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

    {
        float z_re;
        float z_im;
        float z_magnitude;

        /* Z = -R / H = -R * conj(H) / |H|^2。 */
        z_re = -(CIR_FEEDBACK_RESISTOR_OHM * h_re) /
               h_magnitude_squared;
        z_im = -(CIR_FEEDBACK_RESISTOR_OHM * h_im) /
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
#endif

#if CIR_DUT_TOPOLOGY == 1u
    if (!CircuitID_IsFinite(output->z_mag_ohm) ||
        !CircuitID_IsFinite(output->z_mag_dB) ||
        !CircuitID_IsFinite(output->z_phase_deg) ||
        !CircuitID_IsFinite(output->y_mag_siemens) ||
        !CircuitID_IsFinite(output->y_phase_deg))
    {
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

static float CircuitID_FrequencyToHz(uint32_t raw_frequency)
{
#if CIR_FREQ_IS_RAW
    float frequency_hz = (float)raw_frequency;

  #if CIR_FREQ_UNIT == 2u
    frequency_hz *= 1000000.0f;
  #elif CIR_FREQ_UNIT == 1u
    frequency_hz *= 1000.0f;
  #endif

    return frequency_hz;
#else
    return ((float)raw_frequency * CIR_SAMPLE_RATE_HZ) /
           (float)CIR_FFT_POINTS;
#endif
}
