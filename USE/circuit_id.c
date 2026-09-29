#include "circuit_id.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define CIR_PIF                 3.14159265358979323846f
#define CIR_DEN_GUARD           1e-20f

/* Private function prototypes -----------------------------------------------*/
static float CircuitID_FreqToHz(uint32_t raw);

/* Exported functions --------------------------------------------------------*/
void CircuitID_Init(void)
{
    /* Placeholder for state/averaging init when chirp/noise mode is used */
}

uint8_t CircuitID_Compute(const FFT_Point_t *in, FRF_Point_t *out)
{
    float ui_re, ui_im, uo_re, uo_im;
    float ui_mag2, h_re, h_im, h_mag;

    if ((in == (const FFT_Point_t *)0) || (out == (FRF_Point_t *)0))
    {
        return 0u;
    }

    ui_re = in->ui_re;
    ui_im = in->ui_im;
    uo_re = in->uo_re;
    uo_im = in->uo_im;

    ui_mag2 = (ui_re * ui_re) + (ui_im * ui_im);

    /* Noise gate */
    if (ui_mag2 <= (CIR_UI_MIN_MAG * CIR_UI_MIN_MAG))
    {
        out->valid = 0u;
        return 0u;
    }

    /* H = Uo * Ui* / |Ui|^2  (H1 estimate, complex division) */
    h_re = ((uo_re * ui_re) + (uo_im * ui_im)) / ui_mag2;
    h_im = ((uo_im * ui_re) - (uo_re * ui_im)) / ui_mag2;

    h_mag = sqrtf((h_re * h_re) + (h_im * h_im));

    out->frequency_hz = CircuitID_FreqToHz(in->freq);
    out->magnitude_dB = 20.0f * log10f(h_mag + CIR_DEN_GUARD);
    out->phase_deg    = atan2f(h_im, h_re) * 180.0f / CIR_PIF;
    out->valid        = 1u;

#if CIR_DUT_TOPOLOGY != 0u
    /* Impedance path (placeholder; refine once the DUT topology is fixed):
       topology 1: Z_DUT = R_ref * (1/H - 1)
       topology 2: Z_DUT = R_ref / H
       Compute from h_re/h_im when needed. */
#endif

    return 1u;
}

/* Private functions ---------------------------------------------------------*/
static float CircuitID_FreqToHz(uint32_t raw)
{
#if CIR_FREQ_IS_RAW
    float f = (float)raw;
  #if CIR_FREQ_UNIT == 2u
    f *= 1000000.0f;
  #elif CIR_FREQ_UNIT == 1u
    f *= 1000.0f;
  #endif
    return f;
#else
    return ((float)raw * CIR_SAMPLE_RATE_HZ) / (float)CIR_FFT_POINTS;
#endif
}
