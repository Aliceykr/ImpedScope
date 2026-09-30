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
    float ui_mag2, h_re, h_im, h_mag2, h_mag;

    if ((in == (const FFT_Point_t *)0) || (out == (FRF_Point_t *)0))
    {
        return 0u;
    }

    ui_re = in->ui_re;
    ui_im = in->ui_im;
    uo_re = in->uo_re;
    uo_im = in->uo_im;

    ui_mag2 = (ui_re * ui_re) + (ui_im * ui_im);

    /* Reference noise gate */
    if (ui_mag2 <= (CIR_UI_MIN_MAG * CIR_UI_MIN_MAG))
    {
        out->valid = 0u;
        return 0u;
    }

    /* H = Uo * Ui* / |Ui|^2  (H1 estimate, complex division) */
    h_re = ((uo_re * ui_re) + (uo_im * ui_im)) / ui_mag2;
    h_im = ((uo_im * ui_re) - (uo_re * ui_im)) / ui_mag2;

    h_mag2 = (h_re * h_re) + (h_im * h_im);
    h_mag  = sqrtf(h_mag2);

    out->frequency_hz   = CircuitID_FreqToHz(in->freq);
    out->h_magnitude_dB = 20.0f * log10f(h_mag + CIR_DEN_GUARD);
    out->h_phase_deg    = atan2f(h_im, h_re) * 180.0f / CIR_PIF;

#if CIR_DUT_TOPOLOGY == 1u
    /* Response noise gate: Uo ~ 0 means Z -> infinity (open circuit) */
    if (h_mag2 <= (CIR_UO_MIN_MAG * CIR_UO_MIN_MAG))
    {
        out->valid = 0u;
        return 0u;
    }

    /* Z = -R/H = -R * conj(H)/|H|^2 */
    {
        float z_re = -(CIR_R_FB_OHM * h_re) / h_mag2;
        float z_im = -(CIR_R_FB_OHM * h_im) / h_mag2;
        float z_mag = CIR_R_FB_OHM / h_mag;

        out->z_mag_ohm   = z_mag;
        out->z_mag_dB    = 20.0f * log10f(z_mag + CIR_DEN_GUARD);
        out->z_phase_deg = atan2f(z_im, z_re) * 180.0f / CIR_PIF;
    }

    /* Y = -H/R */
    {
        float y_re = -h_re / CIR_R_FB_OHM;
        float y_im = -h_im / CIR_R_FB_OHM;
        float y_mag = h_mag / CIR_R_FB_OHM;

        out->y_mag_siemens = y_mag;
        out->y_phase_deg   = atan2f(y_im, y_re) * 180.0f / CIR_PIF;
    }
#endif

    out->valid = 1u;
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
