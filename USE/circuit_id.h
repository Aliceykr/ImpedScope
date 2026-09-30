#ifndef __CIRCUIT_ID_H
#define __CIRCUIT_ID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi_receiver.h"

/* ===========================================================================
 * Black-box circuit frequency-response identification (runs on MCU).
 *
 * Front end (per whiteboard): transimpedance (I/V) amplifier, virtual ground.
 *   DUT admittance Y, feedback resistor R:
 *     I  = Ui * Y
 *     Uo = -I * R
 *     H  = Uo/Ui = -Y*R
 *     Y  = -H/R
 *     Z  = 1/Y = -R/H
 *
 * XYZ output (DUT impedance):
 *   X = frequency (Hz), Y = |Z| (ohm), Z = arg(Z) (deg)
 * ===========================================================================*/

/* ---- Frequency field: bin -> Hz ---- */
/* 1 = the FPGA freq field is already a real frequency; 0 = it is an FFT bin */
#define CIR_FREQ_IS_RAW         1u
/* Unit of the raw frequency when CIR_FREQ_IS_RAW=1: 0=Hz, 1=kHz, 2=MHz */
#define CIR_FREQ_UNIT           0u
/* Used only when CIR_FREQ_IS_RAW=0: f = bin * fs / N */
#define CIR_SAMPLE_RATE_HZ      1000000.0f
#define CIR_FFT_POINTS          1024u

/* ---- Front-end topology ----
 * 0 = transfer ratio only (H = Uo/Ui), no impedance conversion
 * 1 = transimpedance I/V amplifier, virtual ground (whiteboard):
 *       Y = -H/R,  Z = -R/H
 */
#define CIR_DUT_TOPOLOGY        1u

/* Feedback / reference resistor R of the I/V amplifier, in ohms */
#define CIR_R_FB_OHM            1000.0f

/* ---- Excitation ----
 * 0 = swept sine (single tone; one frame per point, no averaging)
 * 1 = chirp, 2 = white noise (require multi-frame averaging)
 */
#define CIR_EXCITATION          0u
#define CIR_AVG_FRAMES          1u

/* Noise gates: reject points whose reference/response magnitude is too small */
#define CIR_UI_MIN_MAG          1e-4f   /* |Ui| gate (H unreliable)        */
#define CIR_UO_MIN_MAG          1e-6f   /* |Uo| gate (Z -> open circuit)  */

/* Frequency-response result */
typedef struct
{
    float frequency_hz;   /* X: frequency                       */

    /* Transfer function H = Uo/Ui */
    float h_magnitude_dB;  /* |H| in dB                          */
    float h_phase_deg;     /* arg(H), degrees                    */

#if CIR_DUT_TOPOLOGY == 1u
    /* DUT impedance Z = -R/H */
    float z_mag_ohm;       /* Y: |Z|, ohms                       */
    float z_mag_dB;        /* |Z| in dB (20log10)                */
    float z_phase_deg;     /* Z: arg(Z), degrees                 */

    /* DUT admittance Y = -H/R */
    float y_mag_siemens;   /* |Y|, siemens                       */
    float y_phase_deg;     /* arg(Y), degrees                    */
#endif

    uint8_t valid;         /* 1 = reliable, 0 = rejected         */
} FRF_Point_t;

/* Exported functions --------------------------------------------------------*/
void CircuitID_Init(void);

/* Computes the frequency response of one FFT point.
   Returns 1 and sets out->valid=1 on success; 0 on bad input / noise gate. */
uint8_t CircuitID_Compute(const FFT_Point_t *in, FRF_Point_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CIRCUIT_ID_H */
