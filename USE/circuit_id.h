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
 * Computes, per frequency point:
 *   H(f) = Uo(f) / Ui(f)   (complex, H1 estimate)
 *   X = frequency (Hz), Y = magnitude (dB), Z = phase (deg)
 * ===========================================================================*/

/* ---- Frequency field: bin -> Hz ---- */
/* 1 = the FPGA freq field is already a real frequency; 0 = it is an FFT bin */
#define CIR_FREQ_IS_RAW         1u
/* Unit of the raw frequency when CIR_FREQ_IS_RAW=1: 0=Hz, 1=kHz, 2=MHz */
#define CIR_FREQ_UNIT           0u
/* Used only when CIR_FREQ_IS_RAW=0: f = bin * fs / N */
#define CIR_SAMPLE_RATE_HZ      1000000.0f
#define CIR_FFT_POINTS          1024u

/* ---- DUT topology / reference resistor (PLACEHOLDER, not used in ratio
 * mode; kept so impedance can be added later) ----
 * 0 = transfer ratio Uo/Ui only (no impedance)
 * 1 = DUT in series with R_ref; Ui = total voltage, Uo = voltage on R_ref
 * 2 = DUT in series with R_ref; Ui = voltage on DUT, Uo = voltage on R_ref
 */
#define CIR_DUT_TOPOLOGY        0u
#define CIR_R_REF_OHM           1000.0f

/* ---- Excitation ----
 * 0 = swept sine (single tone; one frame per point, no averaging)
 * 1 = chirp, 2 = white noise (require multi-frame averaging)
 */
#define CIR_EXCITATION          0u
#define CIR_AVG_FRAMES          1u

/* Noise gate: points whose |Ui| is below this are rejected as unreliable */
#define CIR_UI_MIN_MAG          1e-4f

/* Frequency-response result (XYZ) */
typedef struct
{
    float frequency_hz;   /* X: frequency */
    float magnitude_dB;   /* Y: |H| in dB   */
    float phase_deg;      /* Z: phase of H  */
    uint8_t valid;        /* 1 = reliable, 0 = rejected */
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
