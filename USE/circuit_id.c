#include "circuit_id.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define CIR_PIF                 3.14159265358979323846f  /* 圆周率 π（float）*/
#define CIR_DEN_GUARD           1e-20f                   /* 对数/除法的保护量，避免 log(0) */

/* Private function prototypes -----------------------------------------------*/
/* 把频点字段（频率值或 bin 号）换算为以 Hz 为单位的实际频率 */
static float CircuitID_FreqToHz(uint32_t raw);

/* Exported functions --------------------------------------------------------*/
void CircuitID_Init(void)
{
    /* 为 chirp/白噪声模式下的多帧平均状态初始化预留；当前扫频正弦模式无需处理 */
}

uint8_t CircuitID_Compute(const FFT_Point_t *in, FRF_Point_t *out)
{
    float ui_re, ui_im, uo_re, uo_im;   /* Ui、Uo 的实部与虚部 */
    float ui_mag2;                      /* |Ui|^2 */
    float h_re, h_im, h_mag2, h_mag;    /* H 的实部、虚部、|H|^2、|H| */

    /* 入参合法性检查 */
    if ((in == (const FFT_Point_t *)0) || (out == (FRF_Point_t *)0))
    {
        return 0u;
    }

    ui_re = in->ui_re;
    ui_im = in->ui_im;
    uo_re = in->uo_re;
    uo_im = in->uo_im;

    ui_mag2 = (ui_re * ui_re) + (ui_im * ui_im);

    /* 参考噪声门限：|Ui| 过小则 H 会被噪声严重放大，直接判无效 */
    if (ui_mag2 <= (CIR_UI_MIN_MAG * CIR_UI_MIN_MAG))
    {
        out->valid = 0u;
        return 0u;
    }

    /* H = Uo·Ui* / |Ui|^2（H1 估计，复数除法）。
       乘以 Ui 的共轭 Ui* 可在最小二乘意义下抑制输出端噪声。
       复数乘法 (Uo)(Ui*)：
         实部 = uo_re*ui_re + uo_im*ui_im
         虚部 = uo_im*ui_re - uo_re*ui_im  */
    h_re = ((uo_re * ui_re) + (uo_im * ui_im)) / ui_mag2;
    h_im = ((uo_im * ui_re) - (uo_re * ui_im)) / ui_mag2;

    h_mag2 = (h_re * h_re) + (h_im * h_im);
    h_mag  = sqrtf(h_mag2);

    /* 传递函数 H 的频率、幅度(dB)、相位(°) */
    out->frequency_hz   = CircuitID_FreqToHz(in->freq);
    out->h_magnitude_dB = 20.0f * log10f(h_mag + CIR_DEN_GUARD);
    out->h_phase_deg    = atan2f(h_im, h_re) * 180.0f / CIR_PIF;

#if CIR_DUT_TOPOLOGY == 1u
    /* 响应噪声门限：Uo≈0 意味着 |H|≈0、Z 趋于无穷（被测件开路）*/
    if (h_mag2 <= (CIR_UO_MIN_MAG * CIR_UO_MIN_MAG))
    {
        out->valid = 0u;
        return 0u;
    }

    /* 阻抗 Z = -R/H = -R·conj(H)/|H|^2。
       负号来自反相（虚地）运放，直接进入复数运算，
       相位用 atan2 计算即可自然体现 180° 翻转，无需手动加减。 */
    {
        float z_re = -(CIR_R_FB_OHM * h_re) / h_mag2;
        float z_im = -(CIR_R_FB_OHM * h_im) / h_mag2;
        float z_mag = CIR_R_FB_OHM / h_mag;   /* |Z| = R/|H| */

        out->z_mag_ohm   = z_mag;
        out->z_mag_dB    = 20.0f * log10f(z_mag + CIR_DEN_GUARD);
        out->z_phase_deg = atan2f(z_im, z_re) * 180.0f / CIR_PIF;
    }

    /* 导纳 Y = -H/R（与阻抗互为倒数）*/
    {
        float y_re = -h_re / CIR_R_FB_OHM;
        float y_im = -h_im / CIR_R_FB_OHM;
        float y_mag = h_mag / CIR_R_FB_OHM;   /* |Y| = |H|/R */

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
    /* freq 字段已经是实际频率，按其单位换算为 Hz */
    float f = (float)raw;
  #if CIR_FREQ_UNIT == 2u
    f *= 1000000.0f;     /* MHz -> Hz */
  #elif CIR_FREQ_UNIT == 1u
    f *= 1000.0f;        /* kHz -> Hz */
  #endif
    return f;
#else
    /* freq 字段是 FFT bin 号：f = bin * fs / N */
    return ((float)raw * CIR_SAMPLE_RATE_HZ) / (float)CIR_FFT_POINTS;
#endif
}
