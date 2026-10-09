#include "circuit_id.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "screen.h"

#define CIR_PI              3.14159265358979323846f
/* log10f 的参数不能为 0，加一个极小值避免得到 -Inf。 */
#define CIR_LOG_GUARD       1e-20f

static float CircuitID_FrequencyToHz(uint32_t raw_frequency);
static uint8_t CircuitID_IsFinite(float value);

/* 目前没有需要初始化的状态。保留此函数，供以后的多帧平均等功能使用，main.c 启动时调用。 */
void CircuitID_Init(void)
{
}

/* 计算一个频点的传递函数 H、阻抗 Z 和导纳 Y。成功返回 1；失败返回 0，且输出全部清零。 */
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

    /* 先清零输出，失败返回时调用方拿到的全是 0。 */
    (void)memset(output, 0, sizeof(*output));

    /* 配置检查：反馈电阻必须为正，频率为 0 无意义。 */
    if ((CIR_FEEDBACK_RESISTOR_OHM <= 0.0f) ||
        (input->freq == 0u))
    {
        Screen_ReportError(SCREEN_E07_CIR_CFG);
        return 0u;
    }

    ui_re = input->ui_re;
    ui_im = input->ui_im;
    uo_re = input->uo_re;
    uo_im = input->uo_im;

    /* 输入必须是有限数，NaN 或 Inf 会让后面所有结果都无效。 */
    if (!CircuitID_IsFinite(ui_re) || !CircuitID_IsFinite(ui_im) ||
        !CircuitID_IsFinite(uo_re) || !CircuitID_IsFinite(uo_im))
    {
        Screen_ReportError(SCREEN_E08_CIR_NAN_IN);
        return 0u;
    }

    /* |Ui|² 太小时，H = Uo / Ui 会被噪声放大，结果失真，直接丢弃。 */
    ui_magnitude_squared = (ui_re * ui_re) + (ui_im * ui_im);
    if (!CircuitID_IsFinite(ui_magnitude_squared) ||
        ui_magnitude_squared <= (CIR_INPUT_MIN_MAG * CIR_INPUT_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E09_CIR_UI_LOW);
        return 0u;
    }

    /* |Uo| 太小说明输出几乎没有信号，无法可靠计算。 */
    uo_magnitude_squared = (uo_re * uo_re) + (uo_im * uo_im);
    if (!CircuitID_IsFinite(uo_magnitude_squared) ||
        uo_magnitude_squared <= (CIR_OUTPUT_MIN_MAG * CIR_OUTPUT_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E10_CIR_UO_LOW);
        return 0u;
    }

    /* H = Uo / Ui。分子和分母同乘 conj(Ui)，分母变成实数 |Ui|²，便于计算。 */
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

    /* 幅值换算成 dB：20·log10|H|；相位 = atan2(虚部, 实部)，再从弧度换算成度。 */
    output->frequency_hz = CircuitID_FrequencyToHz(input->freq);
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
    /* |H| 太小时 Z 会趋于无穷大，结果不可靠，丢弃该频点。 */
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

        /* Z = -R / H。分子分母同乘 conj(H)，分母变成实数 |H|²。
         * 结果：实部 = -R*h_re/|H|²，虚部 = +R*h_im/|H|²。 */
        z_re = -(CIR_FEEDBACK_RESISTOR_OHM * h_re) /
               h_magnitude_squared;
        z_im = (CIR_FEEDBACK_RESISTOR_OHM * h_im) /
               h_magnitude_squared;
        /* |Z| = R / |H|，分子 R 是实数，只需除以 |H|。 */
        z_magnitude = CIR_FEEDBACK_RESISTOR_OHM / h_magnitude;

        /* 阻抗幅值转 dB；相位同样用 atan2 求出并换算成度。 */
        output->z_mag_ohm = z_magnitude;
        output->z_mag_dB = 20.0f * log10f(z_magnitude + CIR_LOG_GUARD);
        output->z_phase_deg = atan2f(z_im, z_re) * 180.0f / CIR_PI;
    }

    {
        float y_re;
        float y_im;
        float y_magnitude;

        /* Y = -H / R。R 是实数，所以实部和虚部都直接除以 R。 */
        y_re = -h_re / CIR_FEEDBACK_RESISTOR_OHM;
        y_im = -h_im / CIR_FEEDBACK_RESISTOR_OHM;
        y_magnitude = h_magnitude / CIR_FEEDBACK_RESISTOR_OHM;

        output->y_mag_siemens = y_magnitude;
        output->y_phase_deg = atan2f(y_im, y_re) * 180.0f / CIR_PI;
    }
#endif

#if CIR_DUT_TOPOLOGY == 1u
    /* Z、Y 的最终检查，任一结果非有限则整个频点作废。 */
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

/* 判断是否为有限数：NaN 不等于自身，Inf 超出 float 范围，两者都排除。 */
static uint8_t CircuitID_IsFinite(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

/* freq 是实际频率时只做单位换算；是 FFT bin 时，频率 = bin × 采样率 ÷ FFT 点数。 */
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
