#include "calib.h"

#include <math.h>

#define CALIB_PI        3.14159265358979323846
#define CALIB_COUNT     (sizeof(s_calib) / sizeof(s_calib[0u]))

/*
 * 标定表:5 个测量频率点。
 *
 * 当前全部为恒等值,即不做任何修正。实测方法:
 * 用已知精密电阻(如 1 kohm / 0.1%)做直通标定,对 5 个频率记录实测 H_cal(f),
 * 与理论值比较得到每个频点的
 *   mag_ratio = |H_true| / |H_meas|
 *   phase_deg = angle(H_meas) - angle(H_true)
 * 标定时的采样参数(窗口周期数、Fs)必须与实测一致。
 */
static const Calib_Point_t s_calib[] = {
    {   100.0f, 1.0f, 0.0f },
    {   300.0f, 1.0f, 0.0f },
    {  1000.0f, 1.0f, 0.0f },
    {  3000.0f, 1.0f, 0.0f },
    { 10000.0f, 1.0f, 0.0f },
};

uint8_t Calib_Lookup(float frequency_hz, float *mag_ratio, float *phase_deg)
{
    uint32_t index;
    uint32_t nearest = 0u;
    float    best = 0.0f;
    uint8_t  found = 0u;

    if ((mag_ratio == (float *)0) || (phase_deg == (float *)0))
    {
        return 0u;
    }

    for (index = 0u; index < (uint32_t)CALIB_COUNT; ++index)
    {
        float diff = s_calib[index].frequency_hz - frequency_hz;

        if (diff < 0.0f)
        {
            diff = -diff;
        }

        if (found == 0u)
        {
            best = diff;
            nearest = index;
            found = 1u;
        }
        else if (diff < best)
        {
            best = diff;
            nearest = index;
        }
    }

    if (found == 0u)
    {
        /* 表为空:恒等值。 */
        *mag_ratio = 1.0f;
        *phase_deg = 0.0f;
        return 0u;
    }

    *mag_ratio = s_calib[nearest].mag_ratio;
    *phase_deg = s_calib[nearest].phase_deg;
    return 1u;
}

uint8_t Calib_Apply(float frequency_hz, double *h_re, double *h_im)
{
    float   mag_ratio;
    float   phase_deg;
    uint8_t found;
    double  theta;
    double  cos_t;
    double  sin_t;
    double  re;
    double  im;

    if ((h_re == (double *)0) || (h_im == (double *)0))
    {
        return 0u;
    }

    found = Calib_Lookup(frequency_hz, &mag_ratio, &phase_deg);

    /* 旋转 -phase_deg:H_corrected = H * mag_ratio * e^{-j*phase} */
    theta = -(double)phase_deg * CALIB_PI / 180.0;
    cos_t = cos(theta);
    sin_t = sin(theta);

    re = *h_re;
    im = *h_im;

    *h_re = ((re * cos_t) - (im * sin_t)) * (double)mag_ratio;
    *h_im = ((re * sin_t) + (im * cos_t)) * (double)mag_ratio;

    return found;
}