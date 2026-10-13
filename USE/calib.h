#ifndef CALIB_H
#define CALIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 双路标定。
 *
 * 目的:测出两路(ADC + 模拟前端)的增益比与固定相位差。跨阻前端与两路调理电路
 * 不可能完全一致,这些固定偏差会直接进入 H,进而进入 |Y|、phiY、G、B。
 *
 * 修正关系:实测 H 乘以 mag_ratio 并旋转 -phase_deg 得到真值。
 *   H_corrected = H_meas * mag_ratio * e^{-j*phase_deg}
 *
 * 当前表内全部为恒等值(1.0, 0.0),即不做任何修正;待实测直通数据后填数。
 */

/* 单点标定修正。 */
typedef struct
{
    float frequency_hz;   /* 标定频率点 */
    float mag_ratio;      /* 增益修正比,恒等 = 1.0f */
    float phase_deg;      /* 相位修正量(度),恒等 = 0.0f */
} Calib_Point_t;

/* 按频率就近查找修正量;落在两个标定点之间取较近者。
 * 未找到(频率超出标定范围)时输出恒等值(1.0, 0.0)并返回 0。 */
uint8_t Calib_Lookup(float frequency_hz, float *mag_ratio, float *phase_deg);

/* 对复数 H 施加修正。返回 1 表示命中标定点,0 表示用了恒等值(未修正)。 */
uint8_t Calib_Apply(float frequency_hz, double *h_re, double *h_im);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_H */