#ifndef __CIRCUIT_ID_H
#define __CIRCUIT_ID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi_receiver.h"

/* ===========================================================================
 * 黑盒电路频响辨识模块（在 MCU 上运行，float 运算）
 * ---------------------------------------------------------------------------
 * 前端电路（见白板）：运放构成跨阻（I/V）放大器，反相端虚地。
 *   设被测件导纳为 Y、反馈电阻为 R：
 *     电流 I  = Ui * Y
 *     输出 Uo = -I * R
 *     传递函数 H = Uo/Ui = -Y*R
 *     导纳   Y = -H/R
 *     阻抗   Z = 1/Y = -R/H
 *
 * 最终 XYZ 输出（被测件阻抗）：
 *   X = 频率(Hz)，Y = |Z|(Ω)，Z = arg(Z)(°)
 * ===========================================================================*/

/* ---- 频点字段：bin 号 -> 实际频率 ---- */
/* 1 = FPGA 发来的 freq 字段已经是实际频率；0 = 发来的是 FFT bin 号 */
#define CIR_FREQ_IS_RAW         1u
/* 当 CIR_FREQ_IS_RAW=1 时，原始频率的单位：0=Hz, 1=kHz, 2=MHz */
#define CIR_FREQ_UNIT           0u
/* 仅当 CIR_FREQ_IS_RAW=0 时使用：f = bin * fs / N */
#define CIR_SAMPLE_RATE_HZ      1000000.0f  /* 采样率 fs */
#define CIR_FFT_POINTS          1024u       /* FFT 点数 N */

/* ---- 前端拓扑 ----
 * 0 = 仅计算传递比 H = Uo/Ui，不做阻抗换算
 * 1 = 跨阻 I/V 放大器、虚地法（白板方案）：Y = -H/R，Z = -R/H
 */
#define CIR_DUT_TOPOLOGY        1u

/* I/V 放大器的反馈（参考）电阻 R，单位 Ω */
#define CIR_R_FB_OHM            1000.0f

/* ---- 激励信号 ----
 * 0 = 扫频正弦（单频点，一帧即可，无需平均）
 * 1 = chirp，2 = 白噪声（需要多帧平均）
 */
#define CIR_EXCITATION          0u
#define CIR_AVG_FRAMES          1u

/* 噪声门限：参考/响应幅度过小的频点判为不可靠并剔除 */
#define CIR_UI_MIN_MAG          1e-4f   /* |Ui| 门限（小于它，H 不可靠）*/
#define CIR_UO_MIN_MAG          1e-6f   /* |Uo| 门限（小于它，Z 趋于开路）*/

/* 频响计算结果 */
typedef struct
{
    float frequency_hz;   /* X：频率 */

    /* 传递函数 H = Uo/Ui */
    float h_magnitude_dB;  /* |H|，单位 dB */
    float h_phase_deg;     /* arg(H)，单位 ° */

#if CIR_DUT_TOPOLOGY == 1u
    /* 被测件阻抗 Z = -R/H */
    float z_mag_ohm;       /* Y：|Z|，单位 Ω */
    float z_mag_dB;        /* |Z|，单位 dB（20log10）*/
    float z_phase_deg;     /* Z：arg(Z)，单位 ° */

    /* 被测件导纳 Y = -H/R */
    float y_mag_siemens;   /* |Y|，单位 S（西门子）*/
    float y_phase_deg;     /* arg(Y)，单位 ° */
#endif

    uint8_t valid;         /* 1 = 结果可靠，0 = 被门限剔除 */
} FRF_Point_t;

/* Exported functions --------------------------------------------------------*/
/* 初始化（为 chirp/噪声模式下的平均状态预留，当前为空）*/
void CircuitID_Init(void);

/* 计算单个 FFT 频点的频响。
   成功返回 1 且 out->valid=1；入参错误或被噪声门限剔除时返回 0。 */
uint8_t CircuitID_Compute(const FFT_Point_t *in, FRF_Point_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CIRCUIT_ID_H */
