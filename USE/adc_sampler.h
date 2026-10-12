#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 两个 ADC 各自单通道，由同一个 TIM6 TRGO 触发，DMA 循环写入各自的缓冲区。
 *   ADC1，PA0_C（ADC1_INP0）：DUT 电压 Ui
 *   ADC2，PA1_C（ADC2_INP1）：电流转换后的电压 Uo
 *
 * 频率由 SPI 给出。每收到一个新频率，开始一个窗口：在两路上做该频率的单频 DFT，
 * 窗口结束后得到 Ui、Uo 的复数分量，交给 CircuitID_Compute() 计算 H、Z、Y。
 * 两路由同一 DMA 块边界开始累加，样本严格对齐。
 */

/* TIM6 触发频率：APB1 定时器时钟 120 MHz，PSC=11、ARR=9，得 1 MHz。 */
#define ADC_SAMPLER_RATE_HZ          1000000.0f

/* 每个半缓冲区（一个 DMA 块）的采样点数。 */
#define ADC_SAMPLER_PAIRS_PER_HALF   64u

/* 窗口长度：约覆盖 CYCLES 个信号周期，并限制在 [WINDOW_MIN, WINDOW_MAX] 之间。
 * 低频时 WINDOW_MAX 会限制周期数，此时窗口不再是整数周期，会有泄漏误差。 */
#define ADC_SAMPLER_WINDOW_CYCLES    10u
#define ADC_SAMPLER_WINDOW_MIN       1024u
#define ADC_SAMPLER_WINDOW_MAX       1000000u

/* 一个窗口的单频 DFT 结果。
 * 幅值换算：|U| = 2 * sqrt(re^2 + im^2) / sample_count，单位为 ADC 码值。
 * 两路使用同一个 sample_count，所以 H = Uo / Ui 与归一化无关。 */
typedef struct
{
    uint32_t frequency_hz;
    uint32_t sample_count;
    double   ui_re;
    double   ui_im;
    double   uo_re;
    double   uo_im;
} ADC_Window_t;

/* 校准并启动两路 DMA，再启动 TIM6；成功返回 1，失败返回 0。 */
uint8_t ADC_Sampler_Init(void);

/* 设定当前频率并开始新窗口。频率相同则不重启；频率非法返回 0。 */
uint8_t ADC_Sampler_SetFrequency(uint32_t frequency_hz);

/* 取出最近完成的窗口；没有新窗口时返回 0。 */
uint8_t ADC_Sampler_GetWindow(ADC_Window_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SAMPLER_H */
