#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 两个 ADC 各自单通道采样，DMA 循环写入各自的缓冲区，由同一个 TIM6 TRGO 触发：
 *   ADC1，PA0_C（ADC1_INP0）：DUT 电压
 *   ADC2，PA1_C（ADC2_INP1）：电流转换后的电压
 * 当前只保存最近一对采样，数据流相关的上层接口待定。
 */

/* 每个半缓冲区包含的采样对数。DMA 每填满半个缓冲区触发一次回调。 */
#define ADC_SAMPLER_PAIRS_PER_HALF   64u

/* 码值到电压的换算：16 位满量程 65535，参考电压暂按 3.3 V，待实测确认。 */
#define ADC_SAMPLER_FULL_SCALE       65535.0f
#define ADC_SAMPLER_VREF_V           3.3f

typedef struct
{
    uint16_t dut_voltage_raw;       /* 通道 0，16 位原始码值 */
    uint16_t current_voltage_raw;   /* 通道 1，16 位原始码值 */
    uint32_t sequence;              /* 每完成一个半缓冲区加 1，0 表示尚无数据 */
} ADC_Sample_t;

/* 校准并启动 ADC1、ADC2 的 DMA 采样，再启动 TIM6；成功返回 1，失败返回 0。 */
uint8_t ADC_Sampler_Init(void);

/* 取最近一对采样；尚无数据时返回 0。 */
uint8_t ADC_Sampler_GetLatest(ADC_Sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SAMPLER_H */
