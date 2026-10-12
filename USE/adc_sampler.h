#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * ADC1 连续双通道采样，DMA 循环写入缓冲区。
 *   扫描顺序 rank1 = 通道 0（PA0，ADC1_INP0）：DUT 电压
 *            rank2 = 通道 1（PA1，ADC1_INP17）：电流转换后的电压
 * 当前只保存最近一对采样，数据流相关的上层接口待定。
 */

/* 每个半缓冲区包含的采样对数。DMA 每填满半个缓冲区触发一次回调。 */
#define ADC_SAMPLER_PAIRS_PER_HALF   64u

typedef struct
{
    uint16_t dut_voltage_raw;       /* 通道 0，16 位原始码值 */
    uint16_t current_voltage_raw;   /* 通道 1，16 位原始码值 */
    uint32_t sequence;              /* 每完成一个半缓冲区加 1，0 表示尚无数据 */
} ADC_Sample_t;

/* 启动 ADC1 连续扫描 DMA 采样；成功返回 1，失败返回 0。 */
uint8_t ADC_Sampler_Init(void);

/* 取最近一对采样；尚无数据时返回 0。 */
uint8_t ADC_Sampler_GetLatest(ADC_Sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SAMPLER_H */
