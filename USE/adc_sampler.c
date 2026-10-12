#include "adc_sampler.h"

#include "adc.h"
#include "tim.h"

/* 扫描序列长度：每次扫描依次转换通道 0 和通道 1。 */
#define ADC_SAMPLER_SCAN_LEN         2u
#define ADC_SAMPLER_HALF_LEN         (ADC_SAMPLER_PAIRS_PER_HALF * ADC_SAMPLER_SCAN_LEN)
#define ADC_SAMPLER_BUF_LEN          (2u * ADC_SAMPLER_HALF_LEN)

/* DMA 按半字（16 位）写入，缓冲区用 uint16_t。 */
static uint16_t adc_buf[ADC_SAMPLER_BUF_LEN];
static ADC_Sample_t adc_latest;

/* 取半缓冲区中最后一对采样作为最近结果。只在 DMA 中断中写入。 */
static void ADC_Sampler_Publish(const uint16_t *half)
{
    const uint16_t *last = &half[ADC_SAMPLER_HALF_LEN - ADC_SAMPLER_SCAN_LEN];

    adc_latest.dut_voltage_raw = last[0];
    adc_latest.current_voltage_raw = last[1];
    adc_latest.sequence++;
}

uint8_t ADC_Sampler_Init(void)
{
    /* HAL 要求 pData 为 uint32_t*，实际按半字读写。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_SAMPLER_BUF_LEN) != HAL_OK)
    {
        return 0u;
    }

    /* ADC 由 TIM6 TRGO 触发，定时器不启动就不会有采样。 */
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
        return 0u;
    }

    return 1u;
}

uint8_t ADC_Sampler_GetLatest(ADC_Sample_t *out)
{
    uint32_t primask;

    if (out == (ADC_Sample_t *)0)
    {
        return 0u;
    }

    /* 关中断复制，避免读到 DMA 中断写了一半的结果。 */
    primask = __get_PRIMASK();
    __disable_irq();
    *out = adc_latest;
    __set_PRIMASK(primask);

    return (out->sequence != 0u) ? 1u : 0u;
}

/* 前半缓冲区填满。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_Publish(&adc_buf[0]);
    }
}

/* 后半缓冲区填满。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_Publish(&adc_buf[ADC_SAMPLER_HALF_LEN]);
    }
}
