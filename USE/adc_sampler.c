#include "adc_sampler.h"

#include "adc.h"
#include "tim.h"

/* 两个 ADC 各自单通道，DMA 各用一个缓冲区；每个半缓冲区 PAIRS_PER_HALF 个采样。 */
#define ADC_SAMPLER_BUF_LEN          (2u * ADC_SAMPLER_PAIRS_PER_HALF)

/* DMA 按半字（16 位）写入，缓冲区用 uint16_t。 */
static uint16_t adc1_buf[ADC_SAMPLER_BUF_LEN];
static uint16_t adc2_buf[ADC_SAMPLER_BUF_LEN];
static ADC_Sample_t adc_latest;

/* 两个 ADC 由同一个 TIM6 TRGO 触发，采样时刻接近。
 * ADC1 的回调负责递增 sequence，ADC2 只更新电流值。 */
static void ADC_Sampler_UpdateDut(const uint16_t *half)
{
    adc_latest.dut_voltage_raw = half[ADC_SAMPLER_PAIRS_PER_HALF - 1u];
    adc_latest.sequence++;
}

static void ADC_Sampler_UpdateCurrent(const uint16_t *half)
{
    adc_latest.current_voltage_raw = half[ADC_SAMPLER_PAIRS_PER_HALF - 1u];
}

uint8_t ADC_Sampler_Init(void)
{
    /* H7 上电后必须校准，否则偏移和增益误差会直接进入读数。 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 0u;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return 0u;
    }

    /* HAL 要求 pData 为 uint32_t*，实际按半字读写。两路 DMA 都要先装载，再启动定时器。 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC_SAMPLER_BUF_LEN) != HAL_OK)
    {
        return 0u;
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, ADC_SAMPLER_BUF_LEN) != HAL_OK)
    {
        return 0u;
    }

    /* 两路 ADC 都由 TIM6 TRGO 触发，定时器不启动就不会有采样。 */
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
        ADC_Sampler_UpdateDut(&adc1_buf[0]);
    }
    else if (hadc->Instance == ADC2)
    {
        ADC_Sampler_UpdateCurrent(&adc2_buf[0]);
    }
}

/* 后半缓冲区填满。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_UpdateDut(&adc1_buf[ADC_SAMPLER_PAIRS_PER_HALF]);
    }
    else if (hadc->Instance == ADC2)
    {
        ADC_Sampler_UpdateCurrent(&adc2_buf[ADC_SAMPLER_PAIRS_PER_HALF]);
    }
}
