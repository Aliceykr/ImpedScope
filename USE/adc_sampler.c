#include "adc_sampler.h"

#include <math.h>

#include "adc.h"
#include "tim.h"

#define ADC_SAMPLER_TWO_PI           6.283185307179586

/* DMA 按半字（16 位）写入，缓冲区用 uint16_t。两路各一个，每路两个半区。 */
static uint16_t adc1_buf[2u * ADC_SAMPLER_PAIRS_PER_HALF];
static uint16_t adc2_buf[2u * ADC_SAMPLER_PAIRS_PER_HALF];

/* 单路累加器。DMA 每回调一次处理一个块，块序号两路各自计数。 */
typedef struct
{
    uint32_t blocks;        /* 已处理的块数，即下一个块的序号 */
    uint32_t start_block;   /* 本窗口起始块序号，两路相同 */
    uint32_t count;         /* 本窗口已累计的采样点数 */
    uint8_t  done;          /* 本窗口已累计满 */
    double   rot_re;        /* 当前权重 e^{-j w n} */
    double   rot_im;
    double   acc_re;        /* sum x * w */
    double   acc_im;
    double   w_re;          /* sum w，用于去除直流 */
    double   w_im;
    double   sum_x;         /* sum x，用于去除直流 */
} ADC_Acc_t;

static ADC_Acc_t s_acc_dut;             /* ADC1：Ui */
static ADC_Acc_t s_acc_cur;             /* ADC2：Uo */
static uint32_t  s_frequency_hz;
static uint32_t  s_target;              /* 窗口长度，0 表示尚未设定频率 */
static double    s_step_re;             /* e^{-j w}，每采样点推进一次 */
static double    s_step_im;
static ADC_Window_t s_window;
static volatile uint8_t s_ready;

static void ADC_Sampler_ResetAcc(ADC_Acc_t *acc, uint32_t start_block)
{
    acc->start_block = start_block;
    acc->count = 0u;
    acc->done = 0u;
    acc->rot_re = 1.0;
    acc->rot_im = 0.0;
    acc->acc_re = 0.0;
    acc->acc_im = 0.0;
    acc->w_re = 0.0;
    acc->w_im = 0.0;
    acc->sum_x = 0.0;
}

/* 处理一个 DMA 块（ADC_SAMPLER_PAIRS_PER_HALF 个采样点）。只在 DMA 中断中调用。 */
static void ADC_Sampler_Accumulate(ADC_Acc_t *acc, const uint16_t *block)
{
    uint32_t block_index = acc->blocks;
    uint32_t index;

    acc->blocks++;

    /* 无窗口、窗口尚未开始、或本窗口已满，都不累计。 */
    if ((s_target == 0u) || (block_index < acc->start_block) || (acc->done != 0u))
    {
        return;
    }

    for (index = 0u; (index < ADC_SAMPLER_PAIRS_PER_HALF) && (acc->count < s_target); ++index)
    {
        double x = (double)block[index];
        double next_re;
        double next_im;

        acc->acc_re += x * acc->rot_re;
        acc->acc_im += x * acc->rot_im;
        acc->w_re += acc->rot_re;
        acc->w_im += acc->rot_im;
        acc->sum_x += x;
        acc->count++;

        /* 权重推进一个采样点：rot *= step。 */
        next_re = (acc->rot_re * s_step_re) - (acc->rot_im * s_step_im);
        next_im = (acc->rot_re * s_step_im) + (acc->rot_im * s_step_re);
        acc->rot_re = next_re;
        acc->rot_im = next_im;
    }

    if (acc->count >= s_target)
    {
        acc->done = 1u;
    }
}

/* 去直流后的单频分量：S = sum (x - mean) * w。 */
static void ADC_Sampler_Phasor(const ADC_Acc_t *acc, double *re, double *im)
{
    double mean = acc->sum_x / (double)acc->count;

    *re = acc->acc_re - (mean * acc->w_re);
    *im = acc->acc_im - (mean * acc->w_im);
}

/* 两路都累计满时，写出结果并立即开始下一个窗口。 */
static void ADC_Sampler_TryFinish(void)
{
    uint32_t next_start;

    if ((s_acc_dut.done == 0u) || (s_acc_cur.done == 0u))
    {
        return;
    }

    s_window.frequency_hz = s_frequency_hz;
    s_window.sample_count = s_target;
    ADC_Sampler_Phasor(&s_acc_dut, &s_window.ui_re, &s_window.ui_im);
    ADC_Sampler_Phasor(&s_acc_cur, &s_window.uo_re, &s_window.uo_im);
    s_ready = 1u;

    /* 下一窗口从两路中较靠后的块开始，保证两路起点相同。 */
    next_start = (s_acc_dut.blocks > s_acc_cur.blocks) ? s_acc_dut.blocks : s_acc_cur.blocks;
    ADC_Sampler_ResetAcc(&s_acc_dut, next_start);
    ADC_Sampler_ResetAcc(&s_acc_cur, next_start);
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
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, 2u * ADC_SAMPLER_PAIRS_PER_HALF) != HAL_OK)
    {
        return 0u;
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, 2u * ADC_SAMPLER_PAIRS_PER_HALF) != HAL_OK)
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

uint8_t ADC_Sampler_SetFrequency(uint32_t frequency_hz)
{
    uint32_t target;
    uint32_t start;
    uint32_t primask;
    double   w;
    double   step_re;
    double   step_im;

    if ((frequency_hz == 0u) || ((double)frequency_hz > ((double)ADC_SAMPLER_RATE_HZ / 2.0)))
    {
        return 0u;
    }

    /* 同一频率不重启窗口，避免 FPGA 重复发送时窗口永远完不成。 */
    if ((frequency_hz == s_frequency_hz) && (s_target != 0u))
    {
        return 1u;
    }

    target = (uint32_t)(((double)ADC_SAMPLER_WINDOW_CYCLES * (double)ADC_SAMPLER_RATE_HZ /
                         (double)frequency_hz) + 0.5);
    if (target < ADC_SAMPLER_WINDOW_MIN)
    {
        target = ADC_SAMPLER_WINDOW_MIN;
    }
    if (target > ADC_SAMPLER_WINDOW_MAX)
    {
        target = ADC_SAMPLER_WINDOW_MAX;
    }

    w = ADC_SAMPLER_TWO_PI * (double)frequency_hz / (double)ADC_SAMPLER_RATE_HZ;
    step_re = cos(w);
    step_im = -sin(w);

    primask = __get_PRIMASK();
    __disable_irq();

    /* 两路的 blocks 相差不超过一个块，取较大值作为共同起点。 */
    start = (s_acc_dut.blocks > s_acc_cur.blocks) ? s_acc_dut.blocks : s_acc_cur.blocks;
    s_frequency_hz = frequency_hz;
    s_target = target;
    s_step_re = step_re;
    s_step_im = step_im;
    s_ready = 0u;
    ADC_Sampler_ResetAcc(&s_acc_dut, start);
    ADC_Sampler_ResetAcc(&s_acc_cur, start);

    __set_PRIMASK(primask);

    return 1u;
}

uint8_t ADC_Sampler_GetWindow(ADC_Window_t *out)
{
    uint32_t primask;
    uint8_t  ready;

    if (out == (ADC_Window_t *)0)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    ready = s_ready;
    if (ready != 0u)
    {
        *out = s_window;
        s_ready = 0u;
    }
    __set_PRIMASK(primask);

    return ready;
}

/* 前半块填满：DUT 为 ADC1，电流为 ADC2。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_Accumulate(&s_acc_dut, &adc1_buf[0]);
    }
    else if (hadc->Instance == ADC2)
    {
        ADC_Sampler_Accumulate(&s_acc_cur, &adc2_buf[0]);
    }
    ADC_Sampler_TryFinish();
}

/* 后半块填满。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_Accumulate(&s_acc_dut, &adc1_buf[ADC_SAMPLER_PAIRS_PER_HALF]);
    }
    else if (hadc->Instance == ADC2)
    {
        ADC_Sampler_Accumulate(&s_acc_cur, &adc2_buf[ADC_SAMPLER_PAIRS_PER_HALF]);
    }
    ADC_Sampler_TryFinish();
}
