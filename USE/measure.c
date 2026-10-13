#include "measure.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "ad9833.h"
#include "adc.h"
#include "screen.h"
#include "tim.h"

/* 这三个常数原来分散在 adc_sampler.c / circuit_id.c / calib.c,合并后集中定义。 */
#define MEASURE_PI         3.14159265358979323846
#define MEASURE_TWO_PI     6.283185307179586
#define MEASURE_LOG_GUARD  1e-20

/* ==========================================================================
 * 1. 频率扫描计划(原 freq_plan.c)
 * ========================================================================== */

/*
 * 5 个测量频率,与赛题要求一致。顺序扫描 100 -> 300 -> 1000 -> 3000 -> 10000 Hz。
 *
 * 这 5 个值之比为 1:3:10:30:100,在 Fs = 300 kHz 下 Fs/f 全为整数,
 * 窗口取 10 个周期即为整数个周期(校验表见 measure.h 的 ADC_SAMPLER_WINDOW_CYCLES)。
 */
static const uint32_t s_freq_plan[FREQ_PLAN_COUNT] = {
    100u, 300u, 1000u, 3000u, 10000u
};

void Freq_SourceSet(uint32_t frequency_hz)
{
    /* 必须输出与 Freq_PlanGet() 表值完全一致的频率:DFT 的旋转因子用的是表值,
     * 两者不一致会让频率轴错位、窗口不再是整数周期,而且不会有任何报错。
     * AD9833_SetFrequency() 内部按 f × 2^28 / MCLK 取整,25 MHz 下分辨率 0.093 Hz。 */
    if (frequency_hz == 0u)
    {
        return;
    }

    if (AD9833_SetFrequency(frequency_hz) == 0u)
    {
        Screen_ReportError(SCREEN_E19_AD9833_SPI);
    }
}

uint32_t Freq_PlanGet(uint8_t index)
{
    if (index >= (uint8_t)FREQ_PLAN_COUNT)
    {
        return 0u;
    }

    return s_freq_plan[index];
}

/* ==========================================================================
 * 2. ADC 采样与单频 DFT(原 adc_sampler.c)
 * ========================================================================== */

/* 双 ADC 同步模式下，DMA 的一个字就是一对采样点：
 *   bit[15:0]  = ADC1（主）的结果 = Ui，PA0_C / ADC1_INP0
 *   bit[31:16] = ADC2（从）的结果 = Uo，PA1_C / ADC2_INP1
 * 一对结果由硬件在同一时刻采样后打包进公共数据寄存器 CDR，只由 ADC1 的 DMA
 * （DMA1_Stream2）按字搬运，所以缓冲区是 uint32_t。 */
static uint32_t adc_pair_buf[2u * ADC_SAMPLER_PAIRS_PER_HALF];

#define ADC_PAIR_UI(word)   ((uint16_t)((uint32_t)(word) & 0xFFFFu))
#define ADC_PAIR_UO(word)   ((uint16_t)(((uint32_t)(word) >> 16) & 0xFFFFu))

/* 窗口累加器。两路共用一份权重和一份块序号，所以合在一个结构里：
 * 硬件保证同一个字里的 Ui、Uo 来自同一采样时刻，不存在跨路对齐问题。
 * （独立模式下两路各自触发、各自计数，还得取较大块序号来对齐，已不再需要。） */
typedef struct
{
    uint32_t blocks;        /* 已处理的块数，即下一个块的序号 */
    uint32_t start_block;   /* 本窗口起始块序号 */
    uint32_t count;         /* 本窗口已累计的采样点数（两路相同） */
    uint8_t  done;          /* 本窗口已累计满 */
    double   rot_re;        /* 当前权重 e^{-j w n} */
    double   rot_im;
    double   ui_re;         /* sum Ui * w */
    double   ui_im;
    double   uo_re;         /* sum Uo * w */
    double   uo_im;
    double   w_re;          /* sum w，用于去除直流 */
    double   w_im;
    double   ui_sum;        /* sum Ui，用于去除直流 */
    double   uo_sum;        /* sum Uo，用于去除直流 */
} ADC_Acc_t;

static ADC_Acc_t s_acc;
static uint32_t  s_frequency_hz;
static uint32_t  s_target;              /* 窗口长度，0 表示尚未设定频率 */
static double    s_step_re;             /* e^{-j w}，每采样点推进一次 */
static double    s_step_im;
static ADC_Window_t s_window;
static volatile uint8_t s_ready;

static void ADC_Sampler_ResetAcc(ADC_Acc_t *acc, uint32_t start_block);
static void ADC_Sampler_Accumulate(const uint32_t *block);
static void ADC_Sampler_Phasor(const ADC_Acc_t *acc, double sum_x, double acc_re, double acc_im,
                               double *re, double *im);
static void ADC_Sampler_TryFinish(void);

static void ADC_Sampler_ResetAcc(ADC_Acc_t *acc, uint32_t start_block)
{
    acc->start_block = start_block;
    acc->count = 0u;
    acc->done = 0u;
    acc->rot_re = 1.0;
    acc->rot_im = 0.0;
    acc->ui_re = 0.0;
    acc->ui_im = 0.0;
    acc->uo_re = 0.0;
    acc->uo_im = 0.0;
    acc->w_re = 0.0;
    acc->w_im = 0.0;
    acc->ui_sum = 0.0;
    acc->uo_sum = 0.0;
}

/* 处理一个 DMA 块（ADC_SAMPLER_PAIRS_PER_HALF 对采样点）。只在 DMA 中断中调用。
 * 一个字里两路的结果来自同一采样时刻，所以用同一个权重、同一个块序号。 */
static void ADC_Sampler_Accumulate(const uint32_t *block)
{
    uint32_t block_index = s_acc.blocks;
    uint32_t index;

    s_acc.blocks++;

    /* 无窗口、窗口尚未开始、或本窗口已满，都不累计。 */
    if ((s_target == 0u) || (block_index < s_acc.start_block) || (s_acc.done != 0u))
    {
        return;
    }

    for (index = 0u; (index < ADC_SAMPLER_PAIRS_PER_HALF) && (s_acc.count < s_target); ++index)
    {
        double ui = (double)ADC_PAIR_UI(block[index]);
        double uo = (double)ADC_PAIR_UO(block[index]);
        double next_re;
        double next_im;

        s_acc.ui_re += ui * s_acc.rot_re;
        s_acc.ui_im += ui * s_acc.rot_im;
        s_acc.uo_re += uo * s_acc.rot_re;
        s_acc.uo_im += uo * s_acc.rot_im;
        s_acc.w_re += s_acc.rot_re;
        s_acc.w_im += s_acc.rot_im;
        s_acc.ui_sum += ui;
        s_acc.uo_sum += uo;
        s_acc.count++;

        /* 权重推进一个采样点：rot *= step。 */
        next_re = (s_acc.rot_re * s_step_re) - (s_acc.rot_im * s_step_im);
        next_im = (s_acc.rot_re * s_step_im) + (s_acc.rot_im * s_step_re);
        s_acc.rot_re = next_re;
        s_acc.rot_im = next_im;
    }

    if (s_acc.count >= s_target)
    {
        s_acc.done = 1u;
    }
}

/* 去直流后的单频分量：S = sum (x - mean) * w。 */
static void ADC_Sampler_Phasor(const ADC_Acc_t *acc, double sum_x, double acc_re, double acc_im,
                               double *re, double *im)
{
    double mean = sum_x / (double)acc->count;

    *re = acc_re - (mean * acc->w_re);
    *im = acc_im - (mean * acc->w_im);
}

/* 窗口累计满时，写出结果并立即开始下一个窗口。 */
static void ADC_Sampler_TryFinish(void)
{
    if (s_acc.done == 0u)
    {
        return;
    }

    s_window.frequency_hz = s_frequency_hz;
    s_window.sample_count = s_target;
    ADC_Sampler_Phasor(&s_acc, s_acc.ui_sum, s_acc.ui_re, s_acc.ui_im, &s_window.ui_re, &s_window.ui_im);
    ADC_Sampler_Phasor(&s_acc, s_acc.uo_sum, s_acc.uo_re, s_acc.uo_im, &s_window.uo_re, &s_window.uo_im);
    s_ready = 1u;

    /* 下一窗口从本窗口结束后的下一个块开始：两路共用一个块序号，起点天然一致。 */
    ADC_Sampler_ResetAcc(&s_acc, s_acc.blocks);
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

    /* 双 ADC 同步模式：一次 TIM6 TRGO 让 ADC1（主）和 ADC2（从）在同一时刻开始转换，
     * 一对结果打包在公共数据寄存器 CDR 里，由 ADC1 的 DMA 按字搬进 adc_pair_buf。
     * 只能用主 ADC 的句柄调用；从 ADC 的 DMA 在同步模式下不使用（其配置保留在
     * Core/Src/adc.c 里，只有切回独立模式时才会用到）。
     * 必须在两路都校准之后再调用：校准会先把 ADC 关掉。 */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_pair_buf, 2u * ADC_SAMPLER_PAIRS_PER_HALF) != HAL_OK)
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

    /* 同一频率不重启窗口，避免重复设置时窗口永远完不成。 */
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

    w = MEASURE_TWO_PI * (double)frequency_hz / (double)ADC_SAMPLER_RATE_HZ;
    step_re = cos(w);
    step_im = -sin(w);

    primask = __get_PRIMASK();
    __disable_irq();

    /* 同步模式下两路共用一个块序号，当前块序号就是下一个窗口的起点。 */
    start = s_acc.blocks;
    s_frequency_hz = frequency_hz;
    s_target = target;
    s_step_re = step_re;
    s_step_im = step_im;
    s_ready = 0u;
    ADC_Sampler_ResetAcc(&s_acc, start);

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

/* 前半块填满。同步模式下只有主 ADC（ADC1）的 DMA 在跑。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_Accumulate(&adc_pair_buf[0]);
    }
    ADC_Sampler_TryFinish();
}

/* 后半块填满。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Sampler_Accumulate(&adc_pair_buf[ADC_SAMPLER_PAIRS_PER_HALF]);
    }
    ADC_Sampler_TryFinish();
}

/* ==========================================================================
 * 3. 两路标定(原 calib.c)
 * ========================================================================== */

/*
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

static uint8_t Calib_Lookup(float frequency_hz, float *mag_ratio, float *phase_deg);
static uint8_t Calib_Apply(float frequency_hz, double *h_re, double *h_im);

/* 按频率就近查找修正量;落在两个标定点之间取较近者。
 * 未找到(频率超出标定范围)时输出恒等值(1.0, 0.0)并返回 0。 */
static uint8_t Calib_Lookup(float frequency_hz, float *mag_ratio, float *phase_deg)
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

/* 对复数 H 施加修正。返回 1 表示命中标定点,0 表示用了恒等值(未修正)。 */
static uint8_t Calib_Apply(float frequency_hz, double *h_re, double *h_im)
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
    theta = -(double)phase_deg * MEASURE_PI / 180.0;
    cos_t = cos(theta);
    sin_t = sin(theta);

    re = *h_re;
    im = *h_im;

    *h_re = ((re * cos_t) - (im * sin_t)) * (double)mag_ratio;
    *h_im = ((re * sin_t) + (im * cos_t)) * (double)mag_ratio;

    return found;
}

/* ==========================================================================
 * 4. 传递函数 / 阻抗 / 导纳(原 circuit_id.c)
 * ========================================================================== */

static uint8_t CircuitID_IsFinite(double value);

void CircuitID_Init(void)
{
    /* 当前为单频窗口模式，暂时没有运行时状态。 */
}

uint8_t CircuitID_Compute(const ADC_Window_t *input, FRF_Point_t *output)
{
    double n;
    double ui_mag;
    double uo_mag;
    double ui_mag2;
    double h_re;
    double h_im;
    double h_magnitude_squared;
    double h_magnitude;

    if ((input == (const ADC_Window_t *)0) ||
        (output == (FRF_Point_t *)0))
    {
        return 0u;
    }

    (void)memset(output, 0, sizeof(*output));

    if (CIR_FEEDBACK_RESISTOR_OHM <= 0.0f)
    {
        Screen_ReportError(SCREEN_E07_CIR_CFG);
        return 0u;
    }

    if ((input->frequency_hz == 0u) || (input->sample_count == 0u))
    {
        Screen_ReportError(SCREEN_E07_CIR_CFG);
        return 0u;
    }

    if (!CircuitID_IsFinite(input->ui_re) || !CircuitID_IsFinite(input->ui_im) ||
        !CircuitID_IsFinite(input->uo_re) || !CircuitID_IsFinite(input->uo_im))
    {
        Screen_ReportError(SCREEN_E08_CIR_NAN_IN);
        return 0u;
    }

    /* 单频幅值 = 2 |S| / N，单位为 ADC 码值。 */
    n = (double)input->sample_count;
    ui_mag = 2.0 * sqrt((input->ui_re * input->ui_re) + (input->ui_im * input->ui_im)) / n;
    uo_mag = 2.0 * sqrt((input->uo_re * input->uo_re) + (input->uo_im * input->uo_im)) / n;

    if (ui_mag < (double)CIR_INPUT_MIN_AMP)
    {
        Screen_ReportError(SCREEN_E09_CIR_UI_LOW);
        return 0u;
    }

    if (uo_mag < (double)CIR_OUTPUT_MIN_AMP)
    {
        Screen_ReportError(SCREEN_E10_CIR_UO_LOW);
        return 0u;
    }

    /* H = Uo * conj(Ui) / |Ui|^2。两路 sample_count 相同，归一化约掉。 */
    ui_mag2 = (input->ui_re * input->ui_re) + (input->ui_im * input->ui_im);
    h_re = ((input->uo_re * input->ui_re) + (input->uo_im * input->ui_im)) / ui_mag2;
    h_im = ((input->uo_im * input->ui_re) - (input->uo_re * input->ui_im)) / ui_mag2;

    /* 标定修正:扣掉两路增益差与固定相位差。放在这里,后面的 |H|、angle(H)、Z、Y
     * 全部基于修正后的 H。表内当前为恒等值(1.0, 0.0),此调用不改变结果。 */
    (void)Calib_Apply((float)input->frequency_hz, &h_re, &h_im);

    h_magnitude_squared = (h_re * h_re) + (h_im * h_im);
    if (!CircuitID_IsFinite(h_re) || !CircuitID_IsFinite(h_im) ||
        !CircuitID_IsFinite(h_magnitude_squared))
    {
        Screen_ReportError(SCREEN_E11_CIR_H_NAN);
        return 0u;
    }

    h_magnitude = sqrt(h_magnitude_squared);

    output->frequency_hz = (float)input->frequency_hz;
    output->h_magnitude_dB = (float)(20.0 * log10(h_magnitude + MEASURE_LOG_GUARD));
    output->h_phase_deg = (float)(atan2(h_im, h_re) * 180.0 / MEASURE_PI);

    if (!CircuitID_IsFinite(output->frequency_hz) ||
        !CircuitID_IsFinite(output->h_magnitude_dB) ||
        !CircuitID_IsFinite(output->h_phase_deg))
    {
        Screen_ReportError(SCREEN_E12_CIR_OUT_NAN);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

#if CIR_DUT_TOPOLOGY == 1u
    /* H 过小表示阻抗趋于无穷，结果不可靠。 */
    if (h_magnitude_squared <= ((double)CIR_H_MIN_MAG * (double)CIR_H_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E13_CIR_H_LOW);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

    {
        double r = (double)CIR_FEEDBACK_RESISTOR_OHM;
        double z_re;
        double z_im;
        double z_magnitude;

        /* Z = -R / H = -R * conj(H) / |H|^2。 */
        z_re = -(r * h_re) / h_magnitude_squared;
        z_im = (r * h_im) / h_magnitude_squared;
        z_magnitude = r / h_magnitude;

        output->z_mag_ohm = (float)z_magnitude;
        output->z_mag_dB = (float)(20.0 * log10(z_magnitude + MEASURE_LOG_GUARD));
        output->z_phase_deg = (float)(atan2(z_im, z_re) * 180.0 / MEASURE_PI);
    }

    {
        double r = (double)CIR_FEEDBACK_RESISTOR_OHM;
        double y_re;
        double y_im;
        double y_magnitude;

        /* Y = -H / R。 */
        y_re = -h_re / r;
        y_im = -h_im / r;
        y_magnitude = h_magnitude / r;

        output->y_mag_siemens = (float)y_magnitude;
        output->y_phase_deg = (float)(atan2(y_im, y_re) * 180.0 / MEASURE_PI);
        output->y_conductance_siemens = (float)y_re;   /* G = Re[Y] */
        output->y_susceptance_siemens = (float)y_im;   /* B = Im[Y] */
    }

    if (!CircuitID_IsFinite(output->z_mag_ohm) ||
        !CircuitID_IsFinite(output->z_mag_dB) ||
        !CircuitID_IsFinite(output->z_phase_deg) ||
        !CircuitID_IsFinite(output->y_mag_siemens) ||
        !CircuitID_IsFinite(output->y_phase_deg) ||
        !CircuitID_IsFinite(output->y_conductance_siemens) ||
        !CircuitID_IsFinite(output->y_susceptance_siemens))
    {
        Screen_ReportError(SCREEN_E14_CIR_ZY_NAN);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }
#endif

    output->valid = 1u;
    return 1u;
}

static uint8_t CircuitID_IsFinite(double value)
{
    return (uint8_t)((value == value) &&
                     (value <= DBL_MAX) &&
                     (value >= -DBL_MAX));
}
