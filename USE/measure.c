#include "measure.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "adc.h"
#include "screen.h"
#include "tim.h"
#include "relay_range.h"
#include "uart_debug.h"

/* 三个 π 常数集中在此定义。 */
#define MEASURE_PI         3.14159265358979323846
#define MEASURE_TWO_PI     6.283185307179586
#define MEASURE_LOG_GUARD  1e-20
#define MEASURE_SQRT2      1.4142135623730951

/* ==========================================================================
 * 1. 测量频率
 * ========================================================================== */

/*
 * 默认频点:1 kHz。运行时频率由 Measure_SetFrequency() 维护,本表只作默认值/
 * 标定基准;应用层收到屏幕命令后可以覆盖它。
 *
 * 激励源在外部 —— 信号源直接接到 Ui 节点,固件不再驱动任何 DDS。
 * 频率轴约定:外部信号源的输出频率必须与 Measure_GetFrequency() 一致,
 * 否则频率轴错位、窗口不再是整数周期(泄漏不为零),且不会有任何报错。
 *
 * 1 kHz 与采样率的关系:Fs = 300 kHz,Fs/f = 300 为整数,
 * 窗口取 100 个周期即 30000 点 / 100 ms,是整数个周期,泄漏为零(见 measure.h)。
 */
static const uint32_t s_freq_plan[FREQ_PLAN_COUNT] = {
    1000u
};

/*
 * 激励源接口。当前为空实现:激励由外部信号源提供,固件不驱动任何器件,
 * 因此不需要做任何事,也不会上报 E13。
 *
 * 保留这个函数的意义是维持调用契约 —— main.c 仍按"设定频率"的语义调用它,
 * 将来若重新接入片内 DDS 或可编程信号源,只需在此处实现,调用点不用动。
 *
 * 注意:它不做的正是"改变实际激励频率"这件事,所以外部信号源的输出频率
 * 必须与 Measure_GetFrequency() 一致,固件无法校验这一点(见 README 已知限制)。
 */
void Freq_SourceSet(uint32_t frequency_hz)
{
    (void)frequency_hz;   /* 外部信号源,固件不参与设定 */
}

uint32_t Freq_PlanGet(uint8_t index)
{
    if (index >= (uint8_t)FREQ_PLAN_COUNT)
    {
        return 0u;
    }

    return s_freq_plan[index];
}

/* 运行时频率。屏幕命令写、主循环读;32 位访问在 Cortex-M7 上是原子的。 */
#define MEASURE_FREQ_MIN_HZ   100u
#define MEASURE_FREQ_MAX_HZ   100000u

static volatile uint32_t s_requested_frequency_hz = 1000u;

uint32_t Measure_GetFrequency(void)
{
    return s_requested_frequency_hz;
}

uint8_t Measure_SetFrequency(uint32_t frequency_hz)
{
    if ((frequency_hz < MEASURE_FREQ_MIN_HZ) || (frequency_hz > MEASURE_FREQ_MAX_HZ))
    {
        return 0u;
    }

    s_requested_frequency_hz = frequency_hz;
    return 1u;
}

/* ==========================================================================
 * 2. ADC 采样与单频 DFT
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
 * 硬件保证同一个字里的 Ui、Uo 来自同一采样时刻，不存在跨路对齐问题。 */
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
    double   ui_sq;         /* sum Ui^2，用于算交流有效值 */
    double   uo_sq;         /* sum Uo^2，用于算交流有效值 */
    uint32_t ui_clip_low;    /* sum(Ui <= CIR_CLIP_LOW_CODES),削波检测 */
    uint32_t ui_clip_high;   /* sum(Ui >= CIR_CLIP_HIGH_CODES)          */
    uint32_t uo_clip_low;    /* sum(Uo <= CIR_CLIP_LOW_CODES)           */
    uint32_t uo_clip_high;   /* sum(Uo >= CIR_CLIP_HIGH_CODES)          */
} ADC_Acc_t;

static ADC_Acc_t s_acc;
static uint32_t  s_frequency_hz;
static uint32_t  s_target;              /* 窗口长度，0 表示尚未设定频率 */
static double    s_step_re;             /* e^{-j w}，每采样点推进一次 */
static double    s_step_im;
static ADC_Window_t s_window;
static volatile uint8_t s_ready;

/* 本窗口内发生过 ADC 溢出/DMA 错误时置位,窗口完成时据此弃掉该窗口。
 * 由 HAL_ADC_ErrorCallback()(中断上下文)写,ADC_Sampler_TryFinish() 读并清除。 */
static volatile uint8_t s_ovr_discard;

static void ADC_Sampler_ResetAcc(ADC_Acc_t *acc, uint32_t start_block);
static void ADC_Sampler_Accumulate(const uint32_t *block);
static void ADC_Sampler_Phasor(const ADC_Acc_t *acc, double sum_x, double acc_re, double acc_im,
                               double *re, double *im);
static void ADC_Sampler_TryFinish(void);
static double ADC_Sampler_AcRms(double sum_sq, double sum_x, uint32_t count);

/* 去掉直流后的交流有效值：RMS = sqrt(sum(x^2)/N - mean^2)。N 为 0 时返回 0。 */
static double ADC_Sampler_AcRms(double sum_sq, double sum_x, uint32_t count)
{
    double mean;
    double variance;

    if (count == 0u)
    {
        return 0.0;
    }

    mean = sum_x / (double)count;
    variance = (sum_sq / (double)count) - (mean * mean);

    if (variance <= 0.0)
    {
        return 0.0;
    }

    return sqrt(variance);
}

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
    acc->ui_sq = 0.0;
    acc->uo_sq = 0.0;
    acc->ui_clip_low = 0u;
    acc->ui_clip_high = 0u;
    acc->uo_clip_low = 0u;
    acc->uo_clip_high = 0u;
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
        uint16_t ui_code = ADC_PAIR_UI(block[index]);
        uint16_t uo_code = ADC_PAIR_UO(block[index]);
        double ui = (double)ui_code;
        double uo = (double)uo_code;
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
        s_acc.ui_sq += ui * ui;
        s_acc.uo_sq += uo * uo;

        /* 削波检测:用原始码值判断是否贴量程边。必须在去直流之前判断 ——
         * 削顶样本本身就是"被钳住的极值",去直流不会改变它贴边的事实,
         * 但用原始码值最直观也最省事。两路各自独立统计。 */
        if (ui_code <= (uint16_t)CIR_CLIP_LOW_CODES)
        {
            s_acc.ui_clip_low++;
        }
        else if (ui_code >= (uint16_t)CIR_CLIP_HIGH_CODES)
        {
            s_acc.ui_clip_high++;
        }
        else
        {
            /* 既未贴低边也未贴高边,无需统计 */
        }

        if (uo_code <= (uint16_t)CIR_CLIP_LOW_CODES)
        {
            s_acc.uo_clip_low++;
        }
        else if (uo_code >= (uint16_t)CIR_CLIP_HIGH_CODES)
        {
            s_acc.uo_clip_high++;
        }

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

    /* 本窗口内出现过 ADC 溢出或 DMA 错误:样本时间轴已不可信,整个窗口作废。
     * 不置 s_ready,让主循环继续等下一个干净窗口(下一个窗口最长约 100 ms,
     * 仍在 500 ms 超时预算内,所以正常的偶发溢出不会引起 E12)。
     * 错误码已由 HAL_ADC_ErrorCallback() 记入 E17/E18。 */
    if (s_ovr_discard != 0u)
    {
        s_ovr_discard = 0u;
        ADC_Sampler_ResetAcc(&s_acc, s_acc.blocks);
        return;
    }

    s_window.frequency_hz = s_frequency_hz;
    s_window.sample_count = s_target;
    ADC_Sampler_Phasor(&s_acc, s_acc.ui_sum, s_acc.ui_re, s_acc.ui_im, &s_window.ui_re, &s_window.ui_im);
    ADC_Sampler_Phasor(&s_acc, s_acc.uo_sum, s_acc.uo_re, s_acc.uo_im, &s_window.uo_re, &s_window.uo_im);

    /* 交流有效值：与单频幅值一起构成相干性判据（见 measure.h 的 CIR_MIN_COHERENCE）。 */
    s_window.ui_rms = ADC_Sampler_AcRms(s_acc.ui_sq, s_acc.ui_sum, s_acc.count);
    s_window.uo_rms = ADC_Sampler_AcRms(s_acc.uo_sq, s_acc.uo_sum, s_acc.count);

    /* 削波统计:供 CircuitID_Compute() 判定是否削顶。计数在 DMA 中断里累加,
     * 这里只是搬运,不在此处判定 —— 判定逻辑集中放在有效性检查里。 */
    s_window.clip.ui_low = s_acc.ui_clip_low;
    s_window.clip.ui_high = s_acc.ui_clip_high;
    s_window.clip.uo_low = s_acc.uo_clip_low;
    s_window.clip.uo_high = s_acc.uo_clip_high;
    s_ready = 1u;

    /* 下一窗口从本窗口结束后的下一个块开始：两路共用一个块序号，起点天然一致。 */
    ADC_Sampler_ResetAcc(&s_acc, s_acc.blocks);
}

uint8_t ADC_Sampler_Init(void)
{
    /* 先核对双 ADC 同步模式是否真的生效(配置在 adc.c 的 USER CODE 区写入)。
     * 这是一个"静默失效"防护:若 CCR 被改回独立模式,两路不再是同一时刻采样;
     * 若 DMA 被改回半字对齐,还会丢掉 Uo —— 两种情况都不会自己报错,只会让
     * H、Z、Y 悄悄偏掉(相干性判据也挡不住,因为两路各自看着都像单频)。
     * 故在上电初始化阶段就回读 CCR 核对,不符直接判定为致命(E16)。 */
    if (LL_ADC_GetMultimode(ADC12_COMMON) != LL_ADC_MULTI_DUAL_REG_SIMULT)
    {
        return 2u;      /* 双 ADC 同步模式未生效 -> 调用方报 E16 */
    }

    /* 数据打包格式也必须是 32~10 位档(一个 32 位字装一对结果),否则 DMA 按字
     * 搬运时拿到的不是"低半字 Ui + 高半字 Uo"的布局。 */
    if (LL_ADC_GetMultiDMATransfer(ADC12_COMMON) != LL_ADC_MULTI_REG_DMA_RES_32_10B)
    {
        return 2u;      /* 数据打包格式不符 -> 调用方报 E16 */
    }

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

    /* 使能 ADC 全局中断,让溢出(OVR)能被上报。HAL_ADCEx_MultiModeStart_DMA() 会
     * 打开 ADC_IT_OVR,但前提是 NVIC 里 ADC_IRQn 已使能,否则溢出被静默吞掉,
     * 结果就是用陈旧样本算出的频响而不报任何错。
     * 放在这里(而不是 .ioc 的 NVIC 配置)是为了让它在 CubeMX 重新生成后依然有效 ——
     * 本工程已有先例:EXTI4 与 PA4 也是同样理由在 USER CODE 区手动配置的。
     * 优先级取 5(数值越大越低),低于 DMA1_Stream2 的 0,让搬运优先。
     * 处理函数见 Core/Src/stm32h7xx_it.c 的 ADC_IRQHandler()。 */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

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
    s_ovr_discard = 0u;      /* 新窗口重新开始,不继承上一个窗口的错误标记 */
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

uint8_t ADC_Sampler_Restart(void)
{
    uint32_t start;
    uint32_t primask;

    if (s_target == 0u)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    /* 从当前块序号重新开始，保留频率、步进和窗口长度。 */
    start = s_acc.blocks;
    s_ready = 0u;
    s_ovr_discard = 0u;
    ADC_Sampler_ResetAcc(&s_acc, start);

    __set_PRIMASK(primask);

    return 1u;
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

/* ADC 错误回调(覆盖 HAL 的 weak 定义)。两条来路:
 *   1) 溢出(OVR):本轮转换完成时上一次的结果还没被 DMA 取走。配置是
 *      ADC_OVR_DATA_PRESERVED,所以 HAL 把 OVR 当作错误上报,会走到这里。
 *      两个 ADC 同步模式下 CDR 只有一个,溢出意味着某个采样时刻的结果被覆盖,
 *      本窗口的时间轴不再可信 —— 弃掉该窗口,不拿它去算 H/Z/Y。
 *   2) DMA 传输错误:结果根本没搬进来,同样弃窗口。
 * 两者都只计数上报,不终止测量:下一个窗口会重新开始。 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
    {
        return;
    }

    if ((hadc->ErrorCode & HAL_ADC_ERROR_OVR) != 0u)
    {
        s_ovr_discard = 1u;
        Screen_ReportError(SCREEN_E17_ADC_OVR);
    }

    if ((hadc->ErrorCode & HAL_ADC_ERROR_DMA) != 0u)
    {
        s_ovr_discard = 1u;
        Screen_ReportError(SCREEN_E18_ADC_DMA);
    }
}

/* ==========================================================================
 * 3. 两路标定
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
 * 标定表：1 kHz / 10 kHz 两个频点。
 *
 * 参考 DUT：纯电阻，实际 1010 Ω；Rf = 1500 Ω。
 * 旧表下实测（稳定窗口）：
 *   1 kHz : G≈996.35 µS, B≈0.48 µS, P≈0.027°, H≈3.49 dB
 *   10 kHz: G≈966 µS,    B≈1 µS,    P≈0.070°, H≈3.22 dB
 *
 * 反推恒等表原始值后，按 R_ref=1010 Ω 重算：
 *   1 kHz : |Y_raw|≈981.6 µS, P_raw≈-1.186° → mag_ratio≈1.009
 *   10 kHz: |Y_raw|≈960.2 µS, P_raw≈-11.559° → mag_ratio≈1.031
 *
 * mag_ratio = |H_true| / |H_meas|，phase_deg = angle(H_meas)-angle(H_true)
 * 纯电阻的 H_true 相位为 180°，因此 phase_deg 就等于屏幕 P。
 */
static const Calib_Point_t s_calib[] = {
    {  1000.0f, 1.009f, -1.186f },
    { 10000.0f, 1.031f, -11.559f },
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
 * 4. 传递函数 / 阻抗 / 导纳
 * ========================================================================== */

static uint8_t CircuitID_IsFinite(double value);
static double CircuitID_Coherence(double magnitude, double rms);

/* 运行时反馈电阻（Ω）。默认取 CIR_FEEDBACK_RESISTOR_OHM，relay_range 模块
 * 上电后由应用层调用 CircuitID_SetFeedbackResistor() 同步固定 R_eff。 */
static float s_r_feedback = CIR_FEEDBACK_RESISTOR_OHM;

void CircuitID_Init(void)
{
    /* 当前为单频窗口模式，暂时没有运行时状态。 */
}

void CircuitID_SetFeedbackResistor(float ohm)
{
    if (ohm > 0.0f)
    {
        s_r_feedback = ohm;
    }
}

float CircuitID_GetFeedbackResistor(void)
{
    return s_r_feedback;
}

/* 相干性 kappa = 单频幅值 / (sqrt(2) * 交流有效值)。
 * 纯单频为 1.0；窗口里只有噪声时远小于 1（约 sqrt(2/N)）。
 * rms 为 0（全直流或无采样点）时返回 0，按"没有信号"处理。 */
static double CircuitID_Coherence(double magnitude, double rms)
{
    if (rms <= 0.0)
    {
        return 0.0;
    }

    return magnitude / (MEASURE_SQRT2 * rms);
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
    double ui_coh;
    double uo_coh;

    if ((input == (const ADC_Window_t *)0) ||
        (output == (FRF_Point_t *)0))
    {
        return 0u;
    }

    (void)memset(output, 0, sizeof(*output));

    if (s_r_feedback <= 0.0f)
    {
        Screen_ReportError(SCREEN_E02_CIR_CFG);
        return 0u;
    }

    if ((input->frequency_hz == 0u) || (input->sample_count == 0u))
    {
        Screen_ReportError(SCREEN_E02_CIR_CFG);
        return 0u;
    }

    if (!CircuitID_IsFinite(input->ui_re) || !CircuitID_IsFinite(input->ui_im) ||
        !CircuitID_IsFinite(input->uo_re) || !CircuitID_IsFinite(input->uo_im))
    {
        Screen_ReportError(SCREEN_E03_CIR_NAN_IN);
        return 0u;
    }

    /* 单频幅值 = 2 |S| / N，单位为 ADC 码值。 */
    n = (double)input->sample_count;
    ui_mag = 2.0 * sqrt((input->ui_re * input->ui_re) + (input->ui_im * input->ui_im)) / n;
    uo_mag = 2.0 * sqrt((input->uo_re * input->uo_re) + (input->uo_im * input->uo_im)) / n;

    /* 相干性判据：幅值门限只能说明"有不小的交流分量"，说不出这分量是不是单频。
     * 输入悬空、信号源没接时，噪声落在测量频点上的分量照样能过幅值门限，于是屏幕
     * 会显示一堆看着正常的 |Y|、φY —— 这里用 kappa 把它挡下来。 */
    ui_coh = CircuitID_Coherence(ui_mag, input->ui_rms);
    uo_coh = CircuitID_Coherence(uo_mag, input->uo_rms);

    /* 先送到串口屏（t7、t8），再走幅值/相干性门限。
     * 放在门限之前是有意的：幅值过小（E04/E05）恰恰是"信号源没接"最需要看两路
     * 本底的情形；若放在后面，t7、t8 会停留在上一窗口的陈旧读数。
     * 送的是去直流后的交流有效值（RMS，单位 V），可直接与万用表、示波器的读数对照；
     * 单频分量的峰值约等于它的 sqrt(2) 倍（相干性接近 1 时该关系成立）。
     * 门限 CIR_INPUT_MIN_AMP / CIR_OUTPUT_MIN_AMP 用的仍是峰值码值，不在这里换算。 */
    (void)Screen_SetDebug((float)(input->ui_rms * ADC_SAMPLER_LSB_TO_V),
                          (float)(input->uo_rms * ADC_SAMPLER_LSB_TO_V),
                          (float)ui_coh, (float)uo_coh);

    if (ui_mag < (double)CIR_INPUT_MIN_AMP)
    {
        Screen_ReportError(SCREEN_E04_CIR_UI_LOW);
        return 0u;
    }

    if (uo_mag < (double)CIR_OUTPUT_MIN_AMP)
    {
        Screen_ReportError(SCREEN_E05_CIR_UO_LOW);
        return 0u;
    }

    /* 削波检测:统计落在 ADC 量程两端贴边区(<= 64 或 >= 65471)的样本比例。
     *
     * 为什么必须单独检测:削平后的波形仍带很强的基波分量,幅值门限和相干性
     * 判据都拦不住 —— 计算表明即使削掉 95% 的峰值,kappa 仍有 0.91,远高于
     * 门限 0.2。若不检测,屏幕会显示看着完全正常的 |Y|、phiY、G、B。
     *
     * 用整数运算比较(sum * 1000 vs N * permille),不引入浮点除法。
     * 判据门限取 1%:干净信号(哪怕幅度到轨)为 0%,真实削顶在 4% 以上。
     *
     * 注意:检出削波后【仍然继续计算并显示结果】,只是报一个错误码提示
     * 这次读数可能不可信。与 E17/E18(时间轴不可信)不同,削顶时波形是完整的,
     * 结果仍有参考价值,该由使用者判断,不该由固件替他丢弃。 */
    {
        uint32_t n_points = input->sample_count;
        uint32_t ui_clip = input->clip.ui_low + input->clip.ui_high;
        uint32_t uo_clip = input->clip.uo_low + input->clip.uo_high;

        if ((ui_clip * 1000u) > (n_points * (uint32_t)CIR_CLIP_MAX_PERMILLE))
        {
            Screen_ReportError(SCREEN_E19_ADC_CLIP_UI);
        }

        if ((uo_clip * 1000u) > (n_points * (uint32_t)CIR_CLIP_MAX_PERMILLE))
        {
            Screen_ReportError(SCREEN_E20_ADC_CLIP_UO);
        }
    }

    if (ui_coh < (double)CIR_MIN_COHERENCE)
    {
        Screen_ReportError(SCREEN_E14_CIR_UI_INCOH);
        return 0u;
    }

    if (uo_coh < (double)CIR_MIN_COHERENCE)
    {
        Screen_ReportError(SCREEN_E15_CIR_UO_INCOH);
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
        Screen_ReportError(SCREEN_E06_CIR_H_NAN);
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
        Screen_ReportError(SCREEN_E07_CIR_OUT_NAN);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

#if CIR_DUT_TOPOLOGY == 1u
    /* H 过小表示阻抗趋于无穷，结果不可靠。 */
    if (h_magnitude_squared <= ((double)CIR_H_MIN_MAG * (double)CIR_H_MIN_MAG))
    {
        Screen_ReportError(SCREEN_E08_CIR_H_LOW);
        (void)memset(output, 0, sizeof(*output));
        return 0u;
    }

    {
        double r = (double)s_r_feedback;
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
        double r = (double)s_r_feedback;
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
        Screen_ReportError(SCREEN_E09_CIR_ZY_NAN);
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


/* ==========================================================================
 * 5. 应用层：测量循环、扫描、按键请求
 * ========================================================================== */

/* 单个频率等待窗口完成的超时。默认 1 kHz 时窗口 100 ms;屏幕最低档 100 Hz 时
 * 100 周期窗口为 1 s,所以取 1500 ms 留 500 ms 裕量。
 * 注意这是墙钟时间;不要改成"循环计数",HAL_Delay(1) 因 SysTick 的 +1 分频
 * 实际可能耗 2 ms,计数式超时会翻倍。 */
#define MEASURE_WAIT_TIMEOUT_MS   1500u

/* 内部状态 */
static uint8_t s_scan_request;
static uint8_t s_range_request;
static uint8_t s_scan_fail_count;

/* 请求函数（由 screen.c 调用） */
void Measure_RequestScan(void)
{
    s_scan_request = 1u;
}

void Measure_RequestNextRange(void)
{
    s_range_request = 1u;
}

/* 外部信号源只会在 100/300/1k/3k/10k 五档之间切换。
 * 依次用五个候选频率开一个 DFT 窗口，按 Ui 的单频幅值和相干性选最可信的一档。
 * 候选顺序把常用的 1k、10k 放前面; 命中一个高相干候选就提前结束。
 * 返回选中的频率; 五档都没有可信信号时返回 0, 并保持默认 1 kHz。 */
static uint32_t FreqScan_Select(void)
{
  static const uint32_t candidates[] = { 1000u, 10000u, 3000u, 300u, 100u };
  uint32_t best_f = 0u;
  double   best_score = 0.0;
  uint8_t  i;

  Screen_ClearResult();

  for (i = 0u; i < (uint8_t)(sizeof(candidates) / sizeof(candidates[0u])); ++i)
  {
    uint32_t     f = candidates[i];
    uint32_t     deadline;
    uint8_t      got = 0u;
    ADC_Window_t window;

    if (ADC_Sampler_SetFrequency(f) == 0u)
    {
      continue;
    }

    deadline = HAL_GetTick() + MEASURE_WAIT_TIMEOUT_MS;
    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
      /* 检测 B1 按键：扫描期间用户可能想切档，提前结束当前频率的扫描 */
      if (Screen_PeekKey() == SCREEN_KEY_B1)
      {
        break;  /* 退出扫描，让主循环处理 B1 */
      }

      if (ADC_Sampler_GetWindow(&window) != 0u)
      {
        got = 1u;
        break;
      }
      Screen_Poll();
      HAL_Delay(1u);
    }

    if ((got == 0u) || (window.sample_count == 0u))
    {
      continue;
    }

    {
      double n = (double)window.sample_count;
      double ui_mag = 2.0 * sqrt((window.ui_re * window.ui_re) +
                                 (window.ui_im * window.ui_im)) / n;
      double ui_coh = 0.0;
      double score;

      if (window.ui_rms > 0.0)
      {
        ui_coh = ui_mag / (1.41421356237 * window.ui_rms);
      }

      score = ui_mag * ui_coh;
      if ((ui_mag > (double)CIR_INPUT_MIN_AMP) && (ui_coh > 0.5) &&
          (score > best_score))
      {
        best_score = score;
        best_f = f;

        /* 相干性接近 1, 已经确定是这一档, 不必再等更低的候选频率。 */
        if (ui_coh > 0.95)
        {
          break;
        }
      }
    }
  }

  if (best_f == 0u)
  {
    (void)Measure_SetFrequency(1000u);
    (void)ADC_Sampler_SetFrequency(1000u);
    return 0u;
  }

  (void)Measure_SetFrequency(best_f);
  (void)ADC_Sampler_SetFrequency(best_f);
  (void)Screen_SetActiveFreq(best_f);
  return best_f;
}

/* 初始化测量模块：CircuitID → Relay → ADC → 首次扫描 */
void Measure_Init(void)
{
  CircuitID_Init();

  /* 反馈电阻量程：三路继电器上电只闭合默认档 K2（1.5 kΩ），
   * 再把固定 R_eff 交给 CircuitID。必须在 ADC_Sampler_Init() 之前完成。 */
  Relay_Init();
  CircuitID_SetFeedbackResistor(Relay_GetOhm());

  Screen_SetStage("MODULE");

  {
    /* 只调用一次:返回 2 = 双 ADC 同步配置未生效(报 E16),0 = 其它初始化失败(E01)。
     * 前者是"静默失效"防护,配置丢失不会自己报错,只会让 H/Z/Y 悄悄偏掉。 */
    uint8_t adc_init = ADC_Sampler_Init();

    if (adc_init == 2u)
    {
      Screen_Fatal(SCREEN_E16_ADC_DUAL_MODE);
    }
    if (adc_init != 1u)
    {
      Screen_Fatal(SCREEN_E01_HAL_INIT);
    }
  }
  Screen_SetStage("SCAN");
  {
    uint32_t detected = FreqScan_Select();

    Screen_SetStage("LOOP");
    if (detected == 0u)
    {
      Screen_ReportError(SCREEN_E04_CIR_UI_LOW);
    }
  }
}

/* 主测量循环：执行一次测量周期或处理请求 */
void Measure_Service(void)
{
    uint32_t f;
    ADC_Window_t window;
    FRF_Point_t frf_point;
    uint32_t deadline;
    uint8_t  got = 0u;
    uint8_t  b1_abort = 0u;

    /* 处理切档请求 */
    if (s_range_request != 0u)
    {
        s_range_request = 0u;
        uint8_t next = Relay_Next();       /* 阻塞约 20 ms：先断后合 */

        CircuitID_SetFeedbackResistor(Relay_GetOhm());
        Screen_ClearResult();
        Screen_ClearError();
        Screen_FlushRange();
        (void)ADC_Sampler_Restart();
        UartDebug_Printf("RANGE %u %ld\r\n", (unsigned)next, (long)Relay_GetOhm());
        return;
    }

    /* 处理重扫请求 */
    if (s_scan_request != 0u)
    {
        s_scan_request = 0u;
        Screen_SetStage("SCAN");
        if (FreqScan_Select() != 0u)
        {
            Screen_SetStage("LOOP");
        }
        else
        {
            Screen_SetStage("LOOP");
            Screen_ReportError(SCREEN_E04_CIR_UI_LOW);
        }
        s_scan_fail_count = 0u;
        return;
    }

    /* 自动重扫机制：连续失败 3 次后自动重新扫描（与 b0 无关） */
    if (s_scan_fail_count >= 3u)
    {
      Screen_SetStage("SCAN");
      if (FreqScan_Select() != 0u)
      {
        Screen_SetStage("LOOP");
      }
      else
      {
        Screen_SetStage("LOOP");
        Screen_ReportError(SCREEN_E04_CIR_UI_LOW);
      }
      s_scan_fail_count = 0u;
      return;
    }

    f = Measure_GetFrequency();

    /* 外部信号源必须同步到同一频率,固件只改自己的 DFT/窗口频率。 */
    Freq_SourceSet(f);                    /* 空实现:激励由外部信号源提供,固件不设定 */
    if (ADC_Sampler_SetFrequency(f) == 0u)
    {
      Screen_ReportError(SCREEN_E02_CIR_CFG);
      Screen_ClearResult();
      ++s_scan_fail_count;
      Screen_Poll();
      HAL_Delay(10u);
      return;
    }

    /* ADC 已按 f 开始新窗口,这时才让 t10 显示 f。
     * 若显示频率变了,清掉上一频率的读数,避免 t0~t9 还是旧频率的结果。 */
    if (Screen_SetActiveFreq(f) != 0u)
    {
      Screen_ClearResult();
    }

    /* 等本频率的窗口完成。用墙钟比较,不用循环计数(见 MEASURE_WAIT_TIMEOUT_MS)。
     * 期间检测 B1：如果用户在等待窗口时按 b1 切档，提前结束当前窗口并立即处理。 */
    deadline = HAL_GetTick() + MEASURE_WAIT_TIMEOUT_MS;
    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
      /* 检测 B1 按键：窥视队列，发现 B1 则提前结束窗口 */
      if (Screen_PeekKey() == SCREEN_KEY_B1)
      {
        b1_abort = 1u;
        break;
      }

      if (ADC_Sampler_GetWindow(&window) != 0u)
      {
        got = 1u;
        break;
      }
      Screen_Poll();
      HAL_Delay(1u);
    }

    /* B1 提前终止：丢弃当前窗口，立即处理切档 */
    if (b1_abort != 0u)
    {
      uint8_t next = Relay_Next();       /* 阻塞约 20 ms：先断后合 */

      CircuitID_SetFeedbackResistor(Relay_GetOhm());
      Screen_ClearResult();
      Screen_ClearError();
      Screen_FlushRange();
      (void)ADC_Sampler_Restart();
      UartDebug_Printf("RANGE %u %ld\r\n", (unsigned)next, (long)Relay_GetOhm());
      return;
    }

    if (got == 0u)
    {
      Screen_ReportError(SCREEN_E12_WINDOW_TIMEOUT);
      Screen_ClearResult();               /* 不留上一频率的陈旧读数 */
      ++s_scan_fail_count;
      return;
    }

    /* 新窗口开始计算前清掉上一窗口的错误显示；若本窗口仍异常，
     * CircuitID_Compute() 会通过 Screen_ReportError() 重新置位。 */
    Screen_ClearError();

    if (CircuitID_Compute(&window, &frf_point) != 0u)
    {
      Screen_SetResult(&frf_point);
      UartDebug_LogMeasurement(&frf_point, &window, Relay_Get());
      s_scan_fail_count = 0u;
    }
    else
    {
      Screen_ClearResult();               /* 计算失败同样清掉旧读数 */
      ++s_scan_fail_count;
    }
}

