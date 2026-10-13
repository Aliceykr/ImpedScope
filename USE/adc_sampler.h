#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 两个 ADC 各自单通道,由同一个 TIM6 TRGO 触发,DMA 循环写入各自的缓冲区。
 *   ADC1,PA0_C(ADC1_INP0):DUT 电压 Ui
 *   ADC2,PA1_C(ADC2_INP1):电流转换后的电压 Uo
 *
 * 频率由主循环的扫描计划给出(freq_plan)。每设一个新频率就开始一个窗口:在两路上
 * 做该频率的单频 DFT,窗口结束后得到 Ui、Uo 的复数分量,交给 CircuitID_Compute()
 * 计算 H、Z、Y。两路由同一 DMA 块边界开始累加,样本严格对齐。
 */

/* TIM6 触发频率:APB1 定时器时钟 120 MHz(APB1 = 60 MHz,分频不为 1 故定时器时钟加倍),
 * PSC=3、ARR=99 得 120 MHz / (3+1) / (99+1) = 300 kHz。
 *
 * 取 300 kHz 的依据:5 个测量频率 100/300/1000/3000/10000 Hz 之比为 1:3:10:30:100,
 * 故 Fs/f 全为整数,窗口取 10 个周期即为整数个周期,DFT 泄漏为零。
 *
 * 注意:本宏是纯软件参数,只用于 DFT 旋转因子与窗口长度计算。ADC 的硬件触发率由
 * tim.c 中 TIM6 的 PSC/ARR 决定(经 TRGO 触发两路 ADC),两者没有编译期或运行期
 * 联动,必须成对修改;只改一边会让 DFT 频率轴静默错 3.33 倍且不报任何错。 */
#define ADC_SAMPLER_RATE_HZ          300000.0f

/* 每个半缓冲区(一个 DMA 块)的采样点数。 */
#define ADC_SAMPLER_PAIRS_PER_HALF   64u

/* 窗口长度:覆盖 CYCLES 个信号周期,并限制在 [WINDOW_MIN, WINDOW_MAX] 之间。
 *
 * Fs = 300 kHz 时,5 个频率的窗口长度整数周期校验:
 *   f (Hz) | Fs/f | N(10 周期) | 是否被钳位
 *     100  | 3000 |   30000    | 否
 *     300  | 1000 |   10000    | 否
 *    1000  |  300 |    3000    | 否
 *    3000  |  100 |    1000    | 否
 *   10000  |   30 |     300    | 否
 * 5 行全部落在 [WINDOW_MIN, WINDOW_MAX] 内,无钳位,窗口恒为整数个周期。
 *
 * 注意:WINDOW_MIN 仅作安全下限,对上面 5 个目标频率不生效。不要为提高平均
 *       深度把它调大,那会让 3000/10000 Hz 的窗口被抬高而重新引入非整数周期。 */
#define ADC_SAMPLER_WINDOW_CYCLES    10u
#define ADC_SAMPLER_WINDOW_MIN       256u
#define ADC_SAMPLER_WINDOW_MAX       400000u

/* 一个窗口的单频 DFT 结果。
 * 幅值换算:|U| = 2 * sqrt(re^2 + im^2) / sample_count,单位为 ADC 码值。
 * 两路使用同一个 sample_count,所以 H = Uo / Ui 与归一化无关。 */
typedef struct
{
    uint32_t frequency_hz;
    uint32_t sample_count;
    double   ui_re;
    double   ui_im;
    double   uo_re;
    double   uo_im;
} ADC_Window_t;

/* 校准并启动两路 DMA,再启动 TIM6;成功返回 1,失败返回 0。 */
uint8_t ADC_Sampler_Init(void);

/* 设定当前频率并开始新窗口。频率相同则不重启;频率非法返回 0。 */
uint8_t ADC_Sampler_SetFrequency(uint32_t frequency_hz);

/* 取出最近完成的窗口;没有新窗口时返回 0。 */
uint8_t ADC_Sampler_GetWindow(ADC_Window_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ADC_SAMPLER_H */