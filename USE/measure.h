#ifndef MEASURE_H
#define MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 测量核心(由原 freq_plan / adc_sampler / circuit_id / calib 四个模块合并)。
 *
 * 一次测量的完整链路:
 *
 *   Freq_PlanGet(k)                 取第 k 个扫描频率
 *     -> Freq_SourceSet(f)          设定 AD9833(SPI1 主机,见 USE/ad9833.c)
 *     -> ADC_Sampler_SetFrequency(f)  开一个采样窗口
 *          ADC1 为主、ADC2 为从的硬件同步采样,由 TIM6 TRGO 触发,一对结果打包进
 *          CDR,由 ADC1 的 DMA 循环写入缓冲区;
 *          窗口内对两路做该频率的单频 DFT,得到 Ui、Uo 的复数分量
 *     -> ADC_Sampler_GetWindow()    取出窗口结果
 *     -> CircuitID_Compute()        算 H、Z、Y,并做两路标定修正
 *
 * 合并的理由:freq_plan 决定测量频率,adc_sampler 的 Fs 与窗口长度决定 DFT 频率轴,
 * circuit_id 与 calib 只消费 DFT 结果。四者共享同一个"频率轴"约定,拆成四个模块时
 * 该约定散落在四个头文件里;合并后集中在同一屏代码内,便于对照检查。
 */

/* ==========================================================================
 * 1. 频率扫描计划
 * ========================================================================== */

/* 赛题要求的 5 个测量频率个数。 */
#define FREQ_PLAN_COUNT   5u

/*
 * 设置信号源(AD9833)到目标频率。内部调用 USE/ad9833.c 的 AD9833_SetFrequency(),
 * 失败时上报 E19;调用前必须先 AD9833_Init()。
 *
 * 输出频率必须与 Freq_PlanGet() 返回的表值完全一致 —— DFT 用表值算旋转因子,
 * 两者不一致会让频率轴错位、窗口不再是整数周期(泄漏不为零),而且不会有任何报错。
 */
void Freq_SourceSet(uint32_t frequency_hz);

/* 取第 index 个扫描频率(0..FREQ_PLAN_COUNT-1);越界返回 0。 */
uint32_t Freq_PlanGet(uint8_t index);

/* ==========================================================================
 * 2. ADC 采样与单频 DFT
 * ========================================================================== */

/*
 * ADC1(主) 与 ADC2(从) 工作在双 ADC 规则组同步模式(ADC_DUALMODE_REGSIMULT):
 *   ADC1,PA0_C(ADC1_INP0):DUT 电压 Ui
 *   ADC2,PA1_C(ADC2_INP1):电流转换后的电压 Uo
 *
 * 一次 TIM6 TRGO 让两路在同一时刻开始采样(不再是"两路各自被同一个触发沿启动",
 * 那种方式两路之间还有几百纳秒的采样时刻偏差)。一对结果打包进公共数据寄存器
 * CDR 的一个 32 位字:
 *   bit[15:0]  = ADC1（主）= Ui
 *   bit[31:16] = ADC2（从）= Uo
 * 只由 ADC1 的 DMA(DMA1_Stream2,按字搬运)循环写入缓冲区,从 ADC 不使用 DMA。
 *
 * 每设一个新频率就开始一个窗口:在两路上做该频率的单频 DFT,窗口结束后得到
 * Ui、Uo 的复数分量,交给 CircuitID_Compute() 计算 H、Z、Y。
 * 两个字段来自同一次采样,故两路累加时共用同一个权重和同一个块序号,不存在对齐问题。
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

/* 校准两路 ADC,再启动双 ADC 同步转换(只用主 ADC 的 DMA),最后启动 TIM6;成功返回 1,失败返回 0。 */
uint8_t ADC_Sampler_Init(void);

/* 设定当前频率并开始新窗口。频率相同则不重启;频率非法返回 0。 */
uint8_t ADC_Sampler_SetFrequency(uint32_t frequency_hz);

/* 取出最近完成的窗口;没有新窗口时返回 0。 */
uint8_t ADC_Sampler_GetWindow(ADC_Window_t *out);

/* ==========================================================================
 * 3. 传递函数 H、阻抗 Z、导纳 Y
 * ========================================================================== */

/*
 * 黑盒电路频响辨识。频率由本模块第 1 节的扫描计划给出,Ui、Uo 由第 2 节的 ADC
 * 采样与 DFT 得到。前端是跨阻 I/V 放大器,运放反相端近似虚地,反馈电阻为 R:
 *   I = Ui * Y       Uo = -I * R
 *   H = Uo / Ui      Y  = -H / R
 *   Z = 1 / Y = -R / H
 *
 * 注意:两路前端的增益不同会使 |H| 有固定偏差,靠第 4 节的标定修正。
 */

/* 1：计算阻抗和导纳；0：只计算传递函数。 */
#define CIR_DUT_TOPOLOGY            1u

/* R 取值依据:Ui = 1 Vpp -> 幅度 ±0.5 V;ADC 范围 0~3.3 V,静态偏置
 * 1.65 V。不削顶要求 0.5*|H| < 1.5 V,取安全裕量 |H| <= 2,则 R <= 2/|Y|max
 * = 2/10 mS = 200 ohm。单挡即可覆盖 0.1~10 mS 全范围(最弱点 0.1 mS 仍约
 * 200 LSB)。原 1000 ohm 在 10 mS 时会削顶。 */
#define CIR_FEEDBACK_RESISTOR_OHM   200.0f

/* 单频幅值下限，单位：ADC 码值（2 * |S| / N）。低于此值视为噪声，待实测确定。 */
#define CIR_INPUT_MIN_AMP           4.0f
#define CIR_OUTPUT_MIN_AMP          4.0f

/* |H| 下限：低于此值阻抗趋于无穷，结果不可靠。 */
#define CIR_H_MIN_MAG               1e-6f

typedef struct
{
    float frequency_hz;
    float h_magnitude_dB;
    float h_phase_deg;

#if CIR_DUT_TOPOLOGY == 1u
    float z_mag_ohm;
    float z_mag_dB;
    float z_phase_deg;
    float y_mag_siemens;
    float y_phase_deg;
    float y_conductance_siemens;    /* G = Re[Y] = -h_re/R,单位 S */
    float y_susceptance_siemens;    /* B = Im[Y] = -h_im/R,单位 S */
#endif

    uint8_t valid;
} FRF_Point_t;

/* 初始化频响辨识模块。当前无状态需要初始化。 */
void CircuitID_Init(void);

/* 由一个窗口的 Ui、Uo 计算 H、Z、Y(含标定修正)；成功且结果有效时返回 1，否则返回 0。 */
uint8_t CircuitID_Compute(const ADC_Window_t *input, FRF_Point_t *output);

#ifdef __cplusplus
}
#endif

#endif /* MEASURE_H */
