#ifndef MEASURE_H
#define MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 测量核心:测量频率、ADC 采样与单频 DFT、两路标定、传递函数与阻抗。
 *
 * 默认单频点(1 kHz)测量,激励由外部信号源提供;频率可由应用层调用
 * Measure_SetFrequency() 在运行时修改(范围 100~100000 Hz)。
 *
 * 一次测量的完整链路:
 *
 *   Measure_GetFrequency()          取当前测量频率(默认 1 kHz,可运行时改)
 *     -> Freq_SourceSet(f)          空实现:激励在外部,固件不设定
 *     -> ADC_Sampler_SetFrequency(f)  开一个采样窗口
 *          ADC1 为主、ADC2 为从的硬件同步采样,由 TIM6 TRGO 触发,一对结果打包进
 *          CDR,由 ADC1 的 DMA 循环写入缓冲区;
 *          窗口内对两路做该频率的单频 DFT,得到 Ui、Uo 的复数分量
 *     -> ADC_Sampler_GetWindow()    取出窗口结果
 *     -> CircuitID_Compute()        算 H、Z、Y,并做两路标定修正
 *
 * 这几个环节共享同一个频率轴约定:Measure_GetFrequency() 的值既是外部信号源的
 * 应有输出,也是 DFT 旋转因子的来源;两者必须一致,否则频率轴错位且不报错。
 */

/* ==========================================================================
 * 1. 测量频率
 * ========================================================================== */

/* 默认频率计划个数。当前只放一个 1 kHz 默认值;屏幕可在运行时覆盖。 */
#define FREQ_PLAN_COUNT   1u

/*
 * 保留的接口:激励源在外部(信号源直连 Ui),固件不再驱动任何 DDS。
 * 本函数当前是空实现,只为保持调用契约,实现见 measure.c 第 1 节。
 *
 * 约定:输出频率必须与 Measure_GetFrequency() 一致 —— DFT 的旋转因子用的是
 * 运行时频率,两者不一致会让频率轴错位、窗口不再是整数周期(泄漏不为零),
 * 且不会有任何报错。
 */
void Freq_SourceSet(uint32_t frequency_hz);

/* 取第 index 个测量频率(0..FREQ_PLAN_COUNT-1);越界返回 0。 */
uint32_t Freq_PlanGet(uint8_t index);

/* 运行时测量频率。默认 1000 Hz;应用层收到屏幕命令后调用 Measure_SetFrequency()。
 * 只改变固件的 DFT/窗口频率,不改变外部信号源,源必须同步到同一频率。
 * 范围 100~100000 Hz:100 Hz 时 100 周期窗口为 1 s,主循环超时已放宽到 1500 ms;
 * 高于 100 kHz 时窗口点数接近 ADC_SAMPLER_WINDOW_MIN,不再是整数周期。 */
uint32_t Measure_GetFrequency(void);
uint8_t Measure_SetFrequency(uint32_t frequency_hz);

/* ==========================================================================
 * 2. ADC 采样与单频 DFT
 * ========================================================================== */

/*
 * ADC1(主) 与 ADC2(从) 工作在双 ADC 规则组同步模式(ADC_DUALMODE_REGSIMULT):
 *   ADC1,PA0_C(ADC1_INP0):DUT 电压 Ui
 *   ADC2,PA1_C(ADC2_INP1):电流转换后的电压 Uo
 *
 * 一次 TIM6 TRGO 让两路在同一时刻开始采样,两路之间没有采样时刻偏差。一对结果
 * 打包进公共数据寄存器 CDR 的一个 32 位字:
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
 * 取 300 kHz 的依据:测量频率 1 kHz 时 Fs/f = 300000/1000 = 300 为整数,
 * 窗口取整数个周期即可做到 DFT 泄漏为零;同时 300 kHz 对 1 kHz 有 300 倍过采样,
 * 1 kHz 的谐波(2k/3k/...)都落在远离测量频点的 DFT 栅格上。
 *
 * 注意:本宏是纯软件参数,只用于 DFT 旋转因子与窗口长度计算。ADC 的硬件触发率由
 * tim.c 中 TIM6 的 PSC/ARR 决定(经 TRGO 触发两路 ADC),两者没有编译期或运行期
 * 联动,必须成对修改;只改一边会让 DFT 频率轴静默错位且不报任何错。 */
#define ADC_SAMPLER_RATE_HZ          300000.0f

/* 每个半缓冲区(一个 DMA 块)的采样点数。 */
#define ADC_SAMPLER_PAIRS_PER_HALF   64u

/* 窗口长度:覆盖 CYCLES 个信号周期,并限制在 [WINDOW_MIN, WINDOW_MAX] 之间。
 *
 * 单频点 1 kHz 时:Fs/f = 300(整数),取 100 个周期
 *   N = 100 × 300 = 30000 点,窗口时长 = 30000 / 300000 = 100 ms
 * N 落在 [WINDOW_MIN, WINDOW_MAX] 内,不被钳位,窗口恒为整数个周期,泄漏为零。
 *
 * 取 100 个周期是为了压低噪声底:相干性判据的噪声底 kappa ≈ sqrt(2/N),
 * N = 30000 时约 0.008,相对门限 CIR_MIN_COHERENCE(0.2)有 20 倍以上余量。
 * 相比 10 个周期(3000 点),相干增益 sqrt(N) 从 55 提升到 173。
 *
 * 窗口时长 100 ms 必须明显小于主循环的等待超时(默认 1 kHz;100 Hz 时为 1 s,
 * 所以超时取 1500 ms)。
 * 不要为提高平均深度把 CYCLES 抬到窗口时长接近该超时值,那会让每个窗口都报 E12。
 *
 * 注意:WINDOW_MIN 仅作安全下限,对 1 kHz 不生效。 */
#define ADC_SAMPLER_WINDOW_CYCLES    100u
#define ADC_SAMPLER_WINDOW_MIN       256u
#define ADC_SAMPLER_WINDOW_MAX       400000u

/* ADC 的参考电压 VREF+，单位 V。16 位分辨率下满量程 65535 个码值对应 VREF+，
 * 所以 电压 = 码值 * VREF+ / 65535。
 *
 * 本工程按 VREF+ = VDDA = 3.3 V 计算，**必须与实际接线一致**：如果板上 VREF+ 由
 * 独立基准（例如 2.5 V）驱动，或 VDDA 不是 3.3 V，就改这里，否则 t7、t8 显示的
 * 电压会整体差一个固定比例。这个比例不影响 H、Z、Y（它们都是两路之比或对 R 的
 * 比值），只影响"屏幕上的电压是不是真值"。
 * 校验方法：在 PA0_C 上加一个已知直流电压，看 t7 的读数是否成比例。 */
#define ADC_SAMPLER_VREF_V           3.3f

/* ADC 码值 → 伏特。只用于 t7、t8 显示，不参与 H、Z、Y 的计算。 */
#define ADC_SAMPLER_LSB_TO_V         ((double)ADC_SAMPLER_VREF_V / 65535.0)

/* 削波(贴 ADC 量程边)检测的配置与计数。
 *
 * 为什么需要它:ADC 是单端 0~65535,超出量程的样本被钳在边界上。钳平后的波形
 * 仍带很强的基波分量,所以幅值门限和相干性判据都拦不住 —— 屏幕上会显示看着
 * 完全正常的 |Y|、phiY、G、B,而数据其实是错的。这是本工程唯一一类"静默给出
 * 错误结果"的失效,必须在代码里显式检测。
 *
 * 用贴边阈值而不是精确等于 0/65535:真实削顶时受噪声与运放输出阻抗影响,
 * 边界样本会在 0 附近抖动若干码值,不会精确停在 0。
 *
 * 局限:本检测抓的是"ADC 端已经贴边"。若前端电路在 ADC 之前就自己饱和
 * (例如运放先削顶),ADC 看到的是尚未贴边的"平顶"波形,这里抓不到;
 * 那种情况只能靠相干性判据(削顶产生谐波使 kappa 下降)间接反映。 */
#define CIR_CLIP_LOW_CODES          64u     /* <= 此码值算贴低边(0 V 侧)      */
#define CIR_CLIP_HIGH_CODES         65471u  /* >= 此码值算贴高边(3.3 V 侧)    */

/* 贴边样本占窗口的比例上限,超过即判定削波,单位千分之一。
 *
 * 取 1%(10 permille)的依据 —— 用解析式算出的贴边比例:
 *   干净正弦,幅度到 1.60 V(距 1.65 V 轨道还有 0.05 V)  ->  0%
 *   恰好触轨,幅度 1.65 V                                  ->  4.0%
 *   轻微削顶 0.05 V                                        -> 16.0%
 *   直流偏置错(中点 0 V,负半周全削)                       -> 50.2%
 * 干净与削顶之间是一个很宽的间隙(0% vs >=4%),所以门限取在中间,
 * 既不会因噪声误报,也不会漏掉轻微削顶。窗口 30000 点时 1% = 300 个样本。
 *
 * 注意:判定时用整数运算(sum * 1000 > N * 10),不引入浮点除法。 */
#define CIR_CLIP_MAX_PERMILLE       10u

/* 一个窗口内两路贴量程边界的样本计数。 */
typedef struct
{
    uint32_t ui_low;    /* Ui 贴低边(0 V 侧)的样本数 */
    uint32_t ui_high;   /* Ui 贴高边(3.3 V 侧)的样本数 */
    uint32_t uo_low;    /* Uo 贴低边的样本数 */
    uint32_t uo_high;   /* Uo 贴高边的样本数 */
} ADC_ClipCount_t;
/* 一个窗口的单频 DFT 结果。
 * 幅值换算:|U| = 2 * sqrt(re^2 + im^2) / sample_count,单位为 ADC 码值;
 * 换成电压用 ADC_SAMPLER_LSB_TO_V(= VREF+ / 65535)。
 * 该幅值是单频分量的峰值,其 1/sqrt(2) 即该分量的有效值。
 * 两路使用同一个 sample_count,所以 H = Uo / Ui 与归一化无关。
 *
 * ui_rms / uo_rms 是去掉直流后的交流有效值,只用于判断窗口里到底有没有单频信号
 * (相干性判据),不参与 H、Z、Y 的计算。 */
typedef struct
{
    uint32_t frequency_hz;
    uint32_t sample_count;
    double   ui_re;
    double   ui_im;
    double   uo_re;
    double   uo_im;
    double   ui_rms;        /* 窗口内 Ui 去掉直流后的交流有效值，单位 ADC 码值 */
    double   uo_rms;        /* 窗口内 Uo 去掉直流后的交流有效值，单位 ADC 码值 */
    ADC_ClipCount_t clip;   /* 窗口内两路贴 ADC 量程边界的样本计数,用于削波判定 */
} ADC_Window_t;

/* 校准两路 ADC,再启动双 ADC 同步转换(只用主 ADC 的 DMA),最后启动 TIM6。
 *
 * 返回值区分失败原因,便于串口屏显示确切错误码:
 *   1 = 成功
 *   0 = 通用失败(校准或 DMA/定时器启动失败)          -> 调用方报 E01
 *   2 = 双 ADC 同步模式或数据打包格式未生效(CCR 回读不符)-> 调用方报 E16
 *
 * 2 是"静默失效"防护:.ioc 没有多模式选项,若 adc.c USER CODE 区的配置被丢失,
 * 或 DMA 对齐被改回半字,都会让 H/Z/Y 悄悄偏掉且不报错,所以在这里拦下。 */
uint8_t ADC_Sampler_Init(void);

/* 设定当前频率并开始新窗口。频率相同则不重启;频率非法返回 0。 */
uint8_t ADC_Sampler_SetFrequency(uint32_t frequency_hz);

/* 取出最近完成的窗口;没有新窗口时返回 0。 */
uint8_t ADC_Sampler_GetWindow(ADC_Window_t *out);

/* 以当前频率强制重开一个窗口（同频也重开）。没有设定过频率时返回 0。
 * 换档后调用，丢弃切换瞬间的样本。 */
uint8_t ADC_Sampler_Restart(void);

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
 * 注意:两路前端的增益不同会使 |H| 有固定偏差,靠第 3 节的标定修正。
 */

/* 1：计算阻抗和导纳；0：只计算传递函数。
 *
 * 0 是合法的编译期选项：此时 FRF_Point_t 不含 Z/Y 字段，screen.c 里依赖 Y 的
 * t1~t4 由同一宏条件编译为显示 "-"。可用工程全局宏覆盖本值，不必改源文件。 */
#ifndef CIR_DUT_TOPOLOGY
#define CIR_DUT_TOPOLOGY            1u
#endif

/* 固定反馈电阻（1.5 kΩ）。relay_range 模块上电只闭合默认档 K2，R_eff 恒为
 * 1500 Ω；应用层在 Relay_Init() 后调用 CircuitID_SetFeedbackResistor() 同步。
 * R 只参与 Y=-H/R 换算，不改变前端增益。 */
#define CIR_FEEDBACK_RESISTOR_OHM   1500.0f

/* 单频幅值下限，单位：ADC 码值（2 * |S| / N）。低于此值只剩量化噪声，结果没有
 * 意义；这只是一道兜底门槛，"有没有信号"由下面的相干性判据负责。 */
#define CIR_INPUT_MIN_AMP           4.0f
#define CIR_OUTPUT_MIN_AMP          4.0f

/* 相干性下限。定义 kappa = (2|S|/N) / (sqrt(2) * RMS_ac)：
 *   窗口里是单频正弦时 kappa = 1.0；
 *   窗口里只有噪声时 kappa 约为 sqrt(2/N)（N = 30000 时约 0.008，差一百多倍）。
 * 该判据与信号绝对大小无关,不需要现场标定,回答的就是"到底有没有在驱动被测电路"：
 * 输入悬空、信号源没接时,噪声落在测量频点上的分量仍可能让幅值门限
 * 通过(实测就是这样),但 kappa 会远低于此值,于是报 E14/E15。
 * 取 0.2 留了 20 倍以上的噪声余量,同时容忍波形带少量谐波失真(方波约 0.9、三角波约 1.0)。 */
#define CIR_MIN_COHERENCE           0.2f

/* |H| 下限:低于此值阻抗趋于无穷,结果不可靠。 */
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

/* 运行时反馈电阻（Ω）。Relay_Init() 后由应用层调用一次，固定为 1500 Ω；
 * <=0 时 CircuitID_Compute() 报 E02（CIR_CFG）。 */
void  CircuitID_SetFeedbackResistor(float ohm);
float CircuitID_GetFeedbackResistor(void);

/* 由一个窗口的 Ui、Uo 计算 H、Z、Y(含标定修正)；成功且结果有效时返回 1，否则返回 0。 */
uint8_t CircuitID_Compute(const ADC_Window_t *input, FRF_Point_t *output);

#ifdef __cplusplus
}
#endif

#endif /* MEASURE_H */
