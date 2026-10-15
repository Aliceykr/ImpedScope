# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上。主循环按 `measure.c` 第 1 节的测量频率设定激励:当前为**单频点 1 kHz**,激励由**外部信号源**直接加到 `Ui` 节点(板上原 DDS 已移除,见第 12 节)。STM32 用两路 ADC 同时采集 DUT 的电压 `Ui` 和电流转换后的电压 `Uo`,在该频率上做单频 DFT,计算导纳 `|Y|`、φY、G、B,并把结果显示在串口屏上。

主要功能：

- 测量频率为**单频点 1 kHz**(`FREQ_PLAN_COUNT = 1`);激励由外部信号源提供,固件不驱动任何 DDS
- ADC1、ADC2 硬件同步采样(PA0_C、PA1_C,主从同步模式),由 TIM6 触发,结果打包进 CDR、DMA 循环写入缓冲区
- 每个频率开一个采样窗口，对两路做单频 DFT，得到 `Ui`、`Uo` 的复数分量
- 传递函数 `H = Uo / Ui`，阻抗 `Z = -R / H`，导纳 `Y = -H / R`
- 单频幅值过小时丢弃该频点
- UART4 驱动串口屏，显示频率、阻抗和运行状态

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ measure.h/.c        测量核心:测量频率、ADC 采样与单频 DFT、
│  │                      H/Z/Y 计算、双路标定修正
│  └─ screen.h/.c         串口屏 UART4 输出和错误上报
├─ Core/
│  ├─ Src/main.c          系统初始化和主循环（USER CODE 区）
│  ├─ Src/spi.c           SPI1 主机配置（CubeMX 生成，USER CODE 区留空）
│  ├─ Src/stm32h7xx_it.c  中断入口
│  ├─ Src/usart.c         UART4 配置（CubeMX 生成）
│  ├─ Src/adc.c           ADC1、ADC2 配置（CubeMX 生成，单通道、TIM6 触发、主从同步模式）
│  └─ Src/tim.c           TIM6 配置（CubeMX 生成，TRGO 触发 ADC）
├─ Drivers/               STM32 HAL 和 CMSIS
├─ MDK-ARM/               Keil 工程文件
└─ 9_30.ioc               STM32CubeMX 配置文件
```

`USE/` 保存用户逻辑。CubeMX 重新生成代码时应保留该目录，不修改 CubeMX 生成的 `.c` 文件中的业务逻辑。

## 3. 硬件模型

DUT 接在 `Ui` 节点和虚地之间。运放反相端接近 0 V（虚地），DUT 的电流全部流过反馈电阻 `R`：

```text
信号源 ── Ui ── DUT ──┬── 虚地（运放反相端）
                      │
                      └── 反馈电阻 R ── 运放输出 ── Uo
```

电路关系：

```text
I  = Ui × Y        流过 DUT 的电流（Y 为 DUT 导纳）
Uo = -I × R        运放把电流转换为输出电压
H  = Uo / Ui = -Y × R
```

由此可以从 `H` 反推 DUT 的导纳和阻抗。

## 4. 数据处理流程

```text
主循环(measure.c 第 1 节的测量频率)─> 取频率 f(当前:1 kHz)
                        │  Freq_SourceSet(f)  ── 空实现:激励在外部,固件不设定
                        ▼
          ADC_Sampler_SetFrequency(f)  ── 设定频率，开始新的采样窗口
                        │
                        │  DMA 中断中从 CDR 拆出两路样本并累加单频 DFT
                        ▼
          ADC_Sampler_GetWindow()     ── 窗口完成时取出 Ui、Uo 的复数分量
                        │  （带墙钟超时；超时报 E12 并清结果）
                        ▼
          CircuitID_Compute()         ── 计算 H、Z、Y、G、B，标定修正，检查有效性
                        │  有效结果
                        ▼
          Screen_SetResult()          ── 保存最近一次有效结果
                        │
                        ▼
          Screen_Poll()               ── 每 500 ms 通过 UART4 刷新串口屏
```

主循环逐个取 `Freq_PlanGet(k)` 的频率。当前 `FREQ_PLAN_COUNT = 1`,所以每轮只测 1 kHz,循环结构保留以便将来恢复多频点。窗口完成后调用计算函数,成功(返回 1)才更新显示;失败或超时会清掉结果,屏幕显示 "-" 而不是上一轮的陈旧读数。

## 5. 计算方法

采样窗口与 DFT（第 2 节）、H、Z、Y（第 4 节）都在 [USE/measure.c](USE/measure.c) 中，两路标定表在第 3 节。

### 5.1 采样窗口

测量频率 `f` 对应一个窗口,窗口长度为:

```text
N = round(100 × fs / f),限制在 [256, 400000] 之间
fs = 300 kHz（TIM6 触发频率）
```

窗口覆盖 **100 个信号周期**。当前 f = 1 kHz、fs = 300 kHz 时:Fs/f = 300(整数),N = 100 × 300 = **30000 点**,窗口时长 **100 ms**,是整数个周期,理论上无泄漏。

取 100 个周期(而非更少)是为了压低噪声底:相干性判据的噪声底 κ ≈ sqrt(2/N),N = 30000 时约 **0.008**,相对门限 `CIR_MIN_COHERENCE`(0.2)有 20 倍以上余量;相干增益 sqrt(N) ≈ 173。

⚠️ 窗口时长必须明显小于主循环的等待超时(`MEASURE_WAIT_TIMEOUT_MS` = 500 ms)。当前 100 ms 安全;若把 `ADC_SAMPLER_WINDOW_CYCLES` 抬到窗口接近 500 ms,每个窗口都会报 E12。

```text
ω = 2π f / fs
S = Σ (x_n − x̄) · e^{−j ω n}，n = 0 … N−1
x̄ 为窗口内的平均值（去除直流）
```

`S_i` 对应 `Ui`（ADC1），`S_o` 对应 `Uo`（ADC2）。

单频幅值换算为 ADC 码值：

```text
|U| = 2 |S| / N
```

`|U|` 低于 `CIR_INPUT_MIN_AMP` 报 E04，低于 `CIR_OUTPUT_MIN_AMP` 报 E05。两个门限都是占位值，待实测确定。

幅值门限只说明"这个频点上有多少能量"，说不出这些能量是不是来自信号源，所以另有一个与信号绝对大小无关的相干性判据：

```text
kappa = |U| / (sqrt(2) · RMS_ac)
```

`RMS_ac` 是同一个窗口里去直流后的交流有效值。窗口里是单频正弦时 `kappa = 1.0`；窗口里只有噪声时 `kappa ≈ sqrt(2/N)`（N = 30000 时约 0.008，差一百多倍）。`kappa` 低于 `CIR_MIN_COHERENCE`（默认 0.2）就报 E14（`Ui`）/ E15（`Uo`）—— 这正是"信号源没接、输入悬空"的情形：噪声落在测量频点上的分量能让幅值门限通过，却过不了相干性判据。两路的电压与相干性同时显示在串口屏的 `t7`、`t8`（第 7 节），便于现场判断本底有多大。

`t7`、`t8` 显示的是**去直流后的交流有效值（RMS）**，单位 V，由 `ADC_SAMPLER_LSB_TO_V = ADC_SAMPLER_VREF_V / 65535` 换算得到，可以直接和万用表、示波器的读数对照。本工程按 `VREF+ = VDDA = 3.3 V` 计算；板上基准不同的要改 `ADC_SAMPLER_VREF_V`（见第 6 节）。单频分量的峰值约为显示值的 `sqrt(2)` 倍（相干性接近 1 时该关系成立），门限 `CIR_INPUT_MIN_AMP` / `CIR_OUTPUT_MIN_AMP` 用的是峰值码值，不随基准变化。

### 5.3 传递函数 H = Uo / Ui

分子分母同乘 `conj(Ui)`，分母变为实数 `|Ui|²`：

```text
|Ui|²  = Ui_re² + Ui_im²
H_re   = (Uo_re·Ui_re + Uo_im·Ui_im) / |Ui|²
H_im   = (Uo_im·Ui_re − Uo_re·Ui_im) / |Ui|²
|H|²   = H_re² + H_im²
```

两路的窗口长度相同，`H` 与 DFT 的归一化无关，`H` 是无量纲的比值。

`H` 的幅值和相位：

```text
|H|       = √(|H|²)
增益 (dB) = 20 · log10(|H|)
相位 (°)  = atan2(H_im, H_re) × 180 / π
```

`H` 的结果若不是有限数，报 E06。

### 5.4 频率

测量频率由 `measure.c` 第 1 节的表给出(当前:单频点 1 kHz)。

### 5.5 阻抗 Z 和导纳 Y（`CIR_DUT_TOPOLOGY = 1` 时计算）

阻抗：

```text
Z = -R / H = -R · conj(H) / |H|²

Z_re = -R · H_re / |H|²
Z_im = +R · H_im / |H|²
|Z| = R / |H|
Z 相位 = atan2(Z_im, Z_re) × 180 / π
```

若 `|H|² ≤ CIR_H_MIN_MAG²`，Z 趋于无穷大，结果不可靠，报 E08 并丢弃。

导纳：

```text
Y = -H / R

Y_re = -H_re / R
Y_im = -H_im / R
|Y| = |H| / R
Y 相位 = atan2(Y_im, Y_re) × 180 / π
```

Z 或 Y 的任一结果非有限，报 E09 并丢弃。

`|Z|` 只依赖 `|H|` 和 `R`，不需要 ADC 的参考电压。但 `|H|` 的绝对值会受两路增益差影响，见第 11 节。

### 5.6 输出

| 字段 | 含义 | 单位 |
|---|---|---|
| `frequency_hz` | 测量频率(来自 `Freq_PlanGet()`) | Hz |
| `h_magnitude_dB` / `h_phase_deg` | 传递函数增益和相位 | dB / ° |
| `z_mag_ohm` / `z_phase_deg` | 阻抗幅值和相位 | Ω / ° |
| `y_mag_siemens` / `y_phase_deg` | 导纳幅值和相位 | S / ° |
| `y_conductance_siemens` | 电导 G = Re[Y] | S |
| `y_susceptance_siemens` | 电纳 B = Im[Y] | S |
| `z_mag_dB` | 阻抗幅值 dB | dB |
| `valid` | 1 表示结果有效 | — |

计算失败时输出结构全部清零，`valid = 0`。

串口屏显示频率、`|Y|`、φY、G、B（见第 7 节），另加状态/错误行 t5、心跳行 t6 与两路诊断行 t7、t8，以及传递函数行 t9。

### 5.7 数值示例

以 1 µF 电容为 DUT，`R = 200 Ω`，`f = 1 kHz`。信号源 1 Vpp（`Ui` 幅度 0.5 V）：

```text
Y  = jωC = j · 2π·1000·1e-6           ≈ j 6.283e-3 S
|Y|= √(G²+B²)                          ≈ 6.283 mS     ← 显示 t1
φY = atan2(B, G)                       = +90.00°      ← 显示 t2
G  = Re[Y]                             = 0.000 mS     ← 显示 t3
B  = Im[Y]                             ≈ +6.283 mS    ← 显示 t4

H  = -Y·R ≈ -j 1.257                   （|H| = 1.257，∠H = -90°）
Uo 幅度 = 0.5 V × |H|                  ≈ 0.63 V       （不削顶）
|Z| = 1/|Y|                            ≈ 159 Ω
```

**符号约定自检**：电容的 φY 应为 **+90°**（电流超前电压）。若屏幕显示 **-90°**，
说明 `measure.c` 里 `Y = -H/R` 的符号约定与本节不一致。

## 6. 参数配置

| 宏 | 所在文件 | 默认值 | 说明 |
|---|---|---|---|
| `ADC_SAMPLER_RATE_HZ` | `measure.h` | 300000 | ADC 采样率（TIM6 触发频率） |
| `ADC_SAMPLER_PAIRS_PER_HALF` | `measure.h` | 64 | 每半个 DMA 缓冲区的采样对数（一对 = `Ui`、`Uo` 一次同步采样） |
| `ADC_SAMPLER_WINDOW_CYCLES` | `measure.h` | 100 | 每个窗口覆盖的信号周期数(1 kHz 时为 30000 点 / 100 ms) |
| `ADC_SAMPLER_WINDOW_MIN` | `measure.h` | 256 | 窗口最短采样点数(对 1 kHz 不生效) |
| `ADC_SAMPLER_WINDOW_MAX` | `measure.h` | 400000 | 窗口最长采样点数 |
| `CIR_DUT_TOPOLOGY` | `measure.h` | 1 | 1：计算 Z 和 Y；0：只计算 H |
| `CIR_FEEDBACK_RESISTOR_OHM` | `measure.h` | 200.0 | 反馈电阻 R 的标称值，实际值需标定 |
| `CIR_INPUT_MIN_AMP` | `measure.h` | 4.0 | `|Ui|` 幅值下限（码值），占位，待实测 |
| `CIR_OUTPUT_MIN_AMP` | `measure.h` | 4.0 | `|Uo|` 幅值下限（码值），占位，待实测 |
| `CIR_MIN_COHERENCE` | `measure.h` | 0.2 | 相干性下限 `|U| / (sqrt(2)·RMS_ac)`，低于此值报 E14/E15 |
| `ADC_SAMPLER_VREF_V` | `measure.h` | 3.3 | ADC 参考电压 VREF+（V），只影响 `t7`/`t8` 显示的电压，必须与实际接线一致 |
| `CIR_H_MIN_MAG` | `measure.h` | 1e-6 | `|H|` 下限，低于此值 Z 不可靠 |


PA4 原先用作 DDS 的软件片选,现未使用,`.ioc` 里不分配。

## 7. 串口屏

UART4，9600 8N1，PC10 TX / PC11 RX。每 500 ms 发送 10 条指令（t0~t9），每条以 `FF FF FF` 结尾。串口屏页面需要建立同名文本框 `t0`~`t8`。

| 控件 | 内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `f:1000`，无结果时 `f:-` |
| `t1` | 导纳模 \|Y\|（mS） | `Y:1.000`，无结果时 `Y:-` |
| `t2` | 导纳角 φY（°） | `P:-12.34`，无结果时 `P:-` |
| `t3` | 电导 G = Re[Y]（mS） | `G:0.977`，无结果时 `G:-` |
| `t4` | 电纳 B = Im[Y]（mS） | `B:-0.213`，无结果时 `B:-` |
| `t5` | 运行阶段或最近一次错误 | `stage:LOOP`、`stage:E12 WINDOW_TIMEOUT x12` |
| `t9` | 传递函数 H 的幅值（dB）与相位（°） | `H:-3.97dB A:-88.62`，无结果时 `H:-` |
| `t6` | 心跳，每刷新周期 +1 | `beat:1234` |
| `t7` | 窗口诊断：`Ui` 交流有效值（V）与相干性 kappa | `Ui:0.1234V k:0.98`，尚无窗口时 `Ui:-` |
| `t8` | 窗口诊断：`Uo` 交流有效值（V）与相干性 kappa | `Uo:0.0988V k:0.97`，尚无窗口时 `Uo:-` |

### 7.1 运行阶段

`main.c` 的 USER CODE 区调用 `Screen_SetStage()` 打点，顺序为：

`BOOT → CLK → PERIPH → MODULE → SCAN → LOOP`

`BOOT`、`CLK` 发生在 UART 初始化之前，只保存在内存中，屏幕上看不到。`PERIPH` 起屏幕可见，设置时立即阻塞发送并保持约 150 ms。

### 7.2 错误码

非致命错误通过 `Screen_ReportError(code)` 计数，`t5` 显示最近一次错误及累计次数（16 位，溢出后回绕）。新阶段或新错误会覆盖之前的显示。

致命错误由 `Screen_Fatal()` 或 `Error_Handler()` 触发，关中断后阻塞发送，`t5` 锁定显示。

| 码 | 名称 | 位置 | 触发条件 | 类型 |
|---|---|---|---|---|
| E01 | HAL_INIT | CubeMX 生成的 `MX_*_Init()`，`ADC_Sampler_Init()` | 外设初始化或 ADC 校准、DMA 启动失败 | 致命 |
| E02 | CIR_CFG | `measure.c` | `R ≤ 0`、频率为 0 或窗口为空 | 非致命 |
| E03 | CIR_NAN_IN | `measure.c` | 窗口的 Ui、Uo 分量非有限 | 非致命 |
| E04 | CIR_UI_LOW | `measure.c` | `\|Ui\|` 幅值低于 `CIR_INPUT_MIN_AMP` | 非致命 |
| E05 | CIR_UO_LOW | `measure.c` | `\|Uo\|` 幅值低于 `CIR_OUTPUT_MIN_AMP` | 非致命 |
| E06 | CIR_H_NAN | `measure.c` | H 非有限 | 非致命 |
| E07 | CIR_OUT_NAN | `measure.c` | 频率、dB、相位输出非有限 | 非致命 |
| E08 | CIR_H_LOW | `measure.c` | `\|H\|` 过小，无法计算 Z | 非致命 |
| E09 | CIR_ZY_NAN | `measure.c` | Z 或 Y 非有限 | 非致命 |
| E10 | UART_TX | `screen.c` | `HAL_UART_Transmit_IT()` 返回非 OK | 非致命 |
| E11 | UART_BUSY | `screen.c` | 上一条指令未发完，本周期跳过 | 非致命 |
| E12 | WINDOW_TIMEOUT | `main.c` | 等待 ADC 窗口完成超时 | 非致命 |
| E13 | DDS_SPI | — | **已停用。** 固件不再驱动 DDS,保留编号占位 | — |
| E14 | CIR_UI_INCOH | `measure.c` | `Ui` 有交流分量但不是单频（`kappa < CIR_MIN_COHERENCE`），信号源未接或未输出 | 非致命 |
| E15 | CIR_UO_INCOH | `measure.c` | `Uo` 有交流分量但不是单频（`kappa < CIR_MIN_COHERENCE`） | 非致命 |
| E16 | ADC_DUAL_MODE | `measure.c` 的 `ADC_Sampler_Init()` | 双 ADC 同步模式或数据打包格式未生效(CCR 回读不符) | 致命 |
| E17 | ADC_OVR | `measure.c` 的 `HAL_ADC_ErrorCallback()` | ADC 溢出,本窗口样本时间轴不可信 | 非致命 |
| E18 | ADC_DMA | `measure.c` 的 `HAL_ADC_ErrorCallback()` | ADC 的 DMA 传输错误 | 非致命 |
| E19 | ADC_CLIP_UI | `measure.c` 的 `CircuitID_Compute()` | `Ui` 波形贴 ADC 量程边(削顶) | 非致命 |
| E20 | ADC_CLIP_UO | `measure.c` 的 `CircuitID_Compute()` | `Uo` 波形贴 ADC 量程边(削顶) | 非致命 |

时钟配置失败发生在 UART 就绪之前，屏幕无法显示，未纳入错误码表，只能用调试器定位。

## 8. 已知限制

- **窗口泄漏。** Fs = 300 kHz 下测量频率 1 kHz 的窗口是整数个周期(见 5.1 节),理论上泄漏为零;残余误差只来自晶振频率偏差与外部信号源实际输出与标称值的偏差。
- **两路增益未标定。** 两路 ADC 的增益误差会直接进入 `H`。见第 11 节。
- **两路输入路径有差异。** ADC1/ADC2 工作在规则组同步模式（见 10.2 节），采样时刻完全相同；残余相位误差来自 PA0_C/PA1_C 两个模拟开关与源阻抗的带宽差异（第 11 节第 2 点）。
- **每轮测量要等一个完整的窗口(100 ms)。** 窗口未完成就拿不到结果。超时上限是 `MEASURE_WAIT_TIMEOUT_MS`(500 ms),窗口时长必须明显小于它。
- **幅值门限是占位值。** `CIR_INPUT_MIN_AMP`、`CIR_OUTPUT_MIN_AMP` 为 4 个码值，需要用实际噪声水平确定。这两道门限只保证"有不小的交流分量"，判断"信号源是不是真的在驱动被测电路"靠相干性判据 `CIR_MIN_COHERENCE`（见 5.2 节），它不需要现场标定。
- **外部信号源的频率精度不在固件控制内。** 激励由外部信号源提供,固件无法校验其实际频率。若源偏离 1 kHz,DFT 会在错误频率上取分量,表现为 κ 下降或相位误差,而屏幕上与"DUT 特性"无法区分。请使用频率精度足够的信号源,或先用示波器/频率计校验。

## 9. 编译与维护

使用 Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx` 编译。工程基于：

- STM32CubeMX 6.18
- STM32H7 HAL FW 1.13
- Arm Compiler 6（ArmClang）V6.24 —— `.uvprojx` 的 `pArmCC` 已钉到 V6.24，换机器装的是别的版本时要同步改这一项
- 系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz
- SPI1 内核时钟 75 MHz（PLL2），ADC 时钟 37.5 MHz（PLL2 经 /2）

重新生成 CubeMX 代码后，请检查：

1. Keil 工程中仍包含 `USE/measure.c` 和 `USE/screen.c`。
2. 头文件路径仍包含 `../USE`。
3. `.ioc` 与 `spi.c` 中 SPI1 仍为主机、16 位、CPOL=High / CPHA=1Edge(Mode 2)、软件 NSS、波特率预分频 8;SPI1 不使用 DMA 和中断。当前 SPI1 无从设备。
4. `spi.c` 的 MspInit 只配置 PB5/PG9/PG11 三根 SPI1 引脚,不碰 PA4。
5. `main.c` 的 USER CODE 区不调用任何 DDS 驱动(激励源在外部,`Freq_SourceSet()` 是空实现)。
6. `main.c` 的 USER CODE 区仍调用 `Screen_Init()`、`Screen_SetStage()`、`ADC_Sampler_Init()`、`CircuitID_Init()`、`Freq_PlanGet()`/`Freq_SourceSet()` 和 `Screen_Poll()`。
7. `.ioc` 中 UART4 仍为 9600 8N1（PC10/PC11），且 UART4 全局中断已使能。
8. `.ioc` 中 ADC1、ADC2 均为单通道、连续转换关闭、DMA 循环、由 TIM6 TRGO 触发,ADC 时钟分频为 `ADC_CLOCK_ASYNC_DIV2`;`.ioc` 的 `Dma.ADC1.2` 对齐为 **Word**(`DMA_*DATAALIGN_WORD`),与同步模式的 32 位 CDR 一致。
9. `main.c` 的 USER CODE 区在 `MX_TIM6_Init()` 之后调用 `ADC_Sampler_Init()`。
10. `adc.c` 的双 ADC 同步配置(`ADC_DUALMODE_REGSIMULT`、`DualModeData`、`TwoSamplingDelay`)**已放在 `USER CODE BEGIN ADC1_Init 2` 区**,CubeMX 重新生成不会覆盖,无需再手工改回;`.ioc` 里没有多模式选项属于正常现象。ADC1 的 DMA 字对齐同上。

## 10. ADC 采样

ADC1（主）与 ADC2（从）工作在双 ADC 规则组同步模式，各自对一路电压做单通道采样：一次 TIM6 TRGO 让两路在同一时刻开始转换，一对结果打包进公共数据寄存器 CDR 的一个 32 位字（低半字 ADC1 = `Ui`，高半字 ADC2 = `Uo`），由 ADC1 的 DMA 循环写入缓冲区。采样结果不再直接显示，而是用于窗口 DFT。代码见 [USE/measure.c](USE/measure.c) 第 2 节。

### 10.1 通道与引脚

| 通道 | 引脚 | ADC 通道 | 含义 |
|---|---|---|---|
| ADC1 | PA0_C | `ADC_CHANNEL_0`（ADC1_INP0） | DUT 电压 `Ui` |
| ADC2 | PA1_C | `ADC_CHANNEL_1`（ADC2_INP1） | 电流转换后的电压 `Uo` |

`_C` 为直连快速通道，经 SYSCFG 模拟开关接入 ADC。引脚配置为模拟模式，无上下拉。模拟开关为 `OPEN`，ADC 从 `_C` 焊盘采样。`PA0_C` 与普通 `PA0` 是两个焊盘，中间靠模拟开关连接，同一时刻只能选其中一条路径。

### 10.2 采样时序

- 多模式为同步模式（`ADC_DUALMODE_REGSIMULT`，16 位分辨率对应 `ADC_DUALMODEDATAFORMAT_32_10_BITS`）。由 TIM6 TRGO（上升沿）触发：一次触发同时启动两路。定时器 PSC=3、ARR=99，按 120 MHz 计数时钟计算，采样速率为 300 kHz。
- 关闭连续转换，每次触发完成一次单通道转换。采样时间为 1.5 个 ADC 周期。
- ADC 时钟由 PLL2 的 75 MHz 经 /2 分频为 37.5 MHz，低于 16 位 ADC 的时钟上限（50 MHz）。
- ADC 在 `ADC_Sampler_Init()` 中先回读 CCR 核对同步配置(见下条),再做 `ADC_CALIB_OFFSET_LINEARITY` 校准(两路都做),然后用主 ADC 的句柄调用 `HAL_ADCEx_MultiModeStart_DMA()` 启动同步转换,接着使能 `ADC_IRQn`(用于溢出上报),最后启动 TIM6。顺序不能变:校准会先把 ADC 关掉。
- 只有 ADC1 的 DMA(`DMA1_Stream2`,`DMA_REQUEST_ADC1`)在搬运,外围与存储都是 32 位字对齐、循环模式;每半个缓冲区(64 对样本)回调一次。ADC2 的 DMA 配置(`DMA1_Stream1`)保留在 `adc.c` 里,但同步模式下不会启动。
- `HAL_ADCEx_MultiModeConfigChannel()` 写的是两个 ADC 共用的 CCR,而 CCR 里的 DUAL/DAMDF/DELAY 位只有在两个 ADC 都处于关闭状态时才写得进去(HAL 内部按“任一 ADC 已使能就整段跳过”判断)。因此这几行**放在 `adc.c` 的 `USER CODE BEGIN ADC1_Init 2` 区**(在 `HAL_ADC_Init()` 之后、任何校准或启动调用之前),CubeMX 重新生成不会覆盖。
- **配置有运行期回读校验。** `ADC_Sampler_Init()` 会读回 CCR 核对同步模式(`ADC_DUALMODE_REGSIMULT`)与数据打包格式(`32_10_BITS`);不符时返回 2,`main.c` 据此报 **E16 并走致命路径**。这挡住的是“静默退回独立模式”:那种情况下两路不再同时采样,`H` 会悄悄偏掉,而相干性判据挡不住(两路各自看着都像单频)。
### 10.3 窗口与对齐

- `ADC_Sampler_SetFrequency(f)` 设定频率，并从当前块序号开始新窗口。同步模式下两路只有一个块序号，不存在起点对齐问题。
- DMA 回调里从每个 32 位字中同时取出两路样本、按同一个权重一并累加，所以两路的累加点数必然相同，到齐即窗口完成。
- 累加器同时累计两路样本的平方和，窗口完成时算出各自的交流有效值（`ui_rms`、`uo_rms`），供 `CircuitID_Compute()` 做相干性判据（见 5.2 节）。这两个值不参与 H、Z、Y 的计算。
- `ADC_Sampler_GetWindow()` 取出完成的窗口，主循环交给 `CircuitID_Compute()`。
- 同一频率重复调用 `SetFrequency` 不会重启窗口。
- **溢出(OVR)会作废当前窗口。** 双 ADC 同步模式下 CDR 只有一个,溢出意味着某个采样时刻的结果被覆盖,本窗口的时间轴不再可信。`HAL_ADC_ErrorCallback()`(覆盖 HAL 的 weak 定义)在检出 `HAL_ADC_ERROR_OVR` 时记 E17、置弃窗标志,窗口累计满时直接丢弃、不置 `s_ready`,由主循环等下一个干净窗口;DMA 传输错误记 E18 并同样弃窗。两者都不终止测量。此机制依赖 `ADC_IRQn` 在 NVIC 中使能 —— 由 `ADC_Sampler_Init()` 完成(放在该处是为了不被 CubeMX 重新生成覆盖),处理函数是 `stm32h7xx_it.c` 里 USER CODE 区的 `ADC_IRQHandler()`。当前 300 kHz 下 DMA 余量充足,正常不会触发。

### 10.4 接口

| 函数 / 类型 | 说明 |
|---|---|
| `uint8_t ADC_Sampler_Init(void)` | 校准两路 ADC 并启动同步转换（只用主 ADC 的 DMA），再启动 TIM6。`main.c` 中调用，失败报 E01 |
| `uint8_t ADC_Sampler_SetFrequency(uint32_t)` | 设定频率并开始新窗口。频率非法返回 0 |
| `uint8_t ADC_Sampler_GetWindow(ADC_Window_t *)` | 取出已完成的窗口；没有新窗口时返回 0 |
| `ADC_Window_t` | `frequency_hz`、`sample_count`,`ui_re`、`ui_im`、`uo_re`、`uo_im`(单频 DFT 的复数分量),`ui_rms`、`uo_rms`(去直流后的交流有效值,用于相干性判据),以及 `clip`(贴量程边界的样本计数,用于削波检测) |

采样速率 300 kHz。改动时**必须同时**修改 TIM6 的 PSC/ARR（硬件触发率）与 `ADC_SAMPLER_RATE_HZ`（DFT 频率轴）—— 两者没有任何编译期或运行期联动，只改一处会让频率轴静默错位且不报任何错。

### 10.5 削波检测

ADC 是单端 0~65535,超出量程的样本会被钳在边界上。**削平后的波形仍带很强的基波分量**,所以幅值门限和相干性判据都拦不住它 —— 这是本工程唯一一类会"静默给出错误结果"的失效,因此单独做了检测。

**为什么 kappa 拦不住(实测计算):**

| 削掉的峰值比例 | kappa | 现有门限 0.2 能拦住? |
|---|---|---|
| 50% | 0.974 | 否 |
| 70% | 0.950 | 否 |
| 95% | 0.910 | 否 |

即使削掉 95% 的峰值,kappa 仍有 0.91,远高于门限。直流偏置错(负半周被削平)时 kappa = 0.92,同样放行。

**检测方法:** 逐个样本统计落在量程两端贴边区的点数(码值 ≤ 64 或 ≥ 65471),占窗口比例超过 **1%** 即判定削波,`Ui` 报 E19、`Uo` 报 E20。

阈值取值依据(解析式算出的贴边比例):

| 情形 | 贴边比例 |
|---|---|
| 干净正弦,幅度到 1.60 V(距 1.65 V 轨道还有 0.05 V) | 0% |
| 恰好触轨,幅度 1.65 V | 4.0% |
| 轻微削顶 0.05 V | 16.0% |
| 直流偏置错(中点 0 V,负半周全削) | 50.2% |

干净与削顶之间有很宽的间隙(0% vs ≥4%),门限取 1% 既不会因噪声误报,也不会漏掉轻微削顶。

**检出后仍照常计算并显示结果**,只是报错误码提示这次读数可能不可信。这与 E17/E18 不同:溢出和 DMA 错误会让样本时间轴不可信,必须作废窗口;削顶时波形是完整的,结果仍有参考价值,该由使用者判断而不是由固件替他丢弃。

**局限:** 本检测抓的是"ADC 端已经贴边"。若前端电路在 ADC 之前就自己饱和(例如运放先削顶),ADC 看到的是尚未贴边的"平顶"波形,这里抓不到。那种情况只能靠 kappa 下降(削顶产生谐波)间接反映,而上一段的表说明这种反映很弱 —— 所以**两道防线都不保证能发现运放级削顶**,根本办法仍是把前端偏置和幅度做对。

### 10.6 待确认

- 通道映射（PA0_C → INP0，PA1_C → INP1）依据工程原有配置和 ST 社区资料，请以 CubeMX 引脚列表为准。
- 采样速率 300 kHz 是按测量频率 1 kHz 的整数周期条件选定的(见 5.1 节)。需要其他速率时,要同时改 TIM6 的 PSC/ARR、`ADC_SAMPLER_RATE_HZ` **和 `.ioc` 里的 TIM6**,否则用 CubeMX 重新生成代码会把硬件触发率改回去,导致频率轴静默错位。
- 窗口覆盖 100 个信号周期(N = 30000 点 / 100 ms)和上下限是初始值,需要结合实测调整;调整时必须保证窗口时长远小于 `MEASURE_WAIT_TIMEOUT_MS`(500 ms)。
- 双 ADC 同步模式没有体现在 `.ioc` 里(`.ioc` 只有 `ADC1.master=1`,没有多模式选项),但配置已放进 `adc.c` 的 `USER CODE` 区,重新生成不会丢失;`.ioc` 的 DMA 对齐也已同步为 Word。若该配置仍因故失效,`ADC_Sampler_Init()` 的回读校验会报 E16 而不是静默降级(见 10.2 节)。

## 11. 标定方法

本工程的两路 ADC 直接测量 `Ui` 和 `Uo`，没有放大或分压。误差主要来自以下几个方面：

1. **两路 ADC 的增益不同。** HAL 的校准只修正偏移和线性度，不修正增益。16 位 ADC 的增益误差常见为 0.1% 到 0.5%，会让 `|H|` 有固定偏差。
2. **两路的输入路径不同。** PA0_C 和 PA1_C 的模拟开关、寄生电容和源阻抗不同，对高频信号的带宽也不同，会引起相位偏差。两路采样时刻已由硬件同步（见 10.2 节），所以残余相位差只来自这条输入路径，用 11.1 节的直通测试标定掉即可。
3. **反馈电阻 `R` 的实际值。** 代码中的 200 Ω 是标称值，电阻有公差，`|Z|` 和 `|Y|` 的绝对值会直接跟着偏。

### 11.1 直通测试（标定两路的增益比和相位差）

把 `Ui` 和 `Uo` 的 DUT 连接点短接到同一个信号源上。理论上两路测到的是同一个信号，`H = 1`，即 0 dB、0°。

1. 对每个标定频率 `f_k`，记录程序算出的 `H_thru(f_k)`（即 `h_magnitude_dB` 和 `h_phase_deg`，或复数形式）。
2. `H_thru(f_k)` 与理论值 1 的偏差，就是两路的增益比和相位差。
3. 对于测量的任意频点 `f`，修正后的传递函数为：

```text
H_true(f) = H_meas(f) / H_thru(f)
```

`H_thru(f)` 需要按频率存成表。注意 `measure.c` 第 3 节的 `Calib_Lookup()` 是**就近取值、不做插值**：当前 5 个扫描频点与标定点一一重合，所以没有影响，但改动扫描频率表时必须同步改标定表。

### 11.2 反馈电阻 R 的标定

用一个已知阻抗的器件（如精密电阻或电容）代替 DUT，测量 `|Z|`，与器件的标称值比较。若两者不一致，就可以反算出实际的 `R`，并更新 `CIR_FEEDBACK_RESISTOR_OHM`（当前为 200.0f）。

### 11.3 标定数据的使用

- 直通测试测得的 `H_thru(f)` 折算成 `measure.c` 标定表（第 3 节）里的两个字段:`mag_ratio = 1/|H_thru|`,`phase_deg = angle(H_thru)`。这样 `H_corrected = H_meas * mag_ratio * e^{-j*phase_deg}` 正好等于 `H_meas / H_thru`。
- 标定时的采样参数（窗口长度、频率点、采样速率）必须与实际测量时一致，否则标定结果不适用。
- **代码已实现接口但表内是恒等值**：`measure.c` 第 3 节的标定表 `s_calib[]` 全为 `{1.0f, 0.0f}`，`Calib_Apply()` 已接入 `CircuitID_Compute()`（文件内静态函数），即当前不做任何修正。实测出上表的数值后填进该表即可生效。在标定之前，`|Y|`、`G`、`B` 的绝对值不可信；φY 只剩两路输入路径差异这一小项。

### 11.4 采样时刻的硬件同步

两路触发时刻偏差已在硬件层面消除：ADC1（主）与 ADC2（从）工作在双 ADC 规则组同步模式，一次 TIM6 触发让两路在同一时刻开始采样，一对结果打包进 CDR 的一个 32 位字，由 ADC1 的 DMA 搬运。实现要点见 10.2 节，重新生成 CubeMX 代码后的必查项见第 9 节检查表第 8、10 条。

采样时刻的同步完全由硬件完成，不依赖标定表，也不会随频率和源阻抗变化而失配。

## 12. 激励源

> **当前版本没有板载 DDS。** 原先驱动 AD9833 的模块(`USE/ad9833.c`、`USE/ad9833.h`)已删除,
> `Freq_SourceSet()` 改为空实现。激励由**外部信号源**提供,直接加到 `Ui` 节点。
>
> 下面保留 SPI1 的配置说明,因为 `Core/Src/spi.c` 仍在工程里且仍被初始化(当前无从设备)。

### 12.1 SPI1 配置

SPI1 仍然初始化,但不连接任何从设备。现有参数:

| 项目 | 配置 | 备注 |
|---|---|---|
| 模式 | 主机 | |
| 数据宽度 | 16 位 | 接新器件前需按该器件手册复核 |
| CPOL / CPHA | High / 1Edge(Mode 2) | |
| NSS | 软件 | |
| 波特率 | 75 MHz(PLL2Q)/ 8 = 9.375 MHz | |
| DMA / 中断 | 不使用 | |

引脚(PB5 = MOSI、PG11 = SCK、PG9 = MISO)由 `spi.c` 的 MspInit 配置。PA4 原先用作 DDS 的软件片选,**现已不使用**,`.ioc` 里不分配。

### 12.2 外部激励源的接入

激励源接到 `Ui` 节点(PA0_C / ADC1_INP0),这是唯一需要接入信号的地方。

关键约束:

- **波形:** 正弦。谐波会污染单频 DFT 的结果。
- **频率:** 必须与 `Freq_PlanGet()` 的表值一致(当前 1 kHz)。固件**无法校验**这一点 —— 源频率偏离会表现为 κ 下降或相位误差,与“DUT 特性”在屏幕上无法区分(见第 8 节)。
- **直流电平:** `Ui` 与运放虚地(反相端)的直流电位必须一致。若不一致,DUT 上会压着直流而产生直流电流灌入跨阻放大器,轻则吃掉动态范围,重则使 `Uo` 削顶。削顶会被固件检出并报 **E19/E20**(见 10.4 节),但那时 `|Y|`、φY、G、B 已经不可信了 —— 报错只是告诉你"这次读数不能要"。
- **幅度:** 由前端电路的量程决定,不受固件约束(代码只检查幅值下限 `CIR_INPUT_MIN_AMP` / `CIR_OUTPUT_MIN_AMP`,以及上限的削波检测)。`Uo` 不削顶仍是结果可信的前提。
