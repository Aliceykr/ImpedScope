# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上。主循环按内部扫描计划（`measure.c` 第 1 节）依次设定 5 个测试频点（100/300/1k/3k/10k Hz），激励由 AD9833 经 SPI1 产生（见第 12 节）。STM32 用两路 ADC 同时采集 DUT 的电压 `Ui` 和电流转换后的电压 `Uo`，在该频率上做单频 DFT，计算导纳 `|Y|`、φY、G、B，并把结果显示在串口屏上。

主要功能：

- AD9833（SPI1 主机）产生正弦激励，5 个频点由内部扫描计划给出；换频无毛刺，失败报 E13
- ADC1、ADC2 硬件同步采样（PA0_C、PA1_C，主从同步模式），由 TIM6 触发，结果打包进 CDR、DMA 循环写入缓冲区
- 每个频率开一个采样窗口，对两路做单频 DFT，得到 `Ui`、`Uo` 的复数分量
- 传递函数 `H = Uo / Ui`，阻抗 `Z = -R / H`，导纳 `Y = -H / R`
- 单频幅值过小时丢弃该频点
- UART4 驱动串口屏，显示频率、阻抗和运行状态

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ ad9833.h/.c         AD9833 DDS 激励源驱动（SPI1 主机，PA4 作软件 FSYNC）
│  ├─ measure.h/.c        测量核心：5 频率扫描计划、ADC 采样与单频 DFT、
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
主循环（measure.c 的扫描计划）──> 取第 k 个频率 f（100/300/1k/3k/10k）
                        │  Freq_SourceSet(f)  ── 经 SPI1 写 AD9833 频率寄存器（失败报 E13）
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

主循环按 100 -> 300 -> 1000 -> 3000 -> 10000 Hz 顺序扫描，每个频点等一个窗口完成。窗口完成后调用计算函数，成功（返回 1）才更新显示；失败或超时会清掉结果，屏幕显示 "-" 而不是上一频点的陈旧读数。

## 5. 计算方法

采样窗口与 DFT（第 2 节）、H、Z、Y（第 4 节）都在 [USE/measure.c](USE/measure.c) 中，两路标定表在第 3 节。

### 5.1 采样窗口

每个频率 `f` 对应一个窗口，窗口长度为：

```text
N = round(10 × fs / f)，限制在 [256, 400000] 之间
fs = 300 kHz（TIM6 触发频率）
```

窗口约覆盖 10 个信号周期。低频时 `N` 会被上限截断，窗口不再是整数周期，会有泄漏误差。最短窗口为 1024 点，约 1 ms。

### 5.2 单频 DFT

两路是硬件同步采样，一对样本来自 CDR 的同一个 32 位字（低半字 ADC1 = `Ui`，高半字 ADC2 = `Uo`），采样时刻完全相同。对每一路：

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

频率由内部扫描计划给出（`measure.c` 第 1 节）。

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
| `frequency_hz` | 频率（来自内部扫描计划） | Hz |
| `h_magnitude_dB` / `h_phase_deg` | 传递函数增益和相位 | dB / ° |
| `z_mag_ohm` / `z_phase_deg` | 阻抗幅值和相位 | Ω / ° |
| `y_mag_siemens` / `y_phase_deg` | 导纳幅值和相位 | S / ° |
| `y_conductance_siemens` | 电导 G = Re[Y] | S |
| `y_susceptance_siemens` | 电纳 B = Im[Y] | S |
| `z_mag_dB` | 阻抗幅值 dB | dB |
| `y_mag_siemens` / `y_phase_deg` | 导纳幅值和相位 | S / ° |
| `valid` | 1 表示结果有效 | — |

计算失败时输出结构全部清零，`valid = 0`。

串口屏显示频率、`|Y|`、φY、G、B，另加一行状态/错误与一行心跳（见第 7 节）。

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
| `ADC_SAMPLER_WINDOW_CYCLES` | `measure.h` | 10 | 每个窗口覆盖的信号周期数 |
| `ADC_SAMPLER_WINDOW_MIN` | `measure.h` | 256 | 窗口最短采样点数（对 5 个目标频率不生效） |
| `ADC_SAMPLER_WINDOW_MAX` | `measure.h` | 400000 | 窗口最长采样点数 |
| `CIR_DUT_TOPOLOGY` | `measure.h` | 1 | 1：计算 Z 和 Y；0：只计算 H |
| `CIR_FEEDBACK_RESISTOR_OHM` | `measure.h` | 200.0 | 反馈电阻 R 的标称值，实际值需标定 |
| `CIR_INPUT_MIN_AMP` | `measure.h` | 4.0 | `|Ui|` 幅值下限（码值），占位，待实测 |
| `CIR_OUTPUT_MIN_AMP` | `measure.h` | 4.0 | `|Uo|` 幅值下限（码值），占位，待实测 |
| `CIR_H_MIN_MAG` | `measure.h` | 1e-6 | `|H|` 下限，低于此值 Z 不可靠 |
| `AD9833_MCLK_HZ` | `ad9833.h` | 25000000 | AD9833 主时钟，必须与实际接线一致（第 12 节） |

PA4（AD9833 的 FSYNC）由 [USE/ad9833.c](USE/ad9833.c) 手动配置成推挽输出，`.ioc` 里不分配 PA4 —— CubeMX 不支持同一引脚既做 `SPI1_NSS` 又做普通 GPIO，而 AD9833 需要软件片选。详见第 12 节。

## 7. 串口屏

UART4，9600 8N1，PC10 TX / PC11 RX。每 500 ms 发送 7 条指令（t0~t6），每条以 `FF FF FF` 结尾。串口屏页面需要建立同名文本框 `t0`~`t6`。

| 控件 | 内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `f:1000`，无结果时 `f:-` |
| `t1` | 导纳模 \|Y\|（mS） | `Y:1.000`，无结果时 `Y:-` |
| `t2` | 导纳角 φY（°） | `P:-12.34`，无结果时 `P:-` |
| `t3` | 电导 G = Re[Y]（mS） | `G:0.977`，无结果时 `G:-` |
| `t4` | 电纳 B = Im[Y]（mS） | `B:-0.213`，无结果时 `B:-` |
| `t5` | 运行阶段或最近一次错误 | `stage:LOOP`、`stage:E12 WINDOW_TIMEOUT x12` |
| `t6` | 心跳，每刷新周期 +1 | `beat:1234` |

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
| E13 | AD9833_SPI | `ad9833.c` 的调用方（`main.c` / `measure.c`） | SPI1 写 AD9833 寄存器失败（HAL 返回非 OK 或超时） | 非致命 |

时钟配置失败发生在 UART 就绪之前，屏幕无法显示，未纳入错误码表，只能用调试器定位。

## 8. 已知限制

- **窗口泄漏。** Fs = 300 kHz 下 5 个频率的窗口都是整数个周期（见 5.1 节），理论上泄漏为零；残余误差只来自晶振频率偏差与 AD9833 实际输出与标称值的偏差。
- **两路增益未标定。** 两路 ADC 的增益误差会直接进入 `H`。见第 11 节。
- **两路输入路径有差异。** ADC1/ADC2 工作在规则组同步模式（见 10.2 节），采样时刻完全相同；残余相位误差来自 PA0_C/PA1_C 两个模拟开关与源阻抗的带宽差异（第 11 节第 2 点）。
- **每个频点要等一个完整窗口。** 换频率会重启窗口；窗口长度从 1 ms（10 kHz）到 100 ms（100 Hz）。窗口未完成就切下一个频率，该频点拿不到结果。
- **幅值门限是占位值。** `CIR_INPUT_MIN_AMP`、`CIR_OUTPUT_MIN_AMP` 为 4 个码值，需要用实际噪声水平确定。
- **AD9833 频率字量化。** 频率字是 28 位整数，25 MHz MCLK 下分辨率 0.093 Hz。100 Hz 处取整误差最大可达 ±0.047 Hz（约 460 ppm），在一个 10 周期窗口内引入约 0.5~0.9° 的相位误差与极小的泄漏；1 kHz 以上可忽略。详见第 12.4 节。

## 9. 编译与维护

使用 Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx` 编译。工程基于：

- STM32CubeMX 6.18
- STM32H7 HAL FW 1.13
- Arm Compiler 6（ArmClang）V6.24 —— `.uvprojx` 的 `pArmCC` 已钉到 V6.24，换机器装的是别的版本时要同步改这一项
- 系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz
- SPI1 内核时钟 75 MHz（PLL2），ADC 时钟 37.5 MHz（PLL2 经 /2）

重新生成 CubeMX 代码后，请检查：

1. Keil 工程中仍包含 `USE/ad9833.c`、`USE/measure.c` 和 `USE/screen.c`。
2. 头文件路径仍包含 `../USE`。
3. `.ioc` 与 `spi.c` 中 SPI1 仍为主机、16 位、CPOL=High / CPHA=1Edge（Mode 2）、软件 NSS、波特率预分频 8；SPI1 不使用 DMA 和中断。
4. `spi.c` 的 MspInit 只配置 PB5/PG9/PG11 三根 SPI1 引脚，不碰 PA4（PA4 由 `ad9833.c` 配成 GPIO 输出）。
5. `main.c` 的 USER CODE 区在 `MX_SPI1_Init()` 之后调用 `AD9833_Init()`，失败报 E13。
6. `main.c` 的 USER CODE 区仍调用 `Screen_Init()`、`Screen_SetStage()`、`ADC_Sampler_Init()`、`CircuitID_Init()`、`Freq_PlanGet()`/`Freq_SourceSet()` 和 `Screen_Poll()`。
7. `.ioc` 中 UART4 仍为 9600 8N1（PC10/PC11），且 UART4 全局中断已使能。
8. `.ioc` 中 ADC1、ADC2 均为单通道、连续转换关闭、DMA 循环、由 TIM6 TRGO 触发，ADC 时钟分频为 `ADC_CLOCK_ASYNC_DIV2`。**注意双 ADC 同步模式在 `.ioc` 里没有对应选项**，重新生成会把 `adc.c` 的 `multimode.Mode` 改回 `ADC_MODE_INDEPENDENT`，要按第 10 条手工改回来。
9. `main.c` 的 USER CODE 区在 `MX_TIM6_Init()` 之后调用 `ADC_Sampler_Init()`。
10. `adc.c` 中 ADC1 的多模式配置为 `ADC_DUALMODE_REGSIMULT`（连同 `DualModeData`、`TwoSamplingDelay`），且 `HAL_ADCEx_MultiModeConfigChannel()` 必须留在 `HAL_ADC_Init()` 之后、任何校准或启动调用之前；ADC1 的 DMA 必须是字（32 位）对齐——同步模式读的是 32 位 CDR，半字对齐会把 ADC2 的数据丢掉。详见 10.2 节。

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
- ADC 在 `ADC_Sampler_Init()` 中先做 `ADC_CALIB_OFFSET_LINEARITY` 校准（两路都做），再用主 ADC 的句柄调用 `HAL_ADCEx_MultiModeStart_DMA()` 启动同步转换，最后启动 TIM6。顺序不能变：校准会先把 ADC 关掉。
- 只有 ADC1 的 DMA（`DMA1_Stream2`，`DMA_REQUEST_ADC1`）在搬运，外围与存储都是 32 位字对齐、循环模式；每半个缓冲区（64 对样本）回调一次。ADC2 的 DMA 配置（`DMA1_Stream1`）保留在 `adc.c` 里，但同步模式下不会启动。
- `HAL_ADCEx_MultiModeConfigChannel()` 写的是两个 ADC 共用的 CCR，而 CCR 里的 DUAL/DAMDF/DELAY 位只有在两个 ADC 都处于关闭状态时才写得进去（HAL 内部按"任一 ADC 已使能就整段跳过"判断）。因此这几行必须留在 `HAL_ADC_Init()` 之后、任何校准或启动调用之前；挪到后面不会报错，只会静默退回独立模式。

### 10.3 窗口与对齐

- `ADC_Sampler_SetFrequency(f)` 设定频率，并从当前块序号开始新窗口。同步模式下两路只有一个块序号，不存在起点对齐问题。
- DMA 回调里从每个 32 位字中同时取出两路样本、按同一个权重一并累加，所以两路的累加点数必然相同，到齐即窗口完成。
- `ADC_Sampler_GetWindow()` 取出完成的窗口，主循环交给 `CircuitID_Compute()`。
- 同一频率重复调用 `SetFrequency` 不会重启窗口。

### 10.4 接口

| 函数 / 类型 | 说明 |
|---|---|
| `uint8_t ADC_Sampler_Init(void)` | 校准两路 ADC 并启动同步转换（只用主 ADC 的 DMA），再启动 TIM6。`main.c` 中调用，失败报 E01 |
| `uint8_t ADC_Sampler_SetFrequency(uint32_t)` | 设定频率并开始新窗口。频率非法返回 0 |
| `uint8_t ADC_Sampler_GetWindow(ADC_Window_t *)` | 取出已完成的窗口；没有新窗口时返回 0 |
| `ADC_Window_t` | `frequency_hz`、`sample_count`，以及 `ui_re`、`ui_im`、`uo_re`、`uo_im`（单频 DFT 的复数分量） |

采样速率 300 kHz。改动时**必须同时**修改 TIM6 的 PSC/ARR（硬件触发率）与 `ADC_SAMPLER_RATE_HZ`（DFT 频率轴）—— 两者没有任何编译期或运行期联动，只改一处会让频率轴静默错位且不报任何错。

### 10.5 待确认

- 通道映射（PA0_C → INP0，PA1_C → INP1）依据工程原有配置和 ST 社区资料，请以 CubeMX 引脚列表为准。
- 采样速率 300 kHz 是按 5 个频点的整数周期条件选定的（见 5.1 节）。需要其他速率时，要同时改 TIM6 的 PSC/ARR、`ADC_SAMPLER_RATE_HZ` **和 `.ioc` 里的 TIM6**，否则用 CubeMX 重新生成代码会把硬件触发率改回去，导致频率轴静默错位。
- 窗口长度的 10 周期和上下限是初始值，需要结合实测调整。
- 双 ADC 同步模式没有体现在 `.ioc` 里（`.ioc` 只有 `ADC1.master=1`，没有多模式选项）。每次用 CubeMX 重新生成代码后，都要按第 9 节检查表第 8、10 条把多模式与 DMA 对齐改回来，否则会静默退回独立模式（不报错，但重新出现几百纳秒的采样时刻偏差）。

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

## 12. AD9833 信号源

激励由 AD9833 DDS 产生，挂在 MCU 的 SPI1 上。驱动是 [USE/ad9833.h](USE/ad9833.h) / [USE/ad9833.c](USE/ad9833.c)，在 `main.c` 的 USER CODE 2 区由 `AD9833_Init()` 初始化。

### 12.1 接线

| STM32 | 方向 | AD9833 | 说明 |
|---|---|---|---|
| PG11（SPI1_SCK） | 输出 | SCLK | 串行时钟，75 MHz / 8 = 9.375 MHz |
| PB5（SPI1_MOSI） | 输出 | SDATA | 串行数据，MSB 在前 |
| PA4 | 输出 | FSYNC | 片选，低有效。普通 GPIO 推挽，由驱动在每条命令前后手动拉低/拉高 |
| PG9（SPI1_MISO） | — | — | 未使用（AD9833 只写不读） |
| 3.3 V / GND | — | VCC / GND | — |

> ⚠ **MCLK 必须真的是 25 MHz。** `AD9833_MCLK_HZ` 默认按“模块自带 25 MHz 晶振”写。若你的板子靠 MCU 的 MCO 引脚给 AD9833 供时钟，本工程还没有配置 MCO，需要另行配置并确认频率，否则所有输出频率都会按比例偏。

### 12.2 SPI1 配置及理由

| 项 | 值 | 理由 |
|---|---|---|
| 模式 | 主机 | AD9833 是从机 |
| 数据宽度 | 16 位 | 一条 AD9833 写命令正好 16 位，一个 `HAL_SPI_TransmitReceive()` 发完整帧，FSYNC 全程保持低电平 |
| CPOL / CPHA | High / 1Edge（Mode 2） | AD9833 在 SCLK 下降沿锁存数据 |
| NSS | 软件 | 硬件 NSS 在 `SPE=1` 期间一直保持低电平，无法在两条命令之间抬起来给 AD9833 分帧 |
| 波特率 | 75 MHz / 8 = 9.375 MHz | AD9833 的 SCLK 上限 40 MHz |
| 首 bit | MSB | 与 D15 对齐 |

驱动用 `HAL_SPI_TransmitReceive()`（而不是 `HAL_SPI_Transmit()`）：H7 HAL 的阻塞发送路径既不检查也不清 OVR（`SPI_CloseTransfer()` 只在 `State != HAL_SPI_STATE_BUSY_TX` 时才处理 OVR），全双工主机只发不收会让 RX FIFO 堆满。把每帧的 RX 顺手读走即可保持 SPI 状态干净 —— MISO 没接线，读回的数据直接丢弃。

### 12.3 寄存器位定义（数据手册 Table 8）

```text
D15 D14 = 00        控制寄存器
D13 B28      = 0x2000   1：连续两次 16 位写拼成 28 位频率
D12 HLB      = 0x1000   B28=0 时选高/低 14 位
D11 FSEL     = 0x0800   0=FREQ0 1=FREQ1
D10 PSEL     = 0x0400   0=PHASE0 1=PHASE1
D9  保留，写 0
D8  RESET    = 0x0100
D7  SLEEP1   = 0x0080
D6  SLEEP12  = 0x0040
D5  OPBITEN  = 0x0020
D4  保留，写 0          （AD9834 这里是 SIGN/PIB，AD9833 没有这一位）
D3  DIV2     = 0x0008
D2  保留，写 0
D1  MODE     = 0x0002   1=三角波 0=正弦波
D0  保留，写 0
```

Table 15（`OPBITEN=1` 时决定 VOUT）：`DIV2=0` 输出 DAC MSB/2（方波频率 f/2），`DIV2=1` 输出 DAC MSB（方波频率 f）；`OPBITEN=1` 且 `MODE=1` 是保留组合。

以上位定义是逐位核对过的重点：把 `RESET` 写到 D9、`OPBITEN` 写到 D6、`DIV2` 写成 D4、`MODE` 写成 D2 都是常见抄错点，症状分别是“复位/关输出无效”“切方波后没输出”“三角波不生效”。频响测量只用正弦波，`AD9833_SetWave()` 的三角/方波仅供硬件调试。

### 12.4 频率与切换

- 频率字 = `round(f × 2^28 / MCLK)`，由 `AD9833_SetFrequency()` 计算并限幅到 MCLK/2。
- **无毛刺切换**：写“当前没被选中”的那个频率寄存器（FREQ0/FREQ1 乒乓），最后用一次控制字写原子翻转 `FSEL`，相位累加器不清零。
- **量化误差**：25 MHz 下每个 LSB 是 0.093 Hz，取整误差最大 ±0.047 Hz。相对误差在最低频 100 Hz 处最大（约 460 ppm），在 10 周期窗口内约合 0.5~0.9° 相位偏差；1 kHz 以上可忽略。AD9833 换频本身是即时的，驱动不额外等待建立时间。

### 12.5 与扫描计划的配合

`Freq_SourceSet(f)`（`measure.c` 第 1 节）内部调用 `AD9833_SetFrequency(f)`，`f` 就是 `Freq_PlanGet()` 返回的表值，SPI 写失败时报 E13。DFT 的旋转因子用的是同一张表，两者必须严格一致。
