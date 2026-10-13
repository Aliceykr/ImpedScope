# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上。主循环按内部扫描计划（`measure.c` 第 1 节）依次设定 5 个测试频点（100/300/1k/3k/10k Hz），信号源由 AD9833 产生（驱动待接入）。STM32 用两路 ADC 同时采集 DUT 的电压 `Ui` 和电流转换后的电压 `Uo`，在该频率上做单频 DFT，计算导纳 `|Y|`、φY、G、B，并把结果显示在串口屏上。

主要功能：

- SPI1 从机 8 位接收，DMA 每次收一帧（8 字节），收完重新装载
- 硬件 CS（NSS，PA4）界定帧边界，EXTI 双边沿检测帧开始与结束
- 帧头 `0x07 0x08`、帧尾 `0x00 0x00` 校验，短帧丢弃
- 4 字节大端频率解析，记录队列在 DMA 回调中入队，计算在主循环中完成
- ADC1、ADC2 各自单通道采样（PA0_C、PA1_C），共用 TIM6 触发，DMA 循环写入缓冲区
- 每个频率开一个采样窗口，对两路做单频 DFT，得到 `Ui`、`Uo` 的复数分量
- 传递函数 `H = Uo / Ui`，阻抗 `Z = -R / H`，导纳 `Y = -H / R`
- 单频幅值过小时丢弃该频点
- UART4 驱动串口屏，显示频率、阻抗和运行状态

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ measure.h/.c        测量核心：5 频率扫描计划（AD9833 空桩）、
│  │                      ADC 采样与单频 DFT、H/Z/Y 计算、双路标定修正
│  └─ screen.h/.c         串口屏 UART4 输出和错误上报
├─ Core/
│  ├─ Src/main.c          系统初始化和主循环（USER CODE 区）
│  ├─ Src/spi.c           SPI1、DMA、PA4 和 EXTI4 配置（CubeMX 生成，部分在 USER CODE 区）
│  ├─ Src/stm32h7xx_it.c  中断入口（EXTI4 处理函数在 USER CODE 区）
│  ├─ Src/usart.c         UART4 配置（CubeMX 生成）
│  ├─ Src/adc.c           ADC1、ADC2 配置（CubeMX 生成，单通道、TIM6 触发）
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

> ⚠ 频率来源已改为内部扫描计划（`measure.c` 第 1 节），不再由 FPGA 经 SPI1 下发频率帧。
> 原接收实现 `USE/spi_receiver.c/.h` **已从工程删除**，需要时可用
> `git show 9ab44f4:USE/spi_receiver.c` 取回。`main.c` 仍在初始化里调用
> `HAL_NVIC_DisableIRQ(EXTI4_IRQn)` 关掉 PA4 的 CS 边沿中断。
> 以下协议描述仅作历史存档，当前没有任何代码实现它。

## 4. SPI 帧协议（**已废弃**，仅存档）

FPGA 的正式协议尚未确定，下面是临时定义的 8 字节帧。帧头、帧尾与之前相同，中间改为频率。

帧边界由 CS（低有效）决定，每帧固定 8 字节，多字节字段按大端顺序发送：

```text
字节  内容
 0    0x07              帧头
 1    0x08              帧头
 2-5  频率              32 位无符号整数，单位 Hz，大端
 6    0x00              帧尾
 7    0x00              帧尾
```

频率允许范围为 1 Hz 到 500 kHz（ADC 采样率的一半）。超出范围或为 0 的帧被拒绝，报 E03。

旧的 16 字节数据帧（定点分量和峰值 bin）曾以注释形式保留在 `spi_receiver.h/.c` 中；这两个文件已随废弃路径一并删除，内容见 git 历史。

SPI 配置：

- FPGA：主机；STM32：SPI1 从机
- 模式：Mode 0（CPOL=0，CPHA=0），MSB first，8 bit
- NSS：硬件输入（PA4），CS 低有效
- 内核时钟：PLL2 的 75 MHz（`RCC_SPI123CLKSOURCE_PLL2`）。SCK 由 FPGA 提供，内核时钟不决定传输速率。
- 从机能稳定接收的 SCK 上限需用示波器实测确认。ST 社区有 H7 从机在 22.8 MHz 以上出错的报告，参见 [ST 社区讨论](https://community.st.com/t5/stm32-mcus-products/stm32h7-slave-mode-timing-issue-above-22-8-mhz/td-p/822487)。
- DMA：`DMA1_Stream0`，SPI1 RX，普通模式，长度 8 字节，每帧后重新装载

引脚：

| 引脚 | 功能 |
|---|---|
| PB5  | SPI1_MOSI |
| PG9  | SPI1_MISO |
| PG11 | SPI1_SCK |
| PA4  | SPI1_NSS / CS（EXTI4 双边沿）**暂定，待 PCB 确认** |

接收流程：

1. DMA 装载后等待一帧 8 字节。
2. CS 下降沿（帧开始）：若 DMA 未就绪，本帧必丢，报 E02 并重新装载。
3. CS 上升沿（帧结束）：若 DMA 计数不为 0，说明未收满 8 字节（短帧），abort 并报 E03，重新装载。计数为 0 时交给完成回调处理。
4. 收满 8 字节后检查帧头、帧尾和频率范围，通过则放入频率记录队列（深度 4，满时丢弃最旧记录）。
5. SPI 通信错误（溢出、模式错误等）由 `HAL_SPI_ErrorCallback` 处理：报 E03，abort 并重新装载。

协议的宏定义原集中在 `USE/spi_receiver.h`（文件已删除）。

当前协议没有序号或校验和，帧头帧尾只能校验固定字段，不能检测所有载荷错误。

## 5. 数据处理流程

```text
主循环（measure.c 的扫描计划）──> 取第 k 个频率 f（100/300/1k/3k/10k）
                        │  Freq_SourceSet(f)  ── 设定 AD9833（当前为空桩）
                        ▼
          ADC_Sampler_SetFrequency(f)  ── 设定频率，开始新的采样窗口
                        │
                        │  DMA 中断中两路 ADC 累加单频 DFT
                        ▼
          ADC_Sampler_GetWindow()     ── 窗口完成时取出 Ui、Uo 的复数分量
                        │  （带墙钟超时；超时报 E18 并清结果）
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

## 6. 计算方法

采样窗口与 DFT（第 2 节）、H、Z、Y（第 4 节）都在 [USE/measure.c](USE/measure.c) 中，两路标定表在第 3 节。

### 6.1 采样窗口

每个频率 `f` 对应一个窗口，窗口长度为：

```text
N = round(10 × fs / f)，限制在 [256, 400000] 之间
fs = 300 kHz（TIM6 触发频率）
```

窗口约覆盖 10 个信号周期。低频时 `N` 会被上限截断，窗口不再是整数周期，会有泄漏误差。最短窗口为 1024 点，约 1 ms。

### 6.2 单频 DFT

两路 ADC 在同一个 DMA 块边界开始累加，窗口起点相同，因此样本严格对齐。对每一路：

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

`|U|` 低于 `CIR_INPUT_MIN_AMP` 报 E09，低于 `CIR_OUTPUT_MIN_AMP` 报 E10。两个门限都是占位值，待实测确定。

### 6.3 传递函数 H = Uo / Ui

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

`H` 的结果若不是有限数，报 E11。

### 6.4 频率

频率由内部扫描计划给出（`measure.c` 第 1 节），不再由 SPI 帧提供。

### 6.5 阻抗 Z 和导纳 Y（`CIR_DUT_TOPOLOGY = 1` 时计算）

阻抗：

```text
Z = -R / H = -R · conj(H) / |H|²

Z_re = -R · H_re / |H|²
Z_im = +R · H_im / |H|²
|Z| = R / |H|
Z 相位 = atan2(Z_im, Z_re) × 180 / π
```

若 `|H|² ≤ CIR_H_MIN_MAG²`，Z 趋于无穷大，结果不可靠，报 E13 并丢弃。

导纳：

```text
Y = -H / R

Y_re = -H_re / R
Y_im = -H_im / R
|Y| = |H| / R
Y 相位 = atan2(Y_im, Y_re) × 180 / π
```

Z 或 Y 的任一结果非有限，报 E14 并丢弃。

`|Z|` 只依赖 `|H|` 和 `R`，不需要 ADC 的参考电压。但 `|H|` 的绝对值会受两路增益差影响，见第 12 节。

### 6.6 输出

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

串口屏显示频率、`|Y|`、φY、G、B，另加一行状态/错误与一行心跳（见第 8 节）。

### 6.7 数值示例

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
说明 `measure.c` 里 `Y = -H/R` 的负号被删掉了（见 6.5 节）。

## 7. 参数配置

| 宏 | 所在文件 | 默认值 | 说明 |
|---|---|---|---|
| `ADC_SAMPLER_RATE_HZ` | `measure.h` | 300000 | ADC 采样率（TIM6 触发频率） |
| `ADC_SAMPLER_PAIRS_PER_HALF` | `measure.h` | 64 | 每个 DMA 块的采样点数 |
| `ADC_SAMPLER_WINDOW_CYCLES` | `measure.h` | 10 | 每个窗口覆盖的信号周期数 |
| `ADC_SAMPLER_WINDOW_MIN` | `measure.h` | 256 | 窗口最短采样点数（对 5 个目标频率不生效） |
| `ADC_SAMPLER_WINDOW_MAX` | `measure.h` | 400000 | 窗口最长采样点数 |
| `CIR_DUT_TOPOLOGY` | `measure.h` | 1 | 1：计算 Z 和 Y；0：只计算 H |
| `CIR_FEEDBACK_RESISTOR_OHM` | `measure.h` | 200.0 | 反馈电阻 R 的标称值，实际值需标定 |
| `CIR_INPUT_MIN_AMP` | `measure.h` | 4.0 | `|Ui|` 幅值下限（码值），占位，待实测 |
| `CIR_OUTPUT_MIN_AMP` | `measure.h` | 4.0 | `|Uo|` 幅值下限（码值），占位，待实测 |
| `CIR_H_MIN_MAG` | `measure.h` | 1e-6 | `|H|` 下限，低于此值 Z 不可靠 |

CS（PA4）的 GPIO 与 EXTI4 配置在 `spi.c` 的 USER CODE 区，`.ioc` 中 PA4 仍配置为 `SPI1_NSS`。该通道属于已删除的 FPGA 接收路径，现在只在 `main.c` 里被显式关掉中断；后续把 SPI1 改为主机驱动 AD9833 时，PA4 按普通 GPIO 输出使用。

## 8. 串口屏

UART4，9600 8N1，PC10 TX / PC11 RX。每 500 ms 发送 7 条指令（t0~t6），每条以 `FF FF FF` 结尾。串口屏页面需要建立同名文本框 `t0`~`t6`；t5、t6 为本版新增，需在屏上补充。

| 控件 | 内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `f:1000`，无结果时 `f:-` |
| `t1` | 导纳模 \|Y\|（mS） | `Y:1.000`，无结果时 `Y:-` |
| `t2` | 导纳角 φY（°） | `P:-12.34`，无结果时 `P:-` |
| `t3` | 电导 G = Re[Y]（mS） | `G:0.977`，无结果时 `G:-` |
| `t4` | 电纳 B = Im[Y]（mS） | `B:-0.213`，无结果时 `B:-` |
| `t5` | 运行阶段或最近一次错误 | `stage:LOOP`、`stage:E18 WINDOW_TIMEOUT x12` |
| `t6` | 心跳，每刷新周期 +1 | `beat:1234` |

### 8.1 运行阶段

`main.c` 的 USER CODE 区调用 `Screen_SetStage()` 打点，顺序为：

`BOOT → CLK → PERIPH → MODULE → SCAN → LOOP`

`BOOT`、`CLK` 发生在 UART 初始化之前，只保存在内存中，屏幕上看不到。`PERIPH` 起屏幕可见，设置时立即阻塞发送并保持约 150 ms。

### 8.2 错误码

非致命错误通过 `Screen_ReportError(code)` 计数，`t5` 显示最近一次错误及累计次数（16 位，溢出后回绕）。新阶段或新错误会覆盖之前的显示。

致命错误由 `Screen_Fatal()` 或 `Error_Handler()` 触发，关中断后阻塞发送，`t5` 锁定显示。

| 码 | 名称 | 位置 | 触发条件 | 类型 |
|---|---|---|---|---|
| E01 | HAL_INIT | CubeMX 生成的 `MX_*_Init()`，`ADC_Sampler_Init()` | 外设初始化或 ADC 校准、DMA 启动失败 | 致命 |
| E02 | SPI_DMA | — | 已弃用（SPI 接收路径已删除），保留编号 | — |
| E03 | SPI_FRAME | — | 已弃用（SPI 接收路径已删除），保留编号 | — |
| E04 | SPI_NAN | — | 已弃用，保留编号 | — |
| E05 | SPI_LEN | — | 已弃用，保留编号 | — |
| E06 | SPI_QFULL | — | 已弃用（SPI 接收路径已删除），保留编号 | — |
| E07 | CIR_CFG | `measure.c` | `R ≤ 0`、频率为 0 或窗口为空 | 非致命 |
| E08 | CIR_NAN_IN | `measure.c` | 窗口的 Ui、Uo 分量非有限 | 非致命 |
| E09 | CIR_UI_LOW | `measure.c` | `\|Ui\|` 幅值低于 `CIR_INPUT_MIN_AMP` | 非致命 |
| E10 | CIR_UO_LOW | `measure.c` | `\|Uo\|` 幅值低于 `CIR_OUTPUT_MIN_AMP` | 非致命 |
| E11 | CIR_H_NAN | `measure.c` | H 非有限 | 非致命 |
| E12 | CIR_OUT_NAN | `measure.c` | 频率、dB、相位输出非有限 | 非致命 |
| E13 | CIR_H_LOW | `measure.c` | `\|H\|` 过小，无法计算 Z | 非致命 |
| E14 | CIR_ZY_NAN | `measure.c` | Z 或 Y 非有限 | 非致命 |
| E15 | UART_TX | `screen.c` | `HAL_UART_Transmit_IT()` 返回非 OK | 非致命 |
| E16 | UART_BUSY | `screen.c` | 上一条指令未发完，本周期跳过 | 非致命 |
| E17 | BIN_MISMATCH | — | 已弃用（旧的峰值 bin 协议），保留编号 | — |
| E18 | WINDOW_TIMEOUT | `main.c` | 等待 ADC 窗口完成超时（T7） | 非致命 |

时钟配置失败发生在 UART 就绪之前，屏幕无法显示，未纳入错误码表，只能用调试器定位。

## 9. 已知限制

- **窗口泄漏。** Fs = 300 kHz 下 5 个频率的窗口都是整数个周期（见 6.1 节），理论上泄漏为零；残余误差只来自晶振频率偏差与 AD9833 实际输出与标称值的偏差。
- **两路增益和采样时刻未标定。** 两路 ADC 的增益误差和两个独立 ADC 之间的采样时刻偏差会直接进入 `H`。见第 12 节。
- **两个 ADC 不是硬件同步模式。** 它们由同一个 TIM6 TRGO 触发，采样时刻接近，但有几百纳秒的偏差。频率越高，相位误差越大。
- **每个频点要等一个完整窗口。** 换频率会重启窗口；窗口长度从 1 ms（10 kHz）到 100 ms（100 Hz）。窗口未完成就切下一个频率，该频点拿不到结果。
- **幅值门限是占位值。** `CIR_INPUT_MIN_AMP`、`CIR_OUTPUT_MIN_AMP` 为 4 个码值，需要用实际噪声水平确定。

## 10. 编译与维护

使用 Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx` 编译。工程基于：

- STM32CubeMX 6.18
- STM32H7 HAL FW 1.13
- ARMCC V5
- 系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz
- SPI1 内核时钟 75 MHz（PLL2），ADC 时钟 37.5 MHz（PLL2 经 /2）

重新生成 CubeMX 代码后，请检查：

1. Keil 工程中仍包含 `USE/measure.c` 和 `USE/screen.c`。
2. 头文件路径仍包含 `../USE`。
3. SPI1 数据宽度仍为 8 bit，NSS 为硬件输入，DMA 为普通模式（长度 8 字节）。
4. `spi.c` 的 MspInit USER CODE 区仍包含 PA4 的 GPIO 配置和 EXTI4 双边沿配置。
5. `stm32h7xx_it.c` 的 USER CODE 区仍有 `EXTI4_IRQHandler`。`.ioc` 中不需要 `NVIC.EXTI4_IRQn`，EXTI4 的使能由 `spi.c` 负责。
6. `main.c` 的 USER CODE 区仍调用 `Screen_Init()`、`Screen_SetStage()`、`ADC_Sampler_Init()`、`CircuitID_Init()`、`Freq_PlanGet()`/`Freq_SourceSet()` 和 `Screen_Poll()`。
7. `.ioc` 中 UART4 仍为 9600 8N1（PC10/PC11），且 UART4 全局中断已使能。
8. `.ioc` 中 ADC1、ADC2 均为单通道、连续转换关闭、DMA 循环、由 TIM6 TRGO 触发，ADC 时钟分频为 `ADC_CLOCK_ASYNC_DIV2`。
9. `main.c` 的 USER CODE 区在 `MX_TIM6_Init()` 之后调用 `ADC_Sampler_Init()`。

## 11. ADC 采样

ADC1、ADC2 各自对一路电压做单通道采样，由同一个 TIM6 TRGO 触发，DMA 循环写入各自的缓冲区。采样结果不再直接显示，而是用于窗口 DFT。代码见 [USE/measure.c](USE/measure.c) 第 2 节。

### 11.1 通道与引脚

| 通道 | 引脚 | ADC 通道 | 含义 |
|---|---|---|---|
| ADC1 | PA0_C | `ADC_CHANNEL_0`（ADC1_INP0） | DUT 电压 `Ui` |
| ADC2 | PA1_C | `ADC_CHANNEL_1`（ADC2_INP1） | 电流转换后的电压 `Uo` |

`_C` 为直连快速通道，经 SYSCFG 模拟开关接入 ADC。引脚配置为模拟模式，无上下拉。模拟开关为 `OPEN`，ADC 从 `_C` 焊盘采样。`PA0_C` 与普通 `PA0` 是两个焊盘，中间靠模拟开关连接，同一时刻只能选其中一条路径。

### 11.2 采样时序

- 由 TIM6 TRGO（上升沿）触发两路 ADC。定时器 PSC=3、ARR=99，按 120 MHz 计数时钟计算，采样速率为 300 kHz。
- 关闭连续转换，每次触发完成一次单通道转换。采样时间为 1.5 个 ADC 周期。
- ADC 时钟由 PLL2 的 75 MHz 经 /2 分频为 37.5 MHz，低于 16 位 ADC 的时钟上限（50 MHz）。
- ADC 在 `ADC_Sampler_Init()` 中先做 `ADC_CALIB_OFFSET_LINEARITY` 校准，两路都校准，再启动 DMA，最后启动 TIM6。
- DMA 循环模式，半字（16 位）对齐。每个 DMA 块（半个缓冲区，64 点）回调一次，两路各自计数块序号。

### 11.3 窗口与对齐

- `ADC_Sampler_SetFrequency(f)` 设定频率，并从两路共同的块序号开始新窗口。两路块序号取较大者，保证起点相同。
- 每路在 DMA 回调中累加窗口内的样本，两路累加到相同点数后，窗口完成。
- `ADC_Sampler_GetWindow()` 取出完成的窗口，主循环交给 `CircuitID_Compute()`。
- 同一频率重复调用 `SetFrequency` 不会重启窗口。

### 11.4 接口

| 函数 / 类型 | 说明 |
|---|---|
| `uint8_t ADC_Sampler_Init(void)` | 校准并启动两路 DMA，再启动 TIM6。`main.c` 中调用，失败报 E01 |
| `uint8_t ADC_Sampler_SetFrequency(uint32_t)` | 设定频率并开始新窗口。频率非法返回 0 |
- 采样速率 300 kHz。改动时**必须同时**修改 TIM6 的 PSC/ARR（硬件触发率）与 `ADC_SAMPLER_RATE_HZ`（DFT 频率轴）—— 两者没有任何编译期或运行期联动，只改一处会让频率轴静默错位且不报任何错。
| `ADC_Window_t` | `frequency_hz`、`sample_count`，以及 `ui_re`、`ui_im`、`uo_re`、`uo_im`（单频 DFT 的复数分量） |

### 11.5 待确认

- 通道映射（PA0_C → INP0，PA1_C → INP1）依据工程原有配置和 ST 社区资料，请以 CubeMX 引脚列表为准。
- 采样速率 1 MHz 是初始值。需要其他速率时，修改 TIM6 的 PSC 和 ARR，并同步修改 `ADC_SAMPLER_RATE_HZ`。
- 窗口长度的 10 周期和上下限是初始值，需要结合实测调整。

## 12. 标定方法

本工程的两路 ADC 直接测量 `Ui` 和 `Uo`，没有放大或分压。误差主要来自以下几个方面：

1. **两路 ADC 的增益不同。** HAL 的校准只修正偏移和线性度，不修正增益。16 位 ADC 的增益误差常见为 0.1% 到 0.5%，会让 `|H|` 有固定偏差。
2. **两路的输入路径不同。** PA0_C 和 PA1_C 的模拟开关、寄生电容和源阻抗不同，对高频信号的带宽也不同，会引起相位偏差。
3. **两路的采样时刻有偏差。** 两个 ADC 由同一个 TIM6 触发，但不是硬件同步模式，时刻差约为几百纳秒。频率越高，这个偏差造成的相位误差越大。例如 10 kHz 时，0.3 µs 的偏差约为 1.1°，约占 φY 误差预算（约 4.5°）的四分之一。这一项通常比增益误差更重要。
4. **反馈电阻 `R` 的实际值。** 代码中的 200 Ω 是标称值，电阻有公差，`|Z|` 和 `|Y|` 的绝对值会直接跟着偏。

### 12.1 直通测试（标定两路的增益比和相位差）

把 `Ui` 和 `Uo` 的 DUT 连接点短接到同一个信号源上。理论上两路测到的是同一个信号，`H = 1`，即 0 dB、0°。

1. 对每个标定频率 `f_k`，记录程序算出的 `H_thru(f_k)`（即 `h_magnitude_dB` 和 `h_phase_deg`，或复数形式）。
2. `H_thru(f_k)` 与理论值 1 的偏差，就是两路的增益比和相位差。
3. 对于测量的任意频点 `f`，修正后的传递函数为：

```text
H_true(f) = H_meas(f) / H_thru(f)
```

`H_thru(f)` 需要按频率存成表，中间频率用插值。

### 12.2 反馈电阻 R 的标定

用一个已知阻抗的器件（如精密电阻或电容）代替 DUT，测量 `|Z|`，与器件的标称值比较。若两者不一致，就可以反算出实际的 `R`，并更新 `CIR_FEEDBACK_RESISTOR_OHM`（当前为 200.0f）。

### 12.3 标定数据的使用

- 直通测试测得的 `H_thru(f)` 折算成 `measure.c` 标定表（第 3 节）里的两个字段:`mag_ratio = 1/|H_thru|`,`phase_deg = angle(H_thru)`。这样 `H_corrected = H_meas * mag_ratio * e^{-j*phase_deg}` 正好等于 `H_meas / H_thru`。
- 标定时的采样参数（窗口长度、频率点、采样速率）必须与实际测量时一致，否则标定结果不适用。
- **代码已实现接口但表内是恒等值**：`measure.c` 第 3 节的标定表 `s_calib[]` 全为 `{1.0f, 0.0f}`，`Calib_Apply()` 已接入 `CircuitID_Compute()`（现为文件内静态函数），即当前不做任何修正。实测出上表的数值后填进该表即可生效。在标定之前，`|Y|`、`G`、`B` 的绝对值不可信，φY 在两路时刻偏差较小时基本可信。

### 12.4 时刻偏差的根本解决

如果第 3 点的时刻偏差对结果影响太大，需要改为 ADC 双重同步模式（ADC1 为主、ADC2 为从），让两路在同一个采样时刻开始转换。这需要修改 `.ioc` 的多模式设置，并调整 `measure.c` 第 2 节的 DMA 处理。
