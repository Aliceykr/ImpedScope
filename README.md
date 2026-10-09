# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上。FPGA 对被测电路（DUT）的输入电压 `Ui` 和输出电压 `Uo` 做 FFT，把每个频点的复数结果通过 SPI1 发给 STM32。STM32 完成帧同步、数据解析、频响计算，并把结果显示在串口屏上。

主要功能：

- SPI1 从机 8 位接收，DMA 循环接收连续字节流
- 滑动帧同步，帧头帧尾校验，错位后自动重新同步
- IEEE-754 单精度浮点数据解析，拒绝 NaN 和 Inf
- 有效频点队列，避免在 DMA 中断中做浮点计算
- 传递函数 `H = Uo / Ui`
- 跨阻前端下的阻抗 `Z = -R / H` 和导纳 `Y = -H / R`
- 输入、输出幅值过小时丢弃该频点
- UART4 驱动串口屏，显示频率、阻抗和运行状态

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ spi_receiver.h/.c   SPI1 帧接收、DMA 启动和数据解析
│  ├─ circuit_id.h/.c     频响、阻抗和导纳计算
│  └─ screen.h/.c         串口屏 UART4 输出和错误上报
├─ Core/
│  ├─ Src/main.c          系统初始化和主循环（USER CODE 区）
│  ├─ Src/spi.c           SPI1 与 DMA 配置（CubeMX 生成）
│  └─ Src/usart.c         UART4 配置（CubeMX 生成）
├─ Drivers/               STM32 HAL 和 CMSIS
├─ MDK-ARM/               Keil 工程文件
└─ 9_30.ioc               STM32CubeMX 配置文件
```

`USE/` 保存用户逻辑。CubeMX 重新生成代码时应保留该目录，不修改 CubeMX 生成的 `.c` 文件中的业务逻辑。

## 3. 硬件模型

前端使用跨阻放大器。运放反相端接近 0 V（虚地），DUT 的电流全部流过反馈电阻 `R`：

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

## 4. SPI 帧协议

每帧 22 字节，多字节字段按大端顺序发送：

```text
[0xAA]              帧头
[频点 4 字节]        uint32，单位由 CIR_FREQ_UNIT 决定
[Ui 实部 4 字节]     float
[Ui 虚部 4 字节]     float
[Uo 实部 4 字节]     float
[Uo 虚部 4 字节]     float
[0x55]              帧尾
```

SPI 配置：

- FPGA：主机；STM32：SPI1 从机
- 模式：Mode 0（CPOL=0，CPHA=0）
- 数据宽度：8 bit，MSB first
- DMA：`DMA1_Stream0`，SPI1 RX，循环模式，双帧缓冲（缓冲区 44 字节，半传输和全传输各处理一帧）

接收流程：

1. DMA 每收到半个缓冲区或整个缓冲区，回调逐字节送入解析器。
2. 解析器找到帧头 `0xAA` 后收集 22 字节。
3. 检查帧头、帧尾；检查 Ui、Uo 四个字段是否为 NaN 或 Inf。
4. 通过则放入频点队列（最多 3 个，满时丢弃最旧数据）；失败则从候选帧中的下一个 `0xAA` 继续搜索。

协议的宏定义集中在 [USE/spi_receiver.h](USE/spi_receiver.h)。若 FPGA 的标记值、字节序或字段定义变化，修改该文件即可。

当前协议没有长度字段、序号或校验和，帧头帧尾只能帮助恢复字节边界，不能检测所有载荷错误。若 FPGA 协议可调整，建议加入序号和 CRC。

## 5. 数据处理流程

```text
FPGA ──SPI1/DMA──> 字节流
                    │  滑动同步 + 帧校验
                    ▼
              频点队列（最多 3 个）
                    │  主循环中 SPI_Receiver_TryGetPoint()
                    ▼
          CircuitID_Compute()   ── 计算 H、Z、Y，检查有效性
                    │  有效结果
                    ▼
          Screen_SetResult()    ── 保存最近一次有效结果
                    │
                    ▼
          Screen_Poll()         ── 每 500 ms 通过 UART4 刷新串口屏
```

主循环每次取出一个频点，调用计算函数。只有计算成功（返回 1）的结果才会更新显示，失败的频点被丢弃并在串口屏 `t4` 上报对应错误码。

## 6. 计算方法

输入是一个频点 `FFT_Point_t`，输出是 `FRF_Point_t`。计算分为四步，代码见 [USE/circuit_id.c](USE/circuit_id.c)。

### 6.1 输入检查

1. 配置检查：`R > 0` 且 `freq ≠ 0`，否则错误码 E07。
2. 输入检查：`Ui`、`Uo` 的四个分量必须是有限数，否则 E08。
3. 幅值检查：
   - `|Ui|² > CIR_INPUT_MIN_MAG²`，否则 E09（`Ui` 太小，比值会被噪声放大）
   - `|Uo|² > CIR_OUTPUT_MIN_MAG²`，否则 E10（输出几乎没有信号）

### 6.2 传递函数 H = Uo / Ui

分子分母同乘 `conj(Ui)`，分母变为实数 `|Ui|²`，避免复数除法：

```text
|Ui|²  = Ui_re² + Ui_im²
H_re   = (Uo_re·Ui_re + Uo_im·Ui_im) / |Ui|²
H_im   = (Uo_im·Ui_re − Uo_re·Ui_im) / |Ui|²
|H|²   = H_re² + H_im²
```

`H` 的幅值和相位：

```text
|H|       = √(|H|²)
增益 (dB) = 20 · log10(|H|)
相位 (°)  = atan2(H_im, H_re) × 180 / π
```

H 的结果若不是有限数，报 E11。

### 6.3 频率换算

- `CIR_FREQ_IS_RAW = 1`：`freq` 是实际频率，按 `CIR_FREQ_UNIT` 换算到 Hz（0：Hz，1：kHz，2：MHz）。
- `CIR_FREQ_IS_RAW = 0`：`freq` 是 FFT bin，`f = bin × CIR_SAMPLE_RATE_HZ / CIR_FFT_POINTS`。

### 6.4 阻抗 Z 和导纳 Y（`CIR_DUT_TOPOLOGY = 1` 时计算）

阻抗：

```text
Z = -R / H = -R · conj(H) / |H|²

Z_re = -R · H_re / |H|²
Z_im = +R · H_im / |H|²
|Z| = R / |H|
Z 相位 = atan2(Z_im, Z_re) × 180 / π
```

若 `|H|² ≤ CIR_H_MIN_MAG²`，Z 会趋于无穷大，结果不可靠，报 E13 并丢弃。

导纳：

```text
Y = -H / R

Y_re = -H_re / R
Y_im = -H_im / R
|Y| = |H| / R
Y 相位 = atan2(Y_im, Y_re) × 180 / π
```

Z 或 Y 的任一结果非有限，报 E14 并丢弃。

### 6.5 输出

| 字段 | 含义 | 单位 |
|---|---|---|
| `frequency_hz` | 频率 | Hz |
| `h_magnitude_dB` / `h_phase_deg` | 传递函数增益和相位 | dB / ° |
| `z_mag_ohm` / `z_phase_deg` | 阻抗幅值和相位 | Ω / ° |
| `z_mag_dB` | 阻抗幅值 dB | dB |
| `y_mag_siemens` / `y_phase_deg` | 导纳幅值和相位 | S / ° |
| `valid` | 1 表示结果有效 | — |

计算失败时输出结构全部清零，`valid = 0`。

串口屏只显示频率、`|Z|` 和 `Z` 相位三个值。

### 6.6 数值示例

以 1 µF 电容为 DUT，`R = 1000 Ω`，`f = 1 kHz`：

```text
Y = jωC = j · 2π·1000·1e-6 ≈ j 6.28e-3 S
H = -Y·R ≈ -j 6.28
|Z| = 1 / (ωC) ≈ 159 Ω
Z 相位 = -90°
```

若 Z 相位显示为 +90°，说明虚部符号错误。

## 7. 参数配置

主要参数位于 [USE/circuit_id.h](USE/circuit_id.h)：

| 宏 | 默认值 | 说明 |
|---|---|---|
| `CIR_FREQ_IS_RAW` | 1 | 1：freq 是实际频率；0：freq 是 FFT bin |
| `CIR_FREQ_UNIT` | 0 | 0：Hz，1：kHz，2：MHz |
| `CIR_SAMPLE_RATE_HZ` | 1e6 | 采样率，仅 bin 模式使用 |
| `CIR_FFT_POINTS` | 1024 | FFT 点数，仅 bin 模式使用 |
| `CIR_DUT_TOPOLOGY` | 1 | 1：计算 Z 和 Y；0：只计算 H |
| `CIR_FEEDBACK_RESISTOR_OHM` | 1000.0 | 反馈电阻 R |
| `CIR_INPUT_MIN_MAG` | 1e-4 | `|Ui|` 下限 |
| `CIR_OUTPUT_MIN_MAG` | 1e-6 | `|Uo|` 下限 |
| `CIR_H_MIN_MAG` | 1e-6 | `|H|` 下限，低于此值 Z 不可靠 |

`CIR_EXCITATION` 和 `CIR_AVERAGE_FRAMES` 为多帧平均预留，当前未使用。

幅值门限需要根据 FPGA 的 FFT 缩放方式和实际噪声水平校准。

## 8. 串口屏

UART4，9600 8N1，PC10 TX / PC11 RX。每 500 ms 发送 5 条指令，每条以 `FF FF FF` 结尾。串口屏页面需要建立同名文本框 `t0`~`t4`。

| 控件 | 内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `freq:1000`，无结果时 `freq:-` |
| `t1` | 阻抗幅值 \|Z\|（Ω） | `zmag:1234.56`，无结果时 `zmag:-` |
| `t2` | 阻抗相位（°） | `zph:-12.34`，无结果时 `zph:-` |
| `t3` | 心跳计数，每刷新周期 +1 | `beat:1234` |
| `t4` | 运行阶段或最近一次错误 | `stage:LOOP`、`stage:E03 SPI_FRAME x12` |

### 8.1 运行阶段

`main.c` 的 USER CODE 区调用 `Screen_SetStage()` 打点，顺序为：

`BOOT → CLK → PERIPH → MODULE → SPI → LOOP`

`BOOT`、`CLK` 发生在 UART 初始化之前，只保存在内存中，屏幕上看不到。`PERIPH` 起屏幕可见，设置时立即阻塞发送并保持约 150 ms。

### 8.2 错误码

非致命错误通过 `Screen_ReportError(code)` 计数，`t4` 显示最近一次错误及累计次数（16 位，溢出后回绕）。新阶段或新错误会覆盖之前的显示。

致命错误由 `Screen_Fatal()` 或 `Error_Handler()` 触发，关中断后阻塞发送，`t4` 锁定显示。

| 码 | 名称 | 位置 | 触发条件 | 类型 |
|---|---|---|---|---|
| E01 | HAL_INIT | CubeMX 生成的 `MX_*_Init()` | 外设初始化失败（无法区分具体外设） | 致命 |
| E02 | SPI_DMA | `main.c` | `SPI_Receiver_Init()` 失败 | 致命 |
| E03 | SPI_FRAME | `spi_receiver.c` | 帧头或帧尾不匹配 | 非致命 |
| E04 | SPI_NAN | `spi_receiver.c` | 帧内浮点字段为 NaN 或 Inf | 非致命 |
| E05 | SPI_LEN | `spi_receiver.c` | `sizeof(float) ≠ sizeof(uint32_t)` | 致命 |
| E06 | SPI_QFULL | `spi_receiver.c` | 频点队列满，丢弃最旧数据 | 非致命 |
| E07 | CIR_CFG | `circuit_id.c` | `R ≤ 0` 或 `freq = 0` | 非致命 |
| E08 | CIR_NAN_IN | `circuit_id.c` | 输入 Ui、Uo 非有限 | 非致命 |
| E09 | CIR_UI_LOW | `circuit_id.c` | `|Ui|` 过小 | 非致命 |
| E10 | CIR_UO_LOW | `circuit_id.c` | `|Uo|` 过小 | 非致命 |
| E11 | CIR_H_NAN | `circuit_id.c` | H 非有限 | 非致命 |
| E12 | CIR_OUT_NAN | `circuit_id.c` | 频率、dB、相位输出非有限 | 非致命 |
| E13 | CIR_H_LOW | `circuit_id.c` | `|H|` 过小，无法计算 Z | 非致命 |
| E14 | CIR_ZY_NAN | `circuit_id.c` | Z 或 Y 非有限 | 非致命 |
| E15 | UART_TX | `screen.c` | `HAL_UART_Transmit_IT()` 返回非 OK | 非致命 |
| E16 | UART_BUSY | `screen.c` | 上一条指令未发完，本周期跳过 | 非致命 |

时钟配置失败发生在 UART 就绪之前，屏幕无法显示，未纳入错误码表，只能用调试器定位。

## 9. 编译与维护

使用 Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx` 编译。工程基于：

- STM32CubeMX 6.18
- STM32H7 HAL FW 1.13
- ARMCC V5
- 系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz

重新生成 CubeMX 代码后，请检查：

1. Keil 工程中仍包含 `USE/spi_receiver.c`、`USE/circuit_id.c`、`USE/screen.c`。
2. 头文件路径仍包含 `../USE`。
3. SPI1 数据宽度仍为 8 bit，DMA 仍为循环接收。
4. `main.c` 的 USER CODE 区仍调用 `Screen_Init()`、`Screen_SetStage()`、`SPI_Receiver_Init()`、`CircuitID_Init()` 和 `Screen_Poll()`。
5. `.ioc` 中 UART4 仍为 9600 8N1（PC10/PC11），且 UART4 全局中断已使能。
