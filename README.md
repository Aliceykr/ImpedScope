# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上。FPGA 对被测电路（DUT）的输入电压 `Ui` 和输出电压 `Uo` 做 FFT，每个频点通过 SPI1 发送一帧 16 字节数据。STM32 完成帧接收、解析、频响计算，并把结果显示在串口屏上。

主要功能：

- SPI1 从机 8 位接收，DMA 每次收一帧（16 字节），收完重新装载
- 硬件 CS（NSS，PA4）界定帧边界，EXTI 双边沿检测帧开始与结束
- 帧头 `0x07 0x08`、帧尾 `0x00 0x00` 校验，短帧丢弃
- 12 位有符号定点分量、13 位峰值 bin 序号解析
- 记录队列，解析在 DMA 回调中完成，计算在主循环中完成
- 传递函数 `H = Uo / Ui`
- 跨阻前端下的阻抗 `Z = -R / H` 和导纳 `Y = -H / R`
- 输入、输出幅值过小时丢弃该频点
- UART4 驱动串口屏，显示频率、阻抗和运行状态
- ADC1 双通道采样：PA0_C（DUT 电压）、PA1_C（电流转换电压），TIM6 触发，DMA 循环写入缓冲区，串口屏显示电压（见第 11 节）

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ spi_receiver.h/.c   SPI1 帧接收、DMA、CS 边沿和数据解析
│  ├─ circuit_id.h/.c     频响、阻抗和导纳计算
│  ├─ screen.h/.c         串口屏 UART4 输出和错误上报
│  └─ adc_sampler.h/.c    ADC1 双通道定时器触发采样（DMA）
├─ Core/
│  ├─ Src/main.c          系统初始化和主循环（USER CODE 区）
│  ├─ Src/spi.c           SPI1、DMA、PA4 和 EXTI4 配置（CubeMX 生成，部分在 USER CODE 区）
│  ├─ Src/stm32h7xx_it.c  中断入口（EXTI4 由 CubeMX 生成）
│  ├─ Src/usart.c         UART4 配置（CubeMX 生成）
│  ├─ Src/adc.c           ADC1 配置（CubeMX 生成，两路扫描、TIM6 触发）
│  └─ Src/tim.c           TIM6 配置（CubeMX 生成，TRGO 触发 ADC）
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

帧边界由 CS（低有效）决定，每帧固定 16 字节，多字节字段按大端顺序发送：

```text
字节  内容
 0    0x07              帧头
 1    0x08              帧头
 2-3  CH1 虚部          12 位有符号，高 4 位为 0 填充
 4-5  CH1 实部          同上
 6-7  CH0 虚部          同上
 8-9  CH0 实部          同上
10-11 index1            13 位无符号，高 3 位为 0 填充
12-13 index0            13 位无符号，高 3 位为 0 填充
14   0x00              帧尾
15   0x00              帧尾
```

通道映射：**CH0 = Ui，CH1 = Uo**。

12 位分量取低 12 位，按第 11 位做符号扩展。`index` 为 FFT 峰值 bin，有效范围 0 – 512（实信号 FFT 的单边频谱），超出范围的帧被拒绝。

SPI 配置：

- FPGA：主机；STM32：SPI1 从机
- 模式：Mode 0（CPOL=0，CPHA=0），MSB first，8 bit
- NSS：硬件输入（PA4），CS 低有效
- DMA：`DMA1_Stream0`，SPI1 RX，普通模式，长度 16 字节，每帧后重新装载

引脚：

| 引脚 | 功能 |
|---|---|
| PB5  | SPI1_MOSI |
| PG9  | SPI1_MISO |
| PG11 | SPI1_SCK |
| PA4  | SPI1_NSS / CS（EXTI4 双边沿）**暂定，待 PCB 确认** |

接收流程：

1. DMA 装载后等待一帧 16 字节。
2. CS 下降沿（帧开始）：若 DMA 未就绪，本帧必丢，报 E02 并重新装载。
3. CS 上升沿（帧结束）：若 DMA 计数不为 0，说明未收满 16 字节（短帧），abort 并报 E03，重新装载。计数为 0 时交给完成回调处理。
4. 收满 16 字节后检查帧头、帧尾和 `index` 范围，通过则放入记录队列（深度 4，满时丢弃最旧记录）。
5. SPI 通信错误（溢出、模式错误等）由 `HAL_SPI_ErrorCallback` 处理：报 E03，abort 并重新装载。

协议的宏定义集中在 [USE/spi_receiver.h](USE/spi_receiver.h)。若 FPGA 的标记值、字节序或字段定义变化，修改该文件即可。

当前协议没有序号或校验和，帧头帧尾只能校验固定字段，不能检测所有载荷错误。

## 5. 数据处理流程

```text
FPGA ──SPI1/DMA/CS──> 一帧 16 字节
                        │  帧头帧尾校验 + 字段解析
                        ▼
                  记录队列（深度 4）
                        │  主循环中 SPI_Receiver_TryGetRecord()
                        ▼
          CircuitID_Compute()   ── 计算 H、Z、Y，检查有效性
                        │  有效结果
                        ▼
          Screen_SetResult()    ── 保存最近一次有效结果
                        │
                        ▼
          Screen_Poll()         ── 每 100 ms 通过 UART4 刷新串口屏
```

主循环每次取出一条记录并调用计算函数。只有计算成功（返回 1）的结果才会更新显示。失败的记录被丢弃，错误码通过串口屏 `t4` 上报。

## 6. 计算方法

输入是一条记录 `FPGA_Record_t`，输出是 `FRF_Point_t`。代码见 [USE/circuit_id.c](USE/circuit_id.c)。

### 6.1 输入检查

1. 配置检查：`R > 0`，否则报 E07。
2. 频点检查：
   - `index0 != index1`：两通道峰值不在同一频点，丢弃并报 E17。
   - `index0 > 512`：超出实信号 FFT 单边频谱，丢弃并报 E03。
   - `index0 == 0`：零频点，报 E07。
3. 定点还原：分量乘 `CIR_FFT_SCALE` 得到电压，结果必须有限，否则报 E08。
4. 幅值检查：
   - `|Ui|² > CIR_INPUT_MIN_MAG²`，否则报 E09（`Ui` 太小，比值会被噪声放大）
   - `|Uo|² > CIR_OUTPUT_MIN_MAG²`，否则报 E10（输出几乎没有信号）

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

`H` 的结果若不是有限数，报 E11。

### 6.3 频率换算

帧中只给峰值 bin，频率按下式换算：

```text
f = bin × CIR_SAMPLE_RATE_HZ / CIR_FFT_POINTS
```

`CIR_SAMPLE_RATE_HZ` 按 FPGA `adc0_clk = clk_40m` 推断为 40 MSPS，**待硬件实测确认**。

### 6.4 阻抗 Z 和导纳 Y（`CIR_DUT_TOPOLOGY = 1` 时计算）

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

串口屏只显示频率、`|Z|` 和 `Z` 相位。

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
| `CIR_SAMPLE_RATE_HZ` | 4e7 | 采样率（fs），待硬件确认 |
| `CIR_FFT_POINTS` | 1024 | FFT 点数 |
| `CIR_FFT_SCALE` | 1.0 | 12 位定点到电压的比例，待 FPGA 缩放校准 |
| `CIR_DUT_TOPOLOGY` | 1 | 1：计算 Z 和 Y；0：只计算 H |
| `CIR_FEEDBACK_RESISTOR_OHM` | 1000.0 | 反馈电阻 R |
| `CIR_INPUT_MIN_MAG` | 1.0 | `|Ui|` 下限（1 LSB，见下方说明） |
| `CIR_OUTPUT_MIN_MAG` | 1.0 | `|Uo|` 下限（1 LSB，见下方说明） |
| `CIR_H_MIN_MAG` | 1e-6 | `|H|` 下限，低于此值 Z 不可靠 |

算 H 时两路同缩放会约掉，`CIR_FFT_SCALE` 不影响 H；算 `|Z|` 绝对值时必须已知该比例。幅值门限需要根据 FPGA 的 FFT 缩放方式和实际噪声水平校准。
当前 `CIR_INPUT_MIN_MAG` / `CIR_OUTPUT_MIN_MAG` 取 1.0（1 LSB）：12 位定点下 `|U|2` 是非负整数，任何小于 1 的门限都只能拒绝恰好为零的频点，等价于“非零”判断。FPGA 缩放标定后应把 `CIR_FFT_SCALE` 与这两个门限一并改为物理电压值。

CS 引脚在 [USE/spi_receiver.h](USE/spi_receiver.h) 中由 `SPI_CS_GPIO_PORT`、`SPI_CS_PIN` 定义。更换引脚时，需同时修改这两个宏、`.ioc` 中的 PA4 配置，以及 `spi.c` 中的 GPIO 与 EXTI 配置。

## 8. 串口屏

UART4，9600 8N1，PC10 TX / PC11 RX。每 100 ms 发送 5 条指令，每条以 `FF FF FF` 结尾。串口屏页面需要建立同名文本框 `t0`~`t4`。

| 控件 | 内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `freq:1000`，无结果时 `freq:-` |
| `t1` | 阻抗幅值 \|Z\|（Ω） | `zmag:1234.56`，无结果时 `zmag:-` |
| `t2` | 阻抗相位（°） | `zph:-12.34`，无结果时 `zph:-` |
| `t3` | 心跳计数，每刷新周期 +1 | `beat:1234` |
| `t4` | 运行阶段或最近一次错误 | `stage:LOOP`、`stage:E03 SPI_FRAME x12` |

上表为正常模式（`SCREEN_SPI_TEST = 0`）。

### 8.1 通信测试模式

[USE/screen.h](USE/screen.h) 中 `SCREEN_SPI_TEST = 1` 时，串口屏 t0、t1 显示两路 ADC 电压，t2 显示 SPI 帧头校验计数，用于在没有有效频响数据时判断采样和通信是否正常。

| 控件 | 内容 | 示例 | 含义 |
|---|---|---|---|
| `t0` | `dut:N.NNN` | `dut:1.234` | DUT 电压（V），无数据时为 `dut:-` |
| `t1` | `cur:N.NNN` | `cur:0.567` | 电流转换电压（V），无数据时为 `cur:-` |
| `t2` | `hdr:N` | `hdr:118` | 帧头帧尾校验通过的次数 |
| `t3` | `beat:N` | `beat:1234` | 心跳，同正常模式 |
| `t4` | `raw:` 加十六进制 | `raw:0708…0000` | 最近一次收满的 16 字节原始帧 |

判读方法：

- **`dut`、`cur` 显示 `-`**：ADC 尚未出数据，检查 `ADC_Sampler_Init()` 是否成功（E01），以及 TIM6 是否启动。
- **`hdr` 不增加**：SPI 没有收到有效帧，或帧头帧尾不对。查看 `t4` 中的原始字节，并检查 PA4 NSS 与 FPGA 的 `spi_cs`。
- **`hdr` 增加**：SPI 通信和帧格式正常，可以切回 `SCREEN_SPI_TEST = 0` 测频响。

测试模式下 `t4` 显示原始帧，不显示错误码。错误仍然计数，但屏幕上看不到。需要查看错误时，切回正常模式。

### 8.2 运行阶段

`main.c` 的 USER CODE 区调用 `Screen_SetStage()` 打点，顺序为：

`BOOT → CLK → PERIPH → MODULE → SPI → LOOP`

`BOOT`、`CLK` 发生在 UART 初始化之前，只保存在内存中，屏幕上看不到。`PERIPH` 起屏幕可见，设置时立即阻塞发送并保持约 150 ms。

### 8.3 错误码

非致命错误通过 `Screen_ReportError(code)` 计数，`t4` 显示最近一次错误及累计次数（16 位，溢出后回绕）。新阶段或新错误会覆盖之前的显示。

致命错误由 `Screen_Fatal()` 或 `Error_Handler()` 触发，关中断后阻塞发送，`t4` 锁定显示。

| 码 | 名称 | 位置 | 触发条件 | 类型 |
|---|---|---|---|---|
| E01 | HAL_INIT | CubeMX 生成的 `MX_*_Init()` | 外设初始化失败（无法区分具体外设） | 致命 |
| E02 | SPI_DMA | `main.c` / `spi_receiver.c` | DMA 启动失败；CS 下降沿时 DMA 未就绪 | 致命 / 非致命 |
| E03 | SPI_FRAME | `spi_receiver.c` / `circuit_id.c` | 帧头帧尾错、短帧、`index > 512`、SPI 通信错误 | 非致命 |
| E04 | SPI_NAN | — | 已弃用（定点帧无浮点），保留编号 | — |
| E05 | SPI_LEN | — | 已弃用（不再解析 float），保留编号 | — |
| E06 | SPI_QFULL | `spi_receiver.c` | 记录队列满，丢弃最旧数据 | 非致命 |
| E07 | CIR_CFG | `circuit_id.c` | `R ≤ 0` 或 `bin = 0` | 非致命 |
| E08 | CIR_NAN_IN | `circuit_id.c` | 输入（含比例换算后）非有限 | 非致命 |
| E09 | CIR_UI_LOW | `circuit_id.c` | `\|Ui\|` 过小 | 非致命 |
| E10 | CIR_UO_LOW | `circuit_id.c` | `\|Uo\|` 过小 | 非致命 |
| E11 | CIR_H_NAN | `circuit_id.c` | H 非有限 | 非致命 |
| E12 | CIR_OUT_NAN | `circuit_id.c` | 频率、dB、相位输出非有限 | 非致命 |
| E13 | CIR_H_LOW | `circuit_id.c` | `\|H\|` 过小，无法计算 Z | 非致命 |
| E14 | CIR_ZY_NAN | `circuit_id.c` | Z 或 Y 非有限 | 非致命 |
| E15 | UART_TX | `screen.c` | `HAL_UART_Transmit_IT()` 返回非 OK | 非致命 |
| E16 | UART_BUSY | `screen.c` | 上一条指令未发完，本周期跳过 | 非致命 |
| E17 | BIN_MISMATCH | `circuit_id.c` | 两通道峰值 bin 不一致（`index0 != index1`） | 非致命 |

时钟配置失败发生在 UART 就绪之前，屏幕无法显示，未纳入错误码表，只能用调试器定位。

## 9. 已知限制

- **两路峰值 bin 不一致的帧被丢弃（E17）。** 详见 9.1 节。
- **FPGA 配置字与 FFT IP 的定义可能不一致。** 按 `xfft_0` 的 testbench，CH1 的正/逆变换标志位应在配置字 bit 1，而 `axis_config.v` 拼接时将其放在 bit 11。这会影响 CH1 的频谱，由 FPGA 侧处理，MCU 无法检测。
- **FPGA 实际发送字节数待确认。** 按 `data_stute.v` 的状态机推断，第 15 字节可能不会发出。需要用逻辑分析仪确认实际帧长。

### 9.1 两路峰值 bin 不一致（E17）

**现象。** 每帧的 `index0`（CH0，即 Ui 的峰值 bin）和 `index1`（CH1，即 Uo 的峰值 bin）可能不同。`index0 != index1` 时，`CircuitID_Compute()` 丢弃该帧并报 E17。

**为什么需要两路 FFT。** `H = Uo / Ui` 必须在同一频率上相除。只有同一频点的 Ui 和 Uo 才能得到 DUT 的幅值和相位。

**为什么会不一致。** FPGA 的 `xfft_0` 是双通道 FFT，两路在同一次变换中输出同一个 bin 网格，因此 bin 号本身是对齐的。不一致来自 FPGA 的峰值选择逻辑（[axi4s_slave.v](C:/Users/k1350/Desktop/opjjg/dac_staute/dac_staute.srcs/sources_1/new/axi4s_slave.v)）：

- `power_fft0` 和 `power_fft1` 各自与各自的历史最大值比较，各自决定 `index0`、`index1`。
- `fft0_max` 和 `fft1_max` 只在复位时清零，帧与帧之间不清。因此 `index` 只会在超过历史全局最大值的 bin 上更新，很可能卡在早期的峰值上。
- 噪声下或 DUT 在激励频率处衰减很大时（如陷波、高阻抗），Uo 的峰值会落到其他 bin，两路 bin 随之分开。这种情况恰好是最有价值的频点，**因此可能被丢弃得最多**。这一点是根据电路原理推断的，尚未用硬件数据验证。

**MCU 端为什么不能补救。** 每帧只带各通道自己峰值处的复数值。`index0` 处的 CH1 值不在帧中，MCU 无法凭当前帧构造同一频点的 H。

**当前处理（决策 A）。** 两路 bin 不一致的帧直接丢弃并报 E17，不会算出错频点的 H。代价是丢帧数量未知。

**如何验证。** E17 的累计次数显示在 `t4` 上（格式 `E17 BIN_MISMATCH xN`）。在硬件上扫频时观察 E17 的增长速度与有效结果数，即可判断丢帧比例。

**后续方案（需要 FPGA 修改，MCU 侧无法单独完成）。**

| 方案 | 做法 | 结果 |
|---|---|---|
| B | 以 `index0` 为频率，FPGA 同时输出 CH1 在 `index0` 处的值 | 不需要峰值相等，但 FPGA 要增加取值通路 |
| C | FPGA 逐 bin 输出，每帧 `index0 = index1 = k`，峰值搜索删除 | 每个频点都有完整 H，频响曲线最完整；SPI 传输量增加，需确认 MCU 能跟上 |

若 E17 占比过高，建议转向方案 C。方案 C 需要先修复 FPGA 的 FFT 配置字位序（见上方“已知限制”），否则两路频谱本身不可信。

## 10. 编译与维护

使用 Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx` 编译。工程基于：

- STM32CubeMX 6.18
- STM32H7 HAL FW 1.13
- ARMCC V5
- 系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz

重新生成 CubeMX 代码后，请检查：

1. Keil 工程中仍包含 `USE/spi_receiver.c`、`USE/circuit_id.c`、`USE/screen.c`。
2. 头文件路径仍包含 `../USE`。
3. SPI1 数据宽度仍为 8 bit，NSS 为硬件输入，DMA 为普通模式（长度 16 字节）。
4. `spi.c` 的 MspInit USER CODE 区仍包含 PA4 的 GPIO 配置和 EXTI4 双边沿配置。
5. `stm32h7xx_it.c` 中仍有 `EXTI4_IRQHandler`，且 `.ioc` 中 `NVIC.EXTI4_IRQn` 已启用。
6. `main.c` 的 USER CODE 区仍调用 `Screen_Init()`、`Screen_SetStage()`、`SPI_Receiver_Init()`、`CircuitID_Init()` 和 `Screen_Poll()`。
7. `.ioc` 中 UART4 仍为 9600 8N1（PC10/PC11），且 UART4 全局中断已使能。
8. `main.c` 的 USER CODE 区在 `MX_TIM6_Init()` 之后调用 `ADC_Sampler_Init()`；`.ioc` 中 ADC1 为两路扫描、连续转换关闭、由 TIM6 TRGO 触发。

## 11. ADC 采样

ADC1 对两路电压做周期性双通道采样，结果写入 DMA 缓冲区，目前只保留最近一对数据，并在测试界面的 t0、t1 上显示电压。采样结果尚未接入频响计算。代码见 [USE/adc_sampler.c](USE/adc_sampler.c)。

### 11.1 通道与引脚

| 通道 | 引脚 | ADC 通道 | 含义 |
|---|---|---|---|
| rank 1 | PA0_C | `ADC_CHANNEL_0`（ADC1_INP0） | DUT 电压 |
| rank 2 | PA1_C | `ADC_CHANNEL_1`（ADC1_INP1） | 电流转换后的电压 |

`_C` 为直连快速通道，经 SYSCFG 模拟开关接入 ADC。引脚配置为模拟模式，无上下拉。`PA0_C` 与普通 `PA0` 是同一个物理引脚，同一时刻只能选其中一条路径。

### 11.2 采样时序

- 由 TIM6 TRGO（上升沿）触发 ADC。定时器 PSC=11、ARR=9，按 120 MHz 计数时钟计算，触发周期为 1 µs（1 MHz）。
- 关闭连续转换，每次触发完成一次两通道扫描。采样时间为 1.5 个 ADC 周期。ADC 时钟由 PLL2 的 75 MHz 经 /2 分频为 37.5 MHz，低于 16 位 ADC 的时钟上限（50 MHz）。ADC 在 `ADC_Sampler_Init()` 中先做 `ADC_CALIB_OFFSET_LINEARITY` 校准，再启动 DMA。
- DMA1_Stream1 循环模式，半字（16 位）对齐。每填满半个缓冲区，回调保存该半区最后一对采样。

### 11.3 接口

| 函数 / 类型 | 说明 |
|---|---|
| `uint8_t ADC_Sampler_Init(void)` | 启动 ADC1 DMA 和 TIM6，成功返回 1。`main.c` 中调用，失败报 E01 |
| `uint8_t ADC_Sampler_GetLatest(ADC_Sample_t *out)` | 取最近一对采样，尚无数据时返回 0 |
| `ADC_Sample_t` | `dut_voltage_raw`、`current_voltage_raw`：16 位原始码值；`sequence`：每完成一个半区加 1，0 表示无数据 |

### 11.4 待确认

- 串口屏电压按 16 位满量程 65535、参考电压 3.3 V 换算（`ADC_SAMPLER_VREF_V`），实际 VREF 与量程待实测确认。
- 通道映射（PA0_C → INP0，PA1_C → INP1）依据工程原有配置和 ST 社区资料，请以 CubeMX 引脚列表为准。
- 1 MHz 为初始采样速率。需要其他速率时，修改 TIM6 的 PSC 和 ARR 即可。
