# STM32 黑盒电路频响辨识

> 适用硬件：STM32H750XBH6，3.3 V 单电源 · 文档版本：v3.0（2026-10-27），按当前固件实现编写

## 1. 概述

本工程用一颗 STM32H750XBH6 测量未知一端口网络（DUT）的导纳：

- 外部信号源输出 **1 Vpp、1.65 V 直流偏置** 的正弦，接到 `Ui` 节点；
- DUT 接在 `Ui` 与运放虚地（1.65 V）之间，跨阻放大器把 DUT 电流转换成 `Uo`；
- 两路 ADC 同步采样 `Ui`、`Uo`，在选定频率上做单频 DFT，得到 `H = Uo/Ui`；
- 由 `Y = -H/R_eff`、`Z = -R_eff/H` 计算导纳、阻抗、G、B、φY；
- 固件自动在 **5 档频率** 中选择；反馈电阻为 **3 路继电器**，本版上电固定闭合 1.5 kΩ 档、不做自动量程。结果送串口屏，同时通过 USART3 输出调试日志。

测量范围（任务书指标）：`Re[Y]`、`Im[Y]`、`|Y|` 为 **0.1–10 mS**，误差 ≤10%；`φY` 为 **±90°**，误差 ≤5%。**本版默认 1.5 kΩ 档，覆盖 0.188–1.88 mS；b0 双击可切 150 Ω / 15 kΩ 档，0.1–10 mS 全量程需按档标定（见 §8、§15）。**

## 2. 系统架构

### 2.1 模拟前端

```text
外部源 ── Ui ── DUT ──┬── 虚地 1.65 V（运放反相端）
 1 Vpp                │
 1.65 V 偏置          └── R（K1/K2/K3 之一，上电默认 K2 = 1.5 kΩ，双击切档）
                             │
                        继电器触点（三路并联，高电平闭合）
                             │
                        运放输出 Uo（直流 1.65 V）
```

- 运放为 3.3 V 单电源、轨到轨；反相端由同相端偏置在 **1.65 V**，所以虚地和 `Uo` 的直流工作点都是 1.65 V。
- `Uo` 的交流摆幅设计为 0.15–1.5 V（围绕 1.65 V），即 0.15–3.15 V，落在 ADC 的 0–3.3 V 内。
- 三路独立继电器（高电平闭合）各串一只反馈电阻，并联在 `Uo` 与虚地之间：**K1=150 Ω / K2=1.5 kΩ / K3=15 kΩ**。
- 本版上电只闭合 **K2**，上电时 `R_eff = 1500 Ω`（双击可切）；继电器接触电阻（<1 Ω）相对最小档 150 Ω 可忽略，按标称值使用。
- GPIO（**PE2/PE3/PE4**）只驱动继电器模块的逻辑输入，模块自带三极管/MOS 驱动与续流。

### 2.2 数字部分

- ADC1/ADC2 双 ADC 规则组同步采样，TIM6 TRGO 触发 300 kHz，CDR 打包 + DMA。
- UART4（921600）驱动串口屏；USART3（PB10，921600，TX-only）输出调试日志。
- 调试口为 SWD（PA13/PA14），没有调试 UART 复用。
- SPI1 仍初始化但当前无从设备，不参与测量。

## 3. 工程结构

```text
9_30/
├─ USE/
│  ├─ measure.h/.c        测量核心：频率、ADC 采样与单频 DFT、H/Z/Y、标定
│  ├─ screen.h/.c         串口屏 UART4 协议、错误码、b0 按键
│  ├─ relay_range.h/.c    三路继电器反馈电阻（双击切档）
│  └─ uart_debug.h/.c     USART3 调试日志（TX-only、环形缓冲）
├─ Core/
│  ├─ Src/main.c          初始化、自动选频、主循环
│  ├─ Src/usart.c         UART4（屏幕）+ USART3（调试）
│  ├─ Src/adc.c           ADC1/ADC2 双 ADC 同步配置
│  ├─ Src/tim.c           TIM6，TRGO 触发 ADC
│  ├─ Src/spi.c           SPI1（当前无设备）
│  ├─ Src/stm32h7xx_it.c  中断入口（含 ADC/UART4/USART3）
│  └─ Src/gpio.c          CubeMX 生成的 GPIO
├─ Drivers/               STM32H7 HAL、CMSIS
├─ MDK-ARM/               Keil 工程（39 个源文件）
└─ 9_30.ioc               CubeMX 配置
```

`USE/` 是用户逻辑；CubeMX 重新生成时保留该目录。`relay_range` 和 `uart_debug` 已加入 Keil 的 `Application/User/USE` 组。

## 4. 引脚与资源

| 资源 | 引脚 | 配置 | 用途 |
|---|---|---|---|
| ADC1 | PA0_C | `ADC1_INP0`，模拟 | `Ui` |
| ADC2 | PA1_C | `ADC2_INP1`，模拟 | `Uo` |
| UART4 | PC10 / PC11 | AF8，921600 8N1 | 串口屏 TX/RX |
| USART3 | PB10 | AF7，921600 8N1，TX-only | 调试日志 |
| 继电器 K1/K2/K3 | PE2 / PE3 / PE4 | GPIO 推挽输出 | 量程选择（上电默认 K2，b0 双击切档） |
| TIM6 | 内部 | PSC=3，ARR=99 | 300 kHz TRGO |
| SWD | PA13 / PA14 | Serial Wire | 调试/下载 |
| HSE | PH0 / PH1 | 外部晶振 | 系统时钟 |
| LSE | PC14 / PC15 | 32.768 kHz | RTC 备选 |
| SPI1 | PB5 / PG11 / PG9 | MOSI/SCK/MISO | 当前无设备 |

- PB11 不使用（USART3 只配 TX）。
- PA4 不分配。
- 所有模拟输入为 0–3.3 V 单端，无负压。

## 5. 主循环与数据流

```text
上电
  → HAL/时钟/GPIO/DMA/SPI1/UART4/USART3/ADC1/TIM6/ADC2
  → Screen_Init()            启动 UART4 接收（b0）
  → Relay_Init()            上电闭合 K2（1.5 kΩ），等吸合
  → CircuitID_Init()
  → ADC_Sampler_Init()       校准 + 双 ADC 同步 + DMA + TIM6
  → UartDebug_Init()          调试口就绪，输出 banner
  → FreqScan_Select()        五档频率打分
  → 主循环
       b0 单击 → 重扫；双击 → 切档（见 §9）
       f = Measure_GetFrequency()
       Freq_SourceSet(f)               空实现，源在外部
       ADC_Sampler_SetFrequency(f)     同频不重开；换频开新窗口
       ADC_Sampler_GetWindow()         等窗口，超时 1500 ms
       CircuitID_Compute()             算 H/Z/Y/G/B
       Screen_SetResult() + UartDebug_LogMeasurement()
       Screen_Poll()                   每 500 ms 刷新屏幕
```

- 反馈电阻上电固定 1.5 kΩ；b0 **双击**可循环切档（1.5k → 15k → 150 → 1.5k），切档后 `CircuitID_SetFeedbackResistor()` 同步 `R_eff`、丢弃当前窗口并重开。
- 计算失败/超时会清结果，屏幕显示 `-`，不显示陈旧读数。

## 6. 测量原理

### 6.1 采样窗口

```text
N = round(100 × fs / f)，限制在 [256, 400000]
fs = 300 kHz
```

窗口覆盖 100 个信号周期：

| f | N | 窗口时长 |
|---|---|---|
| 100 Hz | 300000 | 1 s |
| 300 Hz | 100000 | 333 ms |
| 1 kHz | 30000 | 100 ms |
| 3 kHz | 10000 | 33.3 ms |
| 10 kHz | 3000 | 10 ms |

窗口时长远小于 `MEASURE_WAIT_TIMEOUT_MS = 1500 ms`；100 Hz 的 1 s 也留了 500 ms 余量。

### 6.2 单频 DFT

```text
ω = 2π f / fs
S = Σ (x_n − x̄) · e^(−j ω n)，n = 0 … N−1
x̄ = 窗口平均值（去直流）
|U| = 2 |S| / N        （ADC 码值，峰值）
```

- `S_i` 对应 `Ui`（ADC1），`S_o` 对应 `Uo`（ADC2）。
- 两路共用同一个 N 和同一个块序号，`H = Uo/Ui` 与归一化无关。

### 6.3 相干性判据

```text
kappa = |U| / (sqrt(2) · RMS_ac)
```

- 纯单频：`kappa = 1.0`；纯噪声：`kappa ≈ sqrt(2/N)`（N=30000 时约 0.008）。
- `kappa < CIR_MIN_COHERENCE`（0.2）→ `Ui` 报 E14、`Uo` 报 E15。
- 幅值下限：`|Ui| < 4` 码 → E04；`|Uo| < 4` 码 → E05。两个门限是占位值，待实测调整。

### 6.4 传递函数、阻抗、导纳

```text
H = Uo / Ui
Z = -R_eff / H
Y = -H / R_eff
G = Re[Y]，B = Im[Y]
```

- `H` 的 dB/相位由 `20·log10(|H|)` 和 `atan2(H_im, H_re)` 得到。
- `|H| < CIR_H_MIN_MAG`（1e-6）→ E08；Z/Y 非有限 → E09。
- `R_eff` 由 `relay_range` 提供，上电恒为 1500 Ω；b0 双击可切为 150/1500/15000 Ω，见第 8、9、13 节。
- 两路 ADC 的增益/相位差由 `s_calib[]` 修正，见第 13 节。

### 6.5 数值示例

1 µF 电容，`R_eff = 115 Ω`（15 Ω 档），1 kHz，源 1 Vpp（`Ui` 峰值 0.5 V）：

```text
Y   = jωC ≈ j 6.283 mS
|Y| ≈ 6.283 mS，φY = +90°，G = 0，B ≈ +6.283 mS
H   = -Y·R_eff ≈ -j 1.571（4.08 dB，-90°）
Uo 峰值 = 0.5 V × 1.571 ≈ 0.785 V（不削顶）
|Z| = 1/|Y| ≈ 159 Ω
```

## 7. 自动选频

`FreqScan_Select()` 在启动和按 b0 时执行：

- 候选顺序：`1 kHz, 10 kHz, 3 kHz, 300 Hz, 100 Hz`；
- 每个候选开一个窗口，算 `Ui` 的单频幅值 `ui_mag` 和相干性 `ui_coh`；
- 打分 `score = ui_mag × ui_coh`，要求 `ui_mag > CIR_INPUT_MIN_AMP` 且 `ui_coh > 0.5`；
- 取最高分；某档 `ui_coh > 0.95` 时提前结束；
- 全部不合格时回到 1 kHz 并返回 0，主循环报 E04。

选频只依赖 `Ui`，与反馈电阻无关；量程默认 1.5 kΩ，由 b0 双击手动切换，无需选档。`Measure_SetFrequency()` 可覆盖频率，范围 100–100000 Hz；外部源必须同步到当前频率。100 Hz 档窗口 1 s，扫描五档可能需要数秒。

## 8. 继电器量程（三档，上电默认 1.5 kΩ）

### 8.1 三路继电器与档位

| 通道 | GPIO | 电阻 | 上电后状态 | 覆盖 \|Y\|（Uo 0.15–1.5 V，参考） |
|---|---|---|---|---|
| K1 | PE2 | 150 Ω | 断开（低） | 1.2–12 mS |
| K2 | PE3 | 1.5 kΩ | **闭合（高，默认档）** | 0.188–1.88 mS |
| K3 | PE4 | 15 kΩ | 断开（低） | 19.9–198.7 µS |

- 高电平 = 继电器线圈得电 = 触点闭合；低电平/GND = 断开。三路**独立**，任意时刻最多闭合一路。
- 每路：`Uo → 继电器触点 → 反馈电阻 → 虚地（1.65 V）`，三路并联；触点只承受小信号（≤1.5 V、≤5 mA）。
- 继电器接触电阻通常 0.05–0.5 Ω，相对最小档 150 Ω 可忽略，`R_eff` 直接按标称值。
- GPIO 只驱动继电器模块的逻辑输入（模块自带三极管/MOS 驱动与续流），不要直接驱动裸线圈。

### 8.2 上电时序与运行期切档

- `MX_GPIO_Init()`（CubeMX 生成）配 PE2/PE3/PE4 为推挽输出并写低；`Relay_Init()` 只做上电时序：三路全低（断开）→ 等 10 ms 释放 → 只把 PE3 拉高（K2 = 1.5 kΩ）→ 等 10 ms 吸合。
- `CircuitID_SetFeedbackResistor(Relay_GetOhm())` 同步固定 `R_eff = 1500 Ω`，与 `s_calib`（Rf=1500）和 `CIR_FEEDBACK_RESISTOR_OHM` 对齐。
- **本版不做自动量程**；运行期切档由 b0 **双击**触发（`Relay_Set()`/`Relay_Next()`，1.5k → 15k → 150 → 1.5k 循环），不按 `Uo` 幅值/削波自动换档。
- 上电三路全断到 K2 闭合之间 TIA 开环、`Uo` 可能撞轨；`Relay_Init()` 在 `ADC_Sampler_Init()` 之前完成，不会采到无效窗口。

## 9. 串口屏（UART4）

- UART4，**921600 8N1**，PC10 TX / PC11 RX，DMA/中断均不使用，阻塞发送。
- 每 500 ms 发一整帧 `t0`–`t10`，每条 `控件名.txt="值"` + `FF FF FF`；条间留 2 ms 给 HMI 重绘。
- 屏幕只显示，频率由固件自动选择；b0 单击 = 强制重扫，双击 = 切换继电器档位。

| 控件 | 内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `f:1000`，无结果显示 `f:-` |
| `t1` | `\|Y\|`（mS） | `Y:1.000` |
| `t2` | φY（°） | `P:-12.34` |
| `t3` | G = Re[Y]（mS） | `G:0.977` |
| `t4` | B = Im[Y]（mS） | `B:-0.213` |
| `t5` | 阶段或最近错误 | `stage:LOOP`、`stage:E12 WINDOW_TIMEOUT x12` |
| `t6` | 心跳，每帧 +1 | `beat:1234` |
| `t7` | `Ui` 交流有效值（V）与 kappa | `Ui:0.1234V k:0.98` |
| `t8` | `Uo` 交流有效值（V）与 kappa | `Uo:0.0988V k:0.97` |
| `t9` | H 的幅值（dB） | `H:-3.97dB` |
| `t10` | 当前反馈电阻档位 | `R:1.5k` |

- `t1`–`t4` 依赖 `CIR_DUT_TOPOLOGY=1`；设为 0（只算 H）时这四行固定显示 `-`。
- 阶段顺序：`BOOT → CLK → PERIPH → MODULE → SCAN → LOOP`；`BOOT`/`CLK` 在 UART 就绪前，屏幕不可见；`PERIPH` 起可见，且保持约 150 ms。
- b0（`freq_change`）发送 `0x01 0x01 0x01`（`printh 010101`）；`Screen_PollKey()` 每轮调用一次：单击（300 ms 窗口内无第二下）= 强制重扫，双击 = 切档。双击判定用中断记录的时间戳，不受 DFT 窗口阻塞影响；有效双击间隔 100–300 ms。
- 档位在 `t10` 显示为 `R:150 / R:1.5k / R:15k`；切档时 `Screen_FlushRange()` 立即补发 `t10`，不等 500 ms 整帧。`Screen_SetActiveFreq()` 只更新当前频率、清上一频率结果并强制下一帧刷新。

## 10. 调试串口（USART3）

- **USART3，PB10 TX（AF7），921600 8N1，TX-only**；PB11 不使用。
- 中断优先级 **5**（UART4/ADC/DMA 为 0），不抢占采样窗口。
- 发送：1024 字节环形缓冲 + `HAL_UART_Transmit_IT()`；主循环只拷贝，不阻塞；缓冲满整行丢弃并计数。
- 速率：每次有效测量一行，最小间隔 `UART_DEBUG_MIN_INTERVAL_MS = 200 ms`（5 Hz）。
- 只发不收，没有下行命令；标定写入/表格导出仍需另加协议。

日志格式（CSV，整数单位，不用 `%f`）：

```text
f=1000,Y_uS=1000,P_mdeg=-12300,G_uS=977,B_uS=-213,H_dB_x100=408,H_deg_x100=16770,R_ohm=1500,range=1,Ui_rms_mV=354,Uo_rms_mV=566
```

- `Y_uS/G_uS/B_uS`：µS（1 mS = 1000 µS）
- `P_mdeg`：毫度（-12.3° → -12300）
- `H_dB_x100 / H_deg_x100`：dB×100、度×100
- `R_ohm`：当前 `R_eff`
- `range`：0=150 Ω、1=1.5 kΩ、2=15 kΩ（当前档位，随切档变化）
- `Ui_rms_mV / Uo_rms_mV`：去直流后的交流有效值（mV）

上电 banner：

```text
BOOT 9_30 debug uart 921600
```

接线：USB-TTL 的 **RX → PB10**，**GND → GND**。若 921600 误码，改 `.ioc` 的 `USART3.BaudRate` 和 `usart.c` 中的 921600 为 115200。USART3 出错时 `screen.c` 的 `HAL_UART_ErrorCallback()` 会清理标志并调用 `UartDebug_OnError()` 重启发送。

## 11. ADC 采样

### 11.1 通道

| 通道 | 引脚 | ADC 通道 | 含义 |
|---|---|---|---|
| ADC1（主） | PA0_C | `ADC1_INP0` | `Ui` |
| ADC2（从） | PA1_C | `ADC2_INP1` | `Uo` |

`_C` 为直连快速通道，经 SYSCFG 模拟开关接入；PA0_C/PA1_C 与普通 PA0/PA1 是两个焊盘。

### 11.2 时序

- 双 ADC 规则组同步模式（`ADC_DUALMODE_REGSIMULT`），16 位，`32_10_BITS` 打包。
- TIM6 TRGO 上升沿同时启动两路；PSC=3、ARR=99、120 MHz → **300 kHz**。
- 采样时间 **1.5 个 ADC 周期**（37.5 MHz 下 40 ns）；16 位转换 8.5 周期，总计约 267 ns，远小于 3.33 µs 触发周期。
- ADC 时钟 PLL2 75 MHz / 2 = **37.5 MHz**。
- 只有 ADC1 的 DMA（DMA1_Stream2，32 位字、循环）搬运 CDR；ADC2 的 DMA 配置保留但同步模式下不启动。
- 每半个缓冲区 64 对样本回调一次。

### 11.3 初始化与保护

- `ADC_Sampler_Init()`：回读 CCR 校验同步模式/打包格式（不符返回 2 → E16 致命）→ 两路 offset/linearity 校准 → `HAL_ADCEx_MultiModeStart_DMA()` → 使能 `ADC_IRQn` → 启动 TIM6。
- OVR（E17）或 DMA 错误（E18）会作废当前窗口；由 `HAL_ADC_ErrorCallback()` 置弃窗标志，`ADC_Sampler_TryFinish()` 丢弃，不置 ready。
- 溢出/DMA 错误不终止测量，主循环等下一个干净窗口。

### 11.4 接口

| 函数 | 说明 |
|---|---|
| `ADC_Sampler_Init()` | 校准、启动同步转换和 TIM6；返回 0/1/2 |
| `ADC_Sampler_SetFrequency(f)` | 设定频率并开新窗口；同频不重开；非法返回 0 |
| `ADC_Sampler_GetWindow(&w)` | 取出已完成窗口；无新窗口返回 0 |
| `ADC_Sampler_Restart()` | 以当前频率强制重开窗口；换档后调用 |
| `ADC_Window_t` | `frequency_hz`、`sample_count`、`ui_re/im`、`uo_re/im`、`ui_rms/uo_rms`、`clip` |

改采样率时必须同时改 TIM6 的 PSC/ARR 和 `ADC_SAMPLER_RATE_HZ`，否则 DFT 频率轴静默错位。

### 11.5 削波检测

- 统计窗口内贴边样本：码值 `≤ CIR_CLIP_LOW_CODES (64)` 或 `≥ CIR_CLIP_HIGH_CODES (65471)`；
- 贴边比例 > `CIR_CLIP_MAX_PERMILLE (1%)` → `Ui` 报 E19、`Uo` 报 E20；
- 削顶时波形仍完整，结果照常计算/显示，但提示不可信；E17/E18 才是必须弃窗的失效。
- 局限：只能抓 ADC 端贴边；运放先饱和（ADC 尚未贴边）抓不到，仍需把前端偏置/幅度做对。

## 12. 参数配置

| 宏 | 文件 | 默认 | 说明 |
|---|---|---|---|
| `ADC_SAMPLER_RATE_HZ` | `measure.h` | 300000 | DFT 频率轴（必须与 TIM6 一致） |
| `ADC_SAMPLER_PAIRS_PER_HALF` | `measure.h` | 64 | 每个 DMA 半缓冲的采样对数 |
| `ADC_SAMPLER_WINDOW_CYCLES` | `measure.h` | 100 | 窗口覆盖的信号周期数 |
| `ADC_SAMPLER_WINDOW_MIN/MAX` | `measure.h` | 256 / 400000 | 窗口点数上下限 |
| `ADC_SAMPLER_VREF_V` | `measure.h` | 3.3 | VREF+/VDDA，只影响 t7/t8 电压 |
| `CIR_DUT_TOPOLOGY` | `measure.h` | 1 | 1=算 Z/Y；0=只算 H |
| `CIR_FEEDBACK_RESISTOR_OHM` | `measure.h` | 1500.0 | 固定反馈电阻（relay_range 同步后使用） |
| `CIR_INPUT_MIN_AMP` / `CIR_OUTPUT_MIN_AMP` | `measure.h` | 4.0 | 幅值下限（码值，占位） |
| `CIR_MIN_COHERENCE` | `measure.h` | 0.2 | 相干性下限 |
| `CIR_H_MIN_MAG` | `measure.h` | 1e-6 | `|H|` 下限 |
| `CIR_CLIP_LOW/HIGH_CODES` | `measure.h` | 64 / 65471 | 贴边码值 |
| `CIR_CLIP_MAX_PERMILLE` | `measure.h` | 10 | 贴边比例上限（1%） |
| `RELAY_COUNT` | `relay_range.h` | 3 | 继电器档位数 |
| `RELAY_DEFAULT_INDEX` | `relay_range.h` | 1 | 默认档（K2 = 1.5 kΩ） |
| `RELAY_SETTLE_MS` / `RELAY_RELEASE_MS` | `relay_range.h` | 10 / 10 | 吸合建立 / 释放等待（ms） |
| `Relay_Set()` / `Relay_Next()` | `relay_range.h` | — | 切档 / 切到下一档（b0 双击触发） |
| `UART_DEBUG_ENABLE` | `uart_debug.h` | 1 | 调试日志开关 |
| `UART_DEBUG_MIN_INTERVAL_MS` | `uart_debug.h` | 200 | 日志最小间隔（ms） |
| `UART_DEBUG_TX_BUFFER_SIZE` | `uart_debug.h` | 1024 | 发送环形缓冲（字节） |
| `MEASURE_WAIT_TIMEOUT_MS` | `main.c` | 1500 | 单窗口等待上限（ms） |
| `SCREEN_REFRESH_PERIOD_MS` | `screen.h` | 500 | 串口屏刷新周期（ms） |
| `SCREEN_DOUBLE_CLICK_MS` | `screen.h` | 300 | 单击/双击判定窗口（ms），须 > 100 ms 按键锁定期 |

## 13. 标定

### 13.1 反馈电阻

上电默认闭合 K2（1.5 kΩ），`R_eff` 由 `CircuitID_SetFeedbackResistor(Relay_GetOhm())` 同步；
1.5 kΩ 档已标定，继电器接触电阻（<1 Ω）可忽略。双击切到 150 Ω / 15 kΩ 档后需按档重标 `R_eff`。

### 13.2 两路直通 H_thru

把 `Ui` 和 `Uo` 短接到同一信号源，理论上 `H = 1`（0 dB、0°）；实测偏差就是两路增益比与相位差：

```text
H_true(f) = H_meas(f) / H_thru(f)
mag_ratio = 1 / |H_thru|
phase_deg = angle(H_thru)
```

- 1.5 kΩ 档按**逐频点**做即可；切到 150 Ω / 15 kΩ 档后 C_off 随档位变化，这两档的 H_thru 需逐档重做。
- 标定时窗口长度、采样率、频率必须与实际测量一致。

### 13.3 当前标定表

`s_calib[]` 目前有两项，在 **1.5 kΩ 反馈**（即本版默认档）下测得，直接适用：

| 频率 | mag_ratio | phase_deg |
|---|---|---|
| 1 kHz | 1.009 | -1.186° |
| 10 kHz | 1.031 | -11.559° |

`Calib_Lookup()` 是**就近取值、不插值**，只标 1k/10k 时 100/300/3k 会取最近点。

## 14. 错误码

非致命错误由 `Screen_ReportError()` 计数，`t5` 显示最近错误和累计次数；新阶段/新错误覆盖显示。致命错误由 `Screen_Fatal()`/`Error_Handler()` 触发，关中断后阻塞发送，`t5` 锁定。

| 码 | 名称 | 位置 | 触发条件 | 类型 |
|---|---|---|---|---|
| E01 | HAL_INIT | `MX_*_Init()`、`ADC_Sampler_Init()` | 外设初始化/校准/DMA 启动失败 | 致命 |
| E02 | CIR_CFG | `measure.c` | `R_eff ≤ 0`、频率 0、窗口空 | 非致命 |
| E03 | CIR_NAN_IN | `measure.c` | Ui/Uo 分量非有限 | 非致命 |
| E04 | CIR_UI_LOW | `measure.c` | `\|Ui\| < CIR_INPUT_MIN_AMP` | 非致命 |
| E05 | CIR_UO_LOW | `measure.c` | `\|Uo\| < CIR_OUTPUT_MIN_AMP` | 非致命 |
| E06 | CIR_H_NAN | `measure.c` | H 非有限 | 非致命 |
| E07 | CIR_OUT_NAN | `measure.c` | 频率/dB/相位输出非有限 | 非致命 |
| E08 | CIR_H_LOW | `measure.c` | `\|H\| < CIR_H_MIN_MAG` | 非致命 |
| E09 | CIR_ZY_NAN | `measure.c` | Z/Y 非有限 | 非致命 |
| E10 | UART_TX | `screen.c` | UART4 发送失败 | 非致命 |
| E11 | UART_BUSY | — | 保留：当前 UART4 用阻塞发送，不触发 | — |
| E12 | WINDOW_TIMEOUT | `main.c` | 等 ADC 窗口超时 | 非致命 |
| E13 | — | — | 已停用，保留编号 | — |
| E14 | CIR_UI_INCOH | `measure.c` | `Ui` 非单频（kappa < 0.2） | 非致命 |
| E15 | CIR_UO_INCOH | `measure.c` | `Uo` 非单频（kappa < 0.2） | 非致命 |
| E16 | ADC_DUAL_MODE | `measure.c` | 双 ADC 同步/打包格式回读不符 | 致命 |
| E17 | ADC_OVR | `measure.c` | ADC 溢出，弃窗 | 非致命 |
| E18 | ADC_DMA | `measure.c` | ADC DMA 错误，弃窗 | 非致命 |
| E19 | ADC_CLIP_UI | `measure.c` | `Ui` 削顶 | 非致命 |
| E20 | ADC_CLIP_UO | `measure.c` | `Uo` 削顶 | 非致命 |

时钟配置失败发生在 UART 就绪前，屏幕无法显示，只能用调试器定位。

## 15. 已知限制

- **默认只测 1.5 kΩ 档**：覆盖约 0.188–1.88 mS；b0 双击可切 150 Ω / 15 kΩ 档（约 1.2–12 mS / 19.9–198.7 µS），0.1–10 mS 全量程仍需按档标定 `R_eff`。
- **继电器模块依赖外部硬件**：必须高电平闭合、GPIO 只接模块逻辑输入；若模块低有效或上电吸合，需改硬件或在 `Relay_Write()` 反转（见 §8）。
- **上电三路全断 → TIA 开环**：`Relay_Init()` 必须在 `ADC_Sampler_Init()` 之前完成；复位/上电期间模块输入建议加 10 kΩ 下拉。
- **自动选频耗时**：100 Hz 窗口 1 s，重新扫描五档可能需要数秒。
- **外部源频率/直流不在固件控制内**：源频率偏离会导致 κ 下降/相位误差；源直流中点必须是 1.65 V，否则 DUT 上会有 1.65 V 直流，TIA 可能饱和。
- **幅值门限是占位值**：`CIR_INPUT_MIN_AMP`/`CIR_OUTPUT_MIN_AMP = 4` 码，需按实际噪声调整。
- **调试串口只发不收**：USART3 TX-only，没有下行命令。
- **档位在 t10 显示**：`R:150 / R:1.5k / R:15k`（当前档，切档后立即更新）；更细的 `R_eff` 可从 USART3 日志的 `range`/`R_ohm` 看。
- **两路增益/相位只标了 1.5 kΩ 档**：默认档 `s_calib` 直接适用；双击切到 150 Ω / 15 kΩ 档后绝对值只可信到相对变化，需按档重标。
- **双击手感**：判定窗口 300 ms，有效双击间隔 100–300 ms；若实测成功率低，调 `SCREEN_DOUBLE_CLICK_MS`（如 350–400）即可。
- **扫描期间双击**：`FreqScan_Select()` 需数秒，期间双击排入队列，待扫描结束、先用旧档测出一个结果后下一轮才切档——屏幕上可能先闪一个新读数再被清掉，属正常时序，不影响最终结果。

## 16. 编译与维护

- Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx`；ArmClang V6.24（`.uvprojx` 的 `pArmCC` 已钉到该版本）。
- CubeMX 6.18，HAL FW 1.13；系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz；ADC 37.5 MHz；SPI1 75 MHz。
- 当前 **39 个源文件**，含 `relay_range.c`、`uart_debug.c`。
- 命令行构建：

```bash
bash .ccg-build/build.sh <outdir>
```

CubeMX 重新生成后检查：

1. Keil 工程仍包含 `measure.c`、`screen.c`、`relay_range.c`、`uart_debug.c`，头文件路径含 `../USE`。
2. `.ioc`：PE2/PE3/PE4 为 `GPIO_Output`（标签 `RELAY_K1/K2/K3`）；PB10 为 `USART3_TX`；UART4/USART3 均 921600；UART4/USART3 全局中断使能。
3. `main.c` USER CODE 区仍调用 `Screen_Init()`、`Relay_Init()`、`CircuitID_Init()`、`ADC_Sampler_Init()`、`UartDebug_Init()`、`FreqScan_Select()`、`Screen_Poll()`、`UartDebug_LogMeasurement()`。
4. `adc.c` 的 `ADC_DUALMODE_REGSIMULT`/`DualModeData`/`TwoSamplingDelay` 仍在 `USER CODE BEGIN ADC1_Init 2` 区；ADC1 的 DMA 为 Word 对齐。
5. TIM6 的 PSC/ARR 与 `ADC_SAMPLER_RATE_HZ` 一致；SPI1 仍为主机但无设备。
6. `relay_range.c`、`uart_debug.c` 在 `Application/User/USE` 组。

## 17. 验证状态

已通过：

- ArmClang 编译 + 链接：**39 files, 0 errors**；Program Size：Code 51322、RO-data 2378、RW-data 28、ZI-data 4988。
- `.ioc` 与代码的引脚/实例一致性检查：PE2/PE3/PE4（`RELAY_K1/K2/K3`）、PB10/USART3、PC10/11/UART4。
- 固件源码（`USE/`、`Core/`）静态检查：无 `analog_switch` / `Switch_*` / `SW_ENABLE` / `Screen_TakeRefreshKey` 残留。

待上板：

- USART3 921600 日志（T1）、与串口屏共存（T2）、速率/丢行（T3/T4）、中断影响（T5）、误码（T6）、回归（T7）。
- 继电器上电默认（R1）：PE2/PE3/PE4 = 0 V / 3.3 V / 0 V（K1 断、K2 合、K3 断）；K2 触点通断约 1.5 kΩ（R2）。
- 上电默认档 `R_eff = 1500 Ω`（R3）、1 kΩ 已知 DUT 的 |Y| ≈ 1 mS（R4）、30 s 观察无自动切档（R5）、断电/上电 10 次重复性（R6）。
- b0 双击切档（编号 B1–B9，与 USART3 的 T、继电器组的 R 区分）：B1 单击 300 ms 后重扫；B2 双击 200 ms 间隔 1.5k→15k 且 `t10` 立即变 `R:15k`；B4 间隔 500 ms 两次按下 = 两次单击（两次重扫）；B5 100 Hz 档下双击仍识别；B6 双击后测量用新 `R_eff`。

## 18. 实施记录

- 继电器三档量程与 b0 双击切档的实施过程记录在 git 历史中；对应 PLAN 文档已完成使命后删除。
