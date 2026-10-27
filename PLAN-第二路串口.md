# PLAN：第二路串口（USART3 调试日志）

- 状态：**v1.0，已实施并编译通过（39 files, 0 errors）**。
- 日期：2026-10-22
- 适用工程：ImpedScope / 9_30（STM32H750XBH6）
- 关联代码：[Core/Src/usart.c](Core/Src/usart.c)、[Core/Inc/usart.h](Core/Inc/usart.h)、[Core/Src/main.c](Core/Src/main.c)、[Core/Src/stm32h7xx_it.c](Core/Src/stm32h7xx_it.c)、[9_30.ioc](9_30.ioc)、[MDK-ARM/9_30.uvprojx](MDK-ARM/9_30.uvprojx)、新增 [USE/uart_debug.c](USE/uart_debug.c) / [USE/uart_debug.h](USE/uart_debug.h)
- 一句话：新增 USART3（PB10 TX，921600 8N1，3.3 V TTL，TX-only）作为**只读调试日志口**，每完成一次有效测量输出一行测量值；UART4 继续专用于串口屏，两者互不干扰。

---

## 0. 结论摘要

1. **实例与引脚**：USART3，PB10 = TX（AF7），3.3 V TTL；**TX-only**，PB11 不使用。
2. **只输出、不接收命令**：本次不做标定写入、不做 Flash 持久化、不做二进制流；日志内容包含 f、|Y|、φY、G、B、H、当前档位、Ui/Uo。
3. **非阻塞**：发送走 1024 字节环形缓冲 + `HAL_UART_Transmit_IT()`，主循环只做内存拷贝，不 `HAL_Delay`、不阻塞测量窗口。
4. **速率**：默认每完成一次有效测量输出一行，最小间隔 200 ms（5 Hz）；1 kHz 测量时最多 5 行/秒，100 Hz 时 1 行/秒。
5. **代码隔离**：协议/缓冲全部放在 `USE/uart_debug.c/.h`；`usart.c/h` 只加 USART3 的 CubeMX 风格初始化；`main.c` 只加两处调用；`stm32h7xx_it.c` 加 `USART3_IRQHandler`。
6. **`.ioc` 同步**：PB10、USART3、NVIC（优先级 5）、`Mcu.IP`/`PinsNb`、`functionlistsort` 全部补上；CubeMX 重新生成后初始化代码可复现。
7. **预算**：一行约 100 字节，921600 下约 1.1 ms；5 Hz 占空比约 0.55%，对测量无可见影响。

---

## 1. 目标与非目标

### 1.1 目标

- 提供一路独立于串口屏的调试日志，PC 端用 USB-TTL 921600 8N1 直接看。
- 每次有效测量输出一行：频率、|Y|、φY、G、B、H、档位、Ui/Uo。
- 不改变 UART4 屏幕协议（t0~t10、b0 按键）。
- 不阻塞主循环、不影响 ADC 窗口、不增加 E12/E17。
- 代码组织可被 CubeMX 重新生成后保留：`.ioc` 管初始化，`USE/uart_debug.c` 管应用层。

### 1.2 非目标（本次不做）

- 不做 PC 下行命令（读/写标定值、导出/导入表格）。
- 不做 Flash 持久化（标定常数仍用编译期常量）。
- 不做 DMA 收发、不做二进制高速流。
- 不占用 UART4、不改变屏幕刷新周期。
- 不做 RS232/RS485 电平转换（只做 3.3 V TTL）。

---

## 2. 已确认输入

| 项目 | 取值 | 说明 |
|---|---|---|
| 用途 | **只输出调试日志** | 不做命令解析 |
| 实例 | **USART3** | APB1，60 MHz |
| 引脚 | **PB10 = TX（TX-only）** | AF7；PB11 不使用 |
| 波特率 | **921600 8N1** | 3.3 V TTL，无流控 |
| 日志内容 | **测量值** | f、|Y|、φY、G、B、H、档位、Ui/Uo |
| 持久化 | **不需要** | 不改 Flash |
| 代码组织 | **新增 `USE/uart_debug.c/.h`** | 中断 + 环形缓冲 |
| 速率 | 每有效结果一行，最小间隔 200 ms | 可改宏 |

---

## 3. 硬件与电气

- PB10 为 USART3_TX 的 AF7；3.3 V TTL，直接接 USB-TTL 的 RX/GND。
- 接线：USB-TTL 的 **RX → PB10**，**GND → GND**；PB11 不使用（TX-only，将来要命令再加）。
- 921600 的时钟：APB1 = 60 MHz，`USARTDIV = 60e6 / (16 × 921600) = 4.069`，16 倍过采样 + 小数分频，误差 <0.2%；建议上线前用示波器/逻辑分析仪看一位宽度。
- 若 USB-TTL 在 921600 下误码率高：把 `.ioc` 的 `USART3.BaudRate` 和代码里的 921600 一起改成 115200（只改两处），协议不变。
- **PB10 必须在板上引出**（见 §10 待确认）；若没有，备选：PA2/PA3（USART2），代码只改引脚/实例。

---

## 4. `.ioc` 改动（精确到键）

在现有 `9_30.ioc` 中追加/修改：

```text
Mcu.Pin18=PB10
Mcu.PinsNb=19

Mcu.IP12=USART3
Mcu.IPNb=13

PB10.Mode=Asynchronous
PB10.Signal=USART3_TX

USART3.BaudRate=921600
USART3.IPParameters=BaudRate

NVIC.USART3_IRQn=true\:5\:0\:false\:false\:true\:true\:true\:true
```

并在 `ProjectManager.functionlistsort` 末尾（`0-MX_CORTEX_M7_Init` 之前）追加：

```text
9-MX_USART3_UART_Init-USART3-false-HAL-true
```

说明：

- `NVIC.USART3_IRQn` 的优先级取 **5**：低于 UART4/ADC/DMA 的 0，避免日志中断插到采样窗口前面。
- `USART3` 的时钟源沿用 `RCC.USART234578Freq_Value = 60000000`（APB1），不需要改 RCC。
- CubeMX 重新生成后应检查：`MX_USART3_UART_Init()` 出现在 `main.c`、`USART3_IRQHandler()` 出现在 `stm32h7xx_it.c`、PB10 为 AF7。
- 本次不改 GPIO 标签（PB10 保持默认），避免和 CubeMX 的引脚命名冲突。

---

## 5. 代码结构

### 5.1 [Core/Inc/usart.h](Core/Inc/usart.h) / [Core/Src/usart.c](Core/Src/usart.c)

追加（CubeMX 生成风格）：

```c
extern UART_HandleTypeDef huart3;
void MX_USART3_UART_Init(void);
```

`MX_USART3_UART_Init()`：与 `MX_UART4_Init()` 同构，实例 USART3、921600、8N1、`UART_MODE_TX`（TX-only）。

`HAL_UART_MspInit()` 增加分支：

```c
else if (uartHandle->Instance == USART3)
{
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}
```

`HAL_UART_MspDeInit()` 增加对应的反初始化分支。

> 这些代码放在 CubeMX 生成区（USART3_MspInit USER CODE 区外），重新生成时由 `.ioc` 复现；如果先手工加、之后又用 CubeMX 生成，代码应一致。

### 5.2 [USE/uart_debug.h](USE/uart_debug.h)（新增）

```c
#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "main.h"
#include "measure.h"

#define UART_DEBUG_ENABLE            1u
#define UART_DEBUG_MIN_INTERVAL_MS   200u   /* 最小输出间隔，5 Hz */
#define UART_DEBUG_TX_BUFFER_SIZE    1024u

void UartDebug_Init(void);
void UartDebug_Write(const uint8_t *data, uint16_t length);
void UartDebug_Printf(const char *format, ...);
void UartDebug_LogMeasurement(const FRF_Point_t *point,
                              const ADC_Window_t *window,
                              uint8_t range_index);
void UartDebug_LogEvent(const char *event);
void UartDebug_OnError(void);

#endif
```

### 5.3 [USE/uart_debug.c](USE/uart_debug.c)（新增）

- **TX 环形缓冲**：`static uint8_t s_tx_buf[1024]; static volatile uint16_t s_head, s_tail;`
  - `UartDebug_Write()`：单生产者（主循环）写 `head`，中断读 `tail`；缓冲区满时**丢弃整行**并 `++s_drop_count`，不阻塞。
  - 空闲时调用 `HAL_UART_Transmit_IT(&huart3, ...)` 启动发送；`HAL_UART_TxCpltCallback()` 里判断 `huart->Instance == USART3`，继续发下一段。
- **格式化**：`UartDebug_Printf()` 用 `vsnprintf` 输出到 160 字节栈缓冲，**只用 `%d/%ld/%lu/%s`，不用 `%f`**（避免 microlib/printf 浮点选项的坑）。
- **换算**：日志用整数单位：
  - `Y_uS = (int32_t)(y_mag_siemens * 1e6f)`；
  - `P_mdeg = (int32_t)(y_phase_deg * 1000.0f)`；
  - `G_uS/B_uS = (int32_t)(... * 1e6f)`；
  - `H_dB_x100 = (int32_t)(h_magnitude_dB * 100.0f)`；
  - `H_deg_x100 = (int32_t)(h_phase_deg * 100.0f)`；
  - `Ui_rms_mV/Uo_rms_mV = (int32_t)(window->ui_rms * ADC_SAMPLER_LSB_TO_V * 1000.0f)`。
- **速率限制**：`UartDebug_LogMeasurement()` 内部 `HAL_GetTick()` 比较，小于 200 ms 直接返回；不排队。
- **无 RX**：不调用 `HAL_UART_Receive_IT`；PB11 不配置。
- **错误处理**：不定义 `HAL_UART_ErrorCallback`（`screen.c` 已有定义，重复定义会链接冲突）；USART3 的错误在 `screen.c` 的回调里加一个分支清除并重启 TX（见 5.6）。

### 5.4 [Core/Src/main.c](Core/Src/main.c)

- `MX_USART3_UART_Init()` 放在 `MX_UART4_Init()` 之后、`MX_ADC1_Init()` 之前（与 UART4 同组）。
- `UartDebug_Init()` 放在 `Screen_Init()` 之后、`CircuitID_Init()` 之前；启动时输出一行 banner：
  ```text
  BOOT 9_30 debug uart 921600
  ```
- 在 `Screen_SetResult(&frf_point)` 之后调用：
  ```c
  UartDebug_LogMeasurement(&frf_point, &window, Switch_Get());
  ```
  - 只在 `CircuitID_Compute()` 返回 1（有效结果）时调用；
  - `window` 是循环内已有的变量，`frf_point` 有效；
  - 不改变 `Screen_Poll()` 的顺序。

### 5.5 [Core/Src/stm32h7xx_it.c](Core/Src/stm32h7xx_it.c)

- 在文件顶部的 extern 区加 `extern UART_HandleTypeDef huart3;`
- 在 `UART4_IRQHandler()` 旁边加：
  ```c
  void USART3_IRQHandler(void)
  {
    HAL_UART_IRQHandler(&huart3);
  }
  ```
- 若 CubeMX 重新生成，这段会由 `.ioc` 的 `NVIC.USART3_IRQn` 自动生成；手工版保持同样形式。

### 5.6 [USE/screen.c](USE/screen.c)

`HAL_UART_ErrorCallback()` 是全局单一定义，不能重复。给它加一个 `else if` 分支：

```c
else if (huart->Instance == USART3)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_FEF | UART_CLEAR_PEF);
    UartDebug_OnError();   /* 复位 TX 状态，从环形缓冲继续发 */
}
```

这样 USART3 出错不会落到 UART4 的逻辑里，也不会产生重复符号。

`screen.c` 需要 `#include "uart_debug.h"`（或在文件顶部前置声明 `void UartDebug_OnError(void);`），否则调用处缺声明。

### 5.7 [MDK-ARM/9_30.uvprojx](MDK-ARM/9_30.uvprojx)

在 `Application/User/USE` 组里追加：

```xml
<File>
  <FileName>uart_debug.c</FileName>
  <FileType>1</FileType>
  <FilePath>../USE/uart_debug.c</FilePath>
</File>
```

`INCLUDEPATH` 已包含 `../USE`，不用改。

---

## 6. 日志格式

默认一行一条有效测量（CSV 风格，便于 PC 解析，也便于人读）：

```text
f=1000,Y_uS=1000,P_mdeg=-12300,G_uS=977,B_uS=-213,H_dB_x100=408,H_deg_x100=16770,R_ohm=1600,range=1,Ui_rms_mV=354,Uo_rms_mV=566
```

- `Y_uS`：|Y| 的微西门子（1 mS = 1000 µS）；
- `P_mdeg`：φY 的毫度（-12.3° → -12300）；
- `G_uS`、`B_uS`：电导/电纳的微西门子；
- `H_dB_x100`、`H_deg_x100`：H 的 dB×100、度×100；
- `R_ohm`：当前档的 R_eff（含 Ron）；
- `range`：0=150 Ω、1=1.5 kΩ、2=15 kΩ、3=150 kΩ；
- `Ui_rms_mV`、`Uo_rms_mV`：去直流后的交流有效值（mV）。

若 `CIR_DUT_TOPOLOGY == 0`（只算 H），日志自动退化为：

```text
f=1000,H_dB_x100=408,H_deg_x100=16770,R_ohm=1600,range=1,Ui_rms_mV=354,Uo_rms_mV=566
```

启动 banner：

```text
BOOT 9_30 debug uart 921600
```

可选事件行（默认只在上电/扫描失败时输出）：

```text
EVT SCAN_FAIL
EVT E12_WINDOW_TIMEOUT
```

---

## 7. 时序与中断

- USART3 优先级 **5**；UART4/ADC/DMA 仍为 0。日志中断不会抢占采样窗口。
- 发送 100 字节 @921600 ≈ 1.1 ms；5 Hz 下平均占空比 ≈0.55%，最坏一行 1.1 ms，对 100 ms 窗口无影响。
- 环形缓冲 1024 字节，可缓存约 8–10 行；缓冲满时丢弃整行并计数，不阻塞主循环。
- 主循环只做 `memcpy` + 返回；`HAL_UART_Transmit_IT()` 只在空闲时启动。
- `HAL_UART_TxCpltCallback()` 只处理 `USART3`，不碰 UART4。
- `screen.c` 的 `HAL_UART_RxCpltCallback()`/`HAL_UART_ErrorCallback()` 已有 `Instance == UART4` 判断，USART3 不会误入屏幕逻辑。

---

## 8. 构建

- 把 `uart_debug.c` 加入 `9_30.uvprojx` 后，源文件数从 38 变为 39。
- 编译：

```bash
bash .ccg-build/build.sh <outdir>
```

- 通过标准：`compiled: 39 files, failures: 0`，链接无重复符号（重点检查 `HAL_UART_TxCpltCallback`、`HAL_UART_ErrorCallback`）。

---

## 9. 测试计划

| 编号 | 测试 | 方法 | 通过标准 |
|---|---|---|---|
| T1 | 日志通路 | USB-TTL RX→PB10、GND；921600 8N1；上电 | 能看到 `BOOT` banner 和测量行 |
| T2 | 屏幕共存 | 观察 t0~t10、按 b0 | 屏幕刷新正常，b0 仍能触发重扫 |
| T3 | 速率 | 1 kHz 与 100 Hz 各跑 30 s | 1 kHz 约 5 行/s，100 Hz 约 1 行/s；无丢行 |
| T4 | 缓冲满 | 暂时把波特率降到 9600 或连续触发 | 丢行计数增加，主循环不卡死，无 E12 |
| T5 | 中断影响 | 对比开/关日志的 E12/E17 计数 | 无新增 E12/E17 |
| T6 | 误码 | 921600 连续 10 分钟 | 无乱码/丢字节；否则降 115200 |
| T7 | 回归 | 频率扫描、换档、屏幕错误码 | 与加串口前一致 |

---

## 10. 风险与待确认

| 风险/待确认 | 影响 | 处理 |
|---|---|---|
| **PB10 是否在板上引出** | 没引出就接不了 USB-TTL | 先核对焊盘；没有则换 PA2/PA3（USART2），代码只改实例/引脚 |
| 921600 下 USB-TTL 误码 | 日志乱码 | 降 115200；`.ioc` 和代码各改一处 |
| 日志速率/格式是否合适 | 终端刷屏或信息不足 | 默认 5 Hz、CSV；可改 `UART_DEBUG_MIN_INTERVAL_MS` 和格式 |
| RX 是否留 | 已确认只 TX | PB11 不配置；将来要命令再加 |
| CubeMX 重新生成覆盖 | `usart.c` 手工段被覆盖 | `.ioc` 已同步，重新生成会复现；`uart_debug.c` 在 USE 不受影响 |
| `HAL_UART_ErrorCallback` 重复定义 | 链接冲突 | 只在 `screen.c` 里加 USART3 分支，模块不再定义 |
| `%f` 不可用 | 日志编译/输出异常 | 全部用整数缩放单位，禁用 `%f` |
| 日志函数重入 | 缓冲错乱 | 单生产者（主循环）+ 单消费者（ISR），16 位索引原子；不在中断里调用 `UartDebug_LogMeasurement` |

**已确认**：只 TX、PB11 不配置；日志速率/格式按默认。**还需要你核对**：

1. PB10 是否真的引出？如果只有 PA2/PA3，告诉我。
2. 921600 的 USB-TTL 是否确定支持？不确定就降 115200，跑通再提速。

---

## 11. 任务拆分

| 阶段 | 内容 | 产出 | 预估 |
|---|---|---|---|
| M1 | 改 `.ioc`：PB10、USART3、NVIC、IP/Pins、functionlist | `.ioc` 可被 CubeMX 识别 | 0.5 h |
| M2 | `usart.c/h`：`MX_USART3_UART_Init` + MSP/DeInit | USART3 能收发 | 0.5 h |
| M3 | 新增 `USE/uart_debug.c/.h`：环形缓冲、格式化、日志行 | 编译通过 | 1–2 h |
| M4 | `main.c`、`stm32h7xx_it.c`、`screen.c` 接入 | 上电能出日志 | 0.5 h |
| M5 | `uvprojx` 加文件 + 编译 + T1–T7 | 测试记录 | 1 h |

---

## 12. 文件改动地图

| 文件 | 改动 | 状态 |
|---|---|---|
| [9_30.ioc](9_30.ioc) | PB10、USART3、NVIC、IP/Pins、functionlist | 已改 |
| [Core/Inc/usart.h](Core/Inc/usart.h) | `huart3`、`MX_USART3_UART_Init` 声明 | 已改 |
| [Core/Src/usart.c](Core/Src/usart.c) | USART3 初始化 + MSP/DeInit | 已改 |
| [Core/Src/main.c](Core/Src/main.c) | `MX_USART3_UART_Init`、`UartDebug_Init`、日志调用 | 已改 |
| [Core/Src/stm32h7xx_it.c](Core/Src/stm32h7xx_it.c) | `USART3_IRQHandler` | 已改 |
| [USE/screen.c](USE/screen.c) | `HAL_UART_ErrorCallback` 加 USART3 分支 | 已改 |
| [USE/uart_debug.c](USE/uart_debug.c) | 新增：环形缓冲、格式化、日志 | 新增 |
| [USE/uart_debug.h](USE/uart_debug.h) | 新增：API/宏 | 新增 |
| [MDK-ARM/9_30.uvprojx](MDK-ARM/9_30.uvprojx) | 加入 `uart_debug.c` | 待改 |

---

## 附录 A：日志示例

```text
BOOT 9_30 debug uart 921600
f=1000,Y_uS=1000,P_mdeg=-12300,G_uS=977,B_uS=-213,H_dB_x100=408,H_deg_x100=16770,R_ohm=1600,range=1,Ui_rms_mV=354,Uo_rms_mV=566
f=1000,Y_uS=1002,P_mdeg=-12280,G_uS=978,B_uS=-213,H_dB_x100=409,H_deg_x100=16772,R_ohm=1600,range=1,Ui_rms_mV=354,Uo_rms_mV=567
```

## 附录 B：USART3 初始化骨架

```c
UART_HandleTypeDef huart3;

void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}
```
