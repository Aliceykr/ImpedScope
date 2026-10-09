# 串口屏显示计划：计算结果、心跳、运行阶段与错误

本文档供实现者（另一个 agent）使用。目标是把计算结果、心跳和工程中的所有错误判断都显示在串口屏的 5 个文本框中。

## 1. 屏幕控件

共 5 个文本框，名称必须与屏幕页面中的对象名完全一致：

| 控件 | 显示内容 | 示例 |
|---|---|---|
| `t0` | 频率（Hz） | `freq:1000` |
| `t1` | 阻抗幅值 \|Z\|（Ω） | `zmag:1234.56` |
| `t2` | 阻抗相位（°） | `zph:-12.34` |
| `t3` | 心跳计数 | `beat:1234` |
| `t4` | 运行阶段或错误 | `stage:LOOP`、`stage:E06 SPI_FRAME x12` |

每个文本框都必须带标签前缀（`freq:`、`zmag:`、`zph:`、`beat:`、`stage:`），不能只显示数字。

旧的 `t0` 测试框和旧的 `t_freq`、`t_zmag`、`t_zph`、`t1`、`t2` 命名全部作废，实现时需要统一改为上表名称。

## 2. 编码

- 串口屏使用 UTF-8 编码。当前所有显示文字为 ASCII，与 UTF-8 兼容，直接发送字节即可。
- 如需显示中文，源文件必须保存为 UTF-8（无 BOM），并确认屏幕字库包含对应字形。本计划默认不使用中文。
- 指令结尾固定为 `0xFF 0xFF 0xFF`，显示文本中不得出现 `0xFF` 字节。

## 3. 通信约定

- UART4，9600 8N1，PC10 TX / PC11 RX，沿用 CubeMX 生成的 `huart4`。
- 指令格式：`控件名.txt="内容"` + `0xFF 0xFF 0xFF`。
- 所有指令放入同一个静态发送缓冲区，每个刷新周期用一次 `HAL_UART_Transmit_IT()` 发送。5 条指令约 175 字节，缓冲区建议增大到 256 字节。
- 不新增 `.c` 或 `.h` 文件，`MDK-ARM/9_30.uvprojx` 不需要改动。
- 代码仍放在 `USE/screen.c` 和 `USE/screen.h` 中，不修改 CubeMX 生成的代码。

## 4. 刷新逻辑

- `Screen_Poll()` 每 500 ms 刷新一次全部 5 个文本框。
- **即使没有有效频点，也必须刷新全部文本框。** 没有结果时，`t0`~`t2` 显示 `freq:-`、`zmag:-`、`zph:-`。当前代码在没有结果时直接返回，必须改掉。
- `beat` 在每个刷新周期递增一次。递增动作放在 UART 状态检查之前，UART 忙时心跳仍然计数。
- 如果 UART 仍在发送上一条指令（`huart4.gState != HAL_UART_STATE_READY`），本周期跳过发送，不阻塞。

## 5. 运行阶段（`t4`）

阶段码沿用以下文本，在 `main.c` 的 `USER CODE` 区域调用 `Screen_SetStage()`：

| 文本 | 设置位置 |
|---|---|
| `BOOT` | `main()` 中 `MPU_Config()` 之前 |
| `CLK` | `SystemClock_Config()` 和 `PeriphCommonClock_Config()` 之后（`USER CODE BEGIN SysInit`） |
| `PERIPH` | `USER CODE BEGIN 2` 开始处，即所有 `MX_*_Init()` 之后 |
| `MODULE` | `CircuitID_Init()`、`Screen_Init()` 之后 |
| `SPI` | `SPI_Receiver_Init()` 成功之后 |
| `LOOP` | `while (1)` 循环开始前 |

阶段设置不写入 CubeMX 生成的代码区，避免重新生成时丢失。

## 6. 错误显示规则

工程中的每个错误判断都要上报到 `t4`，规则如下：

1. **每个错误有唯一码**，格式 `Exx 名称`，如 `E06 SPI_FRAME`。
2. **非致命错误**：`t4` 显示最近一次事件，格式 `stage:Exx 名称 xN`，其中 `N` 是该码自上电以来的累计次数（16 位，溢出后回绕）。新的阶段或错误会覆盖之前的显示。
3. **致命错误**（`Error_Handler()` 触发）：`t4` 锁定显示，不被后续事件覆盖。
4. **中断中不能直接发送串口**。`HAL_SPI_RxCpltCallback` 等 DMA 回调中只记录错误码和计数（使用 `volatile` 变量），由主循环的 `Screen_Poll()` 统一显示。
5. 错误码和计数可以统一放在 `screen.c` 的一个数组中，一个 `Screen_ReportError(uint8_t code)` 接口供所有模块调用。

### 6.1 错误码列表

| 码 | 名称 | 位置 | 触发条件 | 类型 |
|---|---|---|---|---|
| E01 | CLK_OSC | `main.c` 约 175 行 | `HAL_RCC_OscConfig()` 失败 | 致命 |
| E02 | CLK_BUS | `main.c` 约 193 行 | `HAL_RCC_ClockConfig()` 失败 | 致命 |
| E03 | CLK_PER | `main.c` 约 211 行 | `HAL_RCCEx_PeriphCLKConfig()` 失败 | 致命 |
| E04 | HAL_INIT | `spi.c`、`usart.c`（CubeMX 生成） | `MX_SPI1_Init()`、`MX_UART4_Init()` 中的 `Error_Handler()` 调用 | 致命 |
| E05 | SPI_DMA | `main.c` 约 110 行 | `SPI_Receiver_Init()` 返回 0 | 致命 |
| E06 | SPI_FRAME | `spi_receiver.c` 约 51 行 | 帧头或帧尾不匹配（同步丢弃，属于正常现象，但需计数） | 非致命 |
| E07 | SPI_NAN | `spi_receiver.c` 约 62、70、78、86 行 | 帧内浮点字段为 NaN 或 Inf | 非致命 |
| E08 | SPI_LEN | `spi_receiver.c` 约 45 行 | `sizeof(float) != sizeof(uint32_t)` | 致命 |
| E09 | SPI_QFULL | `spi_receiver.c` 约 170 行 | 队列已满，丢弃最旧频点 | 非致命 |
| E10 | CIR_CFG | `circuit_id.c` 约 40 行 | 反馈电阻 R ≤ 0 或频率为 0 | 非致命 |
| E11 | CIR_NAN_IN | `circuit_id.c` 约 53 行 | 输入 Ui、Uo 为 NaN 或 Inf | 非致命 |
| E12 | CIR_UI_LOW | `circuit_id.c` 约 60 行 | \|Ui\| 小于 `CIR_INPUT_MIN_MAG` | 非致命 |
| E13 | CIR_UO_LOW | `circuit_id.c` 约 67 行 | \|Uo\| 小于 `CIR_OUTPUT_MIN_MAG` | 非致命 |
| E14 | CIR_H_NAN | `circuit_id.c` 约 80 行 | H 为非有限值 | 非致命 |
| E15 | CIR_OUT_NAN | `circuit_id.c` 约 94 行 | 频率、dB、相位输出为非有限值 | 非致命 |
| E16 | CIR_H_LOW | `circuit_id.c` 约 102 行 | \|H\| 过小，阻抗无法计算 | 非致命 |
| E17 | CIR_ZY_NAN | `circuit_id.c` 约 145 行 | Z 或 Y 结果为非有限值 | 非致命 |
| E18 | UART_TX | `screen.c` 约 53 行 | `HAL_UART_Transmit_IT()` 返回非 `HAL_OK` | 非致命 |
| E19 | UART_BUSY | `screen.c` 约 36 行 | 上一条指令未发送完，本周期跳过 | 非致命 |

注：行号为当前代码的大致位置，实现前需以实际代码为准。`E04` 的具体生成代码位置无法区分，显示时以 `t4` 中最近的阶段名作为参考。

### 6.2 致命错误处理

`Error_Handler()` 位于 `main.c` 的 `USER CODE` 区域，执行时已调用 `__disable_irq()`，因此有以下限制：

- `HAL_UART_Transmit_IT()` 不会工作，必须改用阻塞发送。
- SysTick 中断已被关闭，`HAL_GetTick()` 不再递增，不能使用有限超时。必须使用 `HAL_MAX_DELAY`。
- 若 UART4 仍处于发送状态，`HAL_UART_Transmit()` 会返回 `HAL_BUSY`，发送前需要先调用 `HAL_UART_Abort(&huart4)`。
- 若错误发生在 `MX_UART4_Init()` 之前，跳过发送。
- 发送内容为 `t4.txt="stage:Exx 名称"` 加 `0xFF 0xFF 0xFF`，然后进入死循环。

## 7. 验收

1. 上电后 `t4` 依次显示 `BOOT`、`CLK`、`PERIPH`、`MODULE`、`SPI`、`LOOP`，每一步都能看到。
2. `t4` 稳定在 `LOOP` 后，`t3` 每 0.5 s 增加 1。
3. 没有 FPGA 数据时，`t0`~`t2` 显示 `freq:-`、`zmag:-`、`zph:-`，`t3` 仍然增加。
4. 有有效频点时，`t0`~`t2` 显示正确数值，格式与标签前缀一致。
5. 人为让主循环卡住，`t3` 停止变化。
6. 发送错误的 SPI 数据（如 NaN 字段），`t4` 显示 `E07` 并且计数增加。
7. 在 `main.c` 中临时调用 `Error_Handler()`，`t4` 显示致命错误码并锁定，不被后续事件覆盖。
8. 在 Keil 中编译通过，并在硬件上逐项确认。

## 8. 注意事项

- 不改变 `USE/spi_receiver.*`、`USE/circuit_id.*` 的计算逻辑和对外接口。错误上报只增加调用，不改变返回值。
- 不改变 UART4 的 CubeMX 配置（波特率、引脚、中断）。
- 修改完成后更新 `README.md` 中关于串口屏的说明，包括 5 个文本框的名称和含义。
