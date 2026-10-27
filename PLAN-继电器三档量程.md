# PLAN：继电器三档量程（上电默认 1.5 kΩ）

- 状态：**v1.0，已实施并编译通过（39 files, 0 errors）**；本文描述当前实现。
- 日期：2026-10-27
- 工程：9_30（STM32H750XBH6）
- 关联：`USE/measure.c/.h`、`USE/screen.c/.h`、`USE/uart_debug.c/.h`、`Core/Src/main.c`、`Core/Src/usart.c`、`9_30.ioc`、`MDK-ARM/9_30.uvprojx`、`README.md`
- 一句话：**删除 74HC4052 模拟开关方案，改用 3 路独立继电器（高电平闭合）；三档 150 Ω / 1.5 kΩ / 15 kΩ；本版不做自动选档，只在上电时闭合 1.5 kΩ 档，固件固定按 1500 Ω 计算；运行期手动切档由 `PLAN-双击切档.md`（b0 双击）实现。**

---

## 0. 已确认决策

| 项目 | 决策 |
|---|---|
| 量程数量 | **3 档** |
| 三档电阻 | **150 Ω / 1.5 kΩ / 15 kΩ** |
| 继电器数量 | **3 路独立** |
| 继电器极性 | **高电平闭合（使能），低电平/GND 断开** |
| 继电器驱动 | **继电器模块，GPIO 直连模块输入**（模块自带三极管/续流） |
| 控制引脚 | **PE2 / PE3 / PE4** |
| 默认档位 | **1.5 kΩ（K2）** |
| 本版是否切档 | **完全不切**：无自动选档、无手动选档 API、无串口/屏幕命令 |
| 旧 `analog_switch` | **从 Keil 工程移除并删除 `USE/analog_switch.c/.h`** |

---

## 1. 目标与非目标

### 1.1 目标

- 用 3 路继电器替换 4052；继电器高电平闭合、低电平断开。
- 三档硬件到位：K1=150 Ω、K2=1.5 kΩ、K3=15 kΩ。
- 上电后只闭合 K2（1.5 kΩ），K1/K3 保持断开。
- 固件固定按 1500 Ω 计算 `H/Z/Y`，与当前 `s_calib`（Rf=1500）和 `CIR_FEEDBACK_RESISTOR_OHM=1500` 一致。
- 删除 4052/模拟开关相关代码、引脚标签和构建项。

### 1.2 非目标（本版明确不做）

- **不做自动量程**：不调用 `Switch_Recommend()` / `Switch_Update()`，不根据 Uo 换档。
- **不做自动选档**（不按 `Uo` 幅值/削波自动换档）；运行期手动切档由 `PLAN-双击切档.md`（b0 双击）实现，不在本计划范围内。
- 不改测量算法、DFT、ADC 时序、串口屏协议。
- 不改 `s_calib` 的 1 kHz/10 kHz 标定值（它们是在 Rf=1500 下测的，正好匹配默认档）。
- 不做三档的逐档标定流程（后续要切档时再做）。
- 不改 USART3 调试口（仍是 PB10、921600、TX-only）。

> 本版只需要**三路继电器硬件 + 上电默认 1.5 kΩ**。K1/K3 硬件接好但固件不驱动闭合；将来要做切档时再扩展。

---

## 2. 硬件方案

### 2.1 档位与继电器映射

| 通道 | GPIO | 电阻 | 上电状态 | 说明 |
|---|---|---|---|---|
| K1 | **PE2** | 150 Ω | **断开（低）** | 本版不用 |
| K2 | **PE3** | 1.5 kΩ | **闭合（高）** | 默认档 |
| K3 | **PE4** | 15 kΩ | **断开（低）** | 本版不用 |

- 高电平 = 继电器线圈得电 = 触点闭合；低电平/GND = 断开。
- 三路**独立**，不是 4052 的编码选择；任意时刻最多闭合一路。
- 本版上电流程：三路先全部拉低 → 等待释放 → 只把 PE3 拉高（K2）→ 等待吸合。

### 2.2 触点接线

每路：`Uo → 继电器触点 → 反馈电阻 → 虚地（1.65 V）`，三路并联在 `Uo` 与虚地之间：

```text
Uo ── K1 触点 ── 150 Ω ──┐
   ├─ K2 触点 ── 1.5 kΩ ──┼── 虚地 1.65 V
   └─ K3 触点 ── 15 kΩ  ──┘
```

- 触点只承受 `Uo` 的小信号（≤1.5 V、≤5 mA），普通信号继电器即可。
- 继电器接触电阻通常 0.05–0.5 Ω，相对 150 Ω 可忽略；本版 `R_eff` 直接按标称值 1500 Ω。
- 如果实际接线是“电阻在 Uo 侧、继电器在虚地侧”，电气等价，按板上实际走线即可；软件只关心最终等效 `R`。

### 2.3 继电器模块注意事项

- 模块输入为 3.3 V 高有效；GPIO 只驱动模块的逻辑输入，**不要直接驱动裸线圈**。
- 模块应自带三极管/MOS 驱动和续流二极管；若没有，需外加，且 GPIO 不能直接接线圈。
- 建议在 PE2/PE3/PE4 到 GND 各加 **10 kΩ 下拉**，保证复位/上电期间模块输入为低、三路断开。
- 上电时三路都断开，TIA 处于开环，`Uo` 可能撞轨；`Relay_Init()` 要尽早闭合 K2，且在 `ADC_Sampler_Init()` 之前完成。
- 继电器吸合/释放有抖动，默认等待 **10 ms**（`RELAY_SETTLE_MS`）；释放等待也取 10 ms（`RELAY_RELEASE_MS`）。
- 若模块是低有效，必须改硬件/加反相，或在 `Relay_Init()` 里反转电平；本计划按**高有效**实现，实施前用万用表确认模块输入极性。

### 2.4 三档覆盖（只作参考，本版只用 1.5 k）

源 1 Vpp（Ui 峰值 0.5 V）、Uo 0.15–1.5 V：

| 档 | R | 覆盖 \|Y\| | 10 mS 时 Uo |
|---|---|---|---|
| K1 | 150 Ω | 1.2–12 mS | 1.25 V |
| K2 | 1.5 kΩ | 0.188–1.88 mS | 削顶 |
| K3 | 15 kΩ | 19.9–198.7 µS | — |

本版上电默认 K2，只覆盖约 0.188–1.88 mS；这是用户当前要求，不是最终量程能力。

---

## 3. 引脚与 `.ioc`

### 3.1 目标引脚

| 引脚 | 标签 | 配置 | 初始 |
|---|---|---|---|
| PE2 | `RELAY_K1` | GPIO_Output，推挽，低速，无上下拉 | 低 |
| PE3 | `RELAY_K2` | GPIO_Output，推挽，低速，无上下拉 | 低（`Relay_Init` 后拉高） |
| PE4 | `RELAY_K3` | GPIO_Output，推挽，低速，无上下拉 | 低 |

- USART3 仍为 **PB10 TX**（921600，TX-only），不要动。
- PE2/PE3 原来标的是 `SW_A0`/`SW_A1`，改成 `RELAY_K1`/`RELAY_K2`；PE4 新增 `RELAY_K3`。
- 若 PE4 在板上不可用，换一个空闲 GPIO，并同步 `.ioc` 和 `relay_range.h` 的宏；优先选非 ADC 脚。

### 3.2 `.ioc` 具体改动

当前相关项（以仓库现状为准）：

```text
Mcu.Pin16=PE2
Mcu.Pin17=PE3
Mcu.Pin18=PB10
Mcu.PinsNb=19

PE2.GPIOParameters=PinState,GPIO_Label
PE2.GPIO_Label=SW_A0
...
PE3.GPIO_Label=SW_A1
...
```

改成：

```text
Mcu.Pin16=PE2
Mcu.Pin17=PE3
Mcu.Pin18=PB10
Mcu.Pin19=PE4
Mcu.PinsNb=20

PE2.GPIOParameters=PinState,GPIO_Label
PE2.GPIO_Label=RELAY_K1
PE2.Locked=true
PE2.PinState=GPIO_PIN_RESET
PE2.Signal=GPIO_Output

PE3.GPIOParameters=PinState,GPIO_Label
PE3.GPIO_Label=RELAY_K2
PE3.Locked=true
PE3.PinState=GPIO_PIN_RESET
PE3.Signal=GPIO_Output

PE4.GPIOParameters=PinState,GPIO_Label
PE4.GPIO_Label=RELAY_K3
PE4.Locked=true
PE4.PinState=GPIO_PIN_RESET
PE4.Signal=GPIO_Output
```

- 不要改 `Mcu.IP`（GPIO 不在 IP 列表）。
- 不要改 `ProjectManager.functionlistsort`。
- `MX_GPIO_Init()`（CubeMX 生成，引脚宏 `RELAY_K1_Pin` 等定义在 `main.h`）配置 PE2/PE3/PE4 为推挽输出并写低；`Relay_Init()` 只做上电时序，**不再重复配置 GPIO**。

---

## 4. 代码改造

### 4.1 删除 `analog_switch`

1. 删除文件：
   - `USE/analog_switch.c`
   - `USE/analog_switch.h`
2. 从 `MDK-ARM/9_30.uvprojx` 的 `Application/User/USE` 组删除 `analog_switch.c` 的 `<File>` 块。
3. 全仓搜索并清掉引用：
   - `analog_switch.h`
   - `Switch_Init`、`Switch_Set`、`Switch_Get`、`Switch_GetOhm`
   - `Switch_Recommend`、`Switch_Update`
   - `SW_ENABLE`、`SW_UON` 等宏
4. `README.md` 中 4052/模拟开关/自动量程的描述改为继电器三档/固定 1.5 kΩ；`PLAN-反馈电阻自动切换.md` 已删除，不再保留。

### 4.2 新增 `USE/relay_range.c/.h`

#### 4.2.1 头文件

```c
#ifndef RELAY_RANGE_H
#define RELAY_RANGE_H

#include "main.h"

/* 三路独立继电器，高电平闭合，低电平断开。
 * 本版上电只闭合默认档 K2（1.5 kΩ）；运行期手动切档见 PLAN-双击切档.md。
 */

#define RELAY_COUNT          3u
#define RELAY_SETTLE_MS      10u   /* 吸合建立 */
#define RELAY_RELEASE_MS     10u   /* 全部断开后的释放等待 */
#define RELAY_DEFAULT_INDEX  1u    /* K2 = 1.5 kΩ */

/* 控制脚引脚宏由 main.h 提供（RELAY_K1_Pin/RELAY_K1_GPIO_Port 等，
 * 与 9_30.ioc 的 GPIO_Label=RELAY_K1/K2/K3 对应，CubeMX 生成）。
 * 本模块不重复定义，避免双份命名。 */

void Relay_Init(void);

/* 当前档位（0=150 Ω、1=1.5 kΩ、2=15 kΩ；上电为 RELAY_DEFAULT_INDEX，切档后随之变化）。 */
uint8_t Relay_Get(void);

/* 当前档的反馈电阻（Ω）。上电为 1500.0f，切档后随之变化。 */
float Relay_GetOhm(void);

#endif
```

**运行期切档接口** `Relay_Set()` / `Relay_Next()` 已由 `PLAN-双击切档.md` 加入；本计划不包含自动量程接口（`Relay_Auto()` 等）。

#### 4.2.2 实现

```c
#include "relay_range.h"

static const float s_ohm[RELAY_COUNT] = {150.0f, 1500.0f, 15000.0f};
static uint8_t s_index = RELAY_DEFAULT_INDEX;

static void Relay_Write(uint8_t index, GPIO_PinState state)
{
    switch (index)
    {
        case 0u: HAL_GPIO_WritePin(RELAY_K1_GPIO_Port, RELAY_K1_Pin, state); break;
        case 1u: HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, state); break;
        case 2u: HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, state); break;
        default: break;
    }
}

void Relay_Init(void)
{
    /* 引脚已由 MX_GPIO_Init()（CubeMX 生成，main.h 的 RELAY_Kx_Pin 宏）
     * 配置为推挽输出并写低；本函数只做上电时序。 */

    /* 先全部断开，等继电器释放。 */
    Relay_Write(0u, GPIO_PIN_RESET);
    Relay_Write(1u, GPIO_PIN_RESET);
    Relay_Write(2u, GPIO_PIN_RESET);
    HAL_Delay(RELAY_RELEASE_MS);

    /* 只闭合默认档 K2 = 1.5 kΩ。 */
    s_index = RELAY_DEFAULT_INDEX;
    Relay_Write(s_index, GPIO_PIN_SET);
    HAL_Delay(RELAY_SETTLE_MS);
}

uint8_t Relay_Get(void)
{
    return s_index;
}

float Relay_GetOhm(void)
{
    return s_ohm[s_index];
}
```

说明：

- 上电时序：三路全低 → 10 ms → K2 高 → 10 ms；不会出现两路同时闭合。
- 本版没有 `Relay_Set()`，所以 `s_index` 只会在 `Relay_Init()` 里被设为 1，之后不变。
- `Relay_GetOhm()` 返回 **1500.0f**，与 `CIR_FEEDBACK_RESISTOR_OHM` 和 `s_calib` 的 Rf=1500 对齐；继电器接触电阻（<1 Ω）忽略。
- 运行期切档（`Relay_Set()` / `Relay_Next()`，保持“先全断、再闭合目标”时序）已由 `PLAN-双击切档.md` 实现；本计划不再重复。

### 4.3 `Core/Src/main.c`

1. 替换 include：
   - 删 `#include "analog_switch.h"`
   - 加 `#include "relay_range.h"`
2. 初始化：
   ```c
   Relay_Init();
   CircuitID_SetFeedbackResistor(Relay_GetOhm());
   ```
   位置仍在 `CircuitID_Init()` 之后、`ADC_Sampler_Init()` 之前（越早闭合 K2 越好）。
3. 删除自动量程相关：
   - `range_ready` 变量；
   - 主循环里 `Switch_Recommend()` / `Switch_Update()` / `Switch_Set()` 整段；
   - `Window_UoPeakV()` 函数和原型（只被量程逻辑使用；删除后避免 unused 警告）。
   - b0 分支里的 `range_ready = 0u;`。
   - `ADC_Sampler_Restart()` 在量程换档处的调用（函数本身保留在 measure，不删）。
4. 日志：
   - `UartDebug_LogMeasurement(&frf_point, &window, Relay_Get());`
5. 结果：主循环变成“等窗口 → `CircuitID_Compute` → 显示/日志”，不再有量程分支。

### 4.4 `USE/measure.c/.h`（只改注释，不改逻辑）

- `measure.h` 里 `CIR_FEEDBACK_RESISTOR_OHM` 注释中的 “analog_switch 模块” 改为 “relay_range 模块”。
- `CIR_FEEDBACK_RESISTOR_OHM` 保持 **1500.0f**。
- `CircuitID_SetFeedbackResistor()` / `ADC_Sampler_Restart()` 保留；本版 `main` 仍调用前者，不再调用后者。
- `s_calib[]` 保持 1 kHz/10 kHz 的现有值（Rf=1500，正好匹配默认档），不要改。

### 4.5 `USE/uart_debug.c/.h`

- `UartDebug_LogMeasurement()` 的 `range_index` 仍由调用方传入；本版传 `Relay_Get()`，恒为 **1**。
- 更新日志注释/README 的 range 映射：**0=150 Ω、1=1.5 kΩ、2=15 kΩ**。
- 日志中 `R_ohm` 将显示 **1500**，`range=1`，与默认档一致。

### 4.6 `MDK-ARM/9_30.uvprojx`

- 删除 `analog_switch.c` 的 `<File>`。
- 在 `Application/User/USE` 组加入：
  ```xml
  <File>
    <FileName>relay_range.c</FileName>
    <FileType>1</FileType>
    <FilePath>../USE/relay_range.c</FilePath>
  </File>
  ```
- 源文件数保持 **39**（删 1 加 1）。

### 4.7 `README.md`

按继电器方案更新：

- 删除 4052/模拟开关/四档/自动量程/PE2-PE3 A/B 编码等描述；
- 改为“三路独立继电器、高电平闭合、PE2/PE3/PE4、K1=150/K2=1.5k/K3=15k”；
- 明确当前固件**上电默认 1.5 kΩ**，不做自动量程；手动切档由 `PLAN-双击切档.md` 实现；
- 更新日志 range 映射、引脚表、错误码说明中与 4052 相关的部分；
- `PLAN-反馈电阻自动切换.md` 已删除（历史方案不再保留）。

---

## 5. 启动时序

```text
HAL_Init / 时钟 / MX_GPIO_Init     （PE2/PE3/PE4 配置为推挽输出并写低，见 main.h/gpio.c）
  → Relay_Init()
        三路全低（断开）
        HAL_Delay(10)                 继电器释放
        PE3 高（K2 = 1.5 kΩ）
        HAL_Delay(10)                 吸合建立
  → CircuitID_SetFeedbackResistor(1500.0f)
  → ADC_Sampler_Init()
  → ...
```

- 三路全低到 K2 高之间，TIA 开环、`Uo` 可能撞轨；此时 ADC 还没启动，不会采到无效窗口。
- 如果模块上电默认吸合或输入浮空，必须加 10 kΩ 下拉/确认模块默认状态；否则 `Relay_Init` 前的状态不可控。

---

## 6. 构建

```bash
bash .ccg-build/build.sh <outdir>
```

通过标准：

```text
compiled: 39 files, failures: 0
RESULT: BUILD OK (0 error)
```

同时检查：

- 全仓 `grep` 不到 `analog_switch` / `Switch_` / `SW_ENABLE`；
- `main.c` 不再有 `Switch_Recommend` / `Switch_Update` / `range_ready`；
- `uvprojx` 中只有 `relay_range.c`，没有 `analog_switch.c`。

---

## 7. 测试与验收

| 编号 | 测试 | 方法 | 通过标准 |
|---|---|---|---|
| R1 | 上电默认 | 万用表测 PE2/PE3/PE4 | **0 V / 3.3 V / 0 V**（K1 断、K2 合、K3 断） |
| R2 | 触点通断 | 断电/上电后用万用表测 Uo 到虚地 | K2 闭合时约 **1.5 kΩ**；K1/K3 断开 |
| R3 | 上电默认档 R | 读 `CircuitID_GetFeedbackResistor()` 或日志 | **1500 Ω**（上电默认档） |
| R4 | 已知 DUT | 接 1 kΩ 0.1% 电阻 | `\|Y\|` 约 **1 mS**（在标定误差内） |
| R5 | 无自动切档 | 观察 30 s（不按键） | 不自动换档；双击 b0 才换档 |
| R6 | 上电重复性 | 断电/上电 10 次 | 每次都是 K2 闭合，无两路同时闭合 |
| R7 | 构建/静态检查 | 编译 + grep | 39 files、0 error；无 `Switch_*`/`analog_switch` 引用 |

验收条件：

- 三路继电器硬件接好，K1=150 Ω、K2=1.5 kΩ、K3=15 kΩ；
- 上电只闭合 K2，`R_eff` 固定 1500 Ω；
- 无自动选档、无运行期切档、无串口/屏幕切档入口；
- 删除 `analog_switch` 文件与构建项；
- 编译通过，屏幕/USART3 日志正常。

---

## 8. 风险与待确认

| 风险/待确认 | 影响 | 处理 |
|---|---|---|
| **PE4 是否可用/已引出** | 接不了 K3 | 核对焊盘；没有则换空闲 GPIO，同步 `.ioc` 和 `relay_range.h` 宏 |
| 继电器模块实际是低有效 | 上电逻辑反了 | 用万用表确认模块输入；低有效则改硬件反相或在 `Relay_Write()` 反转 |
| 继电器模块输入浮空/上电吸合 | 上电瞬间多路闭合或默认档错误 | PE2/PE3/PE4 加 10 kΩ 下拉；确认模块默认状态 |
| 上电三路全断 → TIA 开环 | `Uo` 撞轨 | `Relay_Init()` 在 ADC 启动前完成；10 ms 释放 + 10 ms 吸合 |
| 触点抖动/接触电阻 | 测量不稳定 | 10 ms 建立；接触电阻 <1 Ω，本版忽略；后续切档时按档标定 `R_eff` |
| 删除 `analog_switch` 遗留引用 | 编译失败 | 全仓 grep + 编译检查 |
| `.ioc` PE2/PE3 旧标签 `SW_A0/SW_A1` | CubeMX 生成名称不符 | 改标签为 `RELAY_K1/K2`，PE4 新增 `RELAY_K3` |
| 本版不自动切档 | 默认 1.5 kΩ，b0 双击手动切档 | 自动量程由后续计划实现；手动切档见 PLAN-双击切档.md |

---

## 9. 任务清单（给实施 agent）

1. **硬件核对**：PE2/PE3/PE4 是否引出；继电器模块高有效；三路触点与电阻对应；加 10 kΩ 下拉。
2. **改 `.ioc`**：PE2/PE3 标签改为 `RELAY_K1/K2`；新增 PE4 `RELAY_K3`；`PinsNb=20`；三脚 GPIO_Output 初始低。
3. **删除旧模块**：`git rm USE/analog_switch.c USE/analog_switch.h`；从 `uvprojx` 删除 `analog_switch.c`。
4. **新增 `USE/relay_range.c/.h`**：按 §4.2 实现 `Relay_Init/Get/GetOhm`；切档接口见 `PLAN-双击切档.md`。
5. **改 `main.c`**：替换 include/初始化；删除 `Switch_*` 和量程分支、`range_ready`、`Window_UoPeakV`；日志传 `Relay_Get()`。
6. **改 `measure.h` 注释**：`analog_switch` → `relay_range`；不改编译逻辑。
7. **改 `uvprojx`**：加入 `relay_range.c`；保持 39 个源文件。
8. **改 `README.md`**：按继电器三档/固定 1.5 kΩ 更新；删除 4052/自动量程描述。
9. **编译**：39 files、0 error。
10. **测试**：R1–R7；重点万用表确认 PE 电平和 K2 触点。

---

## 10. 文件改动地图

| 文件 | 改动 |
|---|---|
| `USE/analog_switch.c/.h` | **删除** |
| `USE/relay_range.c/.h` | **新增**：三路继电器、默认 K2、固定 1500 Ω |
| `Core/Src/main.c` | `Relay_Init()`；删除 `Switch_*` 和量程分支；日志传 `Relay_Get()` |
| `USE/measure.h` | 注释 `analog_switch` → `relay_range`；`CIR_FEEDBACK_RESISTOR_OHM` 保持 1500 |
| `USE/uart_debug.c/.h` | 注释 range 映射 0/1/2；调用方传 `Relay_Get()` |
| `9_30.ioc` | PE2/PE3 标签 `RELAY_K1/K2`；新增 PE4 `RELAY_K3`；`PinsNb=20` |
| `MDK-ARM/9_30.uvprojx` | 删 `analog_switch.c`，加 `relay_range.c` |
| `README.md` | 继电器三档、固定 1.5 kΩ、无自动量程；删除 4052 描述 |
| `PLAN-反馈电阻自动切换.md` | 已删除（历史方案不再保留） |

---

## 11. 明确不做（防止实施 agent 加戏）

- 不实现自动量程：不根据 `Uo` 幅值/削波自动换档（`Switch_Recommend()` 那套不复活）。
- 运行期手动切档（`Relay_Set()` / `Relay_Next()`，b0 双击触发）由 `PLAN-双击切档.md` 实现，不在本计划范围。
- 不加 USART3 下行命令、不加 `RELAY_DEFAULT_RANGE` 运行期变量。
- 不在本版做三档 `R_eff` 标定；`s_calib` 保持 1k/10k 的 Rf=1500 标定值。
- 不改 ADC/DFT/屏幕协议/USART3 波特率。
