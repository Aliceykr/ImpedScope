# PLAN：屏幕按钮双击切换继电器档位

> 目标读者：实施 agent（照着做，不要自由发挥）。
> 前置状态：继电器三档 + 固定 1.5 kΩ 已实现并编译通过（39 files / 0 error / Code=50538）。
> 本文只描述**双击切档**这一件事，不涉及自动量程。

---

## 0. 目标与非目标

### 0.1 目标

1. 保留现有 `b0` 按键的**单击 = 强制重扫频率**。
2. 新增 `b0` 的**双击 = 在三档反馈电阻之间循环切换**。
3. 切档后立即在串口屏 `t10` 上显示新档位（不等 500 ms 整帧刷新）。
4. 切档后 `R_eff` 立刻同步，当前不完整窗口丢弃并重开。

### 0.2 非目标（明确不做）

- 不做自动量程：不根据 `Uo` 幅值/削波自动换档（`Switch_Recommend()` 那套不复活）。
- 不做长按、三击、滑动等手势。
- 不把档位写进 Flash，不做掉电保持。
- 不改屏幕 HMI 工程（`.ioc` 之外的那个串口屏工程不在本仓库，文案要人工改，见 §9）。
- 不引入新的 RTOS/定时器中断，判定全部在主循环里完成。

---

## 1. 决定（D1/D2 已由用户确认，D3–D5 仍为默认值）

| # | 决定 | 取值 | 备注 |
|---|---|---|---|
| **D1** | 单击是否保留 | ✅ **已确认：保留**（重扫），判定窗口 **300 ms** | 单击会因此延迟 300 ms 才响应 |
| **D2** | 双击循环方向 | ✅ **已确认：1.5k → 15k → 150 → 1.5k**（`index = (index+1)%3`） | 即先切大电阻 |
| D3 | 切错档导致削顶 | **只报 E20**，不自动回退 | 想自动回退：在 `main.c` 测量失败分支里加一次 `Relay_Next()` 回退，并加防抖计数 |
| D4 | 复位后档位 | **回到 1.5 kΩ**（现状不变） | 想记忆：要加 Flash 写 + `Relay_Init()` 读回，明显超范围 |
| D5 | 屏幕刷新时机 | **立即补发一条 `t10`** | 想省事：删掉 `Screen_FlushRange()` 调用，等 500 ms 整帧 |
| D3 | 切错档导致削顶 | **只报 E20**，不自动回退 | 想自动回退：在 `main.c` 测量失败分支里加一次 `Relay_Next()` 回退，并加防抖计数 |
| D4 | 复位后档位 | **回到 1.5 kΩ**（现状不变） | 想记忆：要加 Flash 写 + `Relay_Init()` 读回，明显超范围 |
| D5 | 屏幕刷新时机 | **立即补发一条 `t10`** | 想省事：删掉 `Screen_FlushRange()` 调用，等 500 ms 整帧 |

其它细节（判定窗口 300 ms、锁定期 100 ms）直接按 §4.5 的常量走，不需要再问。

---

## 2. 现状事实（实施前必须核对，行号以当前 HEAD+工作区为准）

### 2.1 按键接收（`USE/screen.c`）

| 位置 | 内容 |
|---|---|
| `screen.c:66-72` | 按键状态：`s_rx_byte` / `s_rx_armed` / `s_rx_lockout_tick` / `s_ascii_key_match` / `s_ascii_key[]="010101"` / `s_freq_key_pending` |
| `screen.c:328-339` | `Screen_TakeRefreshKey()`：临界区读 `s_freq_key_pending` 并清零，返回 0/1 |
| `screen.c:341-388` | `HAL_UART_RxCpltCallback()`：认 `0x01` 或 ASCII `"010101"`；`screen.c:379-383` 做 100 ms 锁定期，置 `s_freq_key_pending=1` 并 `s_rx_lockout_tick = now+100` |
| `screen.c:390-406` | `HAL_UART_ErrorCallback()`：UART4 出错时清标志并重挂接收 |

**关键约束**：一次按下屏幕会连发 3 个 `0x01`，100 ms 锁定期把它们压成 1 个事件。
因此**双击两下的间隔必须 > 100 ms**，否则第二下会被当成同一次按下的剩余字节吞掉。

### 2.2 主循环（`Core/Src/main.c`）

| 位置 | 内容 |
|---|---|
| `main.c:263-278` | `if ((Screen_TakeRefreshKey() != 0u) \|\| (scan_fail_count >= 3u))` → `Screen_SetStage("SCAN")` → `FreqScan_Select()` |
| `main.c:280-296` | 取频率、`Freq_SourceSet()`、`ADC_Sampler_SetFrequency()` |
| `main.c:296-330` | 等窗口 → `CircuitID_Compute()` → `Screen_SetResult()` + `UartDebug_LogMeasurement(&frf_point, &window, Relay_Get())` |
| `main.c:219-220` | `Relay_Init(); CircuitID_SetFeedbackResistor(Relay_GetOhm());` |

**关键约束（整个方案的核心）**：主循环里 `ADC_Sampler_GetWindow()` 会阻塞等待一个完整 DFT 窗口——
1 kHz 时约 100 ms，100 Hz 时**约 1 s**，超时上限 1500 ms。

所以：

> **双击判定必须使用「中断里记录的时间戳」，不能用「主循环轮询时刻」的时间戳。**
> 否则在 100 Hz 档下，两下点击都发生在阻塞期间，等主循环醒来时时间差已远超 300 ms，
> 双击永远判不出来。

### 2.3 继电器（`USE/relay_range.*`）

| 位置 | 内容 |
|---|---|
| `relay_range.h:27-30` | `RELAY_COUNT 3u` / `RELAY_SETTLE_MS 10u` / `RELAY_RELEASE_MS 10u` / `RELAY_DEFAULT_INDEX 1u` |
| `relay_range.c:5-8` | `s_ohm[] = {150, 1500, 15000}` |
| `relay_range.c:10` | `static uint8_t s_index = RELAY_DEFAULT_INDEX;` |
| `relay_range.c:12-21` | `Relay_Write(index, state)` —— 唯一写引脚的地方 |
| `relay_range.c:23-38` | `Relay_Init()` —— 只做时序：三路全断 → 10 ms → 闭合 K2 → 10 ms |
| `relay_range.h:39-45` | 现有 API 只有 `Relay_Init` / `Relay_Get` / `Relay_GetOhm`，**没有 setter** |

### 2.4 屏幕显示

| 位置 | 内容 |
|---|---|
| `screen.c:481-490` | `Screen_RangeText()` 已按 `Relay_Get()` 映射 `150 / 1.5k / 15k` |
| `screen.c:615-618` | `Screen_BuildCommands()` 里 `t10` = `"R:" + Screen_RangeText()`，随整帧每 500 ms 发一次 |
| `screen.h:13` | `SCREEN_REFRESH_PERIOD_MS 500u` |

### 2.5 可复用的既有 API

| API | 位置 | 用途 |
|---|---|---|
| `CircuitID_SetFeedbackResistor(float)` | `measure.h:272` | 切档后同步 `R_eff` |
| `ADC_Sampler_Restart(void)` | `measure.h:202` | 丢弃当前窗口、重开一个 |
| `Screen_ClearResult(void)` | `screen.h:59` | 清掉用旧档算出的读数 |
| `Screen_ClearError(void)` | `screen.h:69` | 清掉上一档残留的错误码（如 E20） |
| `UartDebug_Printf(...)` | `uart_debug.h` | 可选：切档时打一行调试日志 |

---

## 3. 交互设计

```text
时间 ──────────────────────────────────────────────────────────►

按下①                                       300 ms 到点
  │                                             │
  ├────────── 判定窗口 SCREEN_DOUBLE_CLICK_MS ───┤
  │                                             │
  │  ① 期间又按了一下                           │
  │     → 判定「双击」→ 立即切档（新档位立即上屏）│
  │                                             │
  └── 期间没有第二下 ─────────────────────────────┘
        → 判定「单击」→ 重扫五档频率
```

- 双击：**第二下按下后、下一次主循环轮询时立刻执行**，不受 300 ms 影响。
- 单击：**按下后 300 ms 才执行**（等窗口过期）。手感会略钝，这是区分双击的必要代价。
- 极端情况：主循环阻塞 1 s 期间用户点了两下 → 两点时间戳差 <300 ms → 仍是双击，只是执行晚一点。

---

## 4. 实现

### 4.1 `USE/relay_range.h` —— 新增两个 API

在现有 `Relay_GetOhm()` 之后加：

```c
/* 切到指定档（0=150 Ω、1=1.5 kΩ、2=15 kΩ）。先断开当前档、等释放，
 * 再闭合目标档、等建立。返回 1=成功，0=参数越界（此时继电器状态不变）。
 * 重复切到当前档会直接返回 1，不做任何动作。 */
uint8_t Relay_Set(uint8_t index);

/* 切到下一档（1.5k → 15k → 150 → 1.5k 循环），返回切换后的档位。
 * 内部调用 Relay_Set()，会阻塞约 RELAY_RELEASE_MS + RELAY_SETTLE_MS。 */
uint8_t Relay_Next(void);
```

### 4.2 `USE/relay_range.c` —— 实现

```c
uint8_t Relay_Set(uint8_t index)
{
    if (index >= RELAY_COUNT)
    {
        return 0u;
    }
    if (index == s_index)
    {
        return 1u;              /* 已在目标档，不重复动作 */
    }

    /* 先断开当前档，等触点释放，再闭合目标档：任何时刻最多一路闭合。 */
    Relay_Write(s_index, GPIO_PIN_RESET);
    HAL_Delay(RELAY_RELEASE_MS);

    s_index = index;
    Relay_Write(s_index, GPIO_PIN_SET);
    HAL_Delay(RELAY_SETTLE_MS);

    return 1u;
}

uint8_t Relay_Next(void)
{
    (void)Relay_Set((uint8_t)((s_index + 1u) % RELAY_COUNT));

    return s_index;
}
```

⚠️ **坑（必须注意）**：`Relay_Init()` **不能**改成调用 `Relay_Set(RELAY_DEFAULT_INDEX)`。
因为 `s_index` 初值就是 `RELAY_DEFAULT_INDEX`，`Relay_Set()` 会命中 `index == s_index`
直接返回，结果是**一个继电器都不会闭合**，上电后 TIA 开环、`Uo` 撞轨。
`Relay_Init()` 保持现有写法（无条件写三路 RESET、再写 K2 SET），不要动。

### 4.3 `USE/screen.h` —— 新增常量、事件类型、两个函数

```c
/* 单击/双击判定窗口：按下后等这么久没有第二下，就按单击处理。
 * 必须大于 UART 按键的 100 ms 锁定期，否则第二下会被吞掉。 */
#define SCREEN_DOUBLE_CLICK_MS       300u

/* 按键事件：一次调用最多返回一个。 */
typedef enum
{
    SCREEN_KEY_NONE = 0u,
    SCREEN_KEY_SINGLE,      /* 单击：强制重扫频率 */
    SCREEN_KEY_DOUBLE       /* 双击：切换继电器档位 */
} Screen_KeyEvent_t;

/* 主循环每轮调用一次（必须每轮都调，否则单击永远出不来）。
 * 内部用中断记录的时间戳判定，不受主循环阻塞影响；读即消费。 */
Screen_KeyEvent_t Screen_PollKey(void);

/* 立即补发一条 t10（档位），不等 500 ms 整帧刷新。 */
void Screen_FlushRange(void);
```

同时**删除** `Screen_TakeRefreshKey()`（唯一调用点是 `main.c:265`），并顺手修掉
`screen.h:78` 和 `screen.h:88` 两条过期注释（它们还在说 "t10 显示 H 的相位"）。

### 4.4 `USE/screen.c` —— 状态机

#### (a) 状态变量（替换 `screen.c:72` 的 `s_freq_key_pending`）

```c
/* 按键点击队列：最多缓存 2 次点击的时间戳，由中断写入、主循环消费。
 * 主循环可能被 DFT 窗口阻塞 1 s，所以时间戳必须取自中断，不能取轮询时刻。 */
static volatile uint8_t  s_click_count;        /* 0..2 */
static volatile uint32_t s_click_first_tick;   /* 第一次按下的时刻 */
static volatile uint32_t s_click_last_tick;    /* 最近一次按下的时刻 */
```

`Screen_Init()` 里对应把 `s_freq_key_pending = 0u;`（`screen.c:102`）改成这三项清零。

#### (b) 中断回调（改 `screen.c:376-384`）

原：

```c
if (key_event != 0u)
{
    now = HAL_GetTick();
    if ((int32_t)(now - s_rx_lockout_tick) >= 0)
    {
        s_freq_key_pending = 1u;
        s_rx_lockout_tick = now + 100u;
    }
}
```

新（**只记时间戳，不做任何判定**）：

```c
if (key_event != 0u)
{
    now = HAL_GetTick();
    if ((int32_t)(now - s_rx_lockout_tick) >= 0)
    {
        if (s_click_count == 0u)
        {
            s_click_first_tick = now;
        }
        if (s_click_count < 2u)
        {
            ++s_click_count;
        }
        s_click_last_tick = now;
        s_rx_lockout_tick = now + 100u;     /* 100 ms 锁定期保持原样 */
    }
}
```

#### (c) 判定函数（替换 `screen.c:328-339` 的 `Screen_TakeRefreshKey`）

```c
Screen_KeyEvent_t Screen_PollKey(void)
{
    Screen_KeyEvent_t event = SCREEN_KEY_NONE;
    uint32_t primask;
    uint8_t  count;
    uint32_t first_tick;
    uint32_t last_tick;

    primask = __get_PRIMASK();
    __disable_irq();
    count      = s_click_count;
    first_tick = s_click_first_tick;
    last_tick  = s_click_last_tick;
    __set_PRIMASK(primask);

    if (count >= 2u)
    {
        if ((uint32_t)(last_tick - first_tick) <= SCREEN_DOUBLE_CLICK_MS)
        {
            /* 两下都在窗口内：双击，队列清空。 */
            Screen_ClickReset();
            event = SCREEN_KEY_DOUBLE;
        }
        else
        {
            /* 两下隔太久：第一下算单击，第二下留作新的待定单击。 */
            Screen_ClickKeepLast();
            event = SCREEN_KEY_SINGLE;
        }
    }
    else if (count == 1u)
    {
        if ((uint32_t)(HAL_GetTick() - last_tick) > SCREEN_DOUBLE_CLICK_MS)
        {
            /* 窗口过期，没有第二下：单击。 */
            Screen_ClickReset();
            event = SCREEN_KEY_SINGLE;
        }
    }

    return event;
}
```

两个私有小工具（放在 `Screen_PollKey` 之前）：

```c
static void Screen_ClickReset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_click_count      = 0u;
    s_click_first_tick = 0u;
    s_click_last_tick  = 0u;
    __set_PRIMASK(primask);
}

static void Screen_ClickKeepLast(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_click_count      = 1u;
    s_click_first_tick = s_click_last_tick;
    __set_PRIMASK(primask);
}
```

⚠️ 三击及以上：`s_click_count` 封顶 2，第三下在双击已被消费后才会被记录（因为 100 ms 锁定期），
实际表现为"双击切一档、再单击重扫"。不需要额外处理，但测试时不要连点三下。

#### (d) 立即刷新档位

```c
void Screen_FlushRange(void)
{
    uint16_t length = 0u;
    char value[SCREEN_VALUE_CHARS];

    (void)snprintf(value, sizeof(value), "R:%s", Screen_RangeText());
    length = Screen_AppendCommand(length, "t10", value);
    if (length == 0u)
    {
        return;
    }

    if (HAL_UART_Transmit(&huart4, s_tx_buffer, length, 10u) != HAL_OK)
    {
        Screen_ReportError(SCREEN_E10_UART_TX);
    }
    if (s_rx_armed == 0u)
    {
        Screen_RxStart();
    }
}
```

放在 `Screen_Poll()` 之后即可（`Screen_AppendCommand` / `Screen_RangeText` 已有前置声明，
见 `screen.c:82-84`）。

### 4.5 `Core/Src/main.c` —— 主循环分支

`main.c:265` 的分支整体替换为：

```c
    /* 按键：单击 = 强制重扫；双击 = 切换继电器档位。
     * 必须在每轮循环开头调用一次，单击靠"窗口超时"产生。 */
    key = Screen_PollKey();

    if (key == SCREEN_KEY_DOUBLE)
    {
        uint8_t next = Relay_Next();       /* 阻塞约 20 ms：先断后合 */

        CircuitID_SetFeedbackResistor(Relay_GetOhm());
        Screen_ClearResult();              /* 旧档算出的读数作废 */
        Screen_ClearError();               /* 上一档的 E20 之类残留作废 */
        Screen_FlushRange();               /* t10 立即显示新档位 */
        (void)ADC_Sampler_Restart();       /* 丢掉混了两档的窗口 */
        UartDebug_Printf("RANGE %u %ld\r\n", (unsigned)next, (long)Relay_GetOhm());
        continue;
    }

    if ((key == SCREEN_KEY_SINGLE) || (scan_fail_count >= 3u))
    {
        ...原有重扫分支不变...
    }
```

- 循环体内新增局部变量 `Screen_KeyEvent_t key;`（放在 `uint8_t got = 0u;` 那一组声明里）。
- **顺序不能变**：先 `Relay_Next()` 再 `SetFeedbackResistor()`；先 `Restart()` 再进下一轮。
- `UartDebug_Printf` 是可选项，`uart_debug` 不支持 `%f`，所以 `Relay_GetOhm()` 必须转 `long`。

---

## 5. 时序与常量

| 常量 | 值 | 位置 | 含义 |
|---|---|---|---|
| `SCREEN_DOUBLE_CLICK_MS` | 300 | `screen.h` | 判定窗口 |
| （硬编码） | 100 | `screen.c:382` | 按键去重锁定期，改动风险高，不动 |
| `RELAY_RELEASE_MS` | 10 | `relay_range.h` | 全部断开后的释放等待 |
| `RELAY_SETTLE_MS` | 10 | `relay_range.h` | 吸合建立等待 |
| `SCREEN_REFRESH_PERIOD_MS` | 500 | `screen.h` | 整帧刷新周期，与判定窗口无关 |

**有效双击间隔 = 100 ms ~ 300 ms**。手感说明写进 README。

留 200 ms 的可用余量（300 − 100 锁定期）：人手双击典型 150–250 ms，落在里面；
但比 350 ms 时更挑手速，验收时按 T2（200 ms）和 T4（500 ms）两头各测一次。
若实测发现双击成功率偏低，只需把 `SCREEN_DOUBLE_CLICK_MS` 调到 350–400，其它代码不用动。

时延预算（双击一次）：

```text
第二下按下
  → ≤1 次主循环轮询（最坏 1.5 s，典型 100 ms 内）
  → Relay_Next()        约 20 ms
  → t10 上屏            约 1 ms
```

即典型 100 ms 内看到档位变化；正在 100 Hz 测量时最坏要等当前窗口结束。

---

## 6. 边界与坑（逐条核对）

1. **主循环阻塞**：判定用 ISR 时间戳，绝不能用 `HAL_GetTick()` 在轮询处取时间（见 §2.2）。
2. **`Relay_Init()` 不能复用 `Relay_Set()`**（见 §4.2 的警告）。
3. **切档后必须 `ADC_Sampler_Restart()`**：否则当前窗口前半段是旧 `R_eff`、后半段是新 `R_eff`，
   DFT 算出来的 `H` 是两段的混合，结果看似"正常"其实全错，而且不会报错。
4. **切档后必须 `Screen_ClearResult()`**：否则屏幕还挂着旧档的读数，用户会以为没切。
5. **`s_index` 只在 `Relay_Set` / `Relay_Init` 里改**，其它地方只读。
6. **双击窗口 < 锁定期会失效**：`SCREEN_DOUBLE_CLICK_MS` 一旦改到 ≤100，第二下永远被吞。
7. **UART4 发送冲突**：`Screen_FlushRange()` 与 `Screen_Poll()` 都在主循环上下文，
   `HAL_UART_Transmit` 是阻塞的，不会重入；不要把它挪进中断。
8. **`Screen_FlushRange()` 不递增 `s_beat`、不碰 `s_last_tick`**，整帧刷新节奏保持不变。
9. **切档时如果正好在 `FreqScan_Select()` 里**（单击后重扫期间）：不会发生——同一轮只处理一个事件。
10. **`screen.h` 的旧注释**：`screen.h:78`（"t10 不参与整帧刷新"）和 `screen.h:88`
    （"t10 现在显示 H 的相位"）都已过期，顺手改成"t10 显示反馈电阻档位"。

---

## 7. 测试

| # | 用例 | 预期 |
|---|---|---|
| T1 | 单击 | 300 ms 后进入 `SCAN`，重新扫频；`t10` 不变 |
| T2 | 双击（间隔 200 ms） | 档位 1.5k → 15k，`t10` 立即变 `R:15k`，UART 日志 `R_ohm` 变 15000 |
| T3 | 连续双击三次 | 1.5k → 15k → 150 → 1.5k，循环正确 |
| T4 | 间隔 500 ms 的两下 | 判定为两次单击（扫频两次），档位不变 |
| T5 | 在 100 Hz 档下双击 | 仍能识别为双击（验证 ISR 时间戳方案） |
| T6 | 双击后立刻测量 | 结果用新 `R_eff`，无 E20 误报；1 kΩ DUT 在 1.5k 档应约 1 mS |
| T7 | 切到不适配的档 | 只报 E20/E05，不自动回退；再双击可切回 |
| T8 | 复位 | 回到 1.5 kΩ（`R:1.5k`） |
| T9 | 屏幕掉线 | `Screen_FlushRange()` 发失败只报 E10，不影响主循环 |

上板还要看：切档时听得到继电器动作声；万用表确认任意时刻只有一路闭合。

---

## 8. 文件改动地图

| 文件 | 改动 |
|---|---|
| `USE/relay_range.h` | 新增 `Relay_Set()` / `Relay_Next()` 声明 |
| `USE/relay_range.c` | 实现这两个函数（约 25 行） |
| `USE/screen.h` | 新增 `SCREEN_DOUBLE_CLICK_MS`、`Screen_KeyEvent_t`、`Screen_PollKey()`、`Screen_FlushRange()`；删除 `Screen_TakeRefreshKey()`；修两条过期注释 |
| `USE/screen.c` | 状态变量换成点击队列；ISR 只记时间戳；新增 `Screen_PollKey()` / `Screen_ClickReset()` / `Screen_ClickKeepLast()` / `Screen_FlushRange()` |
| `Core/Src/main.c` | 主循环按键分支拆成"双击切档 / 单击重扫" |
| `README.md` | §5 流程、§8.2（删掉"不做运行期切档"的说法）、§9 按键说明、§12 参数表、§15 限制、§17 待上板 |
| `PLAN-继电器三档量程.md` | §11「明确不做」里"无 `Relay_Set()`、不做运行期切档"要改成"已被 `PLAN-双击切档.md` 取代" |
| `.ccg-build/verify.sh` | `EXPECT_CODE` 基准要按新体积重算；可加一条 `has Core/Src/main.c "SCREEN_KEY_DOUBLE"` |

**不需要改**：`.ioc`、`MDK-ARM/9_30.uvprojx`、`gpio.c`、`main.h`、`measure.c`、`uart_debug.c`。
（`use/relay_range.c` 已在工程里，没有新增源文件。）

---

## 9. 仓库外的人工动作

1. 串口屏 HMI 工程里，按钮 `b0` 的文字建议从"换频"/"刷新"改成"重扫/切档"之类，
   否则用户不知道双击能切档。这一步 agent 做不了，要人改。
2. 屏上如果需要"当前档位"以外的提示（例如切档时闪一下），也在 HMI 工程里做；
   固件只负责把 `t10` 的值改掉。

---

## 10. 验收标准

1. `bash .ccg-build/build.sh <out>`：**39 files / 0 error**。
2. `bash .ccg-build/verify.sh`：PASS（更新基准后）。
3. 全仓 grep 不到 `Screen_TakeRefreshKey`（已删除干净）。
4. §7 的 T1–T9 至少 T1/T2/T4/T6 在板上验证通过。
5. 代码风格与现有一致：4 空格缩进、`/* */` 注释、句尾 `u` 后缀、变量在块首声明。

---

## 11. 明确不做（防止加戏）

- 不实现自动量程，不加"按 Uo 自动换档"。
- 不做长按/三击/双击速度设置界面。
- 不把档位存 Flash，不加"上次档位"记忆。
- 不新增定时器、不新增中断优先级、不改 NVIC 配置。
- 不改 `SCREEN_REFRESH_PERIOD_MS`（500 ms 保持不变）。
- 不动 `Relay_Init()` 的上电时序，不动 `gpio.c` / `.ioc` 的引脚定义。
