#include "screen.h"
#include "relay_range.h"
#include "uart_debug.h"
#include "measure.h"   /* Screen_KeyService 调用 Measure_Request* 函数 */

#include <stdio.h>
#include <string.h>

/* 指令格式：控件名.txt="值" + 0xFF 0xFF 0xFF。 */
#define SCREEN_CMD_END_BYTE          0xFFu
#define SCREEN_CMD_END_COUNT         3u
#define SCREEN_TX_BUFFER_BYTES        384u
#define SCREEN_VALUE_CHARS           40u
#define SCREEN_STAGE_CHARS           16u
#define SCREEN_BLOCKING_TIMEOUT_MS   100u   /* 921600 下一条 t5 远小于 1ms,留足裕量 */
#define SCREEN_STAGE_HOLD_MS         150u   /* 启动阶段肉眼保持时间 */
#define SCREEN_CMD_GAP_MS              2u   /* 指令之间留 2ms 给 HMI 处理/重绘 */

/* 致命路径有界轮询的上限。关中断后 SysTick 已停,HAL 的 tick 超时失效
 * (HAL_MAX_DELAY 永不超时,有限值也因 tick 冻结而永不到期),故这里用
 * 纯计数器兜底,保证 UART 无响应时也能退出,而不是死等。 */
#define SCREEN_FATAL_SPIN_LIMIT      2000000u

/* 错误码对应的短名称，索引 0 不使用。 */
static const char * const s_error_names[SCREEN_ERROR_CODE_MAX + 1u] =
{
    "",
    "HAL_INIT",       /* E01 */
    "CIR_CFG",        /* E02 */
    "CIR_NAN_IN",     /* E03 */
    "CIR_UI_LOW",     /* E04 */
    "CIR_UO_LOW",     /* E05 */
    "CIR_H_NAN",      /* E06 */
    "CIR_OUT_NAN",    /* E07 */
    "CIR_H_LOW",      /* E08 */
    "CIR_ZY_NAN",     /* E09 */
    "UART_TX",        /* E10 */
    "UART_BUSY",      /* E11 */
    "WINDOW_TIMEOUT", /* E12 */
    "DDS_SPI",        /* E13 已停用,保留编号 */
    "CIR_UI_INCOH",   /* E14 */
    "CIR_UO_INCOH",   /* E15 */
    "ADC_DUAL_MODE",  /* E16 */
    "ADC_OVR",        /* E17 */
    "ADC_DMA",        /* E18 */
    "ADC_CLIP_UI",    /* E19 */
    "ADC_CLIP_UO"     /* E20 */
};

/* HAL_UART_Transmit_IT 发送期间缓冲区必须保持有效，故使用静态数组。 */
static uint8_t s_tx_buffer[SCREEN_TX_BUFFER_BYTES];
static FRF_Point_t s_result;
static uint8_t s_has_result;
static uint8_t s_has_debug;
static float   s_debug[4];      /* Ui 电压、Uo 电压（V）、Ui 相干性、Uo 相干性 */
static uint32_t s_last_tick;
static uint32_t s_beat;
static char s_stage[SCREEN_STAGE_CHARS];

/* 错误计数由中断上下文（DMA 回调）与主循环并发访问，必须 volatile。 */
static volatile uint16_t s_error_counts[SCREEN_ERROR_CODE_MAX + 1u];
static volatile uint8_t  s_last_error_code;
static volatile uint8_t  s_fatal_code;
static volatile uint8_t  s_fatal_latched;
static volatile uint8_t  s_uart_ready;

/* 屏幕按键接收: b0/b1/b2 各发一个字节:
 *   b0 = 0x01, b1 = 0x02, b2 = 0x03。
 * 单字节协议,不再需要三字节帧头/超时重同步。 */
static uint8_t s_rx_byte;
static volatile uint8_t s_rx_armed;
static uint32_t s_rx_lockout_tick;

/* 按键事件队列: 中断写入、主循环消费。主循环可能被 DFT 窗口阻塞 1 s,
 * 所以事件必须留在队列里等下一轮,不能只存"当前按键"。 */
#define SCREEN_KEY_QUEUE_SIZE 4u

static volatile uint8_t s_key_queue[SCREEN_KEY_QUEUE_SIZE];
static volatile uint8_t s_key_head;    /* 写指针,中断推进 */
static volatile uint8_t s_key_tail;    /* 读指针,主循环推进 */
static volatile uint16_t s_key_dropped;/* 队列满而丢弃的事件数,仅诊断 */

/* b0 合并/去重机制：同一时间最多保留一个待处理的 b0 请求。
 * 正在扫描时再次按 b0，只记录"还需要再扫一次"，不堆多个事件。 */
static volatile uint8_t s_b0_pending;

/* beat 复位请求：B0 事件到达时设置，由 Screen_Poll() 应用。
 * 这样 b1/b2 绝不会复位 beat，b0 的反馈也更及时。 */
static volatile uint8_t s_beat_reset_pending;

/* 自动扫描的五个候选频率,仅用于校验/记录当前测量频率。 */
#define SCREEN_FREQ_COUNT 5u
static const uint32_t s_freq_hz[SCREEN_FREQ_COUNT] =
{
    100u, 300u, 1000u, 3000u, 10000u
};
static uint32_t s_active_freq_hz;          /* 当前真正在测量的频率 */

static uint16_t Screen_AppendCommand(uint16_t length, const char *name, const char *value);
static uint16_t Screen_BuildCommands(void);
static const char *Screen_RangeText(void);
static void Screen_SendStatusBlocking(const char *value, uint32_t timeout);
static void Screen_SendStatusRaw(const char *value);
static void Screen_RxStart(void);

void Screen_Init(void)
{
    uint8_t index;

    s_has_result = 0u;
    s_has_debug = 0u;
    s_last_tick = HAL_GetTick();
    s_beat = 0u;
    s_last_error_code = 0u;
    s_fatal_latched = 0u;
    s_fatal_code = SCREEN_E01_HAL_INIT;   /* 默认覆盖 MX_*_Init 内的 Error_Handler */
    /* 置 0 而不是 HAL_GetTick()：锁定期判定是"距上次入队不足 100 ms 就丢"，
     * 若初始化成当前 tick，则初始化后 100 ms 内的第一次按键必被丢弃。
     * 置 0 时 HAL_GetTick 上电后从 0 递增，第一个按键必然通过；
     * 32 位回绕后 (uint32_t)(now - 0) 仍是正确的小值。 */
    s_rx_lockout_tick = 0u;
    s_key_head = 0u;
    s_key_tail = 0u;
    s_key_dropped = 0u;
    s_b0_pending = 0u;
    s_beat_reset_pending = 0u;
    s_active_freq_hz = 0u;                /* 0 = 尚未生效 */
    s_rx_armed = 0u;
    for (index = 0u; index <= SCREEN_ERROR_CODE_MAX; ++index)
    {
        s_error_counts[index] = 0u;
    }
    /* 不重置 s_stage：实际调用顺序为
     *   Screen_Init -> SetStage("PERIPH") -> CircuitID_Init -> SetStage("MODULE")
     * 本函数之后 PERIPH 打点会立即覆盖 s_stage。 */
    s_uart_ready = 1u;                    /* 本函数在 MX_UART4_Init 之后调用 */
    Screen_RxStart();                     /* 开始接收 b0/b1/b2 按键 */
    (void)Screen_SetActiveFreq(1000u);    /* 默认测量频率 1 kHz */
}

void Screen_SetResult(const FRF_Point_t *result)
{
    s_result = *result;
    s_has_result = 1u;
}

void Screen_ClearResult(void)
{
    s_has_result = 0u;
}

void Screen_SetDebug(float ui_volt, float uo_volt, float ui_coherence, float uo_coherence)
{
    s_debug[0] = ui_volt;
    s_debug[1] = uo_volt;
    s_debug[2] = ui_coherence;
    s_debug[3] = uo_coherence;
    s_has_debug = 1u;
}

void Screen_SetStage(const char *stage)
{
    char value[SCREEN_VALUE_CHARS];

    if (s_fatal_latched != 0u)
    {
        return;
    }
    if (stage == (const char *)0)
    {
        stage = "?";
    }
    (void)strncpy(s_stage, stage, sizeof(s_stage) - 1u);
    s_stage[sizeof(s_stage) - 1u] = '\0';
    /* 新阶段覆盖未致命错误显示。 */
    s_last_error_code = 0u;

    /* UART 就绪后立即阻塞发送，否则阶段只停留在内存（BOOT/CLK 即如此）。 */
    if (s_uart_ready != 0u)
    {
        (void)snprintf(value, sizeof(value), "stage:%s", s_stage);
        Screen_SendStatusBlocking(value, SCREEN_BLOCKING_TIMEOUT_MS);
        HAL_Delay(SCREEN_STAGE_HOLD_MS);
    }
}

void Screen_ReportError(uint8_t code)
{
    if ((code == 0u) || (code > SCREEN_ERROR_CODE_MAX))
    {
        return;
    }
    if (s_fatal_latched != 0u)
    {
        return;
    }
    s_error_counts[code]++;
    s_last_error_code = code;
}

void Screen_ClearError(void)
{
    if (s_fatal_latched == 0u)
    {
        s_last_error_code = 0u;
    }
}

void Screen_Fatal(uint8_t code)
{
    if ((code != 0u) && (code <= SCREEN_ERROR_CODE_MAX))
    {
        s_fatal_code = code;
    }
    s_fatal_latched = 1u;
    Error_Handler();   /* 不返回 */
}

void Screen_FatalReportBlocking(void)
{
    char value[SCREEN_VALUE_CHARS];

    if (s_uart_ready == 0u)
    {
        return;
    }

    /* 关中断、SysTick 已停：tick 超时不可用，改用有界轮询。 */
    (void)snprintf(value, sizeof(value), "stage:E%02d %s",
                   (unsigned)s_fatal_code, s_error_names[s_fatal_code]);
    Screen_SendStatusRaw(value);
}

/* 致命路径专用的阻塞发送:不 abort、不依赖 HAL 状态与 SysTick。
 * 停掉可能在跑的 IT/DMA 发送请求,清标志,再按计数器有界等待 TXE/TC。
 * 关中断后 SysTick 已停:HAL_MAX_DELAY 永不超时,有限超时也永不
 * 到期(GetTick 差恒为 0),所以必须绕开 HAL 的 tick 超时机制。 */
static void Screen_SendStatusRaw(const char *value)
{
    uint16_t length = 0u;
    uint16_t remaining;
    const uint8_t *cursor;
    uint32_t spin;

    /* 状态/错误通道是 t5(见 Screen_BuildCommands)。 */
    length = Screen_AppendCommand(length, "t5", value);

    /* 丢弃上一次未能完成的传输:只清发送请求位,不动 gState。 */
    ATOMIC_CLEAR_BIT(huart4.Instance->CR1, (USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE));
    ATOMIC_CLEAR_BIT(huart4.Instance->CR3, USART_CR3_DMAT);
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_PEF | UART_CLEAR_FEF);

    cursor = s_tx_buffer;
    remaining = length;
    while (remaining > 0u)
    {
        spin = SCREEN_FATAL_SPIN_LIMIT;
        while ((__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE) == RESET) &&
               (spin != 0u))
        {
            --spin;
        }
        if (spin == 0u)
        {
            break;   /* UART 无响应:放弃本轮,不再死等 */
        }
        huart4.Instance->TDR = (uint8_t)(*cursor & 0xFFu);
        ++cursor;
        --remaining;
    }

    /* 等最后一字节移位完成(TC),同样有界。 */
    spin = SCREEN_FATAL_SPIN_LIMIT;
    while ((__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET) &&
           (spin != 0u))
    {
        --spin;
    }

    /* 本函数直接写了硬件寄存器,HAL 的 gState 需还原,否则后续 HAL 调用
     * 会因 gState 停在 BUSY_TX 而一律返回 HAL_BUSY。 */
    huart4.gState = HAL_UART_STATE_READY;
    huart4.TxXferCount = 0u;
}

/* 阻塞发送一条 t5 指令；发送前清掉异常残留的 TX 状态。 */
static void Screen_SendStatusBlocking(const char *value, uint32_t timeout)
{
    uint16_t length = 0u;

    if (huart4.gState != HAL_UART_STATE_READY)
    {
        /* 只中止发送,不能动 RX:本函数在启动阶段也会被调用,不能把接收一起停掉。 */
        (void)HAL_UART_AbortTransmit(&huart4);
    }
    length = Screen_AppendCommand(length, "t5", value);
    (void)HAL_UART_Transmit(&huart4, s_tx_buffer, length, timeout);
    if (s_rx_armed == 0u)
    {
        Screen_RxStart();
    }
}

/* 启动/重启 UART4 单字节接收。主循环与 RX 回调都可能调用,用临界区防重入。 */
static void Screen_RxStart(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1u) == HAL_OK)
    {
        s_rx_armed = 1u;
    }
    else
    {
        s_rx_armed = 0u;
    }
    __set_PRIMASK(primask);
}

uint8_t Screen_SetActiveFreq(uint32_t frequency_hz)
{
    uint8_t index;

    for (index = 0u; index < SCREEN_FREQ_COUNT; ++index)
    {
        if (s_freq_hz[index] == frequency_hz)
        {
            break;
        }
    }
    if ((index >= SCREEN_FREQ_COUNT) || (frequency_hz == s_active_freq_hz))
    {
        return 0u;
    }

    s_active_freq_hz = frequency_hz;
    /* 档位切换后,上一档的“最近错误”不再适用。 */
    s_last_error_code = 0u;
    /* 强制下一轮 Screen_Poll 立即刷新结果区。t10 由整帧统一更新,
     * 切档时由 Screen_FlushRange() 额外立即补发。 */
    s_last_tick = HAL_GetTick() - SCREEN_REFRESH_PERIOD_MS;
    if (s_rx_armed == 0u)
    {
        Screen_RxStart();
    }

    return 1u;
}

/* 队列空返回 SCREEN_KEY_NONE，非空取出队首并推进读指针。
 * 只在主循环调用，临界区防中断同时改指针。 */
Screen_KeyEvent_t Screen_PollKey(void)
{
    Screen_KeyEvent_t event = SCREEN_KEY_NONE;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_key_tail != s_key_head)
    {
        event = (Screen_KeyEvent_t)s_key_queue[s_key_tail];
        ++s_key_tail;
        if (s_key_tail >= SCREEN_KEY_QUEUE_SIZE)
        {
            s_key_tail = 0u;
        }
    }

    __set_PRIMASK(primask);
    return event;
}

/* 窥视队列中的下一个按键（不消费）。用于 ADC 窗口等待期间检测 B1。 */
Screen_KeyEvent_t Screen_PeekKey(void)
{
    Screen_KeyEvent_t event = SCREEN_KEY_NONE;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_key_tail != s_key_head)
    {
        event = (Screen_KeyEvent_t)s_key_queue[s_key_tail];
    }

    __set_PRIMASK(primask);
    return event;
}

/* 单字节 → 按键事件。0x01=b0、0x02=b1、0x03=b2，其余无效。 */
static Screen_KeyEvent_t Screen_KeyFromByte(uint8_t byte)
{
    switch (byte)
    {
        case 0x01u: return SCREEN_KEY_B0;
        case 0x02u: return SCREEN_KEY_B1;
        case 0x03u: return SCREEN_KEY_B2;
        default:    return SCREEN_KEY_NONE;
    }
}

/* 把一次完整识别到的按键入队。中断里调用，用临界区保护头尾指针。
 * 队列满时丢弃新事件并计数，不阻塞中断。
 * B0 特殊处理：b0 合并/去重，同一时间最多保留一个待处理的 b0 请求。
 * 同时，B0 事件到达时立即设置 beat 复位请求标志。 */
static void Screen_KeyEnqueue(Screen_KeyEvent_t event, uint32_t now)
{
    uint32_t primask;

    if ((uint32_t)(now - s_rx_lockout_tick) < 100u)
    {
        return;                    /* 100 ms 锁定期内：同一帧/同一次点击不重复触发 */
    }

    /* B0 事件到达时立即请求 beat 复位（不等主循环处理） */
    if (event == SCREEN_KEY_B0)
    {
        s_beat_reset_pending = 1u;

        /* b0 合并/去重：如果已有待处理的 b0，忽略新的（避免堆积） */
        primask = __get_PRIMASK();
        __disable_irq();
        if (s_b0_pending != 0u)
        {
            __set_PRIMASK(primask);
            return;                /* 已有待处理的 b0，不再重复入队 */
        }
        s_b0_pending = 1u;
        __set_PRIMASK(primask);
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((uint8_t)((s_key_head + 1u) % SCREEN_KEY_QUEUE_SIZE) == s_key_tail)
    {
        s_key_dropped++;
    }
    else
    {
        s_key_queue[s_key_head] = (uint8_t)event;
        s_key_head++;
        if (s_key_head >= SCREEN_KEY_QUEUE_SIZE)
        {
            s_key_head = 0u;
        }
    }

    __set_PRIMASK(primask);
    s_rx_lockout_tick = now + 100u;
}

/* b0/b1/b2 按键协议:单字节
 *   b0 = 0x01, b1 = 0x02, b2 = 0x03。
 * 单字节互不相同，不需要帧头/收满三字节；中断直接映射并入队。
 * 普通 ASCII 文本不在这三个值里，不会误触发。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uint8_t ch = s_rx_byte;
        Screen_KeyEvent_t event;
        uint32_t now;

        s_rx_armed = 0u;

        event = Screen_KeyFromByte(ch);
        if (event != SCREEN_KEY_NONE)
        {
            now = HAL_GetTick();
            Screen_KeyEnqueue(event, now);
        }

        Screen_RxStart();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_FEF | UART_CLEAR_PEF);
        /* 同样置 0:置 HAL_GetTick() 会让出错后 100 ms 内的按键被丢掉。 */
        s_rx_lockout_tick = 0u;
        Screen_RxStart();
    }
    else if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_FEF | UART_CLEAR_PEF);
        UartDebug_OnError();
    }
}

void Screen_Poll(void)
{
    uint16_t length;

    if (s_rx_armed == 0u)
    {
        Screen_RxStart();
    }

    /* 应用 beat 复位请求（如果 B0 事件到达后设置了标志） */
    if (s_beat_reset_pending != 0u)
    {
        s_beat_reset_pending = 0u;
        Screen_ResetBeat();
    }

    if ((HAL_GetTick() - s_last_tick) < SCREEN_REFRESH_PERIOD_MS)
    {
        return;
    }
    s_last_tick = HAL_GetTick();

    /* 心跳在任何路径下都递增，包括 UART 忙的周期。 */
    s_beat++;

    length = Screen_BuildCommands();
    if (length == 0u)
    {
        return;
    }

    /* 一条一条阻塞发,每条之间留 2 ms 给 HMI 解析和重绘。
     * 921600 下一次性灌 10 条会让屏幕把整页重画,看起来像"卡一下";
     * 分开送,每条都能被及时处理。 */
    {
        uint16_t pos = 0u;

        while (pos < length)
        {
            uint16_t end = length;
            uint16_t index;

            for (index = pos; (uint16_t)(index + 2u) < length; ++index)
            {
                if ((s_tx_buffer[index] == SCREEN_CMD_END_BYTE) &&
                    (s_tx_buffer[index + 1u] == SCREEN_CMD_END_BYTE) &&
                    (s_tx_buffer[index + 2u] == SCREEN_CMD_END_BYTE))
                {
                    end = (uint16_t)(index + 3u);
                    break;
                }
            }

            if (end <= pos)
            {
                break;
            }

            if (HAL_UART_Transmit(&huart4, &s_tx_buffer[pos],
                                  (uint16_t)(end - pos), 10u) != HAL_OK)
            {
                Screen_ReportError(SCREEN_E10_UART_TX);
                break;
            }

            /* 阻塞发送期间 HAL 锁会让 RX 回调里的重挂失败;发完立刻补挂。 */
            if (s_rx_armed == 0u)
            {
                Screen_RxStart();
            }

            pos = end;
            if (pos < length)
            {
                HAL_Delay(SCREEN_CMD_GAP_MS);
            }
        }
    }
}

void Screen_ResetBeat(void)
{
    /* Screen_Poll() 在构建 t6 前先 s_beat++，所以这里置 UINT32_MAX，
     * 下一轮自增后正好回到 0，屏幕先显示 beat:0。 */
    s_beat = 0xFFFFFFFFu;
    s_last_tick = HAL_GetTick() - SCREEN_REFRESH_PERIOD_MS;
}

void Screen_RequestBeatReset(void)
{
    s_beat_reset_pending = 1u;
}

uint8_t Screen_IsB0Pending(void)
{
    return s_b0_pending;
}

void Screen_ClearB0Pending(void)
{
    s_b0_pending = 0u;
}

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

static const char *Screen_RangeText(void)
{
    switch (Relay_Get())
    {
        case 0u: return "150";
        case 1u: return "1.5k";
        case 2u: return "15k";
        default: return "?";
    }
}

static uint16_t Screen_AppendCommand(uint16_t length, const char *name, const char *value)
{
    int written;
    uint16_t remaining = (uint16_t)(SCREEN_TX_BUFFER_BYTES - length);

    written = snprintf((char *)&s_tx_buffer[length], remaining, "%s.txt=\"%s\"", name, value);
    if ((written < 0) || ((uint16_t)written + SCREEN_CMD_END_COUNT > remaining))
    {
        return length;
    }

    length = (uint16_t)(length + (uint16_t)written);
    for (uint8_t index = 0u; index < SCREEN_CMD_END_COUNT; ++index)
    {
        s_tx_buffer[length] = SCREEN_CMD_END_BYTE;
        ++length;
    }

    return length;
}

static uint16_t Screen_BuildCommands(void)
{
    char value[SCREEN_VALUE_CHARS];
    uint16_t length = 0u;
    uint8_t code;

    /* t0:频率 (Hz)。|Y|、G、B 存储单位是 S,串口屏显示 mS,故均 x1000。 */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "f:%.0f", (double)s_result.frequency_hz);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "f:-");
    }
    length = Screen_AppendCommand(length, "t0", value);

#if CIR_DUT_TOPOLOGY == 1u
    /* t1:|Y| (mS) */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "Y:%.3f",
                       (double)s_result.y_mag_siemens * 1000.0);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "Y:-");
    }
    length = Screen_AppendCommand(length, "t1", value);

    /* t2:phiY (度) */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "P:%.2f", (double)s_result.y_phase_deg);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "P:-");
    }
    length = Screen_AppendCommand(length, "t2", value);

    /* t3:G = Re[Y] (mS) */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "G:%.3f",
                       (double)s_result.y_conductance_siemens * 1000.0);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "G:-");
    }
    length = Screen_AppendCommand(length, "t3", value);

    /* t4:B = Im[Y] (mS) */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "B:%.3f",
                       (double)s_result.y_susceptance_siemens * 1000.0);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "B:-");
    }
    length = Screen_AppendCommand(length, "t4", value);
#else
    /* CIR_DUT_TOPOLOGY = 0:只计算 H,FRF_Point_t 没有 Y 字段,四行固定显示 "-"。 */
    length = Screen_AppendCommand(length, "t1", "Y:-");
    length = Screen_AppendCommand(length, "t2", "P:-");
    length = Screen_AppendCommand(length, "t3", "G:-");
    length = Screen_AppendCommand(length, "t4", "B:-");
#endif

    /* t7、t8:窗口诊断量（电压单位 V，k 为相干性）。计算失败时也会更新，所以"没信号"
     * 时这里能看到两路本底电压有多大、相干性有多低。 */
    if (s_has_debug != 0u)
    {
        (void)snprintf(value, sizeof(value), "Ui:%.4fV k:%.2f", (double)s_debug[0], (double)s_debug[2]);
        length = Screen_AppendCommand(length, "t7", value);
        (void)snprintf(value, sizeof(value), "Uo:%.4fV k:%.2f", (double)s_debug[1], (double)s_debug[3]);
        length = Screen_AppendCommand(length, "t8", value);
    }
    else
    {
        length = Screen_AppendCommand(length, "t7", "Ui:-");
        length = Screen_AppendCommand(length, "t8", "Uo:-");
    }

    /* t9:H 的幅值(dB)与相位(°)。H 本身不参与显示,但标定必须读它:
     * 直通测试时两路接同一节点,H 的理论值是 0 dB / 0°,实测偏差就是第 3 节
     * 标定表要填的两个数。|Y|、φY 是在 H 之后才对 H 做变换,读数时反推麻烦,
     * 所以单独显示一行。*/
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "H:%.2fdB",
                       (double)s_result.h_magnitude_dB);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "H:-");
    }
    length = Screen_AppendCommand(length, "t9", value);

    /* t10:当前反馈电阻档位(标称值,便于和三档定义对应)。
     * H 相位 A 不再显示。 */
    (void)snprintf(value, sizeof(value), "R:%s", Screen_RangeText());
    length = Screen_AppendCommand(length, "t10", value);

    /* t6:心跳,每刷新周期 +1 */
    (void)snprintf(value, sizeof(value), "beat:%lu", (unsigned long)s_beat);
    length = Screen_AppendCommand(length, "t6", value);

    /* t5:阶段或最近一次错误 */
    if (s_fatal_latched != 0u)
    {
        (void)snprintf(value, sizeof(value), "stage:E%02d %s",
                       (unsigned)s_fatal_code, s_error_names[s_fatal_code]);
    }
    else
    {
        code = s_last_error_code;
        if (code != 0u)
        {
            (void)snprintf(value, sizeof(value), "stage:E%02d %s x%u",
                            (unsigned)code, s_error_names[code],
                            (unsigned)s_error_counts[code]);
        }
        else
        {
            (void)snprintf(value, sizeof(value), "stage:%s", s_stage);
        }
    }
    length = Screen_AppendCommand(length, "t5", value);

    return length;
}

/* 按键服务：处理 b0/b1/b2 按键，设置相应的请求标志 */
void Screen_KeyService(void)
{
    Screen_KeyEvent_t event = Screen_PollKey();

    if (event == SCREEN_KEY_B0)
    {
        /* B0：请求 beat 复位，设置 pending，通知 measure 层重扫 */
        Screen_RequestBeatReset();
        if (Screen_IsB0Pending() == 0u)
        {
            Screen_ClearB0Pending();  /* 重置标志以便重新设置 */
        }
        /* 直接设置 pending，不调用 ClearB0Pending/IsB0Pending */
        /* 因为 B0 合并/去重逻辑已在 KeyEnqueue 中实现 */
        Measure_RequestScan();
    }
    else if (event == SCREEN_KEY_B1)
    {
        /* B1：请求切换到下一档 */
        Measure_RequestNextRange();
    }
    /* B2：预留，静默忽略 */
}

