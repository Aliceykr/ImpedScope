#include "screen.h"

#include <stdio.h>
#include <string.h>

/* 指令格式：控件名.txt="值" + 0xFF 0xFF 0xFF。 */
#define SCREEN_CMD_END_BYTE          0xFFu
#define SCREEN_CMD_END_COUNT         3u
#define SCREEN_TX_BUFFER_BYTES        256u
#define SCREEN_VALUE_CHARS           40u
#define SCREEN_STAGE_CHARS           16u
#define SCREEN_BLOCKING_TIMEOUT_MS   100u   /* 9600bps 下一条 t5 约 30ms */
#define SCREEN_STAGE_HOLD_MS         150u   /* 启动阶段肉眼保持时间 */

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
    "AD9833_SPI"      /* E13 */
};

/* HAL_UART_Transmit_IT 发送期间缓冲区必须保持有效，故使用静态数组。 */
static uint8_t s_tx_buffer[SCREEN_TX_BUFFER_BYTES];
static FRF_Point_t s_result;
static uint8_t s_has_result;
static uint32_t s_last_tick;
static uint32_t s_beat;
static char s_stage[SCREEN_STAGE_CHARS];

/* 错误计数由中断上下文（DMA 回调）与主循环并发访问，必须 volatile。 */
static volatile uint16_t s_error_counts[SCREEN_ERROR_CODE_MAX + 1u];
static volatile uint8_t  s_last_error_code;
static volatile uint8_t  s_fatal_code;
static volatile uint8_t  s_fatal_latched;
static volatile uint8_t  s_uart_ready;

static uint16_t Screen_AppendCommand(uint16_t length, const char *name, const char *value);
static uint16_t Screen_BuildCommands(void);
static void Screen_SendStatusBlocking(const char *value, uint32_t timeout);
static void Screen_SendStatusRaw(const char *value);

void Screen_Init(void)
{
    uint8_t index;

    s_has_result = 0u;
    s_last_tick = HAL_GetTick();
    s_beat = 0u;
    s_last_error_code = 0u;
    s_fatal_latched = 0u;
    s_fatal_code = SCREEN_E01_HAL_INIT;   /* 默认覆盖 MX_*_Init 内的 Error_Handler */
    for (index = 0u; index <= SCREEN_ERROR_CODE_MAX; ++index)
    {
        s_error_counts[index] = 0u;
    }
    /* 不重置 s_stage：实际调用顺序为
     *   Screen_Init -> SetStage("PERIPH") -> CircuitID_Init -> SetStage("MODULE")
     * 本函数之后 PERIPH 打点会立即覆盖 s_stage。 */
    s_uart_ready = 1u;                    /* 本函数在 MX_UART4_Init 之后调用 */
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

    /* 状态/错误通道是 t5(见 Screen_BuildCommands);t4 已改作 B 读数。 */
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

/* 阻塞发送一条 t5 指令；发送前 abort 残留的 IT 传输。 */
static void Screen_SendStatusBlocking(const char *value, uint32_t timeout)
{
    uint16_t length = 0u;

    if (huart4.gState != HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Abort(&huart4);
    }
    length = Screen_AppendCommand(length, "t5", value);
    (void)HAL_UART_Transmit(&huart4, s_tx_buffer, length, timeout);
}

void Screen_Poll(void)
{
    uint16_t length;

    if ((HAL_GetTick() - s_last_tick) < SCREEN_REFRESH_PERIOD_MS)
    {
        return;
    }
    s_last_tick = HAL_GetTick();

    /* 心跳在任何路径下都递增，包括 UART 忙的周期。 */
    s_beat++;

    /* 上一条还没发完则跳过本周期，不阻塞主循环。 */
    if (huart4.gState != HAL_UART_STATE_READY)
    {
        Screen_ReportError(SCREEN_E11_UART_BUSY);
        return;
    }

    length = Screen_BuildCommands();
    if (HAL_UART_Transmit_IT(&huart4, s_tx_buffer, length) != HAL_OK)
    {
        Screen_ReportError(SCREEN_E10_UART_TX);
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

    /* t6:心跳,每刷新周期 +1 */
    (void)snprintf(value, sizeof(value), "beat:%lu", (unsigned long)s_beat);
    length = Screen_AppendCommand(length, "t6", value);

    /* t5:阶段或最近一次错误(t4 已改作 B 读数) */
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

