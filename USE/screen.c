#include "screen.h"

#include <stdio.h>
#include <string.h>

#include "spi_receiver.h"

/* 指令格式：控件名.txt="值" + 0xFF 0xFF 0xFF。 */
#define SCREEN_CMD_END_BYTE          0xFFu
#define SCREEN_CMD_END_COUNT         3u
#define SCREEN_TX_BUFFER_BYTES        256u
#define SCREEN_VALUE_CHARS           40u
#define SCREEN_STAGE_CHARS           16u
#define SCREEN_BLOCKING_TIMEOUT_MS   100u   /* 9600bps 下一条 t4 约 30ms */
#define SCREEN_STAGE_HOLD_MS         150u   /* 启动阶段肉眼保持时间 */

/* 错误码对应的短名称，索引 0 不使用。 */
static const char * const s_error_names[SCREEN_ERROR_CODE_MAX + 1u] =
{
    "",
    "HAL_INIT",     /* E01 */
    "SPI_DMA",      /* E02 */
    "SPI_FRAME",    /* E03 */
    "SPI_NAN",      /* E04 */
    "SPI_LEN",      /* E05 */
    "SPI_QFULL",    /* E06 */
    "CIR_CFG",      /* E07 */
    "CIR_NAN_IN",   /* E08 */
    "CIR_UI_LOW",   /* E09 */
    "CIR_UO_LOW",   /* E10 */
    "CIR_H_NAN",    /* E11 */
    "CIR_OUT_NAN",  /* E12 */
    "CIR_H_LOW",    /* E13 */
    "CIR_ZY_NAN",   /* E14 */
    "UART_TX",      /* E15 */
    "UART_BUSY",    /* E16 */
    "BIN_MISMATCH"  /* E17 */
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
static void Screen_SendT4Blocking(const char *value, uint32_t timeout);

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
        Screen_SendT4Blocking(value, SCREEN_BLOCKING_TIMEOUT_MS);
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

    /* 关中断状态下调用：SysTick 已停，超时必须用 HAL_MAX_DELAY。 */
    (void)snprintf(value, sizeof(value), "stage:E%02d %s",
                   (unsigned)s_fatal_code, s_error_names[s_fatal_code]);
    Screen_SendT4Blocking(value, HAL_MAX_DELAY);
}

/* 阻塞发送一条 t4 指令；发送前 abort 残留的 IT 传输。 */
static void Screen_SendT4Blocking(const char *value, uint32_t timeout)
{
    uint16_t length = 0u;

    if (huart4.gState != HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Abort(&huart4);
    }
    length = Screen_AppendCommand(length, "t4", value);
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
        Screen_ReportError(SCREEN_E16_UART_BUSY);
        return;
    }

    length = Screen_BuildCommands();
    if (HAL_UART_Transmit_IT(&huart4, s_tx_buffer, length) != HAL_OK)
    {
        Screen_ReportError(SCREEN_E15_UART_TX);
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
#if SCREEN_SPI_TEST == 1u
    SPI_Stats_t stats;
    uint8_t     index;

    SPI_Receiver_GetStats(&stats);
#else
    uint8_t code;
#endif

#if SCREEN_SPI_TEST == 1u
    /* 测试模式：t0~t2 显示收包计数，用于判断 CS、字节数和帧头是否正确。 */
    (void)snprintf(value, sizeof(value), "cs:%lu", (unsigned long)stats.cs_falling);
    length = Screen_AppendCommand(length, "t0", value);

    (void)snprintf(value, sizeof(value), "full:%lu", (unsigned long)stats.frame_full);
    length = Screen_AppendCommand(length, "t1", value);

    (void)snprintf(value, sizeof(value), "hdr:%lu", (unsigned long)stats.frame_ok);
    length = Screen_AppendCommand(length, "t2", value);
#else
    /* t0：频率 */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "freq:%.0f", (double)s_result.frequency_hz);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "freq:-");
    }
    length = Screen_AppendCommand(length, "t0", value);

    /* t1：阻抗幅值 */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "zmag:%.2f", (double)s_result.z_mag_ohm);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "zmag:-");
    }
    length = Screen_AppendCommand(length, "t1", value);

    /* t2：阻抗相位 */
    if (s_has_result != 0u)
    {
        (void)snprintf(value, sizeof(value), "zph:%.2f", (double)s_result.z_phase_deg);
    }
    else
    {
        (void)snprintf(value, sizeof(value), "zph:-");
    }
    length = Screen_AppendCommand(length, "t2", value);
#endif

    /* t3：心跳 */
    (void)snprintf(value, sizeof(value), "beat:%lu", (unsigned long)s_beat);
    length = Screen_AppendCommand(length, "t3", value);

#if SCREEN_SPI_TEST == 1u
    /* t4：最近一次收满的原始帧，格式 raw:十六进制（16 字节）。 */
    (void)snprintf(value, sizeof(value), "raw:");
    for (index = 0u; index < SPI_FRAME_TOTAL_BYTES; ++index)
    {
        (void)snprintf(&value[4u + ((uint16_t)index * 2u)], 3u, "%02X",
                       (unsigned)stats.last_frame[index]);
    }
#else
    /* t4：阶段或最近一次错误 */
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
#endif
    length = Screen_AppendCommand(length, "t4", value);

    return length;
}
