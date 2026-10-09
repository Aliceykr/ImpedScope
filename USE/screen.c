#include "screen.h"

#include <stdio.h>

/* 指令格式：控件名.txt="值" + 0xFF 0xFF 0xFF。 */
#define SCREEN_CMD_END_BYTE          0xFFu
#define SCREEN_CMD_END_COUNT         3u
#define SCREEN_TX_BUFFER_BYTES       128u
#define SCREEN_VALUE_CHARS           24u

/* HAL_UART_Transmit_IT 发送期间缓冲区必须保持有效，故使用静态数组。 */
static uint8_t s_tx_buffer[SCREEN_TX_BUFFER_BYTES];
static FRF_Point_t s_result;
static uint8_t s_has_result;
static uint32_t s_last_tick;

static uint16_t Screen_AppendCommand(uint16_t length, const char *name, const char *value);
static uint16_t Screen_BuildCommands(void);

void Screen_Init(void)
{
    s_has_result = 0u;
    s_last_tick = HAL_GetTick();
}

void Screen_SetResult(const FRF_Point_t *result)
{
    s_result = *result;
    s_has_result = 1u;
}

void Screen_Poll(void)
{
    uint16_t length;

    if (s_has_result == 0u)
    {
        return;
    }

    if ((HAL_GetTick() - s_last_tick) < SCREEN_REFRESH_PERIOD_MS)
    {
        return;
    }

    /* 上一条还没发完则跳过本周期，不阻塞主循环。 */
    if (huart4.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    length = Screen_BuildCommands();
    if (HAL_UART_Transmit_IT(&huart4, s_tx_buffer, length) == HAL_OK)
    {
        s_last_tick = HAL_GetTick();
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

    (void)snprintf(value, sizeof(value), "%.0f", (double)s_result.frequency_hz);
    length = Screen_AppendCommand(length, "t_freq", value);

    (void)snprintf(value, sizeof(value), "%.2f", (double)s_result.z_mag_ohm);
    length = Screen_AppendCommand(length, "t_zmag", value);

    (void)snprintf(value, sizeof(value), "%.2f", (double)s_result.z_phase_deg);
    length = Screen_AppendCommand(length, "t_zph", value);

    return length;
}
