#include "uart_debug.h"
#include "usart.h"
#include "relay_range.h"

#include <stdio.h>
#include <stdarg.h>

#if (UART_DEBUG_ENABLE != 0u)

static uint8_t  s_tx_buf[UART_DEBUG_TX_BUFFER_SIZE];
static volatile uint16_t s_head = 0u;
static volatile uint16_t s_tail = 0u;
static volatile uint16_t s_tx_len = 0u;
static volatile uint8_t  s_tx_busy = 0u;
static volatile uint16_t s_drop_count = 0u;
static uint32_t s_last_log_tick = 0u;
static uint8_t  s_first_log = 1u;

static int32_t UartDebug_Round(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/* 空闲时从环形缓冲取下一段启动中断发送。主循环和 TxCplt 回调都会调用。 */
static void UartDebug_Kick(void)
{
    uint32_t primask;
    uint16_t head;
    uint16_t tail;
    uint16_t len;

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_tx_busy != 0u)
    {
        __set_PRIMASK(primask);
        return;
    }

    head = s_head;
    tail = s_tail;
    if (head == tail)
    {
        __set_PRIMASK(primask);
        return;
    }

    if (head > tail)
    {
        len = (uint16_t)(head - tail);
    }
    else
    {
        len = (uint16_t)(UART_DEBUG_TX_BUFFER_SIZE - tail);
    }

    s_tx_len  = len;
    s_tx_busy = 1u;
    __set_PRIMASK(primask);

    if (HAL_UART_Transmit_IT(&huart3, &s_tx_buf[tail], len) != HAL_OK)
    {
        s_tx_busy = 0u;
    }
}

void UartDebug_Init(void)
{
    s_head = 0u;
    s_tail = 0u;
    s_tx_len = 0u;
    s_tx_busy = 0u;
    s_drop_count = 0u;
    s_last_log_tick = 0u;
    s_first_log = 1u;
}

void UartDebug_Write(const uint8_t *data, uint16_t length)
{
    uint16_t used;
    uint16_t free_space;
    uint16_t head;
    uint16_t index;
    uint32_t primask;

    if ((data == (const uint8_t *)0) || (length == 0u))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    head = s_head;
    used = (head >= s_tail) ? (uint16_t)(head - s_tail)
                            : (uint16_t)(UART_DEBUG_TX_BUFFER_SIZE - s_tail + head);
    free_space = (uint16_t)(UART_DEBUG_TX_BUFFER_SIZE - 1u - used);
    __set_PRIMASK(primask);

    if (length > free_space)
    {
        ++s_drop_count;              /* 缓冲不够就整段丢弃，避免半行日志 */
        return;
    }

    for (index = 0u; index < length; ++index)
    {
        s_tx_buf[head] = data[index];
        head = (uint16_t)((head + 1u) % UART_DEBUG_TX_BUFFER_SIZE);
    }
    s_head = head;

    UartDebug_Kick();
}

void UartDebug_Printf(const char *format, ...)
{
    char buffer[192];
    va_list args;
    int length;

    if (format == (const char *)0)
    {
        return;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }
    if (length >= (int)sizeof(buffer))
    {
        length = (int)sizeof(buffer) - 1;
    }

    UartDebug_Write((const uint8_t *)buffer, (uint16_t)length);
}

void UartDebug_LogEvent(const char *event)
{
    if (event == (const char *)0)
    {
        return;
    }
    UartDebug_Printf("EVT %s\r\n", event);
}

void UartDebug_LogMeasurement(const FRF_Point_t *point,
                              const ADC_Window_t *window,
                              uint8_t range_index)
{
    uint32_t now;
    long y_us;
    long p_mdeg;
    long g_us;
    long b_us;
    long h_db;
    long h_deg;
    long r_ohm;
    long ui_mv;
    long uo_mv;

    if ((point == (const FRF_Point_t *)0) || (window == (const ADC_Window_t *)0))
    {
        return;
    }

    now = HAL_GetTick();
    if ((s_first_log == 0u) &&
        ((int32_t)(now - s_last_log_tick) < (int32_t)UART_DEBUG_MIN_INTERVAL_MS))
    {
        return;
    }
    s_first_log = 0u;
    s_last_log_tick = now;

    h_db  = (long)UartDebug_Round(point->h_magnitude_dB * 100.0f);
    h_deg = (long)UartDebug_Round(point->h_phase_deg * 100.0f);
    r_ohm = (long)UartDebug_Round(Relay_GetOhm());
    ui_mv = (long)UartDebug_Round((float)(window->ui_rms * ADC_SAMPLER_LSB_TO_V * 1000.0));
    uo_mv = (long)UartDebug_Round((float)(window->uo_rms * ADC_SAMPLER_LSB_TO_V * 1000.0));

#if (CIR_DUT_TOPOLOGY == 1u)
    y_us   = (long)UartDebug_Round(point->y_mag_siemens * 1e6f);
    p_mdeg = (long)UartDebug_Round(point->y_phase_deg * 1000.0f);
    g_us   = (long)UartDebug_Round(point->y_conductance_siemens * 1e6f);
    b_us   = (long)UartDebug_Round(point->y_susceptance_siemens * 1e6f);

    UartDebug_Printf("f=%lu,Y_uS=%ld,P_mdeg=%ld,G_uS=%ld,B_uS=%ld,H_dB_x100=%ld,H_deg_x100=%ld,R_ohm=%ld,range=%u,Ui_rms_mV=%ld,Uo_rms_mV=%ld\r\n",
                     (unsigned long)window->frequency_hz,
                     y_us, p_mdeg, g_us, b_us, h_db, h_deg, r_ohm,
                     (unsigned)range_index, ui_mv, uo_mv);
#else
    (void)y_us;
    (void)p_mdeg;
    (void)g_us;
    (void)b_us;

    UartDebug_Printf("f=%lu,H_dB_x100=%ld,H_deg_x100=%ld,R_ohm=%ld,range=%u,Ui_rms_mV=%ld,Uo_rms_mV=%ld\r\n",
                     (unsigned long)window->frequency_hz,
                     h_db, h_deg, r_ohm,
                     (unsigned)range_index, ui_mv, uo_mv);
#endif
}

void UartDebug_LogCompare(const FRF_Point_t *dft, const Spi_Frame_t *spi)
{
    /* 用 SPI 的 H 换算 Y，与 DFT 的 Y 比，输出一行差异（§6.3） */
    double h_re, h_im, y_re, y_im, y_mag, y_phase;
    double r = (double)CircuitID_GetFeedbackResistor();

    /* H_spi = mag * exp(j*phi) */
    h_re = (double)spi->h_mag * cos(spi->h_phase_deg * 3.14159265358979323846 / 180.0);
    h_im = (double)spi->h_mag * sin(spi->h_phase_deg * 3.14159265358979323846 / 180.0);

    /* Y_spi = -H_spi / R */
    y_re = -h_re / r;
    y_im = -h_im / r;

    /* 转换为极坐标 */
    y_mag = sqrt(y_re * y_re + y_im * y_im);
    y_phase = atan2(y_im, y_re) * 180.0 / 3.14159265358979323846;

    /* 计算偏差 */
    double dft_y_mag = (double)dft->y_mag;
    double dft_y_phase = (double)dft->y_phase_deg;

    int32_t dY_permille = (int32_t)((y_mag - dft_y_mag) / dft_y_mag * 1000.0);
    int32_t dP_mdeg = (int32_t)((y_phase - dft_y_phase) * 1000.0);

    /* 输出格式（全整数，符合现有 UartDebug_Printf 的支持范围） */
    UartDebug_Printf("SPI f=%lu,seq=%u,magH_u=%lu,PHI_mdeg=%ld,Y_uS=%lu,P_mdeg=%ld,",
                     (unsigned long)spi->frequency_hz,
                     (unsigned)spi->seq,
                     (unsigned long)(spi->h_mag * 1e6),
                     (long)(spi->h_phase_deg * 1000.0),
                     (unsigned long)(y_mag * 1e6),
                     (long)(y_phase * 1000.0));

    UartDebug_Printf("dY_permille=%ld,dP_mdeg=%ld,ok=%lu,err=%lu,ovr=%lu\r\n",
                     (long)dY_permille,
                     (long)dP_mdeg,
                     (unsigned long)Spi_SlaveValidCount(),
                     (unsigned long)Spi_SlaveErrorCount(),
                     (unsigned long)Spi_SlaveOverrunCount());
}

uint16_t UartDebug_DropCount(void)
{
    return s_drop_count;
}

void UartDebug_OnError(void)
{
    s_tx_busy = 0u;
    (void)HAL_UART_AbortTransmit(&huart3);
    UartDebug_Kick();
}

/* HAL 的弱回调；只处理 USART3，UART4 仍由 screen.c 的阻塞发送负责。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }

    s_tail = (uint16_t)((s_tail + s_tx_len) % UART_DEBUG_TX_BUFFER_SIZE);
    s_tx_busy = 0u;
    UartDebug_Kick();
}

#else /* UART_DEBUG_ENABLE == 0：全部编译为空，避免占用空间 */

void UartDebug_Init(void)
{
}

void UartDebug_Write(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
}

void UartDebug_Printf(const char *format, ...)
{
    (void)format;
}

void UartDebug_LogMeasurement(const FRF_Point_t *point,
                              const ADC_Window_t *window,
                              uint8_t range_index)
{
    (void)point;
    (void)window;
    (void)range_index;
}

void UartDebug_LogEvent(const char *event)
{
    (void)event;
}

void UartDebug_LogCompare(const FRF_Point_t *dft, const Spi_Frame_t *spi)
{
    (void)dft;
    (void)spi;
}

uint16_t UartDebug_DropCount(void)
{
    return 0u;
}

void UartDebug_OnError(void)
{
}

#endif /* UART_DEBUG_ENABLE */
