#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "measure.h"

/*
 * 第二路串口：USART3（PD8 = TX，TX-only），921600 8N1。
 *
 * 只做调试日志输出，不接收命令：
 *   - 每完成一次有效测量输出一行（默认最小间隔 200 ms，5 Hz）；
 *   - 发送走 1024 字节环形缓冲 + HAL_UART_Transmit_IT()，主循环不阻塞；
 *   - 日志用整数单位（uS / mdeg / mV / dB×100），避免 %f 依赖。
 *
 * 与串口屏（UART4）完全独立：UART4 的 RX/TX 和 t0~t10 协议不变。
 */

#ifndef UART_DEBUG_ENABLE
#define UART_DEBUG_ENABLE            1u
#endif

/* 两行日志之间的最小间隔；1 kHz 测量时约 5 行/秒，100 Hz 时约 1 行/秒。 */
#define UART_DEBUG_MIN_INTERVAL_MS   200u

/* 环形缓冲大小。一行约 120 字节，1024 字节可缓存 8 行左右。 */
#define UART_DEBUG_TX_BUFFER_SIZE    1024u

/* 复位环形缓冲与状态；必须在 MX_USART3_UART_Init() 之后调用。 */
void UartDebug_Init(void);

/* 把一段字节写入发送缓冲；缓冲不足时整段丢弃并计数，不阻塞。 */
void UartDebug_Write(const uint8_t *data, uint16_t length);

/* 格式化输出（只支持 %s/%d/%ld/%lu/%u 等整数格式，不支持 %f）。 */
void UartDebug_Printf(const char *format, ...);

/* 输出一行测量值：f、|Y|、phiY、G、B、H、R、档位、Ui/Uo。 */
void UartDebug_LogMeasurement(const FRF_Point_t *point,
                              const ADC_Window_t *window,
                              uint8_t range_index);

/* 输出一行事件，例如 "BOOT"、"SCAN_FAIL"。 */
void UartDebug_LogEvent(const char *event);

/* USART3 出错时由 screen.c 的 HAL_UART_ErrorCallback() 调用，复位发送状态。 */
void UartDebug_OnError(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_DEBUG_H */
