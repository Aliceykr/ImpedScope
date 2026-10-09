#ifndef SCREEN_H
#define SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "circuit_id.h"

/* 串口屏刷新周期。 */
#define SCREEN_REFRESH_PERIOD_MS     500u

/* 复位发送状态并记录起始时间。需在 MX_UART4_Init() 之后调用。 */
void Screen_Init(void);

/* 保存最近一个有效频点的计算结果，下次刷新时显示。 */
void Screen_SetResult(const FRF_Point_t *result);

/* 在主循环中反复调用；到达周期后非阻塞发送 t_freq、t_zmag、t_zph 指令。 */
void Screen_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_H */
