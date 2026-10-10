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

/* 1：通信测试界面（t0~t2 为 SPI 收包计数，t4 为最近原始帧）；
 * 0：正常显示频响结果。 */
#define SCREEN_SPI_TEST              1u

/*
 * 错误码，说明见 README.md 第 8.3 节。0 保留表示"无错误"。
 *
 * E01 是默认致命码：CubeMX 生成的 MX_*_Init() 内部直接调用
 * Error_Handler()，无法区分具体外设，统一显示 E01 HAL_INIT。
 * E04、E05 在 16 字节定点帧方案中已弃用，保留编号占位。
 */
typedef enum
{
    SCREEN_E01_HAL_INIT   = 1u,   /* MX_*_Init 内 Error_Handler，致命，默认码 */
    SCREEN_E02_SPI_DMA    = 2u,   /* DMA 启动失败致命；CS 下降沿 DMA 未 armed 也报此码（非致命路径） */
    SCREEN_E03_SPI_FRAME  = 3u,   /* 帧头帧尾错、短帧、index 越界或两 bin 不一致，非致命 */
    SCREEN_E04_SPI_NAN    = 4u,   /* 已弃用（定点帧无浮点），保留编号 */
    SCREEN_E05_SPI_LEN    = 5u,   /* 已弃用（不再解析 float），保留编号 */
    SCREEN_E06_SPI_QFULL  = 6u,   /* 接收队列满丢点，非致命              */
    SCREEN_E07_CIR_CFG    = 7u,   /* R<=0 或 bin==0，非致命             */
    SCREEN_E08_CIR_NAN_IN = 8u,   /* 输入（含比例换算后）非有限，非致命  */
    SCREEN_E09_CIR_UI_LOW = 9u,   /* |Ui| 过小，非致命                   */
    SCREEN_E10_CIR_UO_LOW = 10u,  /* |Uo| 过小，非致命                   */
    SCREEN_E11_CIR_H_NAN  = 11u,  /* H 非有限，非致命                    */
    SCREEN_E12_CIR_OUT_NAN= 12u,  /* 频响输出非有限，非致命              */
    SCREEN_E13_CIR_H_LOW  = 13u,  /* |H| 过小无法算 Z，非致命            */
    SCREEN_E14_CIR_ZY_NAN = 14u,  /* Z/Y 非有限，非致命                  */
    SCREEN_E15_UART_TX    = 15u,  /* Transmit_IT 返回非 OK，非致命       */
    SCREEN_E16_UART_BUSY  = 16u,  /* 上一条未发完本周期跳过，非致命      */
    SCREEN_E17_BIN_MISMATCH = 17u, /* 两通道峰值 bin 不一致，非致命       */
    SCREEN_ERROR_CODE_MAX = 17u
} Screen_ErrorCode_t;

/* 复位发送状态并记录起始时间；在 MX_UART4_Init() 之后调用。 */
void Screen_Init(void);

/* 保存最近一个有效频点的计算结果，下次刷新时显示。 */
void Screen_SetResult(const FRF_Point_t *result);

/* 记录当前运行阶段（如 "BOOT"、"LOOP"），不立即发送。 */
void Screen_SetStage(const char *stage);

/* 非致命错误上报：只记录计数，中断安全。 */
void Screen_ReportError(uint8_t code);

/* 致命错误：锁定 t4 显示并进入 Error_Handler()。 */
void Screen_Fatal(uint8_t code);

/* 由 Error_Handler() 在关中断后调用：尝试阻塞发送致命码，然后返回。 */
void Screen_FatalReportBlocking(void);

/* 在主循环中反复调用；每 500 ms 非阻塞刷新全部 5 个文本框。 */
void Screen_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_H */
