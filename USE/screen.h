#ifndef SCREEN_H
#define SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "measure.h"

/* 串口屏刷新周期。 */
#define SCREEN_REFRESH_PERIOD_MS     500u

/*
 * 错误码，说明见 README.md 第 7.2 节。0 保留表示"无错误"。
 *
 * E01 是默认致命码：CubeMX 生成的 MX_*_Init() 内部直接调用
 * Error_Handler()，无法区分具体外设，统一显示 E01 HAL_INIT。
 */
typedef enum
{
    SCREEN_E01_HAL_INIT       = 1u,  /* MX_*_Init 内 Error_Handler，致命，默认码 */
    SCREEN_E02_CIR_CFG        = 2u,  /* R<=0、频率为 0 或窗口为空，非致命        */
    SCREEN_E03_CIR_NAN_IN     = 3u,  /* 输入（含比例换算后）非有限，非致命       */
    SCREEN_E04_CIR_UI_LOW     = 4u,  /* |Ui| 过小，非致命                        */
    SCREEN_E05_CIR_UO_LOW     = 5u,  /* |Uo| 过小，非致命                        */
    SCREEN_E06_CIR_H_NAN      = 6u,  /* H 非有限，非致命                         */
    SCREEN_E07_CIR_OUT_NAN    = 7u,  /* 频响输出非有限，非致命                   */
    SCREEN_E08_CIR_H_LOW      = 8u,  /* |H| 过小无法算 Z，非致命                 */
    SCREEN_E09_CIR_ZY_NAN     = 9u,  /* Z/Y 非有限，非致命                       */
    SCREEN_E10_UART_TX        = 10u, /* Transmit_IT 返回非 OK，非致命            */
    SCREEN_E11_UART_BUSY      = 11u, /* 上一条未发完本周期跳过，非致命           */
    SCREEN_E12_WINDOW_TIMEOUT = 12u, /* 等待 ADC 窗口完成超时，非致命            */
    SCREEN_E13_AD9833_SPI     = 13u, /* AD9833 经 SPI1 写寄存器失败，非致命      */
    SCREEN_E14_CIR_UI_INCOH   = 14u, /* Ui 有交流分量但不是单频，信号源未接/未输出 */
    SCREEN_E15_CIR_UO_INCOH   = 15u, /* Uo 有交流分量但不是单频                  */
    SCREEN_E16_ADC_DUAL_MODE  = 16u, /* 双 ADC 同步配置未生效(CCR 回读不符),致命 */
    SCREEN_E17_ADC_OVR        = 17u, /* ADC 溢出(OVR),非致命                    */
    SCREEN_E18_ADC_DMA        = 18u, /* ADC 的 DMA 传输错误,非致命               */
    SCREEN_ERROR_CODE_MAX     = 18u
} Screen_ErrorCode_t;

/* 复位发送状态并记录起始时间；在 MX_UART4_Init() 之后调用。 */
void Screen_Init(void);

/* 保存最近一个有效频点的计算结果，下次刷新时显示。 */
void Screen_SetResult(const FRF_Point_t *result);

/* 保存最近一个窗口的诊断量（两路交流有效值、两路相干性），与计算结果是否有效无关：
 * 计算失败时（正是"没有信号"的情况）t7、t8 仍能显示电压，否则屏幕上只剩一个错误码。
 * 电压单位 V（去直流后的交流有效值），相干性是无量纲的 0~1，1.0 表示纯单频。 */
void Screen_SetDebug(float ui_volt, float uo_volt, float ui_coherence, float uo_coherence);

/* 清除已保存的结果,下一次刷新时 t0~t4 显示 "-"(t7、t8 是窗口诊断量,不受影响)。
 * 用于失败/超时路径,避免显示陈旧读数。 */
void Screen_ClearResult(void);

/* 记录当前运行阶段（如 "BOOT"、"LOOP"），不立即发送。 */
void Screen_SetStage(const char *stage);

/* 非致命错误上报：只记录计数，中断安全。 */
void Screen_ReportError(uint8_t code);

/* 致命错误：锁定 t5 显示并进入 Error_Handler()。 */
void Screen_Fatal(uint8_t code);

/* 由 Error_Handler() 在关中断后调用：尝试阻塞发送致命码，然后返回。 */
void Screen_FatalReportBlocking(void);

/* 在主循环中反复调用;每 500 ms 非阻塞刷新全部 9 个文本框(t0~t8)。 */
void Screen_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_H */
