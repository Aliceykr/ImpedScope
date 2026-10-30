#ifndef SCREEN_H
#define SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "measure.h"   /* Screen_SetResult 使用 FRF_Point_t 类型 */
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
    SCREEN_E13_AD9833_SPI     = 13u, /* 已停用:固件不再驱动 DDS,保留编号占位      */
    SCREEN_E14_CIR_UI_INCOH   = 14u, /* Ui 有交流分量但不是单频，信号源未接/未输出 */
    SCREEN_E15_CIR_UO_INCOH   = 15u, /* Uo 有交流分量但不是单频                  */
    SCREEN_E16_ADC_DUAL_MODE  = 16u, /* 双 ADC 同步配置未生效(CCR 回读不符),致命 */
    SCREEN_E17_ADC_OVR        = 17u, /* ADC 溢出(OVR),非致命                    */
    SCREEN_E18_ADC_DMA        = 18u, /* ADC 的 DMA 传输错误,非致命               */
    SCREEN_E19_ADC_CLIP_UI    = 19u, /* Ui 波形贴 ADC 量程边(削顶),结果不可信  */
    SCREEN_E20_ADC_CLIP_UO    = 20u, /* Uo 波形贴 ADC 量程边(削顶),结果不可信  */
    SCREEN_E21_SPI_LINK       = 21u, /* SPI 从机超过 SPI_SLAVE_ALIVE_MS 没有有效帧 */
    SCREEN_ERROR_CODE_MAX     = 21u
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

/* 清除最近一次非致命错误，使 t5 回到 stage。每个新窗口开始计算前调用；
 * 若本窗口又出错，Screen_ReportError() 会重新置位。 */
void Screen_ClearError(void);

/* 致命错误：锁定 t5 显示并进入 Error_Handler()。 */
void Screen_Fatal(uint8_t code);

/* 由 Error_Handler() 在关中断后调用：尝试阻塞发送致命码，然后返回。 */
void Screen_FatalReportBlocking(void);

/* 在主循环中反复调用;每 500 ms 分条刷新 t0~t9,每条之间留 2 ms 给 HMI 重绘。
 * t10 仍随整帧刷新,切档时由 Screen_FlushRange() 额外立即补发一次。 */
void Screen_Poll(void);

/* 按键事件：一次调用最多返回一个。b2 也会入队，由调用方决定忽略。 */
typedef enum
{
    SCREEN_KEY_NONE = 0u,
    SCREEN_KEY_B0,     /* 单击：强制重扫频率 */
    SCREEN_KEY_B1,     /* 单击：切换继电器档位 */
    SCREEN_KEY_B2      /* 预留：静默忽略 */
} Screen_KeyEvent_t;

/* 屏幕按键 b0/b1/b2：单击即生效，b0 重扫、b1 切档、b2 无动作。
 * 主循环每轮调用一次。中断只负责把完整帧入队，主循环逐次出队，
 * 所以主循环被 DFT 窗口阻塞 1 s 时按键不会丢。读即消费。 */
Screen_KeyEvent_t Screen_PollKey(void);

/* 窥视队列中的下一个按键（不消费）。用于 ADC 窗口等待期间检测 B1。 */
Screen_KeyEvent_t Screen_PeekKey(void);

/* 立即补发一条 t10（档位），不等 500 ms 整帧刷新。 */
void Screen_FlushRange(void);

/* 把 t6 心跳清零；下一次刷新显示 beat:0，之后从 0 继续递增。 */
void Screen_ResetBeat(void);

/* 请求复位 beat（事件到达时调用，由 Screen_Poll 应用）。 */
void Screen_RequestBeatReset(void);

/* 检查是否有待处理的 b0 重扫请求（非破坏性读取）。 */
uint8_t Screen_IsB0Pending(void);

/* 清除 b0 合并标志；正常出队时由 Screen_PollKey() 自动清除。 */
void Screen_ClearB0Pending(void);

/* 按键服务：处理 b0/b1/b2 按键，设置相应的请求标志 */
void Screen_KeyService(void);

/* 自动扫描选定频率后调用,只更新当前测量频率、清掉上一频率结果并强制刷新；
 * t10 显示反馈电阻档位,与本函数无关。 */
uint8_t Screen_SetActiveFreq(uint32_t frequency_hz);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_H */
