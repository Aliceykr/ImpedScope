#ifndef SPI_RECEIVER_H
#define SPI_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/*
 * SPI1 接收协议（FPGA 主机，STM32 从机）
 *
 * 帧边界由 CS（PA4，硬件 NSS，低有效）决定，每帧固定 16 字节：
 *   [帧头 2][12 字节记录][帧尾 2]
 * 记录内为 4 个 12 位有符号分量和两个 13 位峰值 bin 序号，全部定点。
 */

/* 帧头：字节 0、1 */
#define SPI_FRAME_HEADER_0           0x07u
#define SPI_FRAME_HEADER_1           0x08u

/* 帧尾：字节 14、15 */
#define SPI_FRAME_TAIL_0             0x00u
#define SPI_FRAME_TAIL_1             0x00u

/* 帧长与字段位置 */
#define SPI_FRAME_TOTAL_BYTES        16u
#define SPI_FRAME_TAIL_OFFSET        14u

/* 载荷内各字段的字节偏移（相对帧首） */
#define SPI_FRAME_CH1_IM_OFFSET      2u   /* 2 字节 */
#define SPI_FRAME_CH1_RE_OFFSET      4u
#define SPI_FRAME_CH0_IM_OFFSET      6u
#define SPI_FRAME_CH0_RE_OFFSET      8u
#define SPI_FRAME_INDEX1_OFFSET      10u
#define SPI_FRAME_INDEX0_OFFSET      12u

/* SPI1 片选（NSS）引脚，暂定 PA4，待 PCB 确认后修改 */
#define SPI_CS_GPIO_PORT             GPIOA
#define SPI_CS_PIN                   GPIO_PIN_4

/* 接收队列深度，满时丢弃最旧记录。 */
#define SPI_RECORD_QUEUE_SIZE        4u

/* 字段有效位宽 */
#define SPI_FRAME_SAMPLE_BITS        12u  /* 分量有效位，高 4 位为填充 */
#define SPI_FRAME_INDEX_BITS         13u  /* 峰值序号位宽 */
#define SPI_FRAME_INDEX_MAX          1023u /* FFT 1024 点，有效取值 0 – 1023 */

/* 一帧解析后的 FPGA 记录。 */
typedef struct
{
    uint16_t index0;   /* CH0 峰值序号，0 – 1023 */
    uint16_t index1; /* CH1 峰值序号，0 – 1023 */
    int16_t  ch0_re;  /* 12 位有符号，已符号扩展 */
    int16_t  ch0_im;
    int16_t  ch1_re;
    int16_t  ch1_im;
} FPGA_Record_t;

/* 启动 SPI1 从机 DMA 接收；成功返回 1，失败返回 0。 */
uint8_t SPI_Receiver_Init(void);

/* 校验并解析一帧 16 字节原始数据；成功返回 1，失败返回 0。 */
uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FPGA_Record_t *out);

/* 从接收队列取出一条记录；队列为空时返回 0。 */
uint8_t SPI_Receiver_TryGetRecord(FPGA_Record_t *out);

/* SPI 收包统计，用于通信测试显示。 */
typedef struct
{
    uint32_t cs_falling;                          /* CS 下降沿次数 */
    uint32_t frame_full;                          /* 收满 16 字节次数 */
    uint32_t frame_ok;                            /* 帧头帧尾校验通过次数 */
    uint8_t  last_frame[SPI_FRAME_TOTAL_BYTES];   /* 最近一次收满的原始帧 */
} SPI_Stats_t;

/* 取统计快照；与中断更新互斥。 */
void SPI_Receiver_GetStats(SPI_Stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SPI_RECEIVER_H */
