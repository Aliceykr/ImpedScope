#ifndef SPI_RECEIVER_H
#define SPI_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/*
 * SPI1 接收协议
 *
 * FPGA 是 SPI 主机，STM32 是 SPI 从机。每个频点占用一帧 22 字节：
 * [帧头][频点][Ui 实部][Ui 虚部][Uo 实部][Uo 虚部][帧尾]
 *
 * 五个字段均为 4 字节，浮点字段使用 IEEE-754 单精度格式。协议参数
 * 集中在下面定义，后续与 FPGA 对齐时不需要修改解析流程。
 */

#define SPI_FRAME_START_MARKER       0xAAu
#define SPI_FRAME_END_MARKER         0x55u
#define SPI_FRAME_MARKER_BYTES       1u
#define SPI_FRAME_FIELD_BYTES        4u
#define SPI_FRAME_FIELD_COUNT        5u
#define SPI_FRAME_PAYLOAD_BYTES      (SPI_FRAME_FIELD_BYTES * SPI_FRAME_FIELD_COUNT)
#define SPI_FRAME_TOTAL_BYTES        (SPI_FRAME_MARKER_BYTES + \
                                      SPI_FRAME_PAYLOAD_BYTES + \
                                      SPI_FRAME_MARKER_BYTES)
#define SPI_DMA_BUFFER_BYTES         (SPI_FRAME_TOTAL_BYTES * 2u)
#define SPI_POINT_QUEUE_SIZE         4u

/* 1：大端（先接收最高字节）；0：小端（先接收最低字节）。 */
#define SPI_FRAME_BIG_ENDIAN         1u

/* 一帧解析后的频点数据。 */
typedef struct
{
    uint32_t freq;
    float    ui_re;
    float    ui_im;
    float    uo_re;
    float    uo_im;
} FFT_Point_t;

/* 启动 SPI1 从机 DMA 循环接收；成功返回 1，失败返回 0。 */
uint8_t SPI_Receiver_Init(void);

/* 校验并解析一帧原始数据；成功返回 1，失败返回 0。 */
uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out);

/* 从接收队列取出一个完整频点；队列为空时返回 0。 */
uint8_t SPI_Receiver_TryGetPoint(FFT_Point_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SPI_RECEIVER_H */
