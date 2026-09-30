#ifndef __SPI_RECEIVER_H
#define __SPI_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* ===========================================================================
 * SPI 从机字节帧接收模块（8-bit）
 * ---------------------------------------------------------------------------
 * 通信角色：FPGA 为 SPI 主机（其 spi_byte 模块按字节发送），STM32 为从机。
 *
 * 一帧数据的字节布局（共 FRAME_TOTAL_BYTES 字节）：
 *   [起始标记 1B][频点 4B][Ui实部 4B][Ui虚部 4B][Uo实部 4B][Uo虚部 4B][结束标记 1B]
 *
 * 注意：标记值、字节序、字段顺序目前均为占位约定，FPGA 协议确定后只需
 *       修改本文件中的宏即可，无需改动 .c 中的逻辑。
 * ===========================================================================*/

/* ---- 帧头 / 帧尾单字节标记（占位，需与 FPGA 对齐）---- */
#define FRAME_START_MARKER      0xAAu   /* 起始标记（帧头）*/
#define FRAME_STOP_MARKER       0x55u   /* 结束标记（帧尾）*/
#define FRAME_START_BYTES       1u      /* 帧头占用字节数 */
#define FRAME_STOP_BYTES        1u      /* 帧尾占用字节数 */

/* ---- 数据字段定义 ---- */
/* 每个字段都是 32-bit（4 字节），共 5 个字段：
   频点、Ui 实部、Ui 虚部、Uo 实部、Uo 虚部 */
#define FRAME_FIELD_BYTES       4u                          /* 单字段字节数 */
#define FRAME_FIELD_COUNT       5u                          /* 字段个数 */
#define FRAME_PAYLOAD_BYTES     (FRAME_FIELD_BYTES * FRAME_FIELD_COUNT) /* 有效载荷字节数 = 20 */
#define FRAME_TOTAL_BYTES       (FRAME_START_BYTES + FRAME_PAYLOAD_BYTES + FRAME_STOP_BYTES) /* 整帧 = 22 */

/* 每个 32-bit 字段的字节序：
   1 = 先收到的是最高有效字节（大端，MSB first）
   0 = 先收到的是最低有效字节（小端，LSB first） */
#define FRAME_BIG_ENDIAN        1u

/* 解析后的“单个频点 FFT 结果” */
typedef struct
{
    uint32_t freq;     /* 频点（频率值或 FFT bin 号，由 FPGA 约定）*/
    float    ui_re;    /* 输入电压 Ui 的 FFT 实部 */
    float    ui_im;    /* 输入电压 Ui 的 FFT 虚部 */
    float    uo_re;    /* 输出电压 Uo 的 FFT 实部 */
    float    uo_im;    /* 输出电压 Uo 的 FFT 虚部 */
} FFT_Point_t;

/* Exported functions --------------------------------------------------------*/
/* 初始化：启动 SPI1 从机 8-bit、DMA 循环接收，每收满一帧触发一次回调 */
void SPI_Receiver_Init(void);

/* 把一帧原始字节解析为 FFT_Point_t。
   帧头/帧尾校验通过返回 1，否则返回 0。 */
uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_RECEIVER_H */
