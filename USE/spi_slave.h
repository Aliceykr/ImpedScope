#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==========================================================================
 * SPI 从机接收 FPGA 数据驱动
 * ========================================================================== */

/* 模块总开关。
 *   1 = 正常接收 FPGA 帧并与 DFT 比对（默认）
 *   0 = 整个模块编译成空实现：不启动 DMA、不解析、不打印，
 *       Spi_SlavePoll() 恒返回 0，Spi_SlaveIsAlive() 恒返回 1（不报 E21）。
 * 没接 FPGA / 不想看到 E21 时改成 0 即可，measure.c 和 uart_debug.c 无需改动。 */
#define SPI_SLAVE_ENABLE            1u

/* 帧格式常量（与 PLAN-SPI从机与双路并行.md §4 保持一致） */
#define SPI_SLAVE_FRAME_WORDS       10u    /* 每帧 16 位字数 */
#define SPI_SLAVE_FRAME_SLOTS       4u     /* DMA 循环缓冲能放几帧 */
#define SPI_SLAVE_ALIVE_MS          200u   /* 超过这么久没有有效帧 → 认为链路断了 */

#define SPI_SLAVE_HEAD_MAGIC        0xA55Au /* 帧头 */

/* 帧 flags 位定义 */
#define SPI_SLAVE_FLAG_VALID        (1u << 0)   /* 本帧有效 */
#define SPI_SLAVE_FLAG_CLIPPED      (1u << 1)   /* FPGA 侧检测到前端削顶 */
#define SPI_SLAVE_FLAG_FREQ_VALID    (1u << 2)   /* 频率字段有效 */

/* ==========================================================================
 * 调试输出配置（§13）
 * ========================================================================== */

/* 0 = 全关（量产）
 * 1 = 初始化 + 错误 + 每秒统计（默认，稳态约 1 行/秒）
 * 2 = 额外按时间节流打印 SPIRX（联调首选用这个）
 * 3 = 再额外按时间节流 dump SPIRAW 原始字（查字对齐/CS/位序） */
#define SPI_SLAVE_DEBUG_LEVEL       1u

/* 限速：两条都按"最小间隔"节流，与帧率无关 */
#define SPI_SLAVE_DEBUG_RX_PERIOD_MS    100u   /* SPIRX 最快 10 行/秒 ≈ 560 B/s */
#define SPI_SLAVE_DEBUG_RAW_PERIOD_MS   500u   /* SPIRAW 最快 2 行/秒 ≈ 200 B/s */

/* 总量上限：避免长时间开着刷屏（时间节流已经限死带宽，这里再限总行数） */
#define SPI_SLAVE_DEBUG_FRAMES      50u     /* LEVEL>=2 累计最多多少行 SPIRX */
#define SPI_SLAVE_DEBUG_RAW_FRAMES  10u     /* LEVEL>=3 累计最多多少行 SPIRAW */

#define SPI_SLAVE_STAT_PERIOD_MS    1000u   /* 统计行周期 */
#define SPI_SLAVE_ERR_PERIOD_MS     1000u   /* 同类错误最快多久重复一次 */

/* ==========================================================================
 * 帧结构
 * ========================================================================== */
typedef struct
{
    uint16_t seq;            /* 帧序号 */
    uint16_t flags;          /* 标志位 */
    uint32_t frequency_hz;   /* 频率 Hz */
    float    h_mag;          /* 线性幅值 |H| */
    float    h_phase_deg;    /* 度 */
    uint32_t tick;           /* 收到该帧时的 HAL_GetTick() */
} Spi_Frame_t;

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/* 启动 DMA 循环接收。在 Measure_Init() 里调用一次。 */
void Spi_SlaveInit(void);

/* 取"最新一帧"。有新帧（自上次调用以来 CRC 正确且 flags.bit0=1）时返回 1。
 * 没有新帧返回 0，out 不变。 */
uint8_t Spi_SlavePoll(Spi_Frame_t *out);

/* 链路是否还活着：最近 SPI_SLAVE_ALIVE_MS 内收到过任一 CRC 正确的帧。 */
uint8_t Spi_SlaveIsAlive(void);

/* 统计：有效帧数 / CRC 或帧头错 / DMA 溢出。 */
uint32_t Spi_SlaveValidCount(void);
uint32_t Spi_SlaveErrorCount(void);
uint32_t Spi_SlaveOverrunCount(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_SLAVE_H */
