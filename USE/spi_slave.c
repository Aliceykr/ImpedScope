#include "spi_slave.h"
#include "spi.h"
#include "uart_debug.h"

#include <string.h>

/* ==========================================================================
 * 内部状态（§5.2）
 * ========================================================================== */

/* DMA 目标缓冲：4 帧 × 10 字 = 40 个半字 = 80 字节。
 * 必须是 32 位对齐的静态数组，DMA 直接写这里（§11.6）。 */
static uint16_t __attribute__((aligned(4))) s_dma_buf[SPI_SLAVE_FRAME_WORDS * SPI_SLAVE_FRAME_SLOTS];

static volatile uint16_t s_valid_seq;       /* 最近一帧的 seq（中断写、主循环读） */
static volatile uint8_t  s_have_new;        /* 有新帧 */
static volatile uint32_t s_last_valid_tick;
static volatile uint32_t s_valid_count;
static volatile uint32_t s_error_count;
static volatile uint32_t s_overrun_count;
static Spi_Frame_t       s_latest;          /* 最近一帧解析结果 */

/* 重同步状态 */
static volatile uint8_t  s_consecutive_fail; /* 连续失败槽计数 */

/* 统计快照（用于计算 gap） */
static volatile uint16_t s_last_valid_seq_snapshot;

/* ==========================================================================
 * 调试输出状态（§13.4）
 * ========================================================================== */

#if SPI_SLAVE_DEBUG_LEVEL >= 1
static uint32_t s_stat_last_tick;
static uint32_t s_err_last_tick[5];        /* 各类错误上次打印时刻 */
static uint32_t s_last_alive_print_tick;
static uint8_t  s_alive_was_dead;
#endif

#if SPI_SLAVE_DEBUG_LEVEL >= 2
static uint32_t s_rx_last_print_tick;
static uint32_t s_rx_frames_printed;
#endif

#if SPI_SLAVE_DEBUG_LEVEL >= 3
static uint32_t s_raw_last_print_tick;
static uint32_t s_raw_frames_printed;
#endif

/* ==========================================================================
 * CRC-16/CCITT-FALSE
 * 初值 0xFFFF，多项式 0x1021，覆盖 word0..word8（§4.2）
 * ========================================================================== */

static uint16_t CRC16_CCITT_False(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);

        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x8000u) != 0u)
            {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

/* ==========================================================================
 * 帧解析（§5.3）
 * ========================================================================== */

static uint8_t Spi_SlaveParseSlot(const uint16_t *w)
{
    uint8_t result = 0u;

    /* 校验帧头 */
    if (w[0] != SPI_SLAVE_HEAD_MAGIC)
    {
#if SPI_SLAVE_DEBUG_LEVEL >= 1
        /* SPIERR head w0=XXXX slot=N（§13.3） */
        uint32_t now = HAL_GetTick();
        if ((now - s_err_last_tick[0]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[0] = now;
            UartDebug_Printf("SPIERR head w0=%04X slot=%u\r\n", w[0], s_consecutive_fail);
        }
#endif
        s_error_count++;
        s_consecutive_fail++;
        return 0u;
    }

    /* 校验 CRC（word9，覆盖 word0..word8） */
    uint16_t crc_calc = CRC16_CCITT_False((const uint8_t *)w, SPI_SLAVE_FRAME_WORDS * 2u - 2u);
    if (crc_calc != w[SPI_SLAVE_FRAME_WORDS - 1u])
    {
#if SPI_SLAVE_DEBUG_LEVEL >= 1
        /* SPIERR crc seq=XXXX got=XXXX exp=XXXX（§13.3） */
        uint32_t now = HAL_GetTick();
        if ((now - s_err_last_tick[1]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[1] = now;
            UartDebug_Printf("SPIERR crc seq=%u got=%04X exp=%04X\r\n",
                             w[1], w[9], crc_calc);
        }
#endif
        s_error_count++;
        s_consecutive_fail++;
        return 0u;
    }

    /* 帧可信 */
    s_consecutive_fail = 0u;

    /* 填充 s_latest */
    s_latest.seq = w[1];
    s_latest.flags = w[2];
    s_latest.frequency_hz = ((uint32_t)w[3] << 16) | w[4];
    uint32_t mag_val = ((uint32_t)w[5] << 16) | w[6];
    int32_t phi_val = (int32_t)(((uint32_t)w[7] << 16) | w[8]);

    s_latest.h_mag = (float)mag_val / 1e6f;
    s_latest.h_phase_deg = (float)phi_val / 1000.0f;
    s_latest.tick = HAL_GetTick();

    /* flags.bit0 == 1 才认为数据可用 */
    if ((s_latest.flags & SPI_SLAVE_FLAG_VALID) != 0u)
    {
        s_valid_seq = s_latest.seq;
        s_last_valid_tick = s_latest.tick;
        s_valid_count++;
        s_have_new = 1u;
        result = 1u;

#if SPI_SLAVE_DEBUG_LEVEL >= 2
        /* SPIRX seq=XXX fl=X f=XXXX mag=XXXXXXXXX phi=XXXX（§13.3） */
        uint32_t now = HAL_GetTick();
        if ((now - s_rx_last_print_tick) >= SPI_SLAVE_DEBUG_RX_PERIOD_MS)
        {
            if (s_rx_frames_printed < SPI_SLAVE_DEBUG_FRAMES)
            {
                s_rx_last_print_tick = now;
                s_rx_frames_printed++;
                UartDebug_Printf("SPIRX seq=%u fl=%u f=%u mag=%u phi=%d\r\n",
                                 s_latest.seq, s_latest.flags, s_latest.frequency_hz,
                                 mag_val, phi_val);
            }
        }
#endif

#if SPI_SLAVE_DEBUG_LEVEL >= 3
        /* SPIRAW seq=XXX A55A XXXX ...（§13.3） */
        uint32_t now = HAL_GetTick();
        if ((now - s_raw_last_print_tick) >= SPI_SLAVE_DEBUG_RAW_PERIOD_MS)
        {
            if (s_raw_frames_printed < SPI_SLAVE_DEBUG_RAW_FRAMES)
            {
                s_raw_last_print_tick = now;
                s_raw_frames_printed++;
                UartDebug_Printf("SPIRAW seq=%u", s_latest.seq);
                for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_WORDS; i++)
                {
                    UartDebug_Printf(" %04X", w[i]);
                }
                UartDebug_Printf("\r\n");
            }
        }
#endif
    }
    else
    {
#if SPI_SLAVE_DEBUG_LEVEL >= 1
        /* SPIERR nvalid seq=XXX fl=X（§13.3） */
        uint32_t now = HAL_GetTick();
        if ((now - s_err_last_tick[2]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[2] = now;
            UartDebug_Printf("SPIERR nvalid seq=%u fl=%u\r\n",
                             s_latest.seq, s_latest.flags);
        }
#endif
        /* 只更新存活时间戳，不更新数值 */
        s_last_valid_tick = s_latest.tick;
    }

    return result;
}

/* ==========================================================================
 * DMA 回调（§5.3）—— 不要在回调里调用 UartDebug_Printf（§11.8）
 * ========================================================================== */

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        /* 处理前半缓冲（2 帧） */
        for (uint8_t slot = 0u; slot < (SPI_SLAVE_FRAME_SLOTS / 2u); slot++)
        {
            const uint16_t *w = &s_dma_buf[slot * SPI_SLAVE_FRAME_WORDS];
            Spi_SlaveParseSlot(w);
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        /* 处理后半缓冲（2 帧） */
        for (uint8_t slot = 0u; slot < (SPI_SLAVE_FRAME_SLOTS / 2u); slot++)
        {
            const uint16_t *w = &s_dma_buf[(SPI_SLAVE_FRAME_SLOTS / 2u + slot) * SPI_SLAVE_FRAME_WORDS];
            Spi_SlaveParseSlot(w);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        s_overrun_count++;

#if SPI_SLAVE_DEBUG_LEVEL >= 1
        /* SPIERR ovr n=X（§13.3） */
        uint32_t now = HAL_GetTick();
        if ((now - s_err_last_tick[3]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[3] = now;
            UartDebug_Printf("SPIERR ovr n=%u\r\n", s_overrun_count);
        }
#endif

        /* 从机在 OVR/MODF 后会停，尝试重新启动 DMA */
        (void)HAL_SPI_Receive_DMA(hspi, (uint8_t *)s_dma_buf,
                                  SPI_SLAVE_FRAME_WORDS * SPI_SLAVE_FRAME_SLOTS);
    }
}

/* ==========================================================================
 * 对外接口实现
 * ========================================================================== */

void Spi_SlaveInit(void)
{
#if SPI_SLAVE_DEBUG_LEVEL >= 1
    UartDebug_Printf("SPIINIT mode=slave w=%u crc=ccitt lvl=%u\r\n",
                     SPI_SLAVE_FRAME_WORDS, SPI_SLAVE_DEBUG_LEVEL);
#endif

    /* 启动 DMA 循环接收 */
    (void)HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)s_dma_buf,
                              SPI_SLAVE_FRAME_WORDS * SPI_SLAVE_FRAME_SLOTS);
}

uint8_t Spi_SlavePoll(Spi_Frame_t *out)
{
    uint8_t result = 0u;

    /* 读临界区，避免撕裂（§8） */
    __disable_irq();
    if (s_have_new != 0u)
    {
        *out = s_latest;
        result = 1u;
        s_have_new = 0u;
    }
    __enable_irq();

    /* 链路超时检测（§5.4） */
#if SPI_SLAVE_DEBUG_LEVEL >= 1
    uint32_t now = HAL_GetTick();

    /* 每秒统计（§13.3） */
    if ((now - s_stat_last_tick) >= SPI_SLAVE_STAT_PERIOD_MS)
    {
        s_stat_last_tick = now;
        uint32_t ok = Spi_SlaveValidCount();
        uint32_t err = Spi_SlaveErrorCount();
        uint32_t ovr = Spi_SlaveOverrunCount();
        uint16_t drop = UartDebug_DropCount();
        int32_t gap = (int32_t)s_valid_seq - (int32_t)s_last_valid_seq_snapshot;
        s_last_valid_seq_snapshot = s_valid_seq;

        UartDebug_Printf("SPISTAT ok=%lu err=%lu ovr=%lu drop=%u seq=%lu gap=%ld\r\n",
                         ok, err, ovr, drop, s_valid_seq, gap);
    }

    /* 重同步（§5.4） */
    if (s_consecutive_fail >= SPI_SLAVE_FRAME_SLOTS)
    {
        UartDebug_Printf("SPIERR resync fail=%u\r\n", s_consecutive_fail);
        s_consecutive_fail = 0u;
        HAL_SPI_Abort_Receive(&hspi1);
        (void)HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)s_dma_buf,
                                  SPI_SLAVE_FRAME_WORDS * SPI_SLAVE_FRAME_SLOTS);
    }

    /* 链路存活检测 */
    uint32_t age = now - s_last_valid_tick;
    if (age >= SPI_SLAVE_ALIVE_MS)
    {
        /* 链路断了 */
        if (s_alive_was_dead == 0u)
        {
            s_alive_was_dead = 1u;
            /* 每秒 1 条 SPIERR alive age=X（§13.3） */
        }
        if ((now - s_last_alive_print_tick) >= SPI_SLAVE_STAT_PERIOD_MS)
        {
            s_last_alive_print_tick = now;
            UartDebug_Printf("SPIERR alive age=%lu\r\n", age);
        }
    }
    else
    {
        /* 链路恢复 */
        if (s_alive_was_dead != 0u)
        {
            s_alive_was_dead = 0u;
            UartDebug_Printf("SPIERR alive ok\r\n");
        }
    }
#endif

    return result;
}

uint8_t Spi_SlaveIsAlive(void)
{
    uint32_t age = HAL_GetTick() - s_last_valid_tick;
    return (age < SPI_SLAVE_ALIVE_MS) ? 1u : 0u;
}

uint32_t Spi_SlaveValidCount(void)
{
    return s_valid_count;
}

uint32_t Spi_SlaveErrorCount(void)
{
    return s_error_count;
}

uint32_t Spi_SlaveOverrunCount(void)
{
    return s_overrun_count;
}
