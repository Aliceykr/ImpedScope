#include "spi_slave.h"
#include "spi.h"
#include "uart_debug.h"

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
 * 中断上下文事件（避免在中断里打印，§11.8）
 * ========================================================================== */

/* 错误事件标志（位掩码，各自独立，不会被新帧覆盖） */
#define SPI_ERR_HEAD    (1u << 0)
#define SPI_ERR_CRC     (1u << 1)
#define SPI_ERR_NVALID  (1u << 2)
#define SPI_ERR_OVR     (1u << 3)

static volatile uint16_t s_err_flags;       /* 错误标志位掩码 */
static volatile uint16_t s_err_head_w0;     /* head 错误数据 */
static volatile uint16_t s_err_head_slot;
static volatile uint16_t s_err_crc_seq;     /* crc 错误数据 */
static volatile uint16_t s_err_crc_got;
static volatile uint16_t s_err_crc_exp;
static volatile uint16_t s_err_nvalid_seq;  /* nvalid 错误数据 */
static volatile uint16_t s_err_nvalid_fl;
static volatile uint16_t s_err_ovr_n;       /* ovr 错误数据 */

/* 新帧标志（独立于错误标志，不会互相覆盖） */
static volatile uint8_t  s_new_frame_flag;
static volatile uint16_t s_new_frame_seq;
static volatile uint16_t s_new_frame_fl;
static volatile uint32_t s_new_frame_freq;
static volatile uint32_t s_new_frame_mag;
static volatile int32_t  s_new_frame_phi;

/* SPIRAW 快照（10 个字，20 字节） */
static volatile uint16_t s_raw_snapshot[SPI_SLAVE_FRAME_WORDS];
static volatile uint8_t  s_raw_ready;

/* ==========================================================================
 * 调试输出状态（§13.4）
 * ========================================================================== */

#if SPI_SLAVE_DEBUG_LEVEL >= 1
static uint32_t s_stat_last_tick;
static uint32_t s_err_last_tick[4];        /* 各类错误上次打印时刻 */
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
 *
 * 注意：SPI 线上是 MSB first，但 Cortex-M7 是小端，内存里 0xA55A 是 5A A5。
 * 所以固件算 CRC 时必须按"线上字节顺序"（大端）喂数据，而不是内存序（小端）。
 * 这里先把每个 16 位字转成大端字节序，再算 CRC。
 * ========================================================================== */

static uint16_t CRC16_CCITT_False_BE(const uint16_t *words, uint32_t word_count)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0u; i < word_count; i++)
    {
        uint8_t byte_hi = (uint8_t)((words[i] >> 8) & 0xFF);
        uint8_t byte_lo = (uint8_t)(words[i] & 0xFF);

        /* 处理高字节 */
        crc ^= (uint16_t)((uint16_t)byte_hi << 8);
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

        /* 处理低字节 */
        crc ^= (uint16_t)((uint16_t)byte_lo << 8);
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
 * 帧解析（§5.3）—— 只在中断里调用，不做任何打印
 * ========================================================================== */

static uint8_t Spi_SlaveParseSlot(const uint16_t *w)
{
    uint8_t result = 0u;

    /* 校验帧头 */
    if (w[0] != SPI_SLAVE_HEAD_MAGIC)
    {
        s_error_count++;
        s_consecutive_fail++;

        /* 记录 head 错误（独立槽位，不会被新帧覆盖） */
        s_err_flags |= SPI_ERR_HEAD;
        s_err_head_w0 = w[0];
        s_err_head_slot = (uint16_t)s_consecutive_fail;
        return 0u;
    }

    /* 校验 CRC（word9，覆盖 word0..word8）
     * 注意：用大端字节序计算，与 FPGA 线上发送顺序一致 */
    uint16_t crc_calc = CRC16_CCITT_False_BE(w, SPI_SLAVE_FRAME_WORDS - 1u);
    if (crc_calc != w[SPI_SLAVE_FRAME_WORDS - 1u])
    {
        s_error_count++;
        s_consecutive_fail++;

        /* 记录 crc 错误（独立槽位） */
        s_err_flags |= SPI_ERR_CRC;
        s_err_crc_seq = w[1];
        s_err_crc_got = w[9];
        s_err_crc_exp = crc_calc;
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

        /* 记录新帧事件（所有数据放进事件槽位，避免与 s_latest 不同步） */
        s_new_frame_flag = 1u;
        s_new_frame_seq = s_latest.seq;
        s_new_frame_fl = s_latest.flags;
        s_new_frame_freq = s_latest.frequency_hz;
        s_new_frame_mag = mag_val;
        s_new_frame_phi = phi_val;

        /* 保存原始字（供 SPIRAW 使用） */
        for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_WORDS; i++)
        {
            s_raw_snapshot[i] = w[i];
        }
        s_raw_ready = 1u;
    }
    else
    {
        /* 只更新存活时间戳，不更新数值 */
        s_last_valid_tick = s_latest.tick;

        /* 记录 nvalid 错误（独立槽位） */
        s_err_flags |= SPI_ERR_NVALID;
        s_err_nvalid_seq = s_latest.seq;
        s_err_nvalid_fl = s_latest.flags;
    }

    return result;
}

/* ==========================================================================
 * DMA 回调（§5.3）—— 中断上下文，只做解析与计数，不打印（§11.8）
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

        /* 记录 ovr 错误（独立槽位） */
        s_err_flags |= SPI_ERR_OVR;
        s_err_ovr_n = (uint16_t)s_overrun_count;

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
    if (HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)s_dma_buf,
                            SPI_SLAVE_FRAME_WORDS * SPI_SLAVE_FRAME_SLOTS) != HAL_OK)
    {
        /* 启动失败，记录错误（但不影响 DFT 测量） */
        s_error_count++;
#if SPI_SLAVE_DEBUG_LEVEL >= 1
        UartDebug_Printf("SPIINIT failed\r\n");
#endif
    }
}

uint8_t Spi_SlavePoll(Spi_Frame_t *out)
{
    uint8_t result = 0u;
    uint32_t primask;

    /* 读临界区，避免撕裂（§8）
     * 保存恢复 PRIMASK，避免被关中断上下文调用时强行开中断 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_have_new != 0u)
    {
        if (out != NULL)
        {
            *out = s_latest;
            result = 1u;
        }
        s_have_new = 0u;
    }
    __set_PRIMASK(primask);

#if SPI_SLAVE_DEBUG_LEVEL >= 1
    uint32_t now = HAL_GetTick();

    /* 在临界区里一次性拷贝所有需要的数据（避免读到不一致的状态） */
    primask = __get_PRIMASK();
    __disable_irq();
    uint16_t err_flags = s_err_flags;
    uint8_t new_frame = s_new_frame_flag;
    s_new_frame_flag = 0u;  /* 在读数据的同一临界区里清标志，避免丢帧 */
#if SPI_SLAVE_DEBUG_LEVEL >= 2
    uint16_t new_seq = s_new_frame_seq;
    uint16_t new_fl = s_new_frame_fl;
    uint32_t new_freq = s_new_frame_freq;
    uint32_t new_mag = s_new_frame_mag;
    int32_t  new_phi = s_new_frame_phi;
#endif
    uint8_t raw_ready = s_raw_ready;
    uint16_t raw_snapshot[SPI_SLAVE_FRAME_WORDS];
    for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_WORDS; i++)
    {
        raw_snapshot[i] = s_raw_snapshot[i];
    }
    __set_PRIMASK(primask);

    /* 打印错误（各自独立，不会互相覆盖）
     * 只在真正打印后才清对应的 bit，避免节流窗口内的错误被吞 */
    if ((err_flags & SPI_ERR_HEAD) != 0u)
    {
        /* SPIERR head w0=XXXX slot=N（§13.3） */
        if ((now - s_err_last_tick[0]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[0] = now;
            UartDebug_Printf("SPIERR head w0=%04X slot=%u\r\n",
                             s_err_head_w0, s_err_head_slot);

            /* 只在打印成功后才清标志 */
            primask = __get_PRIMASK();
            __disable_irq();
            s_err_flags &= (uint16_t)~SPI_ERR_HEAD;
            __set_PRIMASK(primask);
        }
    }

    if ((err_flags & SPI_ERR_CRC) != 0u)
    {
        /* SPIERR crc seq=XXX got=XXXX exp=XXXX（§13.3） */
        if ((now - s_err_last_tick[1]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[1] = now;
            UartDebug_Printf("SPIERR crc seq=%u got=%04X exp=%04X\r\n",
                             s_err_crc_seq, s_err_crc_got, s_err_crc_exp);

            /* 只在打印成功后才清标志 */
            primask = __get_PRIMASK();
            __disable_irq();
            s_err_flags &= (uint16_t)~SPI_ERR_CRC;
            __set_PRIMASK(primask);
        }
    }

    if ((err_flags & SPI_ERR_NVALID) != 0u)
    {
        /* SPIERR nvalid seq=XXX fl=X（§13.3） */
        if ((now - s_err_last_tick[2]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[2] = now;
            UartDebug_Printf("SPIERR nvalid seq=%u fl=%u\r\n",
                             s_err_nvalid_seq, s_err_nvalid_fl);

            /* 只在打印成功后才清标志 */
            primask = __get_PRIMASK();
            __disable_irq();
            s_err_flags &= (uint16_t)~SPI_ERR_NVALID;
            __set_PRIMASK(primask);
        }
    }

    if ((err_flags & SPI_ERR_OVR) != 0u)
    {
        /* SPIERR ovr n=X（§13.3） */
        if ((now - s_err_last_tick[3]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[3] = now;
            UartDebug_Printf("SPIERR ovr n=%u\r\n", s_err_ovr_n);

            /* 只在打印成功后才清标志 */
            primask = __get_PRIMASK();
            __disable_irq();
            s_err_flags &= (uint16_t)~SPI_ERR_OVR;
            __set_PRIMASK(primask);
        }
    }

    /* 打印新帧（SPIRX/SPIRAW，所有数据来自临界区拷贝的局部变量，保证一致性） */
    if ((new_frame != 0u) && (raw_ready != 0u))
    {
#if SPI_SLAVE_DEBUG_LEVEL >= 2
        /* SPIRX seq=XXX fl=X f=XXXX mag=XXXXXXXXX phi=XXXX（§13.3） */
        if ((now - s_rx_last_print_tick) >= SPI_SLAVE_DEBUG_RX_PERIOD_MS)
        {
            if (s_rx_frames_printed < SPI_SLAVE_DEBUG_FRAMES)
            {
                s_rx_last_print_tick = now;
                s_rx_frames_printed++;
                UartDebug_Printf("SPIRX seq=%u fl=%u f=%lu mag=%lu phi=%ld\r\n",
                                 new_seq, new_fl,
                                 (unsigned long)new_freq,
                                 (unsigned long)new_mag,
                                 (long)new_phi);
            }
        }
#endif

#if SPI_SLAVE_DEBUG_LEVEL >= 3
        /* SPIRAW seq=XXX A55A XXXX ...（§13.3）
         * 打印完整的 10 个原始字，用于逐字对拍
         * 只在打印成功后才清 raw_ready，避免丢帧 */
        if ((now - s_raw_last_print_tick) >= SPI_SLAVE_DEBUG_RAW_PERIOD_MS)
        {
            if (s_raw_frames_printed < SPI_SLAVE_DEBUG_RAW_FRAMES)
            {
                s_raw_last_print_tick = now;
                s_raw_frames_printed++;
                UartDebug_Printf("SPIRAW seq=%u", new_seq);
                for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_WORDS; i++)
                {
                    UartDebug_Printf(" %04X", raw_snapshot[i]);
                }
                UartDebug_Printf("\r\n");

                /* 只在打印成功后才清 raw_ready */
                primask = __get_PRIMASK();
                __disable_irq();
                s_raw_ready = 0u;
                __set_PRIMASK(primask);
            }
        }
#endif
    }

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

        UartDebug_Printf("SPISTAT ok=%lu err=%lu ovr=%lu drop=%u seq=%u gap=%ld\r\n",
                         ok, err, ovr, drop, s_valid_seq, gap);
    }

    /* 链路存活检测（打印部分在调试宏里，检测逻辑本身不在） */
    uint32_t age = now - s_last_valid_tick;
    if (age >= SPI_SLAVE_ALIVE_MS)
    {
        /* 链路断了 */
        if (s_alive_was_dead == 0u)
        {
            s_alive_was_dead = 1u;
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

    /* 重同步（§5.4）—— 不在调试宏里，量产也需要 */
    if (s_consecutive_fail >= SPI_SLAVE_FRAME_SLOTS)
    {
#if SPI_SLAVE_DEBUG_LEVEL >= 1
        UartDebug_Printf("SPIERR resync fail=%u\r\n", s_consecutive_fail);
#endif
        s_consecutive_fail = 0u;
        HAL_SPI_Abort(&hspi1);
        (void)HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)s_dma_buf,
                                  SPI_SLAVE_FRAME_WORDS * SPI_SLAVE_FRAME_SLOTS);
    }

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
