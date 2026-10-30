#include "spi_slave.h"
#include "spi.h"
#include "uart_debug.h"

#if (SPI_SLAVE_ENABLE == 0u)

/* ==========================================================================
 * 关闭时：只提供空实现，measure.c / uart_debug.c 不需要任何条件编译
 * ========================================================================== */

void Spi_SlaveInit(void)
{
    /* 不启动 DMA */
}

uint8_t Spi_SlavePoll(Spi_Frame_t *out)
{
    (void)out;
    return 0u;                  /* 永远没有新帧 */
}

uint8_t Spi_SlaveIsAlive(void)
{
    /* 关闭时视为"链路正常"，否则 measure.c 会每个周期都报 E21。
     * 语义上不是"活着"，而是"不要因为没接 FPGA 就报错"。 */
    return 1u;
}

uint32_t Spi_SlaveValidCount(void)
{
    return 0u;
}

uint32_t Spi_SlaveErrorCount(void)
{
    return 0u;
}

uint32_t Spi_SlaveOverrunCount(void)
{
    return 0u;
}

#else   /* SPI_SLAVE_ENABLE != 0 */

/* ==========================================================================
 * 内部状态（§5.2）
 * ========================================================================== */

#define SPI_SLAVE_DMA_BYTES     (SPI_SLAVE_FRAME_BYTES * SPI_SLAVE_FRAME_SLOTS)
#define SPI_SLAVE_HALF_BYTES    (SPI_SLAVE_DMA_BYTES / 2u)

/* DMA 目标缓冲：SPI_SLAVE_FRAME_SLOTS 帧 × 10 字节，循环写入。
 * 必须是 32 位对齐的静态数组，DMA 直接写这里。 */
static uint8_t __attribute__((aligned(4))) s_dma_buf[SPI_SLAVE_DMA_BYTES];

static volatile uint16_t s_valid_seq;       /* 最近一帧的 seq（中断写、主循环读） */
static volatile uint8_t  s_have_new;        /* 有新帧 */
static volatile uint32_t s_last_valid_tick;
static volatile uint32_t s_valid_count;
static volatile uint32_t s_error_count;
static volatile uint32_t s_overrun_count;
static Spi_Frame_t       s_latest;          /* 最近一帧解析结果 */

/* 字节流解析状态（只在 DMA 回调里改；重启 DMA 时在关中断下清零） */
static volatile uint16_t s_rd;               /* 下一个待解析字节在 s_dma_buf 里的下标 */
static volatile uint16_t s_avail;            /* 已写入但还没解析的字节数 */
static volatile uint8_t  s_head_lost;        /* 当前处于"帧头失配"状态（一段只记一次错） */
static uint32_t          s_last_restart_tick;

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

/* SPIRAW 快照（10 字节） */
static volatile uint8_t s_raw_snapshot[SPI_SLAVE_FRAME_BYTES];
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
 * 初值 0xFFFF，多项式 0x1021，覆盖前 9 字节（byte0..byte8）
 * FPGA 按字节流发送，直接按字节序计算
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
 * 字节流重同步解析（只在中断里调用，不做任何打印）
 *
 * FPGA 每帧 10 字节，但 DMA 启动/重启的时刻可能落在帧中间，缓冲里的帧边界
 * 就和 10 字节槽错开（日志里 head w0=1234/5678/9ABC 就是整字节错位）。
 * 所以不按固定槽切帧，而是在字节流里逐字节滑动找帧头：对上就吃掉 10 字节，
 * 对不上就丢 1 字节再试。不再用 Abort + 重启 DMA 来对齐——重启落在帧中间
 * 反而会制造新的错位。
 * ========================================================================== */

static uint8_t Spi_SlaveParseFrame(const uint8_t *f)
{
    uint16_t head = ((uint16_t)f[0] << 8) | f[1];
    uint16_t tail = ((uint16_t)f[8] << 8) | f[9];

    /* 同时校验帧头 (0x0708) 和帧尾 (0x0000) */
    if ((head != SPI_SLAVE_HEAD_MAGIC) || (tail != SPI_SLAVE_TAIL_MAGIC))
    {
        /* 一段连续失配只记一次错误，避免滑动期间把计数刷爆 */
        if (s_head_lost == 0u)
        {
            s_head_lost = 1u;
            s_error_count++;
            s_err_flags |= SPI_ERR_HEAD;
            s_err_head_w0 = head;
            s_err_head_slot = s_rd;
        }
        return 0u;
    }

    s_head_lost = 0u;

    /* top.v 真实测量帧解析：
     * [0..1] HEAD = 0x0708
     * [2..3] PHASE (16-bit clk count)
     * [4..5] FREQ0 (16-bit clk count)
     * [6..7] FREQ1 (16-bit clk count)
     * [8..9] TAIL = 0x0000
     */
    uint16_t phase_raw = ((uint16_t)f[2] << 8) | f[3];
    uint16_t freq0_raw = ((uint16_t)f[4] << 8) | f[5];
    uint16_t freq1_raw = ((uint16_t)f[6] << 8) | f[7];

    /* 内部序号自增 */
    static uint16_t s_internal_seq = 0;
    s_internal_seq++;
    s_latest.seq = s_internal_seq;
    s_latest.phase_cnt = phase_raw;
    s_latest.freq0_cnt = freq0_raw;
    s_latest.freq1_cnt = freq1_raw;

    /* 换算物理量：时钟 50MHz */
    if (freq0_raw > 0u)
    {
        s_latest.frequency_hz = 50000000u / (uint32_t)freq0_raw;
        float p_deg = (float)phase_raw * 360.0f / (float)freq0_raw;
        if (p_deg > 180.0f)
        {
            p_deg -= 360.0f;
        }
        s_latest.h_phase_deg = p_deg;
        s_latest.flags = SPI_SLAVE_FLAG_VALID | SPI_SLAVE_FLAG_FREQ_VALID;
    }
    else
    {
        s_latest.frequency_hz = 0u;
        s_latest.h_phase_deg = 0.0f;
        s_latest.flags = 0u;
    }

    /* 比较器无法测出线性幅值比，赋 1.0f */
    s_latest.h_mag = 1.0f;
    s_latest.tick = HAL_GetTick();

    s_valid_seq = s_latest.seq;
    s_last_valid_tick = s_latest.tick;
    s_valid_count++;
    s_have_new = 1u;

    /* 记录新帧事件（所有数据放进事件槽位，避免与 s_latest 不同步） */
    s_new_frame_flag = 1u;
    s_new_frame_seq = s_latest.seq;
    s_new_frame_fl = s_latest.flags;
    s_new_frame_freq = s_latest.frequency_hz;
    s_new_frame_mag = 1000000u;
    s_new_frame_phi = (int32_t)(s_latest.h_phase_deg * 1000.0f);

    /* 保存原始字节（供 SPIRAW 使用） */
    for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_BYTES; i++)
    {
        s_raw_snapshot[i] = f[i];
    }
    s_raw_ready = 1u;

    return 1u;
}

static void Spi_SlaveScan(void)
{
    uint8_t f[SPI_SLAVE_FRAME_BYTES];

    while (s_avail >= SPI_SLAVE_FRAME_BYTES)
    {
        for (uint16_t i = 0u; i < SPI_SLAVE_FRAME_BYTES; i++)
        {
            f[i] = s_dma_buf[(s_rd + i) % SPI_SLAVE_DMA_BYTES];
        }

        if (Spi_SlaveParseFrame(f) != 0u)
        {
            s_rd = (uint16_t)((s_rd + SPI_SLAVE_FRAME_BYTES) % SPI_SLAVE_DMA_BYTES);
            s_avail = (uint16_t)(s_avail - SPI_SLAVE_FRAME_BYTES);
        }
        else
        {
            s_rd = (uint16_t)((s_rd + 1u) % SPI_SLAVE_DMA_BYTES);
            s_avail = (uint16_t)(s_avail - 1u);
        }
    }
}

/* 清解析状态并启动 DMA 循环接收（DMA 从缓冲区头开始写，所以下标也归零） */
static HAL_StatusTypeDef Spi_StartDma(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_rd = 0u;
    s_avail = 0u;
    s_head_lost = 0u;
    __set_PRIMASK(primask);

    return HAL_SPI_Receive_DMA(&hspi1, s_dma_buf, SPI_SLAVE_DMA_BYTES);
}

/* ==========================================================================
 * DMA 回调 —— 中断上下文，只做解析与计数，不打印
 * ========================================================================== */

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        s_avail = (uint16_t)(s_avail + SPI_SLAVE_HALF_BYTES);
        Spi_SlaveScan();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        s_avail = (uint16_t)(s_avail + SPI_SLAVE_HALF_BYTES);
        Spi_SlaveScan();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        s_overrun_count++;

        /* 记录 ovr 错误 */
        s_err_flags |= SPI_ERR_OVR;
        s_err_ovr_n = (uint16_t)s_overrun_count;

        /* 从机在 OVR/MODF 后会停，尝试重新启动 DMA */
        (void)Spi_StartDma();
    }
}

/* ==========================================================================
 * 对外接口实现
 * ========================================================================== */

void Spi_SlaveInit(void)
{
#if SPI_SLAVE_DEBUG_LEVEL >= 1
    UartDebug_Printf("SPIINIT mode=slave bytes=%u crc=ccitt lvl=%u\r\n",
                     SPI_SLAVE_FRAME_BYTES, SPI_SLAVE_DEBUG_LEVEL);
#endif

    /* 启动 DMA 循环接收 */
    if (Spi_StartDma() != HAL_OK)
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
    if ((s_have_new != 0u) && (out != NULL))
    {
        *out = s_latest;
        result = 1u;
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
    uint8_t raw_snapshot[SPI_SLAVE_FRAME_BYTES];
    for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_BYTES; i++)
    {
        raw_snapshot[i] = s_raw_snapshot[i];
    }
    __set_PRIMASK(primask);

    /* 打印错误（各自独立，不会互相覆盖）
     * 只在真正打印后才清对应的 bit，避免节流窗口内的错误被吞 */
    if ((err_flags & SPI_ERR_HEAD) != 0u)
    {
        /* SPIERR head w0=XXXX pos=N（失配时的缓冲下标） */
        if ((now - s_err_last_tick[0]) >= SPI_SLAVE_ERR_PERIOD_MS)
        {
            s_err_last_tick[0] = now;
            UartDebug_Printf("SPIERR head w0=%04X pos=%u\r\n",
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
        /* SPIRAW seq=XXX AA 55 12 34 ...（打印原始字节）
         * 打印完整的 10 个原始字节，用于逐字节对拍 */
        if ((now - s_raw_last_print_tick) >= SPI_SLAVE_DEBUG_RAW_PERIOD_MS)
        {
            if (s_raw_frames_printed < SPI_SLAVE_DEBUG_RAW_FRAMES)
            {
                s_raw_last_print_tick = now;
                s_raw_frames_printed++;
                UartDebug_Printf("SPIRAW seq=%u", new_seq);
                for (uint8_t i = 0u; i < SPI_SLAVE_FRAME_BYTES; i++)
                {
                    UartDebug_Printf(" %02X", raw_snapshot[i]);
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

    /* 兜底重启：字节错位已由滑动解析处理，这里只处理"很久没有有效帧"
     * （比如 SPI 外设被 OVR/MODF 停掉）。必须在 NSS 为高（帧间空闲）时重启，
     * 否则 SPE 在帧中间打开会从半个字节开始采样。 */
    {
        uint32_t t = HAL_GetTick();

        if (((int32_t)(t - s_last_valid_tick) >= (int32_t)SPI_SLAVE_RESTART_MS) &&
            ((int32_t)(t - s_last_restart_tick) >= (int32_t)SPI_SLAVE_RESTART_MS) &&
            (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10) == GPIO_PIN_SET))
        {
            s_last_restart_tick = t;
#if SPI_SLAVE_DEBUG_LEVEL >= 1
            UartDebug_Printf("SPIERR restart dma\r\n");
#endif
            (void)HAL_SPI_Abort(&hspi1);
            (void)Spi_StartDma();
        }
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

#endif /* SPI_SLAVE_ENABLE */
