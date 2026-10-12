#include "spi_receiver.h"

#include <string.h>

#include "screen.h"

/* 一次 DMA 只收一帧。 */
static uint8_t s_rx_buffer[SPI_FRAME_TOTAL_BYTES];

/* 单生产者（DMA/EXTI 回调）、单消费者（主循环）记录队列。 */
static FREQ_Record_t s_queue[SPI_RECORD_QUEUE_SIZE];
static volatile uint8_t s_queue_head;
static volatile uint8_t s_queue_tail;

/* DMA 是否已装载并等待一帧。 */
static volatile uint8_t s_dma_armed;

/* CS 是否正在拉低（一帧进行中）：用于区分"正常收满"与"短帧"。 */
static volatile uint8_t s_frame_active;

/* 收包统计，中断中更新，主循环经 SPI_Receiver_GetStats() 读取。 */
static volatile uint32_t s_cs_falling_count;
static volatile uint32_t s_frame_full_count;
static volatile uint32_t s_frame_ok_count;
static uint8_t s_last_frame[SPI_FRAME_TOTAL_BYTES];

static uint8_t SPI_Receiver_Arm(void);
static void    SPI_Receiver_Enqueue(const FREQ_Record_t *record);

#if 0
/* 旧数据帧解析，暂时注释，待新协议确定。 */
static int16_t  SPI_Receiver_ParseSample(const uint8_t *raw, uint16_t offset);
static uint16_t SPI_Receiver_ParseIndex(const uint8_t *raw, uint16_t offset);
#endif

uint8_t SPI_Receiver_Init(void)
{
    s_queue_head = 0u;
    s_queue_tail = 0u;
    s_dma_armed = 0u;
    s_frame_active = 0u;
    s_cs_falling_count = 0u;
    s_frame_full_count = 0u;
    s_frame_ok_count = 0u;

    return SPI_Receiver_Arm();
}

#if 0
/* 旧数据帧解析（定点分量与峰值 bin），暂时注释。 */
uint8_t SPI_Receiver_ParseFrame_OldDataFrame(const uint8_t *raw, FPGA_Record_t *out)
{
    if ((raw == (const uint8_t *)0) || (out == (FPGA_Record_t *)0))
    {
        return 0u;
    }
    (void)memset(out, 0, sizeof(*out));

    if ((raw[0u] != SPI_FRAME_HEADER_0) ||
        (raw[1u] != SPI_FRAME_HEADER_1) ||
        (raw[SPI_FRAME_TAIL_OFFSET] != SPI_FRAME_TAIL_0) ||
        (raw[SPI_FRAME_TAIL_OFFSET + 1u] != SPI_FRAME_TAIL_1))
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        return 0u;
    }
    s_frame_ok_count++;

    out->ch1_im = SPI_Receiver_ParseSample(raw, SPI_FRAME_CH1_IM_OFFSET);
    out->ch1_re = SPI_Receiver_ParseSample(raw, SPI_FRAME_CH1_RE_OFFSET);
    out->ch0_im = SPI_Receiver_ParseSample(raw, SPI_FRAME_CH0_IM_OFFSET);
    out->ch0_re = SPI_Receiver_ParseSample(raw, SPI_FRAME_CH0_RE_OFFSET);
    out->index1 = SPI_Receiver_ParseIndex(raw, SPI_FRAME_INDEX1_OFFSET);
    out->index0 = SPI_Receiver_ParseIndex(raw, SPI_FRAME_INDEX0_OFFSET);

    if ((out->index0 > SPI_FRAME_INDEX_MAX) ||
        (out->index1 > SPI_FRAME_INDEX_MAX))
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }

    return 1u;
}

/* 取 2 字节大端字段，保留低 12 位并按第 11 位符号扩展。 */
static int16_t SPI_Receiver_ParseSample(const uint8_t *raw, uint16_t offset)
{
    uint16_t word = (uint16_t)(((uint16_t)raw[offset] << 8) |
                                raw[offset + 1u]);
    word &= (uint16_t)((1u << SPI_FRAME_SAMPLE_BITS) - 1u);
    if ((word & (1u << (SPI_FRAME_SAMPLE_BITS - 1u))) != 0u)
    {
        word |= (uint16_t)(0xFFFFu << SPI_FRAME_SAMPLE_BITS);
    }
    return (int16_t)word;
}

/* 取 2 字节大端字段，保留低 13 位（无符号 bin 序号）。 */
static uint16_t SPI_Receiver_ParseIndex(const uint8_t *raw, uint16_t offset)
{
    uint16_t word = (uint16_t)(((uint16_t)raw[offset] << 8) |
                                raw[offset + 1u]);
    return (uint16_t)(word & ((1u << SPI_FRAME_INDEX_BITS) - 1u));
}
#endif

/* 解析一帧：帧头、帧尾校验通过后，读取 4 字节大端频率并检查范围。 */
uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FREQ_Record_t *out)
{
    uint32_t frequency;

    if ((raw == (const uint8_t *)0) || (out == (FREQ_Record_t *)0))
    {
        return 0u;
    }
    (void)memset(out, 0, sizeof(*out));

    if ((raw[0u] != SPI_FRAME_HEADER_0) ||
        (raw[1u] != SPI_FRAME_HEADER_1) ||
        (raw[SPI_FRAME_TAIL_OFFSET] != SPI_FRAME_TAIL_0) ||
        (raw[SPI_FRAME_TAIL_OFFSET + 1u] != SPI_FRAME_TAIL_1))
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        return 0u;
    }
    s_frame_ok_count++;

    frequency = ((uint32_t)raw[SPI_FRAME_FREQ_OFFSET]      << 24) |
                ((uint32_t)raw[SPI_FRAME_FREQ_OFFSET + 1u] << 16) |
                ((uint32_t)raw[SPI_FRAME_FREQ_OFFSET + 2u] << 8) |
                 (uint32_t)raw[SPI_FRAME_FREQ_OFFSET + 3u];

    if ((frequency == 0u) || (frequency > SPI_FRAME_FREQ_MAX_HZ))
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        return 0u;
    }

    out->frequency_hz = frequency;
    return 1u;
}

static uint8_t SPI_Receiver_Arm(void)
{
    if (HAL_SPI_Receive_DMA(&hspi1, s_rx_buffer,
                            SPI_FRAME_TOTAL_BYTES) == HAL_OK)
    {
        s_dma_armed = 1u;
        return 1u;
    }
    return 0u;
}

static void SPI_Receiver_Enqueue(const FREQ_Record_t *record)
{
    uint8_t next_head = (uint8_t)((s_queue_head + 1u) % SPI_RECORD_QUEUE_SIZE);

    if (next_head == s_queue_tail)
    {
        Screen_ReportError(SCREEN_E06_SPI_QFULL);
        s_queue_tail = (uint8_t)((s_queue_tail + 1u) % SPI_RECORD_QUEUE_SIZE);
    }

    s_queue[s_queue_head] = *record;
    s_queue_head = next_head;
}

uint8_t SPI_Receiver_TryGetRecord(FREQ_Record_t *out)
{
    uint32_t interrupt_state;
    uint8_t tail;

    if (out == (FREQ_Record_t *)0)
    {
        return 0u;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    tail = s_queue_tail;
    if (tail == s_queue_head)
    {
        if (interrupt_state == 0u)
        {
            __enable_irq();
        }
        return 0u;
    }

    *out = s_queue[tail];
    s_queue_tail = (uint8_t)((tail + 1u) % SPI_RECORD_QUEUE_SIZE);
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }

    return 1u;
}

/* 取统计快照，关中断复制，避免读到中断更新到一半的值。 */
void SPI_Receiver_GetStats(SPI_Stats_t *out)
{
    uint32_t interrupt_state;

    if (out == (SPI_Stats_t *)0)
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    out->cs_falling = s_cs_falling_count;
    out->frame_full = s_frame_full_count;
    out->frame_ok = s_frame_ok_count;
    (void)memcpy(out->last_frame, s_last_frame, sizeof(s_last_frame));
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }
}

/* 一帧收满：解析、入队，然后重新装载 DMA。 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    FREQ_Record_t record;

    if ((hspi != (SPI_HandleTypeDef *)0) && (hspi->Instance == SPI1))
    {
        s_dma_armed = 0u;
        s_frame_active = 0u;
        s_frame_full_count++;
        (void)memcpy(s_last_frame, s_rx_buffer, sizeof(s_last_frame));
        if (SPI_Receiver_ParseFrame(s_rx_buffer, &record) != 0u)
        {
            SPI_Receiver_Enqueue(&record);
        }
        (void)SPI_Receiver_Arm();
    }
}

/* SPI 通信错误（溢出、模式错误等）：中止后重新装载，避免接收停住。 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != (SPI_HandleTypeDef *)0) && (hspi->Instance == SPI1))
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        (void)HAL_SPI_Abort(hspi);
        s_dma_armed = 0u;
        s_frame_active = 0u;
        (void)SPI_Receiver_Arm();
    }
}

/* CS 边沿：下降沿检查 DMA 是否就绪，上升沿检测短帧。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != SPI_CS_PIN)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(SPI_CS_GPIO_PORT, SPI_CS_PIN) == GPIO_PIN_SET)
    {
        /* CS 上升（帧结束）。DMA 计数为 0 表示已收满，完成回调可能还在排队，
         * 交给 RxCpltCallback 处理，这里不做任何动作。 */
        if ((s_frame_active != 0u) &&
            (__HAL_DMA_GET_COUNTER(hspi1.hdmarx) != 0u))
        {
            s_frame_active = 0u;
            (void)HAL_SPI_Abort(&hspi1);
            s_dma_armed = 0u;
            Screen_ReportError(SCREEN_E03_SPI_FRAME);
            (void)SPI_Receiver_Arm();
        }
    }
    else
    {
        /* CS 下降（帧开始）：标记帧进行中；DMA 未装载则本帧必丢。 */
        s_cs_falling_count++;
        s_frame_active = 1u;
        if (s_dma_armed == 0u)
        {
            Screen_ReportError(SCREEN_E02_SPI_DMA);
            (void)SPI_Receiver_Arm();
        }
    }
}
