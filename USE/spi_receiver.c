#include "spi_receiver.h"

#include <string.h>

#include "screen.h"

/* DMA 半传输和全传输回调各处理一帧长度的数据。 */
static uint8_t s_spi_receive_buffer[SPI_DMA_BUFFER_BYTES];
static uint8_t s_frame_candidate[SPI_FRAME_TOTAL_BYTES];
static uint8_t s_frame_candidate_length;

/* 单生产者（DMA 回调）、单消费者（主循环）频点队列。 */
static FFT_Point_t s_point_queue[SPI_POINT_QUEUE_SIZE];
static volatile uint8_t s_point_queue_head;
static volatile uint8_t s_point_queue_tail;

/* 将协议中的 4 个字节还原为一个 32 位字。 */
static uint32_t SPI_Receiver_AssembleWord(const uint8_t *bytes);
static void SPI_Receiver_ProcessBytes(const uint8_t *bytes, uint16_t length);
static void SPI_Receiver_QueuePoint(const FFT_Point_t *point);
static void SPI_Receiver_ConsumeByte(uint8_t byte);

uint8_t SPI_Receiver_Init(void)
{
    s_frame_candidate_length = 0u;
    s_point_queue_head = 0u;
    s_point_queue_tail = 0u;

    /* 编译期约定：IEEE-754 单精度必须和 uint32_t 同宽。 */
    if (sizeof(float) != sizeof(uint32_t))
    {
        Screen_Fatal(SCREEN_E05_SPI_LEN);   /* 不返回 */
    }

    return (HAL_SPI_Receive_DMA(&hspi1,
                                s_spi_receive_buffer,
                                SPI_DMA_BUFFER_BYTES) == HAL_OK) ? 1u : 0u;
}

uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out)
{
    const uint8_t *payload;
    uint32_t raw_value;

    if ((raw == (const uint8_t *)0) || (out == (FFT_Point_t *)0))
    {
        return 0u;
    }

    (void)memset(out, 0, sizeof(*out));

    /* sizeof(float)==sizeof(uint32_t) 已在 SPI_Receiver_Init() 中保证。 */

    if (raw[0u] != SPI_FRAME_START_MARKER ||
        raw[SPI_FRAME_TOTAL_BYTES - 1u] != SPI_FRAME_END_MARKER)
    {
        Screen_ReportError(SCREEN_E03_SPI_FRAME);
        return 0u;
    }

    payload = &raw[SPI_FRAME_MARKER_BYTES];
    out->freq = SPI_Receiver_AssembleWord(&payload[0u]);

    /* IEEE-754 指数全 1 表示 NaN 或无穷大，不接受此类输入。 */
    raw_value = SPI_Receiver_AssembleWord(&payload[4u]);
    if ((raw_value & 0x7F800000u) == 0x7F800000u)
    {
        Screen_ReportError(SCREEN_E04_SPI_NAN);
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }
    (void)memcpy(&out->ui_re, &raw_value, sizeof(out->ui_re));

    raw_value = SPI_Receiver_AssembleWord(&payload[8u]);
    if ((raw_value & 0x7F800000u) == 0x7F800000u)
    {
        Screen_ReportError(SCREEN_E04_SPI_NAN);
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }
    (void)memcpy(&out->ui_im, &raw_value, sizeof(out->ui_im));

    raw_value = SPI_Receiver_AssembleWord(&payload[12u]);
    if ((raw_value & 0x7F800000u) == 0x7F800000u)
    {
        Screen_ReportError(SCREEN_E04_SPI_NAN);
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }
    (void)memcpy(&out->uo_re, &raw_value, sizeof(out->uo_re));

    raw_value = SPI_Receiver_AssembleWord(&payload[16u]);
    if ((raw_value & 0x7F800000u) == 0x7F800000u)
    {
        Screen_ReportError(SCREEN_E04_SPI_NAN);
        (void)memset(out, 0, sizeof(*out));
        return 0u;
    }
    (void)memcpy(&out->uo_im, &raw_value, sizeof(out->uo_im));

    return 1u;
}

static uint32_t SPI_Receiver_AssembleWord(const uint8_t *bytes)
{
#if SPI_FRAME_BIG_ENDIAN
    return ((uint32_t)bytes[0u] << 24) |
           ((uint32_t)bytes[1u] << 16) |
           ((uint32_t)bytes[2u] << 8)  |
           ((uint32_t)bytes[3u]);
#else
    return ((uint32_t)bytes[3u] << 24) |
           ((uint32_t)bytes[2u] << 16) |
           ((uint32_t)bytes[1u] << 8)  |
           ((uint32_t)bytes[0u]);
#endif
}

static void SPI_Receiver_ProcessBytes(const uint8_t *bytes, uint16_t length)
{
    uint16_t index;

    for (index = 0u; index < length; ++index)
    {
        SPI_Receiver_ConsumeByte(bytes[index]);
    }
}

/* 按帧头滑动同步，坏帧或丢字节后继续搜索下一帧。 */
static void SPI_Receiver_ConsumeByte(uint8_t byte)
{
    FFT_Point_t point;
    uint8_t offset;

    if (s_frame_candidate_length == 0u)
    {
        if (byte == SPI_FRAME_START_MARKER)
        {
            s_frame_candidate[0u] = byte;
            s_frame_candidate_length = 1u;
        }
        return;
    }

    s_frame_candidate[s_frame_candidate_length] = byte;
    ++s_frame_candidate_length;

    if (s_frame_candidate_length < SPI_FRAME_TOTAL_BYTES)
    {
        return;
    }

    if (SPI_Receiver_ParseFrame(s_frame_candidate, &point) != 0u)
    {
        SPI_Receiver_QueuePoint(&point);
        s_frame_candidate_length = 0u;
        return;
    }

    for (offset = 1u; offset < SPI_FRAME_TOTAL_BYTES; ++offset)
    {
        if (s_frame_candidate[offset] == SPI_FRAME_START_MARKER)
        {
            s_frame_candidate_length =
                (uint8_t)(SPI_FRAME_TOTAL_BYTES - offset);
            (void)memmove(s_frame_candidate,
                          &s_frame_candidate[offset],
                          s_frame_candidate_length);
            return;
        }
    }

    s_frame_candidate_length = 0u;
}

static void SPI_Receiver_QueuePoint(const FFT_Point_t *point)
{
    uint8_t next_head;

    next_head = (uint8_t)((s_point_queue_head + 1u) % SPI_POINT_QUEUE_SIZE);
    if (next_head == s_point_queue_tail)
    {
        /* 队列满时丢弃最旧数据，保留最新测量值。 */
        Screen_ReportError(SCREEN_E06_SPI_QFULL);
        s_point_queue_tail =
            (uint8_t)((s_point_queue_tail + 1u) % SPI_POINT_QUEUE_SIZE);
    }

    s_point_queue[s_point_queue_head] = *point;
    s_point_queue_head = next_head;
}

uint8_t SPI_Receiver_TryGetPoint(FFT_Point_t *out)
{
    uint32_t interrupt_state;
    uint8_t tail;

    if (out == (FFT_Point_t *)0)
    {
        return 0u;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    tail = s_point_queue_tail;
    if (tail == s_point_queue_head)
    {
        if (interrupt_state == 0u)
        {
            __enable_irq();
        }
        return 0u;
    }

    *out = s_point_queue[tail];
    s_point_queue_tail = (uint8_t)((tail + 1u) % SPI_POINT_QUEUE_SIZE);
    if (interrupt_state == 0u)
    {
        __enable_irq();
    }

    return 1u;
}

/* DMA 半满和全满分别消费环形缓冲区的前、后半段。 */
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != (SPI_HandleTypeDef *)0) && hspi->Instance == SPI1)
    {
        SPI_Receiver_ProcessBytes(s_spi_receive_buffer,
                                 SPI_FRAME_TOTAL_BYTES);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != (SPI_HandleTypeDef *)0) && hspi->Instance == SPI1)
    {
        SPI_Receiver_ProcessBytes(&s_spi_receive_buffer[SPI_FRAME_TOTAL_BYTES],
                                  SPI_FRAME_TOTAL_BYTES);
    }
}
