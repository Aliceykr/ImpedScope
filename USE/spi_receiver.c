#include "spi_receiver.h"

#include <string.h>

/* DMA 缓冲区只保存一帧，DMA 配置为循环模式。 */
static uint8_t s_spi_receive_buffer[SPI_FRAME_TOTAL_BYTES];

/* 将协议中的 4 个字节还原为一个 32 位字。 */
static uint32_t SPI_Receiver_AssembleWord(const uint8_t *bytes);

void SPI_Receiver_Init(void)
{
    /* SPI 和 DMA 已由 CubeMX 初始化，这里只启动接收。 */
    (void)HAL_SPI_Receive_DMA(&hspi1,
                              s_spi_receive_buffer,
                              SPI_FRAME_TOTAL_BYTES);
}

uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out)
{
    const uint8_t *payload;
    uint32_t raw_value;

    if ((raw == (const uint8_t *)0) || (out == (FFT_Point_t *)0))
    {
        return 0u;
    }

    if (raw[0u] != SPI_FRAME_START_MARKER ||
        raw[SPI_FRAME_TOTAL_BYTES - 1u] != SPI_FRAME_END_MARKER)
    {
        return 0u;
    }

    payload = &raw[SPI_FRAME_MARKER_BYTES];
    out->freq = SPI_Receiver_AssembleWord(&payload[0u]);

    /* 按原始位模式复制浮点数，避免未对齐访问和严格别名问题。 */
    raw_value = SPI_Receiver_AssembleWord(&payload[4u]);
    (void)memcpy(&out->ui_re, &raw_value, sizeof(out->ui_re));

    raw_value = SPI_Receiver_AssembleWord(&payload[8u]);
    (void)memcpy(&out->ui_im, &raw_value, sizeof(out->ui_im));

    raw_value = SPI_Receiver_AssembleWord(&payload[12u]);
    (void)memcpy(&out->uo_re, &raw_value, sizeof(out->uo_re));

    raw_value = SPI_Receiver_AssembleWord(&payload[16u]);
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

/* DMA 收满一帧后由 HAL 在中断上下文调用。当前只完成校验和解析。 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    FFT_Point_t point;

    if ((hspi == (SPI_HandleTypeDef *)0) || hspi->Instance != SPI1)
    {
        return;
    }

    if (SPI_Receiver_ParseFrame(s_spi_receive_buffer, &point) != 0u)
    {
        /* 保持中断处理简短，需要计算时由上层任务读取 point。 */
    }
}
