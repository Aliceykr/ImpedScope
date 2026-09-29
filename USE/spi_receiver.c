#include "spi_receiver.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint32_t s_spiRxBuf[SPI_RX_BUF_WORDS];

/* Private function prototypes -----------------------------------------------*/
static void Receiver_ProcessFrame(uint8_t frame);

/* Exported functions --------------------------------------------------------*/
void SPI_Receiver_Init(void)
{
    (void)HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)s_spiRxBuf, SPI_RX_BUF_WORDS);
}

const uint32_t *SPI_Receiver_GetFrame(uint8_t index)
{
    if (index >= SPI_RX_FRAME_COUNT)
    {
        return (const uint32_t *)0;
    }
    return &s_spiRxBuf[(uint32_t)index * FRAME_WORDS];
}

uint8_t SPI_Receiver_ParseFrame(const uint32_t *raw, FFT_Point_t *out)
{
    if ((raw == (const uint32_t *)0) || (out == (FFT_Point_t *)0))
    {
        return 0u;
    }

    if (raw[FRAME_IDX_START] != FRAME_START_MARKER)
    {
        return 0u;
    }
    if (raw[FRAME_IDX_STOP] != FRAME_STOP_MARKER)
    {
        return 0u;
    }

    out->freq = raw[FRAME_IDX_FREQ];
    (void)memcpy(&out->ui_re, &raw[FRAME_IDX_UI_RE], sizeof(float));
    (void)memcpy(&out->ui_im, &raw[FRAME_IDX_UI_IM], sizeof(float));
    (void)memcpy(&out->uo_re, &raw[FRAME_IDX_UO_RE], sizeof(float));
    (void)memcpy(&out->uo_im, &raw[FRAME_IDX_UO_IM], sizeof(float));

    return 1u;
}

/* Private functions ---------------------------------------------------------*/
static void Receiver_ProcessFrame(uint8_t frame)
{
    FFT_Point_t point;

    if (SPI_Receiver_ParseFrame(&s_spiRxBuf[(uint32_t)frame * FRAME_WORDS],
                               &point) != 0u)
    {
        /* Valid frame received: consume point (freq / ui / uo) here */
    }
}

/* HAL callbacks -------------------------------------------------------------*/
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        Receiver_ProcessFrame(0u);   /* first frame in the buffer */
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        Receiver_ProcessFrame(1u);   /* second frame in the buffer */
    }
}
