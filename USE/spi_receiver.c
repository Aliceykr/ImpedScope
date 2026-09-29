#include "spi_receiver.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
/* One frame per DMA round; circular mode re-triggers RxCplt after each frame */
static uint8_t s_spiRxBuf[FRAME_TOTAL_BYTES];

/* Private function prototypes -----------------------------------------------*/
static uint32_t Receiver_AssembleWord(const uint8_t *p);

/* Exported functions --------------------------------------------------------*/
void SPI_Receiver_Init(void)
{
    (void)HAL_SPI_Receive_DMA(&hspi1, s_spiRxBuf, FRAME_TOTAL_BYTES);
}

uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out)
{
    const uint8_t *p;
    uint32_t value;

    if ((raw == (const uint8_t *)0) || (out == (FFT_Point_t *)0))
    {
        return 0u;
    }
    if (raw[0] != FRAME_START_MARKER)
    {
        return 0u;
    }
    if (raw[FRAME_TOTAL_BYTES - 1u] != FRAME_STOP_MARKER)
    {
        return 0u;
    }

    p = &raw[FRAME_START_BYTES];

    out->freq = Receiver_AssembleWord(&p[0u]);

    value = Receiver_AssembleWord(&p[4u]);
    (void)memcpy(&out->ui_re, &value, sizeof(float));
    value = Receiver_AssembleWord(&p[8u]);
    (void)memcpy(&out->ui_im, &value, sizeof(float));
    value = Receiver_AssembleWord(&p[12u]);
    (void)memcpy(&out->uo_re, &value, sizeof(float));
    value = Receiver_AssembleWord(&p[16u]);
    (void)memcpy(&out->uo_im, &value, sizeof(float));

    return 1u;
}

/* Private functions ---------------------------------------------------------*/
static uint32_t Receiver_AssembleWord(const uint8_t *p)
{
#if FRAME_BIG_ENDIAN
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
#else
    return ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] << 8)  |  (uint32_t)p[0];
#endif
}

/* HAL callbacks -------------------------------------------------------------*/
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        FFT_Point_t point;

        if (SPI_Receiver_ParseFrame(s_spiRxBuf, &point) != 0u)
        {
            /* Valid frame: consume point (freq / Ui / Uo) here */
        }
    }
}
