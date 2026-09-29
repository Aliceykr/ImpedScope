#ifndef __SPI_RECEIVER_H
#define __SPI_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* ===========================================================================
 * Frame protocol
 * ---------------------------------------------------------------------------
 * One frame is transferred as a sequence of 32-bit words over SPI1.
 * Payload per frame:
 *   - start frame marker
 *   - frequency point / bin of the FFT result
 *   - FFT real/imaginary parts of Ui
 *   - FFT real/imaginary parts of Uo
 *   - stop frame marker
 * The exact word order and marker VALUES ARE NOT FINALIZED yet; adjust the
 * defines below once the master format is fixed.
 * ===========================================================================*/

/* Word indices inside one frame */
#define FRAME_IDX_START        0u   /* start frame marker  */
#define FRAME_IDX_FREQ         1u   /* corresponding frequency point / bin */
#define FRAME_IDX_UI_RE        2u   /* FFT real part of Ui */
#define FRAME_IDX_UI_IM        3u   /* FFT imaginary part of Ui */
#define FRAME_IDX_UO_RE        4u   /* FFT real part of Uo */
#define FRAME_IDX_UO_IM        5u   /* FFT imaginary part of Uo */
#define FRAME_IDX_STOP         6u   /* stop frame marker */
#define FRAME_WORDS            7u   /* total words per frame */

/* Frame marker values (PLACEHOLDERS - to be aligned with the master) */
#define FRAME_START_MARKER     0xFFFFFFFFu
#define FRAME_STOP_MARKER      0xFFFFFFFEu

/* Number of frames held in the circular DMA buffer (must be >= 2) */
#define SPI_RX_FRAME_COUNT     2u
#define SPI_RX_BUF_WORDS       (FRAME_WORDS * SPI_RX_FRAME_COUNT)

/* Parsed FFT result of one frequency point */
typedef struct
{
    uint32_t freq;     /* frequency point / bin */
    float    ui_re;    /* FFT real part of Ui */
    float    ui_im;    /* FFT imaginary part of Ui */
    float    uo_re;    /* FFT real part of Uo */
    float    uo_im;    /* FFT imaginary part of Uo */
} FFT_Point_t;

/* Exported functions --------------------------------------------------------*/
/* Starts SPI1 slave reception in DMA circular mode */
void SPI_Receiver_Init(void);

/* Returns the raw words of frame `index` (0 .. SPI_RX_FRAME_COUNT-1) */
const uint32_t *SPI_Receiver_GetFrame(uint8_t index);

/* Parses one raw frame into a FFT_Point_t.
   Returns 1 if start/stop markers are valid, 0 otherwise. */
uint8_t SPI_Receiver_ParseFrame(const uint32_t *raw, FFT_Point_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_RECEIVER_H */
