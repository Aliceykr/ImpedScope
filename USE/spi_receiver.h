#ifndef __SPI_RECEIVER_H
#define __SPI_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"

/* ===========================================================================
 * Frame protocol over 8-bit bytes (FPGA is the SPI master, spi_byte module).
 * VALUES/LAYOUT ARE NOT FINALIZED - adjust the defines to match the FPGA.
 *
 *   [start marker][freq 4B][Ui re 4B][Ui im 4B][Uo re 4B][Uo im 4B][stop marker]
 * ===========================================================================*/

/* Single-byte frame markers (PLACEHOLDERS - align with FPGA) */
#define FRAME_START_MARKER      0xAAu
#define FRAME_STOP_MARKER       0x55u
#define FRAME_START_BYTES       1u
#define FRAME_STOP_BYTES        1u

/* Each value is a 32-bit word transferred as 4 bytes */
#define FRAME_FIELD_BYTES       4u
#define FRAME_FIELD_COUNT       5u    /* freq, Ui re, Ui im, Uo re, Uo im */
#define FRAME_PAYLOAD_BYTES     (FRAME_FIELD_BYTES * FRAME_FIELD_COUNT)
#define FRAME_TOTAL_BYTES       (FRAME_START_BYTES + FRAME_PAYLOAD_BYTES + FRAME_STOP_BYTES)

/* Byte order of each 32-bit field:
   1 = first received byte is MSB (big-endian), 0 = LSB first (little-endian) */
#define FRAME_BIG_ENDIAN        1u

/* Parsed FFT result of one frequency point */
typedef struct
{
    uint32_t freq;     /* corresponding frequency point / bin */
    float    ui_re;    /* FFT real part of Ui */
    float    ui_im;    /* FFT imaginary part of Ui */
    float    uo_re;    /* FFT real part of Uo */
    float    uo_im;    /* FFT imaginary part of Uo */
} FFT_Point_t;

/* Exported functions --------------------------------------------------------*/
/* Starts SPI1 slave 8-bit reception in DMA circular mode, one frame per round */
void SPI_Receiver_Init(void);

/* Parses one raw byte frame into a FFT_Point_t.
   Returns 1 if start/stop markers are valid, 0 otherwise. */
uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_RECEIVER_H */
