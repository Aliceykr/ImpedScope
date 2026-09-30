#include "spi_receiver.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
/* DMA 接收缓冲区：大小为一整帧。
   DMA 配置为循环模式，收满一帧后会自动从头开始，并触发一次接收完成回调，
   因此无需在回调里手动重启接收。 */
static uint8_t s_spiRxBuf[FRAME_TOTAL_BYTES];

/* Private function prototypes -----------------------------------------------*/
/* 把连续 4 个字节按约定字节序拼装成一个 32-bit 无符号整数 */
static uint32_t Receiver_AssembleWord(const uint8_t *p);

/* Exported functions --------------------------------------------------------*/
void SPI_Receiver_Init(void)
{
    /* 启动 SPI1 的 DMA 接收：外设 -> 内存，长度为一整帧；
       返回值在此忽略（正常情况下 SPI 与 DMA 已在 main 中完成初始化）。 */
    (void)HAL_SPI_Receive_DMA(&hspi1, s_spiRxBuf, FRAME_TOTAL_BYTES);
}

uint8_t SPI_Receiver_ParseFrame(const uint8_t *raw, FFT_Point_t *out)
{
    const uint8_t *p;   /* 指向有效载荷起始位置 */
    uint32_t value;     /* 临时保存拼装出的 32-bit 原始数据 */

    /* 入参合法性检查 */
    if ((raw == (const uint8_t *)0) || (out == (FFT_Point_t *)0))
    {
        return 0u;
    }

    /* 校验帧头标记 */
    if (raw[0] != FRAME_START_MARKER)
    {
        return 0u;
    }

    /* 校验帧尾标记（位于整帧最后一个字节）*/
    if (raw[FRAME_TOTAL_BYTES - 1u] != FRAME_STOP_MARKER)
    {
        return 0u;
    }

    /* p 指向帧头之后的有效载荷首字节 */
    p = &raw[FRAME_START_BYTES];

    /* 第 1 个字段：频点（按 uint32 使用）*/
    out->freq = Receiver_AssembleWord(&p[0u]);

    /* 第 2~5 个字段：4 个 float（Ui/Uo 的实部、虚部）。
       先拼装成 32-bit 原始位模式，再用 memcpy 原样重解释为 float，
       这样避免了通过指针强转带来的“严格别名/对齐”问题。 */
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
    /* 大端：p[0] 为最高字节，p[3] 为最低字节 */
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
#else
    /* 小端：p[0] 为最低字节，p[3] 为最高字节 */
    return ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] << 8)  |  (uint32_t)p[0];
#endif
}

/* HAL callbacks -------------------------------------------------------------*/
/* SPI 接收完成回调：DMA 收满一帧（22 字节）时由 HAL 在中断上下文调用。 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* 只处理 SPI1 的事件（工程中若有多个 SPI，以此区分）*/
    if (hspi->Instance == SPI1)
    {
        FFT_Point_t point;

        /* 解析本帧；校验通过后 point 中即为该频点的数据 */
        if (SPI_Receiver_ParseFrame(s_spiRxBuf, &point) != 0u)
        {
            /* 有效帧：在此处使用 point（频点 / Ui / Uo）。
               例如调用 CircuitID_Compute() 计算频响。
               注意：本函数运行在中断上下文，应保持简短，避免耗时操作。 */
        }
    }
}
