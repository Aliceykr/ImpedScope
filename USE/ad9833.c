#include "ad9833.h"

/* 频率字是 28 位，这一项就是 2^28。 */
#define AD9833_FREQ_WORD_SCALE      268435456ULL

/* 初始化时预装到 FREQ0 的频率，正式测量前会被 AD9833_SetFrequency() 覆盖。 */
#define AD9833_INIT_FREQ_HZ         1000u

/* 常驻控制字：B28 始终置位（两次 16 位写拼成 28 位频率），其余位按波形/复位状态维护。 */
static uint16_t s_ctrl = AD9833_CTRL_B28;

static void     AD9833_FsyncInit(void);
static uint8_t  AD9833_Write16(uint16_t word);
static uint8_t  AD9833_WriteCtrl(void);
static uint32_t AD9833_FreqWord(uint32_t freq_hz);
static uint8_t  AD9833_WriteFreqReg(uint16_t reg, uint32_t freq_word);

/* FSYNC 引脚不在 CubeMX 里分配（它不支持同一引脚既做 SPI1_NSS 又做普通 GPIO，而 AD9833
 * 需要软件片选），由本驱动手动配成推挽输出。 */
static void AD9833_FsyncInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 先把输出寄存器置高（空闲态），再切成输出，避免上电瞬间误拉低。 */
    HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_PORT, AD9833_FSYNC_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = AD9833_FSYNC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AD9833_FSYNC_GPIO_PORT, &GPIO_InitStruct);
}

/* 写一个 16 位命令：FSYNC 拉低 -> 一个 16 位 SPI 帧 -> FSYNC 拉高。 */
static uint8_t AD9833_Write16(uint16_t word)
{
    uint16_t          rx_dummy = 0u;
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_PORT, AD9833_FSYNC_PIN, GPIO_PIN_RESET);

    /* 这里用 TransmitReceive 而不是 Transmit：全双工主机只发不收时，每个收到的字都会
     * 堆在 RX FIFO 里，堆满后置 OVR；而 H7 HAL 的阻塞发送路径既不检查也不清 OVR
     * （只有中断路径会清，见 stm32h7xx_hal_spi.c）。把每帧的 RX 一并读走，SPI 状态就
     * 始终干净。MISO（PG9）没接任何东西，读回来的是无关数据，丢弃即可。
     * 16 位模式下 Size=1 恰好是一个 16 位帧、MSB 在前，与 AD9833 的 D15 对齐。 */
    status = HAL_SPI_TransmitReceive(&hspi1, (const uint8_t *)&word, (uint8_t *)&rx_dummy,
                                     1u, AD9833_SPI_TIMEOUT_MS);

    HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_PORT, AD9833_FSYNC_PIN, GPIO_PIN_SET);

    return (status == HAL_OK) ? 1u : 0u;
}

static uint8_t AD9833_WriteCtrl(void)
{
    return AD9833_Write16((uint16_t)(AD9833_REG_CTRL | s_ctrl));
}

/* FREQREG = round(f × 2^28 / MCLK)，并限幅到 MCLK/2（AD9833 的输出上限）。 */
static uint32_t AD9833_FreqWord(uint32_t freq_hz)
{
    uint64_t word;

    if (freq_hz > (AD9833_MCLK_HZ / 2u))
    {
        freq_hz = AD9833_MCLK_HZ / 2u;
    }

    word = (((uint64_t)freq_hz * AD9833_FREQ_WORD_SCALE) + (AD9833_MCLK_HZ / 2u)) / AD9833_MCLK_HZ;

    return (uint32_t)(word & 0x0FFFFFFFULL);
}

/* B28=1 时同一个频率寄存器要连写两次：先低 14 位，再高 14 位。 */
static uint8_t AD9833_WriteFreqReg(uint16_t reg, uint32_t freq_word)
{
    if (AD9833_Write16((uint16_t)(reg | (freq_word & 0x3FFFu))) == 0u)
    {
        return 0u;
    }

    return AD9833_Write16((uint16_t)(reg | ((freq_word >> 14) & 0x3FFFu)));
}

uint8_t AD9833_Init(void)
{
    uint8_t ok = 1u;

    AD9833_FsyncInit();

    s_ctrl = AD9833_CTRL_B28;      /* FSEL=0（用 FREQ0）、PSEL=0（用 PHASE0）、正弦波 */

    HAL_Delay(AD9833_POWER_UP_DELAY_MS);   /* 等模块上的 25 MHz 晶振起振 */

    /* 1. 复位：内部寄存器清零，DAC 输出停在中点；同时把 B28 置位，便于下面预装频率。 */
    ok = (uint8_t)(ok & AD9833_Write16((uint16_t)(AD9833_REG_CTRL |
                                                  AD9833_CTRL_B28 |
                                                  AD9833_CTRL_RESET)));
    HAL_Delay(1u);

    /* 2. 复位期间预装 FREQ0 与 PHASE0 = 0。 */
    ok = (uint8_t)(ok & AD9833_WriteFreqReg(AD9833_REG_FREQ0, AD9833_FreqWord(AD9833_INIT_FREQ_HZ)));
    ok = (uint8_t)(ok & AD9833_Write16((uint16_t)(AD9833_REG_PHASE0 | 0x0000u)));

    /* 3. 退出复位，开始输出（控制字里 B28 仍置位、RESET 已清）。 */
    ok = (uint8_t)(ok & AD9833_WriteCtrl());
    HAL_Delay(1u);

    return ok;
}

uint8_t AD9833_SetFrequency(uint32_t frequency_hz)
{
    uint32_t freq_word = AD9833_FreqWord(frequency_hz);

    /* 无毛刺切换：写"当前没被选中"的那个频率寄存器，最后用一次控制字写原子翻转 FSEL。
     * AD9833 在 FSEL 变化时不会清相位累加器，输出相位连续。
     * 注意写入失败时保持 s_ctrl 不变，让下一次调用重试同一条路径。 */
    if ((s_ctrl & AD9833_CTRL_FSEL) != 0u)
    {
        if (AD9833_WriteFreqReg(AD9833_REG_FREQ0, freq_word) == 0u)
        {
            return 0u;
        }
        s_ctrl &= (uint16_t)~AD9833_CTRL_FSEL;
    }
    else
    {
        if (AD9833_WriteFreqReg(AD9833_REG_FREQ1, freq_word) == 0u)
        {
            return 0u;
        }
        s_ctrl |= AD9833_CTRL_FSEL;
    }

    return AD9833_WriteCtrl();
}

uint8_t AD9833_SetWave(uint8_t wave)
{
    /* 先清掉波形相关的三位，避免出现 OPBITEN=1 且 MODE=1 的保留组合。 */
    s_ctrl &= (uint16_t)~(AD9833_CTRL_MODE | AD9833_CTRL_OPBITEN | AD9833_CTRL_DIV2);

    switch (wave)
    {
    case AD9833_WAVE_TRIANGLE:
        s_ctrl |= AD9833_CTRL_MODE;                       /* MODE=1：三角波 */
        break;

    case AD9833_WAVE_SQUARE:
        /* Table 15：OPBITEN=1、MODE=0、DIV2=1 -> 输出 DAC MSB，方波频率 = f。
         * 若把 DIV2 清掉则输出 MSB/2，方波频率 = f/2。 */
        s_ctrl |= (uint16_t)(AD9833_CTRL_OPBITEN | AD9833_CTRL_DIV2);
        break;

    case AD9833_WAVE_SINE:
    default:
        break;                                            /* MODE=0：正弦波 */
    }

    return AD9833_WriteCtrl();
}

uint8_t AD9833_SetPhase(uint16_t phase)
{
    /* PSEL 保持 0，直接写 PHASE0；12 位对应 0~2π。 */
    return AD9833_Write16((uint16_t)(AD9833_REG_PHASE0 | (phase & 0x0FFFu)));
}

uint8_t AD9833_Enable(uint8_t enable)
{
    if (enable != 0u)
    {
        s_ctrl &= (uint16_t)~AD9833_CTRL_RESET;
    }
    else
    {
        s_ctrl |= AD9833_CTRL_RESET;
    }

    return AD9833_WriteCtrl();
}

uint8_t AD9833_Reset(void)
{
    return AD9833_Enable(0u);
}
