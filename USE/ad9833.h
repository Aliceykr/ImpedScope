#ifndef AD9833_H
#define AD9833_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/*
 * AD9833 DDS 信号源驱动（频响测量的激励源）。
 *
 * 硬件连接（复用原 SPI1 的引脚）：
 *   PG11 = SPI1_SCK  ->  AD9833 SCLK
 *   PB5  = SPI1_MOSI ->  AD9833 SDATA
 *   PA4  = GPIO 推挽输出 -> AD9833 FSYNC（低有效片选，由本驱动手动控制）
 *   PG9  = SPI1_MISO    不使用（AD9833 只写不读）
 * 接线不同时只改本文件下面的 AD9833_FSYNC_* 宏并同步 .ioc 的引脚分配。
 *
 * SPI1 必须满足（见 Core/Src/spi.c 与 9_30.ioc）：
 *   主机、SPI_DATASIZE_16BIT、Mode 2（CPOL=1 / CPHA=0，AD9833 在 SCLK 下降沿锁存数据）、
 *   软件 NSS、MSB first，内核时钟 75 MHz / 8 = 9.375 MHz（AD9833 的 SCLK 上限 40 MHz）。
 *
 * 为什么用 16 位帧：一次 HAL_SPI_Transmit 正好发出 16 个时钟，FSYNC 在整帧期间保持
 * 低电平，两个字节之间不会出现时钟间隙。若把 SPI1 改成 8 位，必须连发两个字节
 * （高字节在前）并保证 FSYNC 跨两字节不抬起，否则帧会被截断。
 *
 * 频率字 = round(f × 2^28 / MCLK)，MCLK 必须真的是 25 MHz：模块自带 25 MHz 晶振时
 * 无需额外接线；若 MCLK 由 MCU 的 MCO 提供，需要另行配置 MCO 引脚并确认实际频率，
 * 否则所有输出频率会按比例偏。
 */

/* ---------------- 配置 ---------------- */

/* AD9833 的主时钟。模块自带 25 MHz 晶振时保持默认值。 */
#define AD9833_MCLK_HZ              25000000UL

/* FSYNC（片选）引脚。 */
#define AD9833_FSYNC_GPIO_PORT      GPIOA
#define AD9833_FSYNC_PIN            GPIO_PIN_4

/* 初始化时的上电等待与 SPI 发送超时。 */
#define AD9833_POWER_UP_DELAY_MS    5u
#define AD9833_SPI_TIMEOUT_MS       10u

/* ---------------- 控制寄存器位（AD9833 数据手册 Table 8） ----------------
 *  D13 B28      D12 HLB      D11 FSEL     D10 PSEL
 *  D9  保留=0    D8  RESET    D7  SLEEP1   D6  SLEEP12
 *  D5  OPBITEN  D4  保留=0    D3  DIV2     D2  保留=0
 *  D1  MODE     D0  保留=0
 * D9/D4/D2 手册要求写 0。注意 AD9833 没有 SIGN/PIB 位（那是 AD9834 的 D4），
 * 把 RESET 或 OPBITEN 写到 D9/D6 上是常见的抄错点。
 * -------------------------------------------------------------------------- */
#define AD9833_CTRL_B28             (1u << 13)   /* B28=1：连续两次 16 位写拼 28 位频率 */
#define AD9833_CTRL_HLB             (1u << 12)   /* B28=0 时选高/低 14 位 */
#define AD9833_CTRL_FSEL            (1u << 11)   /* 0=FREQ0 1=FREQ1 */
#define AD9833_CTRL_PSEL            (1u << 10)   /* 0=PHASE0 1=PHASE1 */
#define AD9833_CTRL_RESET           (1u << 8)    /* 1=复位，输出停在中点 */
#define AD9833_CTRL_SLEEP1          (1u << 7)    /* 1=关内部 MCLK */
#define AD9833_CTRL_SLEEP12         (1u << 6)    /* 1=关 DAC */
#define AD9833_CTRL_OPBITEN         (1u << 5)    /* 1=输出 DAC 的 MSB（方波） */
#define AD9833_CTRL_DIV2            (1u << 3)    /* 见 Table 15 */
#define AD9833_CTRL_MODE            (1u << 1)    /* 1=三角波 0=正弦波 */

/* Table 15（OPBITEN=1 时决定 VOUT）：
 *   OPBITEN=1, DIV2=0 -> DAC MSB/2，方波频率 = f/2
 *   OPBITEN=1, DIV2=1 -> DAC MSB，  方波频率 = f
 *   OPBITEN=1 且 MODE=1 是保留组合，不能使用。                                  */

/* ---------------- 寄存器地址（D15:D14） ---------------- */
#define AD9833_REG_CTRL             0x0000u
#define AD9833_REG_FREQ0            0x4000u
#define AD9833_REG_FREQ1            0x8000u
#define AD9833_REG_PHASE0           0xC000u
#define AD9833_REG_PHASE1           0xE000u

/* 波形编号 */
#define AD9833_WAVE_SINE            0u
#define AD9833_WAVE_TRIANGLE        1u
#define AD9833_WAVE_SQUARE          2u   /* 频率 = f（DIV2=1） */

/* ---------------- API ----------------
 * 全部返回 1 表示成功、0 表示本次 SPI 发送失败（驱动内部不报错，
 * 由调用方决定是否 Screen_ReportError）。 */

/* 配置 FSYNC 引脚、复位 AD9833 并预装 FREQ0/PHASE0，退出复位后输出正弦波。 */
uint8_t AD9833_Init(void);

/* 设置输出频率（Hz）。写未被选中的频率寄存器，再原子翻转 FSEL，切换无毛刺。 */
uint8_t AD9833_SetFrequency(uint32_t frequency_hz);

/* 设置波形：AD9833_WAVE_SINE / AD9833_WAVE_TRIANGLE / AD9833_WAVE_SQUARE。
 * 频响测量只能用正弦，三角/方波仅用于硬件调试。 */
uint8_t AD9833_SetWave(uint8_t wave);

/* 设置相位，phase 为 12 位（0~4095 对应 0~2π）。 */
uint8_t AD9833_SetPhase(uint16_t phase);

/* 1 = 正常输出；0 = 复位（输出停在中点）。 */
uint8_t AD9833_Enable(uint8_t enable);

/* 复位：内部寄存器清零，输出中点（不掉电）。 */
uint8_t AD9833_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* AD9833_H */
