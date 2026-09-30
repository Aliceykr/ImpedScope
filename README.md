# 9_30 工程说明

基于 **STM32H750XBH6** 的黑盒电路频响辨识工程。FPGA 作为 SPI 主机，将各频点的
Ui / Uo 的 FFT 结果按字节发给 STM32；STM32 在片上计算被测电路（DUT）的
**频率、阻抗幅度、阻抗相位**（XYZ）。

- 开发环境：Keil MDK-ARM（ARMCC V5）、STM32CubeMX 6.18、HAL FW V1.13
- 主控：STM32H750XBHx，SYSCLK 480 MHz，HCLK 120 MHz，APB 60 MHz

## 目录结构

```
9_30/
├─ USE/                     用户逻辑（CubeMX 不认识、不触碰，重生成不丢）
│  ├─ spi_receiver.h/.c     SPI 从机字节帧接收 + DMA + 帧解析
│  └─ circuit_id.h/.c       频响辨识：H、阻抗 Z、导纳 Y
├─ Core/                    CubeMX 生成
│  ├─ Src/main.c            仅在 USER CODE 区加入初始化调用
│  └─ Src/spi.c             SPI1 / DMA 配置（已改为 8-bit）
├─ Drivers/                 HAL 与 CMSIS
├─ MDK-ARM/                 Keil 工程（9_30.uvprojx）
└─ 9_30.ioc                 CubeMX 工程文件
```

## 硬件连接（前端：跨阻 I/V，虚地法）

```
信号源 ──Ui──┬── DUT（导纳 Y）
             │      │ 电流 I
             │      ▼
             │  运放(−)反相端，反馈电阻 R 接输出
             │  运放(+)同相端接地
             └───────────── Uo（运放输出）
```

运放反相端虚地，DUT 两端电压即 Ui；白板公式：

```
I  = Ui·Y
Uo = −I·R
H  = Uo/Ui = −Y·R
Y  = −H/R
Z  = −R/H
```

## SPI 帧协议（8-bit，每帧 22 字节）

```
[起始 0xAA][频点 4B][Ui实 4B][Ui虚 4B][Uo实 4B][Uo虚 4B][结束 0x55]
```

- FPGA 为主机（spi_byte 模块），STM32 SPI1 为从机，模式 0（CPOL0/CPHA0）
- DMA1_Stream0 / SPI1_RX，循环模式，收满一帧触发 `HAL_SPI_RxCpltCallback`
- 当前字节序为大端（MSB first），数据按 IEEE-754 float 解析

> 标记值、字节序、字段顺序、频点含义（频率值或 bin 号）目前均为占位，
> 最终以 FPGA 实际协议为准，只需修改 `spi_receiver.h` / `circuit_id.h` 中的宏。

## 计算流程

1. DMA 收满一帧 → 校验帧头帧尾 → 解析为 `FFT_Point_t`
2. `CircuitID_Compute()` 计算：
   - H = Uo·Ui\* / |Ui|²（H1 估计，复数除法）
   - 阻抗 Z = −R/H：幅度 Ω（含 dB）、相位 °
   - 导纳 Y = −H/R
3. 噪声门限：|Ui| 过小或 |Uo| 过小（开路）的频点标记 `valid=0` 剔除

XYZ 输出：**X = 频率(Hz)，Y = |Z|(Ω)，Z = arg(Z)(°)**。

## 调用方式

```c
FFT_Point_t fft;   /* 由 SPI 接收解析得到 */
FRF_Point_t frf;
if (CircuitID_Compute(&fft, &frf)) {
    /* frf.frequency_hz / frf.z_mag_ohm / frf.z_phase_deg */
}
```

## 与 CubeMX 解耦 / 重新生成注意事项

- 全部用户逻辑放在 `USE/`，CubeMX 不会修改
- `main.c` 仅在 `USER CODE` 区内有 `#include "spi_receiver.h"` 与
  `SPI_Receiver_Init()`，重生成会保留
- `spi.c` 与 `.ioc` 已同步为 8-bit，重生成不退回
- 若重生成后 Keil 工程丢失 USE 编译组或头文件路径，手动恢复：
  - 将 `USE/spi_receiver.c`、`USE/circuit_id.c` 加入工程
  - Include Paths 增加 `../USE`

## 构建

Keil 中打开 `MDK-ARM/9_30.uvprojx` 编译，当前为 0 Error / 0 Warning。
