# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上，用于接收 FPGA 发送的频域采样结果，并计算被测电路（DUT）的频率响应、阻抗和导纳。

FPGA 对输入电压 `Ui` 和输出电压 `Uo` 做 FFT，将每个频点的复数结果通过 SPI1 发给 STM32。STM32 完成帧校验、数据解析和频响计算。

当前工程已经实现：

- SPI1 从机 8 位接收
- DMA 循环接收固定长度数据帧
- 帧头、帧尾校验
- IEEE-754 单精度浮点数据解析
- 传递函数 `H = Uo / Ui` 计算
- 跨阻 I/V 前端下的阻抗 `Z = -R / H` 和导纳 `Y = -H / R` 计算
- 输入或输出幅值过小时的结果有效性判断

当前 `main()` 没有对计算结果增加串口、网络或存储输出。SPI 回调只负责接收和解析，后续可在上层任务中读取解析结果并调用 `CircuitID_Compute()`。

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ spi_receiver.h/.c   SPI1 帧接收、DMA 启动和数据解析
│  └─ circuit_id.h/.c     频响、阻抗和导纳计算
├─ Core/
│  ├─ Src/main.c          系统初始化和主循环
│  └─ Src/spi.c           SPI1 与 DMA 配置
├─ Drivers/               STM32 HAL 和 CMSIS
├─ MDK-ARM/               Keil 工程文件
└─ 9_30.ioc               STM32CubeMX 配置文件
```

`USE/` 保存用户逻辑，CubeMX 重新生成代码时应保留该目录，并确保 Keil 工程包含两个 `.c` 文件且添加 `../USE` 头文件路径。

## 3. 硬件模型

前端使用跨阻放大器，运放反相端近似虚地，反馈电阻为 `R`：

```text
信号源 ── Ui ── DUT ── 虚地
                  │
                  └── 运放跨阻放大器 ── Uo
```

对应关系为：

```text
I = Ui × Y
Uo = -I × R
H = Uo / Ui = -Y × R
Y = -H / R
Z = -R / H
```

## 4. SPI 帧协议

每帧共 22 字节，字段按大端顺序发送：

```text
[0xAA]
[频点 4 字节]
[Ui 实部 4 字节]
[Ui 虚部 4 字节]
[Uo 实部 4 字节]
[Uo 虚部 4 字节]
[0x55]
```

SPI 配置如下：

- FPGA：主机
- STM32：SPI1 从机
- 模式：Mode 0，CPOL=0，CPHA=0
- 数据宽度：8 bit
- 首位顺序：MSB first
- DMA：`DMA1_Stream0`，SPI1 RX，循环模式

协议相关宏集中在 [USE/spi_receiver.h](USE/spi_receiver.h) 中。若 FPGA 的标记值、字节序或字段定义发生变化，只需修改该文件中的宏和解析顺序。

## 5. 频响计算

`CircuitID_Compute()` 的输入是一个 `FFT_Point_t`，输出是一个 `FRF_Point_t`：

```c
FFT_Point_t input;
FRF_Point_t output;

if (CircuitID_Compute(&input, &output) != 0u) {
    /* output.frequency_hz、output.z_mag_ohm、output.z_phase_deg */
}
```

默认输出可用于 XYZ 数据：

- `X = frequency_hz`：频率，Hz
- `Y = z_mag_ohm`：阻抗幅值，Ω
- `Z = z_phase_deg`：阻抗相位，°

主要计算参数位于 [USE/circuit_id.h](USE/circuit_id.h)：

- `CIR_FREQ_IS_RAW`：频点字段是实际频率还是 FFT bin
- `CIR_FREQ_UNIT`：实际频率单位
- `CIR_FEEDBACK_RESISTOR_OHM`：反馈电阻值
- `CIR_INPUT_MIN_MAG`、`CIR_OUTPUT_MIN_MAG`：有效性门限

## 6. 编译与维护

使用 Keil MDK-ARM 打开 `MDK-ARM/9_30.uvprojx` 编译。工程基于：

- STM32CubeMX 6.18
- STM32H7 HAL FW 1.13
- ARMCC V5
- 系统时钟 480 MHz，HCLK 120 MHz，APB 60 MHz

重新生成 CubeMX 代码后，请检查：

1. `USE/spi_receiver.c` 和 `USE/circuit_id.c` 仍在 Keil 工程中。
2. 工程头文件路径仍包含 `../USE`。
3. SPI1 数据宽度仍为 8 bit，DMA 仍为循环接收。
4. `main.c` 的用户代码区仍调用 `SPI_Receiver_Init()`。
