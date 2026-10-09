# 9_30：STM32 黑盒电路频响辨识

## 1. 工程用途

本工程运行在 **STM32H750XBH6** 上，用于接收 FPGA 发送的频域采样结果，并计算被测电路（DUT）的频率响应、阻抗和导纳。

FPGA 对输入电压 `Ui` 和输出电压 `Uo` 做 FFT，将每个频点的复数结果通过 SPI1 发给 STM32。STM32 完成帧同步、数据解析和频响计算。

当前工程已经实现：

- SPI1 从机 8 位接收
- DMA 循环接收连续字节流，并在帧错位后重新同步
- 帧头、帧尾校验
- IEEE-754 单精度浮点数据解析
- 有效频点队列，避免在 DMA 中断中执行浮点计算
- 传递函数 `H = Uo / Ui` 计算
- 跨阻 I/V 前端下的阻抗 `Z = -R / H` 和导纳 `Y = -H / R` 计算
- 输入或输出幅值过小时的结果有效性判断

主循环从接收队列取出频点并调用 `CircuitID_Compute()`，有效结果交给 `Screen_SetResult()` 保存。UART4（9600 8N1）每 0.5 s 通过 `Screen_Poll()` 向串口屏发送最近一个有效频点的三个参数：`t_freq.txt`（频率，Hz）、`t_zmag.txt`（阻抗幅值 |Z|，Ω）、`t_zph.txt`（阻抗相位，°），每条指令以 `FF FF FF` 结尾。串口屏页面中需要建立同名文本框控件。

## 2. 工程结构

```text
9_30/
├─ USE/
│  ├─ spi_receiver.h/.c   SPI1 帧接收、DMA 启动和数据解析
│  ├─ circuit_id.h/.c     频响、阻抗和导纳计算
│  └─ screen.h/.c         串口屏 UART4 测试输出
├─ Core/
│  ├─ Src/main.c          系统初始化和主循环
│  ├─ Src/spi.c           SPI1 与 DMA 配置
│  └─ Src/usart.c         UART4 配置（CubeMX 生成）
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

每帧共 22 字节，字段按大端顺序发送。DMA 缓冲区长度为两帧，半传输和全传输回调各处理 22 字节；解析器在字节流中搜索帧头，校验失败后继续滑动寻找下一帧：

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
- DMA：`DMA1_Stream0`，SPI1 RX，循环模式，双帧缓冲
- 接收队列：最多保存 3 个待处理频点；队列满时丢弃最旧频点

协议相关宏集中在 [USE/spi_receiver.h](USE/spi_receiver.h) 中。若 FPGA 的标记值、字节序或字段定义发生变化，只需修改该文件中的宏和解析顺序。

当前协议没有长度字段、序号或校验和。帧头/帧尾只能帮助恢复字节边界，不能检测所有载荷位翻转；若 FPGA 协议可调整，建议加入帧序号和 CRC，并让 FPGA 在每帧之间提供片选间隔。

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

计算会拒绝 NaN、无穷大、零频点、无效反馈电阻和非有限中间结果。失败时输出结构清零且 `valid=0`。幅值门限需根据 FPGA FFT 的缩放方式和实际噪声水平校准。

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
4. `main.c` 的用户代码区仍调用 `SPI_Receiver_Init()`、`Screen_Init()` 和 `Screen_Poll()`。
5. `USE/screen.c` 仍在 Keil 工程中；`.ioc` 中 UART4 仍为 9600 8N1（PC10/PC11），且 UART4 全局中断已使能。
