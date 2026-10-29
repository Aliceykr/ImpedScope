#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# CubeMX 重新生成代码后的自检
#
#     bash .ccg-build/verify.sh [输出目录]
#
# 检查三件事：
#   1) 还能编译：39 个源文件、0 error、代码体积没跑偏
#   2) 生成区里的关键配置还在：
#      双 ADC 同步 + 32 位打包 + DMA 字对齐 + 继电器三脚 + 串口/TIM6
#   3) Keil 工程文件没被生成器吃掉：USE 组、../USE 头文件路径
#
# 全部通过退出码 0，否则 1。任何一项 FAIL 都说明"生成后功能会变"。
# ---------------------------------------------------------------------------
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/.ccg-build/verify-out}"

EXPECT_FILES=40
# 基准取自当前版本：非阻塞状态机 + SPI1 从机/DMA + USART3 恢复 + SPI 从机驱动。
EXPECT_CODE=58514
CODE_TOL=500
fail=0

pass() { printf '  [ OK ] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; fail=1; }

# has <相对路径> <固定串> <说明>
has() {
    if grep -qF -- "$2" "$ROOT/$1" 2>/dev/null; then
        pass "$3"
    else
        bad "$3  —— $1 里找不到: $2"
    fi
}

echo "== 1/3 编译 =="
out="$(bash "$ROOT/.ccg-build/build.sh" "$OUT" 2>&1)"
printf '%s\n' "$out" | grep -E 'compiled:|Program Size:|RESULT:' | sed 's/^/  /'

nf="$(printf '%s\n' "$out" | sed -n 's/^compiled: \([0-9][0-9]*\) files.*/\1/p' | head -1)"
nb="$(printf '%s\n' "$out" | sed -n 's/^compiled: [0-9][0-9]* files, failures: \([0-9][0-9]*\).*/\1/p' | head -1)"
code="$(printf '%s\n' "$out" | sed -n 's/.*Program Size: Code=\([0-9][0-9]*\).*/\1/p' | head -1)"

if [ "${nb:-1}" = "0" ]; then pass "0 error"; else bad "编译有错（0 error 不成立）"; fi
if [ "${nf:-0}" = "$EXPECT_FILES" ]; then
    pass "$nf 个源文件"
else
    bad "源文件数 $nf，期望 $EXPECT_FILES（Keil 工程可能丢了文件）"
fi
if [ -n "${code:-}" ]; then
    if [ "$code" -ge "$EXPECT_CODE" ]; then d=$((code - EXPECT_CODE)); else d=$((EXPECT_CODE - code)); fi
    if [ "$d" -le "$CODE_TOL" ]; then
        pass "Code=$code（基准 $EXPECT_CODE）"
    else
        bad "Code=$code 偏离基准 $EXPECT_CODE 超过 $CODE_TOL"
    fi
else
    bad "拿不到 Program Size"
fi

echo "== 2/3 关键配置 =="
# ADC：同步模式在 USER CODE 里，DMA 字对齐在生成区里，两边都要在
has Core/Src/adc.c "ADC_DUALMODE_REGSIMULT"                    "双 ADC 同步模式"
has Core/Src/adc.c "ADC_DUALMODEDATAFORMAT_32_10_BITS"         "32 位打包格式"
has Core/Src/adc.c "ADC_RESOLUTION_16B"                        "16 位分辨率"
has Core/Src/adc.c "ADC_EXTERNALTRIG_T6_TRGO"                  "TIM6 TRGO 触发"
has Core/Src/adc.c "PeriphDataAlignment = DMA_PDATAALIGN_WORD" "DMA 外设侧字对齐"
has Core/Src/adc.c "MemDataAlignment = DMA_MDATAALIGN_WORD"    "DMA 存储侧字对齐"
has Core/Src/adc.c "__HAL_LINKDMA"                             "ADC1 与 DMA 句柄关联"
has Core/Src/dma.c "DMA1_Stream2_IRQn"                         "DMA1 Stream2 中断使能"
has Core/Src/stm32h7xx_it.c "DMA1_Stream2_IRQHandler"          "DMA1 Stream2 中断入口"
# 继电器
has Core/Inc/main.h "RELAY_K1_Pin"                             "K1 引脚宏（main.h）"
has Core/Inc/main.h "RELAY_K2_Pin"                             "K2 引脚宏（main.h）"
has Core/Inc/main.h "RELAY_K3_Pin"                             "K3 引脚宏（main.h）"
has Core/Src/gpio.c "RELAY_K1_Pin"                             "K1 脚已配置（gpio.c）"
has Core/Src/gpio.c "RELAY_K2_Pin"                             "K2 脚已配置（gpio.c）"
has Core/Src/gpio.c "RELAY_K3_Pin"                             "K3 脚已配置（gpio.c）"
# SPI1 从机 + DMA（FPGA 为主机）
has Core/Src/spi.c "SPI_MODE_SLAVE"                            "SPI1 从机模式"
has Core/Src/spi.c "SPI_NSS_HARD_INPUT"                        "SPI1 硬件 NSS 输入"
has Core/Src/spi.c "hdma_spi1_rx"                              "SPI1 RX DMA 句柄"
has Core/Src/spi.c "DMA_REQUEST_SPI1_RX"                       "SPI1 RX DMA 请求"
# SPI 从机 DMA 优先级检查（§14.4）
has Core/Src/spi.c "hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW" "SPI RX DMA 流优先级 Low"
has Core/Src/dma.c "HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0)" "SPI RX DMA NVIC 优先级 1"
# SPI 从机驱动代码检查
has USE/spi_slave.c "Spi_SlaveInit"                            "SPI 从机初始化函数"
has USE/spi_slave.c "Spi_SlavePoll"                            "SPI 从机轮询函数"
has USE/spi_slave.c "Spi_SlaveIsAlive"                         "SPI 从机存活检测"
has USE/spi_slave.c "Spi_SlaveValidCount"                      "SPI 从机有效帧计数"
has USE/spi_slave.c "Spi_SlaveErrorCount"                      "SPI 从机错误帧计数"
has USE/spi_slave.c "Spi_SlaveOverrunCount"                    "SPI 从机溢出计数"
has USE/spi_slave.c "HAL_SPI_RxHalfCpltCallback"               "SPI DMA 半完成回调"
has USE/spi_slave.c "HAL_SPI_RxCpltCallback"                   "SPI DMA 完成回调"
has USE/spi_slave.c "HAL_SPI_ErrorCallback"                    "SPI 错误回调"
has USE/spi_slave.h "SPI_SLAVE_FRAME_WORDS"                    "SPI 帧字数定义"
has USE/spi_slave.h "SPI_SLAVE_FRAME_SLOTS"                    "SPI 帧槽数定义"
has USE/spi_slave.h "Spi_Frame_t"                              "SPI 帧结构定义"
has USE/measure.c "Spi_SlaveInit()"                            "Measure_Init 调用 Spi_SlaveInit"
has USE/measure.c "Spi_SlavePoll(&spi)"                        "Measure_Service 调用 Spi_SlavePoll"
has USE/measure.c "UartDebug_LogCompare(&frf_point, &spi)"     "Measure_Service 调用 UartDebug_LogCompare"
has USE/measure.c "SCREEN_E21_SPI_LINK"                        "Measure_Service 使用 E21 错误码"
has USE/screen.h "SCREEN_E21_SPI_LINK"                         "screen.h 定义 E21 错误码"
has USE/screen.c "SPI_LINK"                                     "screen.c 定义 E21 错误名称"
has USE/uart_debug.c "UartDebug_LogCompare"                    "uart_debug 实现比对函数"
has USE/uart_debug.c "UartDebug_DropCount"                     "uart_debug 实现丢包计数"
has USE/uart_debug.h "UartDebug_LogCompare"                    "uart_debug.h 声明比对函数"
has USE/uart_debug.h "UartDebug_DropCount"                     "uart_debug.h 声明丢包计数"
has 9_30.ioc "Dma.SPI1_RX"                                     ".ioc 里有 SPI1_RX DMA"
has 9_30.ioc "USART3.BaudRate=921600"                          ".ioc 里 USART3 仍在"
has Core/Src/usart.c "MX_USART3_UART_Init"                     "USART3 初始化函数仍在"
# 初始化与按键服务在重构后位于 measure.c / screen.c（main.c 只剩三个高层调用）
has Core/Src/main.c "Measure_Service();"                       "主循环调用 Measure_Service()"
has Core/Src/main.c "Screen_KeyService();"                     "主循环调用 Screen_KeyService()"
has USE/measure.c "Relay_Init();"                              "上电调用 Relay_Init()"
has USE/measure.c "CircuitID_SetFeedbackResistor(Relay_GetOhm());" "同步反馈电阻 R_eff"
has USE/screen.c "SCREEN_KEY_B0"                               "b0 单击重扫分支"
has USE/screen.c "SCREEN_KEY_B1"                               "b1 单击切档分支"
has USE/screen.c "SCREEN_KEY_B2"                               "b2 自动量程分支"
has USE/screen.c "Screen_PollKey"                              "按键出队（Screen_PollKey）"
has USE/screen.c "SCREEN_KEY_QUEUE_SIZE"                       "按键中断入队/主循环出队队列"
has USE/screen.c "Screen_KeyEnqueue"                           "按键入队函数"
has USE/relay_range.c "uint8_t Relay_Next(void)"               "Relay_Next 已实现"
# 自动量程
has USE/measure.c "Measure_AutoRangeStep"                      "自动量程评估函数"
has USE/measure.c "Measure_ToggleAutoRange"                    "自动量程开关"
has USE/measure.h "MEASURE_AR_TARGET_V"                        "自动量程目标电压宏"
has USE/screen.c "Measure_IsAutoRange"                         "t10 显示自动量程状态"
# 时钟/串口
has Core/Src/tim.c "Prescaler = 3"                             "TIM6 PSC=3（300 kHz）"
has Core/Src/tim.c "Period = 99"                               "TIM6 ARR=99"
baud="$(grep -cF 'BaudRate = 921600;' "$ROOT/Core/Src/usart.c" 2>/dev/null || echo 0)"
if [ "$baud" = "2" ]; then pass "UART4 / USART3 均 921600"; else bad "921600 的波特率有 $baud 处，期望 2"; fi

echo "== 3/3 Keil 工程文件 =="
has MDK-ARM/9_30.uvprojx "../USE/relay_range.c" "relay_range.c 仍在工程里"
has MDK-ARM/9_30.uvprojx "../USE/measure.c"     "measure.c 仍在工程里"
has MDK-ARM/9_30.uvprojx "../USE/screen.c"      "screen.c 仍在工程里"
has MDK-ARM/9_30.uvprojx "../USE/uart_debug.c"  "uart_debug.c 仍在工程里"
has MDK-ARM/9_30.uvprojx "../USE/spi_slave.c"   "spi_slave.c 在工程里"
has MDK-ARM/9_30.uvprojx "../USE"               "头文件路径含 ../USE"

echo
if [ "$fail" = "0" ]; then
    echo "RESULT: PASS —— 生成后的代码与基准一致"
    exit 0
fi
echo "RESULT: FAIL —— 上面标 [FAIL] 的项就是被生成器改掉的功能"
exit 1
