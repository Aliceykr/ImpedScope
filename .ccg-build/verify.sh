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

EXPECT_FILES=39
EXPECT_CODE=51322
CODE_TOL=400
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

echo "== 2/3 生成区关键配置 =="
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
has Core/Src/gpio.c "RELAY_K1_Pin|RELAY_K2_Pin|RELAY_K3_Pin"   "三个继电器脚被统一配置"
has Core/Src/main.c "Relay_Init();"                            "上电调用 Relay_Init()"
has Core/Src/main.c "CircuitID_SetFeedbackResistor(Relay_GetOhm());" "同步固定 R_eff=1500"
# b0 双击切档
has Core/Src/main.c "SCREEN_KEY_DOUBLE"                            "b0 双击切档分支"
has Core/Src/main.c "Screen_PollKey();"                            "Screen_PollKey 每轮调用"
has USE/relay_range.c "uint8_t Relay_Next(void)"                   "Relay_Next 已实现"
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
has MDK-ARM/9_30.uvprojx "../USE"               "头文件路径含 ../USE"

echo
if [ "$fail" = "0" ]; then
    echo "RESULT: PASS —— 生成后的代码与基准一致"
    exit 0
fi
echo "RESULT: FAIL —— 上面标 [FAIL] 的项就是被生成器改掉的功能"
exit 1
