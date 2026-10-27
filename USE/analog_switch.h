#ifndef ANALOG_SWITCH_H
#define ANALOG_SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 74HC4052 反馈电阻切换（4 档自动量程）
 *
 * 拓扑（开关在输出侧）：
 *
 *   运放输出 Uo ── COM
 *                    │
 *        OA1   OA2   OA3   OA4
 *         │     │     │     │
 *        15Ω  150Ω  1.5k  15k
 *         └─────┴──┬──┴─────┘
 *                  │
 *            运放反相端（虚地 1.65 V）
 *
 * 控制关系（E# / INH 硬件接 GND）：
 *   E2 = A0，E3 = A1
 *   00 → OA1(15 Ω)
 *   01 → OA2(150 Ω)
 *   10 → OA3(1.5 kΩ)
 *   11 → OA4(15 kΩ)
 *
 * 四档电阻按板上实际网络：15 Ω / 150 Ω / 1.5 kΩ / 15 kΩ。
 * 默认从 15 Ω 起步（最低增益，Uo 最低），再由 Switch_Recommend()/
 * Switch_Update() 自动往上切。
 *
 * Ron 在 3.3 V 下约 100 Ω（不同厂商 80–150 Ω），以实测为准；s_ohm_eff[]
 * 的默认值按“标称值 + 100 Ω”预估，标定后用 Switch_SetCalibratedOhm()
 * 覆盖。15 Ω 档 Ron 占比最大，必须优先标定该档。
 */

/* 1 = 启用自动量程；0 = 退回固定反馈电阻（CIR_FEEDBACK_RESISTOR_OHM），
 * 便于在不装 4052 的板子上做回归测试。 */
#define SW_ENABLE           0u

#define SW_COUNT            4u

/* 换档后等待时间：4052 建立时间 + 运放退出饱和。10 ms 对 100 ms 窗口占 10%，
 * 对 100 Hz 的 1 s 窗口占 1%，可以接受。 */
#define SW_SETTLE_MS        10u

/* 3.3 V 下 4052 的 Ron 估算值，仅用于默认 R_eff；实际以标定为准。 */
#define SW_RON_NOMINAL      100.0f

/* 换档门限（相对 1.65 V 偏置的峰值电压）：
 *   预测下一档 ≤ 1.32 V 才升档（留 12% 余量到 1.5 V）；
 *   当前档 > 1.43 V 立即降档。 */
#define SW_UO_UP_V          1.32f
#define SW_UO_DOWN_V        1.43f

/* 削波判据：贴边样本占比 > 1%（千分之 10）即认为 Uo 削顶。 */
#define SW_CLIP_PERMILLE    10u

/* 4052 控制脚：A0 = E2(PE2)，A1 = E3(PE3)；
 * 00/01/10/11 → 15/150/1.5k/15kΩ。E#(INH) 固定接 GND。 */
#define SW_A0_PORT          GPIOE
#define SW_A0_PIN           GPIO_PIN_2
#define SW_A1_PORT          GPIOE
#define SW_A1_PIN           GPIO_PIN_3

typedef enum
{
    SW_RANGE_15 = 0,    /* 15 Ω  + Ron */
    SW_RANGE_150,       /* 150 Ω + Ron */
    SW_RANGE_1K5,       /* 1.5 kΩ + Ron */
    SW_RANGE_15K        /* 15 kΩ + Ron */
} Switch_Range_t;

/* 初始化 PE2/PE3 为推挽输出，并切到 15 Ω（最小增益，Uo 最低）。
 * 必须在 ADC 启动前调用；内部会等 SW_SETTLE_MS。 */
void Switch_Init(void);

/* 切到指定档。index 越界返回 0；与当前档相同也返回 0（不重复等建立时间）。
 * 返回 1 表示实际切换并已完成建立。 */
uint8_t Switch_Set(uint8_t index);

/* 当前档位（0..SW_COUNT-1）。 */
uint8_t Switch_Get(void);

/* 当前档的反馈电阻（Ω，含 Ron 与标定修正）。CircuitID 用它计算 Y/Z。 */
float Switch_GetOhm(void);

/* 第 index 档的标称电阻（Ω，不含 Ron），用于标定和显示。 */
float Switch_GetNominalOhm(uint8_t index);

/* 写入第 index 档的实测 R_eff（Ω）。标定时使用；默认值 = 标称 + Ron。 */
void Switch_SetCalibratedOhm(uint8_t index, float ohm);

/* 根据当前测得 Uo 峰值电压，推荐“不削顶的最大档”。
 * 只做计算，不切档；返回推荐档位（0..SW_COUNT-1）。 */
uint8_t Switch_Recommend(float uo_peak_v);

/* 运行时换档：削波或 Uo 过高立即降档；预测下一档 ≤ 1.32 V 时升档，
 * 升档需连续 2 个窗口确认（滞回）。返回 1 表示已切档，调用方应丢弃
 * 当前结果、同步 CircuitID 的 R 并重开窗口。 */
uint8_t Switch_Update(float uo_peak_v, uint32_t clip_count, uint32_t sample_count);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_SWITCH_H */
