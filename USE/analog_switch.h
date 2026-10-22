#ifndef ANALOG_SWITCH_H
#define ANALOG_SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 74HC4052 反馈电阻切换（4 档自动量程）
 *
 * 拓扑（与手绘图一致，开关在输出侧）：
 *
 *   运放输出 Uo ── X（COM）
 *                    │
 *         X0    X1    X2    X3
 *         │     │     │     │
 *       150Ω  1.5k   15k   150k
 *         └─────┴──┬──┴─────┘
 *                  │
 *            运放反相端（虚地 1.65 V）
 *
 * 公共端接运放输出，四个掷端各接一只反馈电阻，四只电阻另一端共接虚地。
 * 这样切换时的电荷注入被低阻输出吸收，不会直接打进虚地节点。
 *
 * 四只电阻的推导（任务书：Re/Im/|Y| 0.1–10 mS、误差 ≤10%，φY ≤5%）：
 *
 *   实际激励：信号源 1 Vpp、1.65 V 直流偏置 → 峰值 Ui = 0.5 V。
 *
 *   1) 10 mS 时 DUT 电流 I = Ui_pk × 10 mS = 0.5 V × 10 mS = 5 mA，
 *      与选哪一档无关；Uo = I × (R + Ron) 要 ≤ 1.5 V。
 *   2) R + Ron ≤ 1.5 V / 5 mA = 300 Ω；取 150 Ω：
 *        R_eff = 250 Ω，Uo = 1.25 V（留 17% 余量），Ron 占 40%。
 *      这是 0.1–10 mS 里唯一需要 150 Ω 的点，也是 Ron 占比最大的一档。
 *   3) 其余三档按 10 倍间距取 1.5 kΩ / 15 kΩ / 150 kΩ：
 *        R_eff = 1.6 k / 15.1 k / 150.1 k，
 *        Ron 占 6.25% / 0.66% / 0.067%。
 *   4) 覆盖（Uo 0.15–1.5 V，Ui_pk 0.5 V）：
 *        150 Ω   ：1.2 mS – 12 mS
 *        1.5 kΩ  ：0.188 mS – 1.88 mS
 *        15 kΩ   ：19.9 µS – 198.7 µS
 *        150 kΩ  ：2.0 µS – 20.0 µS（低于任务书下限，作备用）
 *      0.1–10 mS 全程覆盖：10 mS 用 150 Ω（1.25 V），
 *      0.1 mS 用 15 kΩ（0.755 V）。
 *   5) 10 kHz 相位：15 kΩ 的 C_off 极点约 700 kHz，相移 <1°；
 *      150 kΩ 的极点约 70 kHz，相移约 8°，但它不在 0.1–10 mS 内，
 *      不影响 φY 指标。
 *
 * 结论：150 Ω / 1.5 kΩ / 15 kΩ / 150 kΩ（E24，10 倍间距）。
 * 10 mS 档 Ron 占 40%，标定后温漂约 2%/10 °C，仍在 10% 内；
 * 若想进一步降低这一档的 Ron 占比，可换低 Ron 开关（ADG16xx）。
 *
 * 注意：之前按 Ui 峰值 1 V（2 Vpp）推的 27/270/2.7k/27k 不适用于 1 Vpp 源：
 * 0.5 V 峰值下 270 Ω 档在 10 mS 时 Uo=1.85 V 会削顶，27 Ω 档又过小。
 *
 * Ron 在 3.3 V 下约 100 Ω（不同厂商 80–150 Ω），以实测为准；s_ohm_eff[]
 * 里的默认值已按 标称值 + 100 Ω 预估，标定后用 Switch_SetCalibratedOhm()
 * 覆盖。若实测 Ron 明显偏大（>150 Ω），150 Ω 档在 10 mS 会到 1.5 V，
 * 余量很小，需要换低 Ron 开关或降低源幅度。
 */

/* 1 = 启用自动量程；0 = 退回固定反馈电阻（CIR_FEEDBACK_RESISTOR_OHM），
 * 便于在不装 4052 的板子上做回归测试。 */
#define SW_ENABLE           1u

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

/* 4052 控制脚：A = bit0，B = bit1；00/01/10/11 → 150/1.5k/15k/150k。
 * 板上若引出的是别的焊盘，只改这四个宏，同时同步 9_30.ioc。 */
#define SW_A_PORT           GPIOE
#define SW_A_PIN            GPIO_PIN_2
#define SW_B_PORT           GPIOE
#define SW_B_PIN            GPIO_PIN_3

typedef enum
{
    SW_RANGE_150 = 0,   /* 150 Ω  + Ron，10 mS 档 */
    SW_RANGE_1K5,       /* 1.5 kΩ + Ron */
    SW_RANGE_15K,       /* 15 kΩ + Ron */
    SW_RANGE_150K       /* 150 kΩ + Ron，备用低端 */
} Switch_Range_t;

/* 初始化 PE2/PE3 为推挽输出，并切到 150 Ω（最小增益，Uo 最低）。
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
