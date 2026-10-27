#ifndef RELAY_RANGE_H
#define RELAY_RANGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 三路独立继电器反馈电阻切换（固定 1.5 kΩ 版）
 *
 * 拓扑：每路继电器串联一只反馈电阻，并联在运放输出 Uo 与虚地之间：
 *
 *   Uo ── K1 触点 ── 150 Ω  ──┐
 *      ├─ K2 触点 ── 1.5 kΩ ──┼── 虚地 1.65 V
 *      └─ K3 触点 ── 15 kΩ  ──┘
 *
 * 极性：高电平 = 继电器线圈得电 = 触点闭合；低电平/GND = 断开。
 * GPIO 只驱动继电器模块的逻辑输入（模块自带三极管/MOS 驱动与续流）。
 *
 * 本版不做运行期切档：上电只闭合默认档 K2（1.5 kΩ），K1/K3 保持断开，
 * 固件固定按 1500 Ω 计算 H/Z/Y，与 s_calib（Rf=1500）标定值对齐。
 * 后续要做切档时再扩展 Relay_Set()，并保持"先全断、再闭合目标"的时序。
 */

#define RELAY_COUNT          3u
#define RELAY_SETTLE_MS      10u   /* 吸合建立等待 */
#define RELAY_RELEASE_MS     10u   /* 全部断开后的释放等待 */
#define RELAY_DEFAULT_INDEX  1u    /* K2 = 1.5 kΩ */

/* 控制脚引脚宏由 main.h 提供（RELAY_K1_Pin/RELAY_K1_GPIO_Port 等，
 * 与 9_30.ioc 的 GPIO_Label=RELAY_K1/K2/K3 对应，CubeMX 生成）。
 * 本模块不再重复定义，避免双份命名。 */

/* 在 MX_GPIO_Init() 之后调用：引脚已由 CubeMX 生成代码配成推挽输出并写低，
 * 本函数只做上电时序——三路全释放 → 等 10 ms → 只闭合 K2（1.5 kΩ）→ 等吸合。
 * 必须在 ADC_Sampler_Init() 之前调用，避免 TIA 开环期间采到无效窗口。 */
void Relay_Init(void);

/* 当前档位（本版恒为 RELAY_DEFAULT_INDEX）。 */
uint8_t Relay_Get(void);

/* 当前档的反馈电阻（Ω）。本版恒为 1500.0f。 */
float Relay_GetOhm(void);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_RANGE_H */
