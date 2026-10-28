#ifndef RELAY_RANGE_H
#define RELAY_RANGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 三路独立继电器反馈电阻切换（三档 150 Ω / 1.5 kΩ / 15 kΩ）
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
 * 上电只闭合默认档 K2（1.5 kΩ），固件按 1500 Ω 计算 H/Z/Y，
 * 与 s_calib（Rf=1500）标定值对齐；运行期由 Relay_Set()/Relay_Next()
 * 手动切档（b0 双击触发），保持"先全断、再闭合目标"的时序。
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

/* 当前档位（0=150 Ω、1=1.5 kΩ、2=15 kΩ；上电为 RELAY_DEFAULT_INDEX，切档后随之变化）。 */
uint8_t Relay_Get(void);

/* 当前档的反馈电阻（Ω）。上电为 1500.0f，切档后随之变化。 */
float Relay_GetOhm(void);

/* 指定档的标称电阻（Ω）。index 越界返回 0。 */
float Relay_GetNominalOhm(uint8_t index);

/* 切到指定档（0=150 Ω、1=1.5 kΩ、2=15 kΩ）。先断开当前档、等释放，
 * 再闭合目标档、等建立。返回 1=成功，0=参数越界（此时继电器状态不变）。
 * 重复切到当前档会直接返回 1，不做任何动作。 */
uint8_t Relay_Set(uint8_t index);

/* 切到下一档（1.5k → 15k → 150 → 1.5k 循环），返回切换后的档位。
 * 内部调用 Relay_Set()，会阻塞约 RELAY_RELEASE_MS + RELAY_SETTLE_MS。 */
uint8_t Relay_Next(void);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_RANGE_H */
