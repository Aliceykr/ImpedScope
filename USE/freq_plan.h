#ifndef FREQ_PLAN_H
#define FREQ_PLAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 赛题要求的 5 个测量频率。 */
#define FREQ_PLAN_COUNT   5u

/*
 * 设置信号源(AD9833)到目标频率。
 *
 * 当前为桩函数:不驱动任何硬件,仅由调用方保证节拍正确。
 * 后续接入 AD9833 时,在此写 SPI 频率字,并确认输出频率与 Freq_PlanGet() 返回的
 * 表值完全一致 —— adc_sampler 的 DFT 用表值算旋转因子,两者不一致会让频率轴错位,
 * 窗口不再是整数周期(泄漏不为零),而且不会有任何报错。
 * 另外 AD9833 换频后需要建立时间,应在开窗口前等待稳定。
 */
void Freq_SourceSet(uint32_t frequency_hz);

/* 取第 index 个扫描频率(0..FREQ_PLAN_COUNT-1);越界返回 0。 */
uint32_t Freq_PlanGet(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* FREQ_PLAN_H */