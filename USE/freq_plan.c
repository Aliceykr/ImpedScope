#include "freq_plan.h"

/*
 * 5 个测量频率,与赛题要求一致。顺序扫描 100 -> 300 -> 1000 -> 3000 -> 10000 Hz。
 *
 * 这 5 个值之比为 1:3:10:30:100,在 Fs = 300 kHz 下 Fs/f 全为整数,
 * 窗口取 10 个周期即为整数个周期(见 adc_sampler.h 的校验表)。
 */
static const uint32_t s_freq_plan[FREQ_PLAN_COUNT] = {
    100u, 300u, 1000u, 3000u, 10000u
};

void Freq_SourceSet(uint32_t frequency_hz)
{
    (void)frequency_hz;   /* AD9833 未接入,留空(见 freq_plan.h 的说明) */
}

uint32_t Freq_PlanGet(uint8_t index)
{
    if (index >= (uint8_t)FREQ_PLAN_COUNT)
    {
        return 0u;
    }

    return s_freq_plan[index];
}