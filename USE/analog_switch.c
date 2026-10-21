#include "analog_switch.h"

#if (SW_ENABLE != 0u)

/* 四档标称电阻（Ω）。E24 常用值，10 倍间距。 */
static const float s_ohm_nominal[SW_COUNT] =
{
    27.0f, 270.0f, 2700.0f, 27000.0f
};

/* 默认 R_eff = 标称值 + Ron 估算。标定后用 Switch_SetCalibratedOhm() 覆盖。 */
static float s_ohm_eff[SW_COUNT] =
{
    27.0f + SW_RON_NOMINAL,
    270.0f + SW_RON_NOMINAL,
    2700.0f + SW_RON_NOMINAL,
    27000.0f + SW_RON_NOMINAL
};

static uint8_t s_index    = 0u;
static uint8_t s_up_count = 0u;

/* A/B 编码：00 → X0(27Ω)，01 → X1(270Ω)，10 → X2(2.7k)，11 → X3(27k)。 */
static void Switch_WritePins(uint8_t index)
{
    HAL_GPIO_WritePin(SW_A_PORT, SW_A_PIN,
                      ((index & 0x01u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SW_B_PORT, SW_B_PIN,
                      ((index & 0x02u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Switch_Init(void)
{
    GPIO_InitTypeDef init = {0};

    /* 当前固定用 PE2/PE3。若改到别的端口，这里和 9_30.ioc 要一起改。 */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    init.Pin   = SW_A_PIN;
    init.Mode  = GPIO_MODE_OUTPUT_PP;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SW_A_PORT, &init);

    init.Pin = SW_B_PIN;
    HAL_GPIO_Init(SW_B_PORT, &init);

    s_index    = 0u;
    s_up_count = 0u;
    Switch_WritePins(0u);
    HAL_Delay(SW_SETTLE_MS);
}

uint8_t Switch_Set(uint8_t index)
{
    if (index >= SW_COUNT)
    {
        return 0u;
    }
    if (index == s_index)
    {
        return 0u;
    }

    Switch_WritePins(index);
    s_index    = index;
    s_up_count = 0u;
    HAL_Delay(SW_SETTLE_MS);
    return 1u;
}

uint8_t Switch_Get(void)
{
    return s_index;
}

float Switch_GetOhm(void)
{
    return s_ohm_eff[s_index];
}

float Switch_GetNominalOhm(uint8_t index)
{
    if (index >= SW_COUNT)
    {
        return 0.0f;
    }
    return s_ohm_nominal[index];
}

void Switch_SetCalibratedOhm(uint8_t index, float ohm)
{
    if ((index < SW_COUNT) && (ohm > 0.0f))
    {
        s_ohm_eff[index] = ohm;
    }
}

/* 选“预测不超 1.32 V 的最大档”。R_eff 随档位递增，所以最后一个满足的
 * 就是最大的合格档；从 27 Ω 起步时最多三跳就能到 27 kΩ。 */
uint8_t Switch_Recommend(float uo_peak_v)
{
    uint8_t best = s_index;
    uint8_t i;
    float   base;

    if (uo_peak_v <= 0.0f)
    {
        return s_index;
    }

    base = s_ohm_eff[s_index];
    if (base <= 0.0f)
    {
        return s_index;
    }

    for (i = 0u; i < SW_COUNT; ++i)
    {
        float predicted = uo_peak_v * (s_ohm_eff[i] / base);
        if (predicted <= SW_UO_UP_V)
        {
            best = i;
        }
    }

    return best;
}

uint8_t Switch_Update(float uo_peak_v, uint32_t clip_count, uint32_t sample_count)
{
    float predicted;

    if (sample_count == 0u)
    {
        return 0u;
    }

    /* 削波 > 1%：立即降档，安全优先。 */
    if ((clip_count * 1000u) > (sample_count * SW_CLIP_PERMILLE))
    {
        if (s_index > 0u)
        {
            return Switch_Set((uint8_t)(s_index - 1u));
        }
        return 0u;
    }

    /* 接近满摆幅：预防性降档。 */
    if (uo_peak_v > SW_UO_DOWN_V)
    {
        if (s_index > 0u)
        {
            return Switch_Set((uint8_t)(s_index - 1u));
        }
        return 0u;
    }

    /* 预测下一档仍不削顶：升档（连续 2 个窗口确认，避免边界抖动）。
     * 优先大电阻：Ron 占比更小、精度更好、信噪比更高。 */
    if ((s_index + 1u) < SW_COUNT)
    {
        predicted = uo_peak_v * (s_ohm_eff[s_index + 1u] / s_ohm_eff[s_index]);
        if (predicted <= SW_UO_UP_V)
        {
            if (s_up_count < 2u)
            {
                ++s_up_count;
            }
            if (s_up_count >= 2u)
            {
                s_up_count = 0u;
                return Switch_Set((uint8_t)(s_index + 1u));
            }
        }
        else
        {
            s_up_count = 0u;
        }
    }
    else
    {
        s_up_count = 0u;
    }

    return 0u;
}

#else /* SW_ENABLE == 0：退回固定反馈电阻，保留旧行为 */

#include "measure.h"

void Switch_Init(void)
{
}

uint8_t Switch_Set(uint8_t index)
{
    (void)index;
    return 0u;
}

uint8_t Switch_Get(void)
{
    return 0u;
}

float Switch_GetOhm(void)
{
    return (float)CIR_FEEDBACK_RESISTOR_OHM;
}

float Switch_GetNominalOhm(uint8_t index)
{
    (void)index;
    return 0.0f;
}

void Switch_SetCalibratedOhm(uint8_t index, float ohm)
{
    (void)index;
    (void)ohm;
}

uint8_t Switch_Recommend(float uo_peak_v)
{
    (void)uo_peak_v;
    return 0u;
}

uint8_t Switch_Update(float uo_peak_v, uint32_t clip_count, uint32_t sample_count)
{
    (void)uo_peak_v;
    (void)clip_count;
    (void)sample_count;
    return 0u;
}

#endif /* SW_ENABLE */
