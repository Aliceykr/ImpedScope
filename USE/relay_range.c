#include "relay_range.h"

/* 三档标称电阻（Ω）。继电器接触电阻（<1 Ω）相对最小档 150 Ω 可忽略，
 * 1.5 kΩ 档 R_eff 直接按标称值；切到 150 Ω / 15 kΩ 档时再按档标定。 */
static const float s_ohm[RELAY_COUNT] =
{
    150.0f, 1500.0f, 15000.0f
};

static uint8_t s_index = RELAY_DEFAULT_INDEX;

/* 非阻塞切档状态机 */
static RelaySwitchState_t s_switch_state = RELAY_SW_IDLE;
static uint8_t s_switch_target;          /* 目标档位 */
static uint32_t s_switch_deadline;       /* 当前步骤的超时时间 */

void Relay_Write(uint8_t index, GPIO_PinState state)
{
    switch (index)
    {
        case 0u: HAL_GPIO_WritePin(RELAY_K1_GPIO_Port, RELAY_K1_Pin, state); break;
        case 1u: HAL_GPIO_WritePin(RELAY_K2_GPIO_Port, RELAY_K2_Pin, state); break;
        case 2u: HAL_GPIO_WritePin(RELAY_K3_GPIO_Port, RELAY_K3_Pin, state); break;
        default: break;
    }
}

void Relay_Init(void)
{
    /* 引脚已由 MX_GPIO_Init()（CubeMX 生成，main.h 的 RELAY_Kx_Pin 宏）
     * 配置为推挽输出并写低；本函数只做上电时序。 */

    /* 先全部断开，等继电器释放；不会出现两路同时闭合。 */
    Relay_Write(0u, GPIO_PIN_RESET);
    Relay_Write(1u, GPIO_PIN_RESET);
    Relay_Write(2u, GPIO_PIN_RESET);
    HAL_Delay(RELAY_RELEASE_MS);

    /* 只闭合默认档 K2 = 1.5 kΩ。 */
    s_index = RELAY_DEFAULT_INDEX;
    Relay_Write(s_index, GPIO_PIN_SET);
    HAL_Delay(RELAY_SETTLE_MS);
}

uint8_t Relay_Get(void)
{
    return s_index;
}

float Relay_GetOhm(void)
{
    return s_ohm[s_index];
}

float Relay_GetNominalOhm(uint8_t index)
{
    if (index >= RELAY_COUNT)
    {
        return 0.0f;
    }
    return s_ohm[index];
}

uint8_t Relay_Set(uint8_t index)
{
    if (index >= RELAY_COUNT)
    {
        return 0u;
    }
    if (index == s_index)
    {
        return 1u;              /* 已在目标档，不重复动作 */
    }

    /* 先断开当前档，等触点释放，再闭合目标档：任何时刻最多一路闭合。 */
    Relay_Write(s_index, GPIO_PIN_RESET);
    HAL_Delay(RELAY_RELEASE_MS);

    s_index = index;
    Relay_Write(s_index, GPIO_PIN_SET);
    HAL_Delay(RELAY_SETTLE_MS);

    return 1u;
}

uint8_t Relay_Next(void)
{
    (void)Relay_Set((uint8_t)((s_index + 1u) % RELAY_COUNT));

    return s_index;
}

/* ==================== 非阻塞实现 ==================== */

RelaySwitchState_t Relay_GetSwitchState(void)
{
    return s_switch_state;
}

uint8_t Relay_IsSwitching(void)
{
    return (s_switch_state != RELAY_SW_IDLE) ? 1u : 0u;
}

uint8_t Relay_SetAsync(uint8_t index)
{
    if (index >= RELAY_COUNT)
    {
        return 0u;
    }
    if (index == s_index)
    {
        return 1u;  /* 已在目标档 */
    }

    switch (s_switch_state)
    {
        case RELAY_SW_IDLE:
            /* 开始切换：断开当前档 */
            s_switch_target = index;
            Relay_Write(s_index, GPIO_PIN_RESET);
            s_switch_state = RELAY_SW_RELEASING;
            s_switch_deadline = HAL_GetTick() + RELAY_RELEASE_MS;
            return 0u;

        case RELAY_SW_RELEASING:
            /* 等待释放完成 */
            if ((int32_t)(HAL_GetTick() - s_switch_deadline) >= 0)
            {
                /* 释放完成，闭合目标档 */
                s_index = s_switch_target;
                Relay_Write(s_index, GPIO_PIN_SET);
                s_switch_state = RELAY_SW_CLOSING;
                s_switch_deadline = HAL_GetTick() + RELAY_SETTLE_MS;
            }
            return 0u;

        case RELAY_SW_CLOSING:
            /* 等待建立完成 */
            if ((int32_t)(HAL_GetTick() - s_switch_deadline) >= 0)
            {
                s_switch_state = RELAY_SW_IDLE;
                return 1u;  /* 完成 */
            }
            return 0u;
    }

    /* 不应该到这里 */
    s_switch_state = RELAY_SW_IDLE;
    return 1u;
}

uint8_t Relay_NextAsync(void)
{
    uint8_t next = (uint8_t)((s_index + 1u) % RELAY_COUNT);
    return Relay_SetAsync(next);
}
