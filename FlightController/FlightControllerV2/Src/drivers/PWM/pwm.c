#include "stm32f1xx_hal.h"
#include "logging.h"
#include "UAV_Defines.h"

#include "pwm.h"

/*
* Defines
*/

#define LOG_TAG ("PWM")
#define PWM_DEBUG (0)

#if PWM_DEBUG
#define LOG(...) LOGI(__VA_ARGS__)
#else
#define LOG(...)
#endif

#define PWM_FREQUENCY_DEFAULT (50) // 50hz
#define TIM_FREQUENCY (1000000) // 1mhz
#define TIM_PER_CNT (0.001) // ms

#define PWN_MAX_PULSEWIDTH (2) // ms
#define PWM_MIN_PULSEWIDTH (1) // ms

#define DUTYCYCLE_MIN (1000) // PWM_MIN_PULSEWIDTH / TIM_PER_CNT

/*
* Static
*/

extern TIM_HandleTypeDef htim1;

/*
* Prototypes
*/

static bool MX_TIM1_Init(void);
// extern "C" void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/*
* Code
*/

/**
* @brief TIM1 Initialization Function
* @param None
* @retval None
*/
static bool MX_TIM1_Init(void)
{
#if 0
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 64000;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        return false;
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        return false;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        return false;
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        return false;
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        return false;
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
#endif
    return true;
}

bool PWM_Init()
{
    if (!MX_TIM1_Init()) {
        LOGE("PWM Init failed!!\n");
        return false;
    }

    // set all dutycycle to 0 upon init
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR4 = 0;

    return true;
}

void PWM_Start()
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    htim1.Instance->CCR4 = 0;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void PWM_Stop()
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void PWM_SetDutyCycle(PWMChannelType channel, int dutyCycle)
{
    LOG("PWM channel %d, dutycycle %d\r\n", channel, dutyCycle);
    if (dutyCycle > UAV_MOTOR_MAX_DUTYCYCLE) dutyCycle = UAV_MOTOR_MAX_DUTYCYCLE;
    if (dutyCycle < UAV_MOTOR_MIN_DUTYCYCLE) dutyCycle = UAV_MOTOR_MIN_DUTYCYCLE;

    switch (channel) {
    case PWM_CHANNEL_1:
        htim1.Instance->CCR1 = dutyCycle + DUTYCYCLE_MIN;
        break;
    case PWM_CHANNEL_2:
        htim1.Instance->CCR2 = dutyCycle + DUTYCYCLE_MIN;
        break;
    case PWM_CHANNEL_3:
        htim1.Instance->CCR3 = dutyCycle + DUTYCYCLE_MIN;
        break;
    case PWM_CHANNEL_4:
        htim1.Instance->CCR4 = dutyCycle + DUTYCYCLE_MIN;
        break;
    }
}