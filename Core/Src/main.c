/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "measure.h"
#include "screen.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 单个频率等待窗口完成的超时。默认 1 kHz 时窗口 100 ms;屏幕最低档 100 Hz 时
 * 100 周期窗口为 1 s,所以取 1500 ms 留 500 ms 裕量。
 * 注意这是墙钟时间;不要改成"循环计数",HAL_Delay(1) 因 SysTick 的 +1 分频
 * 实际可能耗 2 ms,计数式超时会翻倍。
 *
 * 再低的频率窗口会超过本超时,所以 Measure_SetFrequency() 不接受;
 * 若以后要测更低频,必须继续放宽本超时或降低 ADC_SAMPLER_WINDOW_CYCLES。 */
#define MEASURE_WAIT_TIMEOUT_MS   1500u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t FreqScan_Select(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 外部信号源只会在 100/300/1k/3k/10k 五档之间切换。
 * 依次用五个候选频率开一个 DFT 窗口，按 Ui 的单频幅值和相干性选最可信的一档。
 * 候选顺序把常用的 1k、10k 放前面; 命中一个高相干候选就提前结束。
 * 返回选中的频率; 五档都没有可信信号时返回 0, 并保持默认 1 kHz。 */
static uint32_t FreqScan_Select(void)
{
  static const uint32_t candidates[] = { 1000u, 10000u, 3000u, 300u, 100u };
  uint32_t best_f = 0u;
  double   best_score = 0.0;
  uint8_t  i;

  Screen_ClearResult();

  for (i = 0u; i < (uint8_t)(sizeof(candidates) / sizeof(candidates[0u])); ++i)
  {
    uint32_t     f = candidates[i];
    uint32_t     deadline;
    uint8_t      got = 0u;
    ADC_Window_t window;

    if (ADC_Sampler_SetFrequency(f) == 0u)
    {
      continue;
    }

    deadline = HAL_GetTick() + MEASURE_WAIT_TIMEOUT_MS;
    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
      if (ADC_Sampler_GetWindow(&window) != 0u)
      {
        got = 1u;
        break;
      }
      Screen_Poll();
      HAL_Delay(1u);
    }

    if ((got == 0u) || (window.sample_count == 0u))
    {
      continue;
    }

    {
      double n = (double)window.sample_count;
      double ui_mag = 2.0 * sqrt((window.ui_re * window.ui_re) +
                                 (window.ui_im * window.ui_im)) / n;
      double ui_coh = 0.0;
      double score;

      if (window.ui_rms > 0.0)
      {
        ui_coh = ui_mag / (1.41421356237 * window.ui_rms);
      }

      score = ui_mag * ui_coh;
      if ((ui_mag > (double)CIR_INPUT_MIN_AMP) && (ui_coh > 0.5) &&
          (score > best_score))
      {
        best_score = score;
        best_f = f;

        /* 相干性接近 1, 已经确定是这一档, 不必再等更低的候选频率。 */
        if (ui_coh > 0.95)
        {
          break;
        }
      }
    }
  }

  if (best_f == 0u)
  {
    (void)Measure_SetFrequency(1000u);
    (void)ADC_Sampler_SetFrequency(1000u);
    return 0u;
  }

  (void)Measure_SetFrequency(best_f);
  (void)ADC_Sampler_SetFrequency(best_f);
  (void)Screen_SetActiveFreq(best_f);
  return best_f;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  Screen_SetStage("BOOT");
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  Screen_SetStage("CLK");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  Screen_Init();                       /* 同时启动 b0(freq_change) 按键接收 */
  Screen_SetStage("PERIPH");

  /* 激励源在外部:信号源直接接到 Ui 节点(PA0_C),固件不驱动任何 DDS。
   * 因此这里没有信号源初始化步骤;SPI1 仍然初始化但不再有从设备。 */

  CircuitID_Init();
  Screen_SetStage("MODULE");

  {
    /* 只调用一次:返回 2 = 双 ADC 同步配置未生效(报 E16),0 = 其它初始化失败(E01)。
     * 前者是"静默失效"防护,配置丢失不会自己报错,只会让 H/Z/Y 悄悄偏掉。 */
    uint8_t adc_init = ADC_Sampler_Init();

    if (adc_init == 2u)
    {
      Screen_Fatal(SCREEN_E16_ADC_DUAL_MODE);
    }
    if (adc_init != 1u)
    {
      Screen_Fatal(SCREEN_E01_HAL_INIT);
    }
  }
  Screen_SetStage("SCAN");
  {
    uint32_t detected = FreqScan_Select();

    Screen_SetStage("LOOP");
    if (detected == 0u)
    {
      Screen_ReportError(SCREEN_E04_CIR_UI_LOW);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {
  uint8_t scan_fail_count = 0u;

  while (1)
  {
    uint32_t f;
    ADC_Window_t window;
    FRF_Point_t frf_point;
    uint32_t deadline;
    uint8_t  got = 0u;

    /* HMI 的 b0(freq_change) 现在作为“强制刷新”：
     * 按下后立刻重新扫描五档并重测，不再用硬件复位。 */
    if ((Screen_TakeRefreshKey() != 0u) || (scan_fail_count >= 3u))
    {
      Screen_SetStage("SCAN");
      if (FreqScan_Select() != 0u)
      {
        Screen_SetStage("LOOP");
      }
      else
      {
        Screen_SetStage("LOOP");
        Screen_ReportError(SCREEN_E04_CIR_UI_LOW);
      }
      scan_fail_count = 0u;
    }

    f = Measure_GetFrequency();

    /* 外部信号源必须同步到同一频率,固件只改自己的 DFT/窗口频率。 */
    Freq_SourceSet(f);                    /* 空实现:激励由外部信号源提供,固件不设定 */
    if (ADC_Sampler_SetFrequency(f) == 0u)
    {
      Screen_ReportError(SCREEN_E02_CIR_CFG);
      Screen_ClearResult();
      ++scan_fail_count;
      Screen_Poll();
      HAL_Delay(10u);
      continue;
    }

    /* ADC 已按 f 开始新窗口,这时才让 t10 显示 f。
     * 若显示频率变了,清掉上一频率的读数,避免 t0~t9 还是旧频率的结果。 */
    if (Screen_SetActiveFreq(f) != 0u)
    {
      Screen_ClearResult();
    }

    /* 等本频率的窗口完成。用墙钟比较,不用循环计数(见 MEASURE_WAIT_TIMEOUT_MS)。 */
    deadline = HAL_GetTick() + MEASURE_WAIT_TIMEOUT_MS;
    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
      if (ADC_Sampler_GetWindow(&window) != 0u)
      {
        got = 1u;
        break;
      }
      Screen_Poll();
      HAL_Delay(1u);
    }

    if (got == 0u)
    {
      Screen_ReportError(SCREEN_E12_WINDOW_TIMEOUT);
      Screen_ClearResult();               /* 不留上一频率的陈旧读数 */
      ++scan_fail_count;
      continue;
    }

    /* 新窗口开始计算前清掉上一窗口的错误显示；若本窗口仍异常，
     * CircuitID_Compute() 会通过 Screen_ReportError() 重新置位。 */
    Screen_ClearError();

    if (CircuitID_Compute(&window, &frf_point) != 0u)
    {
      Screen_SetResult(&frf_point);
      scan_fail_count = 0u;
    }
    else
    {
      Screen_ClearResult();               /* 计算失败同样清掉旧读数 */
      ++scan_fail_count;
    }
    Screen_Poll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  Screen_FatalReportBlocking();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

