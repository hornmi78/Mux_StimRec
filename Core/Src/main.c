/* ------------------------------------------------------------------------- */
/*  main.c – AD5624R-3 streamer, one-DMA + TIM6 IRQ guard (works on L432KC)   */
/* ------------------------------------------------------------------------- */
#include "main.h"
#include <math.h>
#include <stdbool.h>

/* ---------- user-tweakable ------------------------------------------------ */
#define USER_WAVE_FREQ_HZ   10.0f      /* output sine frequency (Hz) */
#define USER_WAVE_AMP_V     1.456f        /* peak-to-peak amplitude     */
#define LUT_SIZE            128U
#define VREF                1.25f       /* AD5624R-3 int. ref ×1 */
#define DAC_FS              4095.0f     /* 12-bit full-scale        */

/* ---------- pin aliases --------------------------------------------------- */
#define DAC_SYNC_Pin        GPIO_PIN_4
#define DAC_SYNC_GPIO_Port  GPIOA

/* ---------- CubeMX handles (defined once here) --------------------------- */
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
TIM_HandleTypeDef htim6;

/* ---------- data ---------------------------------------------------------- */
static uint16_t sineLUT[LUT_SIZE];
static uint8_t  txBuf[3];
static uint32_t lutIndex = 0;

/* ---------- helpers ------------------------------------------------------- */
static inline uint32_t makeFrame(uint8_t cmd, uint8_t addr, uint16_t data)
{
    /* C[2:0] A[2:0] D[11:0] … left-aligned to DB23..0               */
    return ((cmd  & 0x07u) << 19) |
           ((addr & 0x07u) << 16) |
           ((data & 0x0FFFu) << 4);
}

/* ------------------------------------------------------------------------- */
/*  TIM6 interrupt – one sample                                              */
/* ------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    /* skip if previous 3-byte DMA still shifting */
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
        return;

    /* scale unity sine to user amplitude */
    float frac = (sineLUT[lutIndex] / DAC_FS) *
                 (USER_WAVE_AMP_V / (2*VREF));
    uint16_t code = (uint16_t)lroundf(frac * DAC_FS);

    uint32_t frame = makeFrame(0x3, 0, code);   /* write & update ch-A */

    txBuf[0] = frame >> 16;
    txBuf[1] = frame >> 8;
    txBuf[2] = frame;

    HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET); /* SYNC ↓ */
    HAL_SPI_Transmit_DMA(&hspi1, txBuf, 3);                              /* 3 bytes */

    lutIndex = (lutIndex + 1U) & (LUT_SIZE - 1U);
}

/* SPI-DMA complete → SYNC ↑ */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);
}

/* ---------- timer helper -------------------------------------------------- */
static void TIM6_SetSampleRate(uint32_t pclk, float fout)
{
    float f_up   = fout * LUT_SIZE;
    uint32_t tck = (uint32_t)lroundf(pclk / f_up);
    if (tck < 2) tck = 2;

    uint32_t psc = (tck - 1U) / 65536U;
    uint32_t arr = (tck / (psc + 1U)) - 1U;

    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_SET_PRESCALER(&htim6,  psc);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_ENABLE(&htim6);
}

/* ---------- app init ------------------------------------------------------ */
static void App_Init(void)
{
    /* build unity LUT */
    for (uint32_t i = 0; i < LUT_SIZE; ++i)
        sineLUT[i] = (uint16_t)lroundf(
                        (sinf(2.0f*M_PI*i/LUT_SIZE)*0.5f + 0.5f) * DAC_FS);

    /* enable internal reference */
    uint32_t ref = makeFrame(0x7, 0, 0x0001);
    uint8_t tmp[3] = {ref>>16, ref>>8, ref};
    HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port,DAC_SYNC_Pin,GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tmp, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port,DAC_SYNC_Pin,GPIO_PIN_SET);

    /* set sample-timer */
    TIM6_SetSampleRate(HAL_RCC_GetPCLK1Freq(), USER_WAVE_FREQ_HZ);
}

/* ---------- main ---------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_TIM6_Init();

    App_Init();

    HAL_TIM_Base_Start_IT(&htim6);   /* start the waveform engine */

    while (1) {
    	__WFI();   				/* CPU sleeps */
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
