/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbusSlave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
#define VREFINT_CAL_ADDR 0x1FFF7A2A
uint16_t VREFINT_CAL;
uint16_t vdda;
uint8_t i;
uint16_t frame[N_SAMPLES * N_FRAMES];
uint8_t frame_8int_V[2 * N_SAMPLES * N_FRAMES];

uint8_t buf[8];
uint8_t RxData[256];
uint8_t TxData[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void ADC_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC->CCR |= ADC_CLOCK_SYNC_PCLK_DIV2;

  ADC1->CR1 = 0;
  ADC1->CR2 = 0;

  ADC1->CR1 |= ADC_RESOLUTION_12B;

  ADC1->SMPR1 = 0;
  ADC1->SMPR1 |= ADC_SMPR1_SMP17_2 | ADC_SMPR1_SMP17_1 | ADC_SMPR1_SMP17_0;
  ADC1->SMPR2 = 0;
  ADC1->SQR1 = 0;
  ADC1->SQR3 = (17 << 0);
  ADC->CCR |= ADC_CCR_TSVREFE;
  ADC1->CR2 |= ADC_CR2_EOCS; // EOC сразу после каждого преобразования
}
void ADC_DMA_Init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  DMA2_Stream0->PAR = ADC1_BASE + 0x4C;
  DMA2_Stream0->M0AR = (uint32_t)frame;
  DMA2_Stream0->NDTR = N_SAMPLES;
  DMA2_Stream0->CR = 0;

  DMA2_Stream0->CR |= DMA_PRIORITY_VERY_HIGH;
  DMA2_Stream0->FCR = 0;

  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0   // 16‑бит в памяти
                      | DMA_SxCR_PSIZE_0 // 16‑бит на периферии
                      | DMA_SxCR_MINC    // инкремент адреса памяти
                      | DMA_SxCR_TCIE;   // прерывание по завершению

  NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}
void TIM2_Init()
{
  // Включение тактирования TIM2
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Сброс настроек таймера
  TIM2->CR1 = 0;
  TIM2->CR2 = 0;
  TIM2->SMCR = 0;
  TIM2->CCER = 0;

  // Настройка предделителя и периода
  TIM2->PSC = 0;   // Предделитель 0 (деление на 1)
  TIM2->ARR = 100; // Период 100 тактов

  // Настройка канала 2 в режиме Toggle
  TIM2->CCMR1 &= ~TIM_CCMR1_OC2M; // Сброс предыдущих настроек
  TIM2->CCMR1 |= (0b011 << 8);    // OC2M = 011 (Toggle mode)

  // Установка значения сравнения
  TIM2->CCR2 = 1; // Значение сравнения для канала 2

  // Настройка режима однократного импульса (One Pulse Mode)
  TIM2->CR1 |= TIM_CR1_OPM; // One Pulse Mode

  // Настройка внешнего триггера (ETR)
  TIM2->SMCR |= (0b100 << 4); // TS = 100 (ETRF)
  TIM2->SMCR |= (0b110 << 0); // SMS = 110 (Trigger mode)

  // Настройка фильтра и полярности ETR
  TIM2->SMCR &= ~(0xF << 8);   // ETF = 0000 (без фильтра)
  TIM2->SMCR &= ~TIM_SMCR_ETP; // ETP = 0 (положительная полярность)

  // Включение ETR
  TIM2->SMCR |= TIM_SMCR_ECE; // External Clock Enable

  // Принудительное обновление регистров
  TIM2->EGR |= TIM_EGR_UG; // Принудительное событие обновления

  // Очистка всех флагов
  TIM2->SR = 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (RxData[0] == SLAVE_ID)
  {
    switch (RxData[1])
    {
    case 0x03:
      readHoldingRegs();
      break;
    case 0x04:
      readInputRegs();
      break;
    case 0x01:
      readCoils();
      break;
    case 0x02:
      readInputs();
      break;
    case 0x06:
      writeSingleReg();
      break;
    case 0x10:
      writeHoldingRegs();
      break;
    case 0x05:
      writeSingleCoil();
      break;
    case 0x0F:
      writeMultiCoils();
      break;
    default:
      modbusException(ILLEGAL_FUNCTION);
      break;
    }
  }
  else if (RxData[0] == 1)
  {
    buf[0] = 1;
  }

  HAL_UARTEx_ReceiveToIdle_IT(&huart5, RxData, 256);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_DMA_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  // MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  VREFINT_CAL = *(uint16_t *)VREFINT_CAL_ADDR;
  ADC_DMA_Init();
  ADC_init();

  ADC1->CR2 |= ADC_CR2_ADON;
  HAL_Delay(1);
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC))
    ;
  uint16_t raw_vrefint = ADC1->DR;
  vdda = (3300 * VREFINT_CAL) / raw_vrefint; // 8-bit разрешение ADC -> 0xFF */

  ADC1->CR1 |= ADC_RESOLUTION_8B;
  ADC1->SQR3 = (9 << 0);
  ADC1->CR2 |= ADC_CR2_CONT;
  ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0; // tim2 cc2
  // заупск таймера TIM2 + OC 2 CHANNEL
  HAL_Delay(10);
  TIM2->CNT = 0;
  TIM2->CCR2 = 1;
  TIM2->CCER |= TIM_CCER_CC2E;
  HAL_UARTEx_ReceiveToIdle_IT(&huart5, RxData, 256);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (buf[0] == 1)
    {
      buf[0] = 0;
      memset(frame, 0, N_FRAMES * N_SAMPLES);
      i = 0;
      TIM2->CCR2 = 1;
      ADC1->CR2 &= ~ADC_CR2_DMA;
      DMA2_Stream0->CR &= ~DMA_SxCR_EN;
      while (DMA2_Stream0->CR & DMA_SxCR_EN)
        ;
      DMA2_Stream0->M0AR = (uint32_t)frame;
      DMA2_Stream0->NDTR = N_SAMPLES;
      DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0;
      DMA2_Stream0->CR |= DMA_SxCR_EN;
      while (!(DMA2_Stream0->CR & DMA_SxCR_EN))
        ;
      ADC1->SR = 0;
      ADC1->CR2 |= ADC_CR2_DMA;
      ADC1->CR2 |= ADC_CR2_EXTEN;
      HAL_Delay(1000);
      for (uint16_t i = 0; i < N_FRAMES * N_SAMPLES; i++)
      {
        uint16_t word = vdda * frame[i] / 0xFF;
        frame_8int_V[2 * i] = (uint8_t)(word & 0xFF);
        frame_8int_V[2 * i + 1] = (uint8_t)((word >> 8) & 0xFF);
      }
      HAL_UART_Transmit(&huart5, frame_8int_V, 2 * N_FRAMES * N_SAMPLES, HAL_MAX_DELAY);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_Init = {0};
  GPIO_Init.Pin = GPIO_PIN_1;
  GPIO_Init.Mode = GPIO_MODE_ANALOG;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_Init);
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