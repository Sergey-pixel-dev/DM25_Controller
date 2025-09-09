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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CLOCK_SYNC_PCLK_DIV2 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
#define VREFINT_CAL_ADDR 0x1FFF7A2A
uint16_t VREFINT_CAL;
uint16_t vdda;
uint16_t frame[MAX_N_FRAMES * MAX_N_SAMPLES];
uint32_t sum;
uint8_t frame_8int_V[2 * MAX_N_SAMPLES * MAX_N_FRAMES + 4];

uint16_t n_samples = MAX_N_SAMPLES;
uint8_t i = 0;

volatile uint8_t uart5_dma_busy = 0;
volatile uint8_t RxData_i = 0;
volatile uint8_t RxData[256];
volatile uint8_t TxData[256];

uint8_t hello[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

const uint8_t last_0_1_us[10] = {
    0, 7, 14, 21, 28, 36, 43, 50, 57, 64};
const uint8_t last_0_01_us[10] = {
    0, 0, 1, 2, 2, 3, 4, 5, 5, 6};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
  ADC->CCR |= ADC_CLOCK_SYNC_PCLK_DIV2;

  ADC1->CR1 = 0;
  ADC1->CR2 = 0;
  ADC2->CR1 = 0;
  ADC2->CR2 = 0;

  ADC1->SMPR1 = 0;
  ADC1->SMPR2 = 0;
  ADC2->SMPR1 = 0;
  ADC2->SMPR2 = 0;

  ADC1->SQR1 = 0;
  ADC1->SQR2 = 0;
  ADC1->SQR3 = 0;
  ADC2->SQR1 = 0;
  ADC2->SQR2 = 0;
  ADC2->SQR3 = 0;
  ADC2->SMPR1 = ADC_SMPR1_SMP10_2 | ADC_SMPR1_SMP11_2 | ADC_SMPR1_SMP12_2 | ADC_SMPR1_SMP13_2;
  ADC2->SMPR2 = ADC_SMPR2_SMP5_2 | ADC_SMPR2_SMP6_2 | ADC_SMPR2_SMP7_2 | ADC_SMPR2_SMP8_2 | ADC_SMPR2_SMP9_2;

  ADC->CCR |= ADC_CCR_TSVREFE;
  ADC1->CR2 |= ADC_CR2_EOCS;
  ADC2->CR2 |= ADC_CR2_EOCS;
}
void DMA_Init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMA1EN;
  // DMA для ADC1
  DMA2_Stream0->PAR = ADC1_BASE + 0x4C;
  DMA2_Stream0->M0AR = (uint32_t)frame;
  DMA2_Stream0->NDTR = n_samples;
  DMA2_Stream0->CR = 0;

  DMA2_Stream0->CR |= DMA_PRIORITY_VERY_HIGH;
  DMA2_Stream0->FCR = 0;

  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0   // 16‑бит в памяти
                      | DMA_SxCR_PSIZE_0 // 16‑бит на периферии
                      | DMA_SxCR_MINC    // инкремент адреса памяти
                      | DMA_SxCR_TCIE;   // прерывание по завершению

  // DMA для UART5
  DMA1_Stream7->PAR = UART5_BASE + 0x04;
  DMA1_Stream7->FCR = 0;
  DMA1_Stream7->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0;

  NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
  NVIC_SetPriority(DMA1_Stream7_IRQn, 0);

  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}
void UART5_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  UART5->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_OVER8;
  UART5->CR3 |= USART_CR3_DMAT | USART_CR3_ONEBIT;
  UART5->BRR = 0x271;

  NVIC_SetPriority(UART5_IRQn, 2);
  NVIC_EnableIRQ(UART5_IRQn);
}

void MeasureVDD()
{
  ADC1->CR2 &= ~ADC_CR2_CONT;
  ADC1->SQR3 = 17;
  ADC1->CR1 &= ~3 << 24;
  ADC1->CR2 &= ~(ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0);
  ADC1->DR;
  ADC1->SR = 0;
  // HAL_Delay(1);
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC))
    ;
  uint16_t raw_vrefint = ADC1->DR;
  vdda = (3300 * VREFINT_CAL) / raw_vrefint;

  ADC1->CR1 |= 1 << 24;
  ADC1->SQR3 = usRegHoldingBuf[2];
  ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0;
  ADC1->CR2 |= ADC_CR2_CONT;
  ADC1->SR = 0;
}
void PrepareADC()
{
  memset(frame, 0, MAX_N_FRAMES * MAX_N_SAMPLES);
  memset(frame_8int_V, 0, 2 * MAX_N_FRAMES * MAX_N_SAMPLES + 4);
  MeasureVDD();
  TIM2->ARR = 100;
}
uint16_t ReadChannel(uint8_t channel)
{
  ADC2->SQR3 = channel;
  ADC2->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC2->SR & ADC_SR_EOC))
    ;
  return ADC2->DR;
}
void UpdateValuesMB()
{
  uint16_t idr = GPIOB->IDR;
  uint8_t new_bits =
      ((idr >> 12) & 0x01) << 0 | /* PB12 → bit0 */
      ((idr >> 13) & 0x01) << 1 | /* PB13 → bit1 */
      ((idr >> 14) & 0x01) << 2 | /* PB14 → bit2 */
      ((idr >> 15) & 0x01) << 3 | /* PB15 → bit3 */
      ((idr >> 4) & 0x01) << 4;   /* PB4  → bit4 */
  if ((usDiscreteBuf[0] & 0x1F) != new_bits)
  {
    usDiscreteBuf[0] = (usDiscreteBuf[0] & ~0x1F) | new_bits;
  }
  for (uint8_t i = 5; i < 14; i++)
  {
    sum = 0;
    for (uint8_t j = 0; j < 10; j++)
    {
      sum += ReadChannel(i);
    }
    usRegInputBuf[i - 5] = vdda * (sum / 10) / 0xFFF;
  }
}

void UART5_Transmit_DMA_Blocking(uint8_t *data, uint16_t size)
{

  while (uart5_dma_busy)
    ;
  DMA1->HIFCR |= DMA_HIFCR_CTCIF7;

  DMA1_Stream7->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream7->CR & DMA_SxCR_EN)
    ;

  DMA1_Stream7->M0AR = (uint32_t)data;
  DMA1_Stream7->NDTR = size;
  uart5_dma_busy = 1;
  DMA1_Stream7->CR |= DMA_SxCR_EN;
  while (!(DMA1_Stream7->CR & DMA_SxCR_EN))
    ;
}
void UART5_ReceiveToIdle_IT()
{
  UART5->CR1 |= USART_CR1_RXNEIE;
}
void UART5_Transmition_Handler()
{
  if (RxData[0] == SLAVE_ID)
  {
    switch (RxData[1])
    {
    case 0x03:
      readHoldingRegs();
      break;
    case 0x04:
      UpdateValuesMB();
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
  RxData_i = 0;
}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  // MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  VREFINT_CAL = *(uint16_t *)VREFINT_CAL_ADDR;
  DMA_Init();
  ADC_init();
  UART5_Init();
  ADC1->CR2 |= ADC_CR2_ADON;
  ADC2->CR2 |= ADC_CR2_ADON;
  HAL_Delay(1);

  // заупск таймера TIM2 + OC 2 CHANNEL
  TIM2->CNT = 0;
  TIM2->CCR2 = 1;
  TIM2->CCER |= TIM_CCER_CC2E;

  MeasureVDD();
  usRegHoldingBuf[0] = 100;
  usRegHoldingBuf[1] = 30;
  usCoilsBuf[0] = 1;
  usRegHoldingBuf[2] = 1;
  usRegHoldingBuf[3] = 4;
  usRegHoldingBuf[5] = 1;
  SetPulse();
  SetHZ();
  StartTimers();
  UART5_ReceiveToIdle_IT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (usCoilsBuf[0] & 0x02 && usCoilsBuf[0] & 0x01)
    {
      usDiscreteBuf[0] |= 0x20;
      while (usCoilsBuf[0] & 0x02 && usCoilsBuf[0] & 0x01)
      {
        i = 0;
        TIM2->CCR2 = 1;
        ADC1->CR2 &= ~ADC_CR2_DMA;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream0->CR & DMA_SxCR_EN)
          ;
        DMA2_Stream0->M0AR = (uint32_t)(frame);
        ADC1->SQR3 = usRegHoldingBuf[2];
        n_samples = usRegHoldingBuf[3];
        frame_8int_V[0] = 0xAA;
        frame_8int_V[1] = 0x55;
        frame_8int_V[2 * MAX_N_FRAMES * n_samples + 2] = 0x55;
        frame_8int_V[2 * MAX_N_FRAMES * n_samples + 3] = 0xAA;
        DMA2_Stream0->NDTR = n_samples;
        DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0;
        DMA2_Stream0->CR |= DMA_SxCR_EN;
        while (!(DMA2_Stream0->CR & DMA_SxCR_EN))
          ;
        ADC1->SR = 0;
        ADC1->CR2 |= ADC_CR2_DMA;
        ADC1->CR2 |= ADC_CR2_EXTEN;

        while (i < MAX_N_FRAMES && usCoilsBuf[0] & 0x02 && usCoilsBuf[0] & 0x01)
          ;
        if (usCoilsBuf[0] & 0x02 && usCoilsBuf[0] & 0x01)
        {
          for (uint16_t i = 0; i < MAX_N_FRAMES * n_samples; i++)
          {
            uint16_t word = vdda * frame[i] / 1024;
            frame_8int_V[2 + 2 * i] = (uint8_t)(word & 0xFF);
            frame_8int_V[2 + 2 * i + 1] = (uint8_t)((word >> 8) & 0xFF);
          }
          UART5_Transmit_DMA_Blocking(frame_8int_V, 2 * MAX_N_FRAMES * n_samples + 4);
        }
      }
      usDiscreteBuf[0] &= ~0x20;
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  htim2.Init.Period = 600;
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
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1098;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3599;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_Init = {0};
  GPIO_Init.Pin = GPIO_PIN_1 | GPIO_PIN_0;
  GPIO_Init.Mode = GPIO_MODE_ANALOG;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);

  GPIO_Init.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOC, &GPIO_Init);

  // Настройка UART5 TX (PC12)
  GPIO_Init.Pin = GPIO_PIN_12;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_Init.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOC, &GPIO_Init);

  // Настройка UART5 RX (PD2)
  GPIO_Init.Pin = GPIO_PIN_2;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_Init.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_Init);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SetHZ()
{
  htim3.Instance->CNT = 0;
  htim3.Instance->PSC = 1099 / usRegHoldingBuf[0];
  htim3.Instance->ARR = (72 * 1000000 / (1099 / usRegHoldingBuf[0] + 1)) / usRegHoldingBuf[0];
}
void SetPulse()
{
  uint16_t shift = usRegHoldingBuf[4];
  htim4.Instance->CCR1 = htim4.Instance->ARR - 72 * (usRegHoldingBuf[1] / 10) - 7 * (usRegHoldingBuf[1] % 10);
  htim4.Instance->CCR4 = htim4.Instance->ARR - 72 * (usRegHoldingBuf[1] / 10) - 7 * (usRegHoldingBuf[1] % 10);
  htim4.Instance->CCR4 -= 72 * (shift / 100);
  htim4.Instance->CCR4 -= last_0_1_us[((shift % 100) / 10)];
  htim4.Instance->CCR4 -= last_0_01_us[shift % 10];
}
void StopTimers()
{
  htim3.Instance->CR1 &= ~TIM_CR1_CEN;
  htim4.Instance->CCER &= ~TIM_CCER_CC1E;
  htim4.Instance->CCER &= ~TIM_CCER_CC4E;
  htim4.Instance->CCER &= ~TIM_CCER_CC2E;
}
void StartTimers()
{
  htim3.Instance->CR1 |= TIM_CR1_CEN;
  htim4.Instance->CCER |= TIM_CCER_CC1E;
  htim4.Instance->CCER |= TIM_CCER_CC4E;
  htim4.Instance->CCER |= TIM_CCER_CC2E;
}
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
