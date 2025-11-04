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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VREFINT_CAL_ADDR 0x1FFF7A2A
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t vdda;
uint8_t RxBufferUART5[RX_BUF_CAPACITY];
uint8_t TxBufferUART5[RX_BUF_CAPACITY];
uint16_t SizeRxBuf = 0;
uint8_t uart_event_data_ready = 0;
uint8_t uart5_tx_dma_busy = 0;

volatile ModbusState_t modbus_state = MODBUS_IDLE;
volatile uint16_t SizeRxBufUART4 = 0;
uint8_t RxBufferUART4[UART4_RX_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void DAC_Init(void);
void DAC_SetVoltage(uint16_t voltage);

void ADC_Init(void);
void MeasureVref(void);
uint16_t ADC_ReadChannel(uint8_t channel);

void DMA_Init(void);

void UART_Init(void);
void UART5_Transmit_DMA_Blocking(uint8_t *data, uint16_t size);

void MODBUS_Timer_Init(void);
inline void Modbus_Timer_Reset(void);
inline void Modbus_Timer_Stop(void);
inline void Modbus_Timer_Start(uint16_t timeout_us);
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
  /* USER CODE BEGIN 2 */
  ADC_Init();
  ADC1->CR2 |= ADC_CR2_ADON;
  DAC_Init();
  DAC->CR |= DAC_CR_EN1;
  DMA_Init();
  UART_Init();
  MODBUS_Timer_Init();
  DMA1_Stream0->CR |= DMA_SxCR_EN;
  DMA1_Stream7->CR |= DMA_SxCR_EN;
  MeasureVref();
  DAC_SetVoltage(400);
  uint32_t last = HAL_GetTick();
  uint32_t cur = last;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    cur = HAL_GetTick();
    if (cur - last > 1000)
    {
      last = cur;
      usRegInputBuf[0] = (uint32_t)vdda * ADC_ReadChannel(1) / 0xFFF;
    }
    /* Обработка Modbus UART4 */
    if (modbus_state == MODBUS_FRAME_COMPLETE)
    {
      /* Фрейм успешно принят */

      /* Проверка адреса слейва */
      if (RxBufferUART4[0] == SLAVE_ID)
      {
        /* Обработка команды */
        switch (RxBufferUART4[1])
        {
        case 0x03:
          /* readHoldingRegs для UART4 */
          break;
        case 0x04:
          /* readInputRegs для UART4 */
          break;
        case 0x06:
          /* writeSingleReg для UART4 */
          break;
        case 0x10:
          /* writeHoldingRegs для UART4 */
          break;
        default:
          /* modbusException(ILLEGAL_FUNCTION); */
          break;
        }
      }

      /* Сброс для приема нового кадра */
      SizeRxBufUART4 = 0;
      modbus_state = MODBUS_IDLE;
    }
    if (uart_event_data_ready)
    {
      uart_event_data_ready = 0;
      if (RxBufferUART5[0] == SLAVE_ID)
      {
        switch (RxBufferUART5[1])
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
          DAC_SetVoltage(usRegHoldingBuf[0]);
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
      DMA1_Stream0->NDTR = RX_BUF_CAPACITY;
      DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
      DMA1_Stream0->CR |= DMA_SxCR_EN;
      while (!(DMA1_Stream0->CR & DMA_SxCR_EN))
        ;
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODER2;
  GPIOA->MODER |= GPIO_MODER_MODER4;

  // UART5: PC12 (TX) уже настроен ранее, сохраняем
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
  // PC12 = AF8 (UART5_TX)
  GPIOC->MODER &= ~(GPIO_MODER_MODER12);
  GPIOC->MODER |= GPIO_MODER_MODER12_1;
  GPIOC->AFR[1] &= ~(0xF << 16);
  GPIOC->AFR[1] |= (8 << 16);
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12;

  // PD2 = AF8 (UART5_RX) + Pull-Up
  GPIOD->MODER &= ~(GPIO_MODER_MODER2);
  GPIOD->MODER |= GPIO_MODER_MODER2_1;
  GPIOD->AFR[0] &= ~(0xF << 8);
  GPIOD->AFR[0] |= (8 << 8);
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
  GPIOD->PUPDR |= GPIO_PUPDR_PUPDR2_0;

  // UART4: PC10 (TX), PC11 (RX), AF8
  // PC10 TX
  GPIOC->MODER &= ~(GPIO_MODER_MODER10);
  GPIOC->MODER |= GPIO_MODER_MODER10_1;      // AF
  GPIOC->AFR[1] &= ~(0xF << 8);              // AFR[1][11:8]
  GPIOC->AFR[1] |= (8 << 8);                 // AF8
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10; // High speed

  // PC11 RX
  GPIOC->MODER &= ~(GPIO_MODER_MODER11);
  GPIOC->MODER |= GPIO_MODER_MODER11_1;      // AF
  GPIOC->AFR[1] &= ~(0xF << 12);             // AFR[1][15:12]
  GPIOC->AFR[1] |= (8 << 12);                // AF8
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11; // High speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR11);
  GPIOC->PUPDR |= GPIO_PUPDR_PUPDR11_0; // Pull-Up на RX

  GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DAC_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1;
}

void ADC_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC->CCR |= 1 << 16; // prescaler = 4
  ADC->CCR |= ADC_CCR_TSVREFE;
  ADC1->CR2 |= ADC_CR2_EOCS;
}

void UART_Init(void)
{
  // UART5
  RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  UART5->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;
  UART5->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
  UART5->BRR = 0x187; // 115200, 45Мгц

  NVIC_SetPriority(UART5_IRQn, 0);
  NVIC_EnableIRQ(UART5_IRQn);
  // UART4
  RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
  UART4->BRR = 0x249F;
  UART4->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE |
                USART_CR1_RXNEIE | USART_CR1_IDLEIE;
  NVIC_SetPriority(UART4_IRQn, 1);
  NVIC_EnableIRQ(UART4_IRQn);
}
void MODBUS_Timer_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC = 899;
  TIM3->ARR = 65535;

  /* Канал 1 (CCR1) - для t1.5 (750 мкс = 75 тиков по 10 мкс) */
  TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M);
  TIM3->CCMR1 |= TIM_CCMR1_OC1M_0; /* PWM mode 1 или Output Compare */
  TIM3->CCER |= TIM_CCER_CC1E;     /* Включаем канал 1 */
  TIM3->DIER |= TIM_DIER_CC1IE;    /* Прерывание на сравнение канала 1 */

  /* Канал 2 (CCR2) - для t3.5 (1750 мкс = 175 тиков по 10 мкс) */
  TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M);
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_0; /* Output Compare */
  TIM3->CCER |= TIM_CCER_CC2E;     /* Включаем канал 2 */
  TIM3->DIER |= TIM_DIER_CC2IE;    /* Прерывание на сравнение канала 2 */

  TIM3->CR1 |= TIM_CR1_OPM;

  NVIC_SetPriority(TIM3_IRQn, 2);
  NVIC_EnableIRQ(TIM3_IRQn);
}

inline void Modbus_Timer_Start(uint16_t timeout_us)
{
  TIM3->ARR = timeout_us / 10;
  TIM3->CNT = 0;
  TIM3->SR = 0;
  TIM3->CR1 |= TIM_CR1_CEN;
}

inline void Modbus_Timer_Stop(void)
{
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->CNT = 0;
}

inline void Modbus_Timer_Reset(void)
{
  TIM3->CNT = 0;
}

void UART4_Transmit(uint8_t *data, uint16_t size)
{
  for (uint16_t i = 0; i < size; i++)
  {
    UART4->DR = data[i];
    while (!(UART4->SR & USART_SR_TXE))
      ;
  }
  while (!(UART4->SR & USART_SR_TC))
    ;
}
void UART5_Transmit_DMA_Blocking(uint8_t *data, uint16_t size)
{

  while (uart5_tx_dma_busy)
    ;
  DMA1_Stream7->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream7->CR & DMA_SxCR_EN)
    ;
  DMA1_Stream7->M0AR = (uint32_t)data;
  DMA1_Stream7->NDTR = size;
  uart5_tx_dma_busy = 1;
  DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
  DMA1_Stream7->CR |= DMA_SxCR_EN;
  while (!(DMA1_Stream7->CR & DMA_SxCR_EN))
    ;
}

void DMA_Init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  // UART5 TX
  DMA1_Stream7->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
  DMA1_Stream7->PAR = (uint32_t)&UART5->DR;
  DMA1_Stream7->FCR = 0;
  // UART5 RX
  DMA1_Stream0->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_1 | DMA_SxCR_MINC;
  DMA1_Stream0->PAR = (uint32_t)&UART5->DR;
  DMA1_Stream0->M0AR = (uint32_t)&RxBufferUART5;
  DMA1_Stream0->NDTR = RX_BUF_CAPACITY;
  DMA1_Stream0->FCR = 0;

  NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  NVIC_SetPriority(DMA1_Stream7_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

uint16_t ADC_ReadChannel(uint8_t channel)
{
  ADC1->SQR3 = channel;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC))
    ;
  uint16_t a = ADC1->DR;
  ADC1->SR = 0;
  return a;
}

void MeasureVref(void)
{
  ADC1->DR;
  ADC1->SR = 0;
  uint16_t raw_vrefint = ADC_ReadChannel(17);
  vdda = (3300 * *(uint16_t *)VREFINT_CAL_ADDR) / raw_vrefint;
  ADC1->SR = 0;
}

void DAC_SetVoltage(uint16_t voltage)
{
  DAC->DHR12R1 = ((uint32_t)voltage * (0xFFF + 1) / vdda) & 0xFFF; // 12 бит должно быть
  DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
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
