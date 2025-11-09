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
#include "modbusSlave.h"

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
uint8_t RxBufferUART5[UART5_TX_BUF_SIZE];
uint8_t TxBufferUART5[UART5_TX_BUF_SIZE];
uint16_t SizeRxBuf = 0;
uint8_t uart5_event_data_ready = 0;
uint8_t uart4_event_data_ready = 0;
uint8_t uart5_tx_dma_busy = 0;
uint8_t uart4_tx_dma_busy = 0;

volatile ModbusState_t modbus_state = MODBUS_IDLE;
volatile uint16_t SizeRxBufUART4 = 0;
uint8_t RxBufferUART4[UART4_RX_BUF_SIZE];
uint8_t TxBufferUART4[UART4_TX_BUF_SIZE];

uint16_t c_a[N_SEGMENTS];
uint16_t c_b[N_SEGMENTS];
uint16_t c_c[N_SEGMENTS];
uint16_t c_d;
uint16_t i_e;
uint16_t c_index = 0;
uint16_t delta_u;
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
void UART4_Transmit_DMA_Blocking(uint8_t *data, uint16_t size);
void MODBUS_Timer_Init(void);

void Calibration(void);
void Set_I(uint16_t i);
void Change_I(void);

void TIM6_Init(void);
uint16_t Get_c_a_by_I(uint16_t i);
uint16_t Get_c_b_by_U_mes(uint16_t u);

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
  TIM6_Init();
  DMA1_Stream0->CR |= DMA_SxCR_EN;
  MeasureVref();
  DAC_SetVoltage(400);
  uint16_t last_i_set;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (uart4_event_data_ready && modbus_state == MODBUS_FRAME_COMPLETE)
    {
      TransmitFuncPtr = &UART4_Transmit_DMA_Blocking;
      RxBuffer = RxBufferUART4;
      TxBuffer = TxBufferUART4;
      if (RxBufferUART4[0] == SLAVE_ID)
      {
        switch (RxBufferUART4[1])
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
      SizeRxBufUART4 = 0;
      uart4_event_data_ready = 0;
      modbus_state = MODBUS_IDLE;
    }
    if (uart5_event_data_ready)
    {
      uart5_event_data_ready = 0;
      if (RxBufferUART5[0] == SLAVE_ID)
      {
        TransmitFuncPtr = &UART5_Transmit_DMA_Blocking;
        RxBuffer = RxBufferUART5;
        TxBuffer = TxBufferUART5;
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
          last_i_set = usRegHoldingBuf[0];
          writeSingleReg();
          if (last_i_set != usRegHoldingBuf[0])
          {
            TIM6->CR1 |= TIM_CR1_CEN;
          }

          break;
        case 0x10:
          last_i_set = usRegHoldingBuf[0];
          writeHoldingRegs();
          if (last_i_set != usRegHoldingBuf[0])
          {
            TIM6->CR1 |= TIM_CR1_CEN;
          }
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
      DMA1_Stream0->NDTR = UART5_RX_BUF_SIZE;
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODER2;
  GPIOA->MODER |= GPIO_MODER_MODER4;

  // UART5: PC12 (TX)
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

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // EXTI

  GPIOB->MODER &= ~(GPIO_MODER_MODER12);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12);
  GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR12_0);
  SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI12);
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PB;
  EXTI->IMR |= EXTI_IMR_MR12;
  EXTI->FTSR |= EXTI_FTSR_TR12;
  EXTI->RTSR &= ~(EXTI_RTSR_TR12);
  NVIC_SetPriority(EXTI15_10_IRQn, 3);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  GPIOB->MODER &= ~(GPIO_MODER_MODER13);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR13);
  GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR13_0);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
  DMA1_Stream0->NDTR = UART5_RX_BUF_SIZE;
  DMA1_Stream0->FCR = 0;

  // UART4 TX
  DMA1_Stream4->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
  DMA1_Stream4->PAR = (uint32_t)&UART4->DR;
  DMA1_Stream4->FCR = 0;

  NVIC_SetPriority(DMA1_Stream0_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  NVIC_SetPriority(DMA1_Stream7_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void DAC_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  DAC->CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1;
}
void DAC_SetVoltage(uint16_t voltage)
{
  DAC->DHR12R1 = ((uint32_t)voltage * (0xFFF + 1) / vdda) & 0xFFF; // 12 бит должно быть
  DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

void ADC_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC->CCR |= 1 << 16; // prescaler = 4
  ADC->CCR |= ADC_CCR_TSVREFE;
  ADC1->CR2 |= ADC_CR2_EOCS;
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

void UART_Init(void)
{
  // UART5
  RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  UART5->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;
  UART5->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
  UART5->BRR = 0x187; // 115200, 45Мгц

  NVIC_SetPriority(UART5_IRQn, 1);
  NVIC_EnableIRQ(UART5_IRQn);
  // UART4
  RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
  UART4->BRR = 0x187;
  UART4->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE |
                USART_CR1_RXNEIE;
  UART4->CR3 |= USART_CR3_DMAT;
  NVIC_SetPriority(UART4_IRQn, 0);
  NVIC_EnableIRQ(UART4_IRQn);
}
void MODBUS_Timer_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->CNT = 1;
  TIM3->PSC = 899;
  TIM3->ARR = 65535;

  TIM3->CCR1 = (MODBUS_T1_5_US / 10);
  TIM3->CCR2 = (MODBUS_T3_5_US / 10);
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM3->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;

  TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;

  TIM3->CR1 |= TIM_CR1_OPM;
  TIM3->EGR |= TIM_EGR_UG;
  TIM3->SR &= ~TIM_SR_UIF;
  NVIC_SetPriority(TIM3_IRQn, 2);
  NVIC_EnableIRQ(TIM3_IRQn);
}
void TIM6_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  TIM6->PSC = 8999;
  TIM6->ARR = 1000;
  TIM6->CNT = 0;
  TIM6->CR1 &= ~TIM_CR1_UDIS;
  TIM6->CR1 |= TIM_CR1_URS;
  TIM6->CR1 &= ~TIM_CR1_OPM;
  TIM6->EGR |= TIM_EGR_UG;
  TIM6->SR &= ~TIM_SR_UIF;
  TIM6->DIER |= TIM_DIER_UIE;
  NVIC_SetPriority(TIM6_DAC_IRQn, 2);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
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
  DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
  uart5_tx_dma_busy = 1;
  DMA1_Stream7->CR |= DMA_SxCR_EN;
  while (!(DMA1_Stream7->CR & DMA_SxCR_EN))
    ;
}
void UART4_Transmit_DMA_Blocking(uint8_t *data, uint16_t size)
{

  while (uart4_tx_dma_busy)
    ;
  DMA1_Stream4->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream4->CR & DMA_SxCR_EN)
    ;
  DMA1_Stream4->M0AR = (uint32_t)data;
  DMA1_Stream4->NDTR = size;
  DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
  uart4_tx_dma_busy = 1;
  DMA1_Stream4->CR |= DMA_SxCR_EN;
  while (!(DMA1_Stream4->CR & DMA_SxCR_EN))
    ;
}

void Calibration(void)
{
  if (usDiscreteBuf[0] & 0x0001 && usCoilsBuf[0] & 0x0001 && c_index < N_SEGMENTS)
  {
    uint16_t u_mes = ADC_ReadChannel(1);
    uint16_t segment = (MIN_U + c_index * (MAX_U - MIN_U));
    c_a[c_index] = ((usRegHoldingBuf[3] / segment) << 12);
    c_a[c_index] |= (usRegHoldingBuf[3] % segment) * 1000 / segment;
    c_b[c_index] = ((usRegHoldingBuf[3] / u_mes) << 12);
    c_b[c_index] |= (usRegHoldingBuf[3] % u_mes) * 1000 / segment;
    c_index++;
    usRegHoldingBuf[0] = MIN_I + c_index * (MAX_I - MIN_I) / N_SEGMENTS;
    Set_I(usRegHoldingBuf[0]);
  }
  else
  {
    c_index = 0;
    usCoilsBuf[0] &= ~0x0001;
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI12_PB;
    usRegHoldingBuf[3] = 0;
    Set_I(usRegHoldingBuf[2]);
  }
}
void Set_I(uint16_t i)
{
  i_set_cur = i;
  usDiscreteBuf[0] |= 0x02;
  TIM6->CR1 |= TIM_CR1_CEN;
}
void Change_I(void)
{
  uint16_t U_mes = ADC_ReadChannel(1);
  uint16_t I_cur = U_mes * Get_c_b_by_U_mes(U_mes);
  uint16_t delta;
  uint16_t I_sp = usRegHoldingBuf[1];
  uint16_t c_a_I_cur = Get_c_a_by_I(I_cur);
  uint16_t c_a_I_cur_sp = Get_c_a_by_I(I_cur + I_sp);
  uint16_t U_delta = 3 * usRegHoldingBuf[1] / 4;
  U_delta = U_delta / (c_a_I_cur >> 12) + (U_delta * 1000) / (c_a_I_cur & 0x0FFF);
  if (I_cur > usRegHoldingBuf[1])
    delta = I_cur - usRegHoldingBuf[1];
  else
    delta = usRegHoldingBuf[1] - I_cur;
  if (delta < usRegHoldingBuf[4])
  {
    TIM6->CR1 &= ~TIM_CR1_CEN;
    return;
  }
  else if (I_cur > usRegHoldingBuf[1])
  {
    DAC_SetVoltage(usRegInputBuf[1] - U_delta);
  }
  else
  {
    DAC_SetVoltage(usRegInputBuf[1] + U_delta);
  }

  uint32_t numerator = (I_cur + I_sp) * (c_a_I_cur >> 12) + (I_cur + I_sp) * (c_a_I_cur & 0x0FFF) / 1000;
  numerator -= I_cur * (c_a_I_cur_sp >> 12) + I_cur * (c_a_I_cur_sp & 0x0FFF) / 1000;
  uint32_t denominator = (c_a_I_cur_sp >> 12) * (c_a_I_cur >> 12) + ((uint32_t)(c_a_I_cur_sp & 0x0FFF) * (c_a_I_cur_sp & 0x0FFF)) / 1000000;
  uint8_t N = numerator / denominator / U_delta;
  TIM6->ARR = 1000 / N;
}
uint16_t Get_c_a_by_I(uint16_t i)
{
  return 10 << 12;
}
uint16_t Get_c_b_by_U_mes(uint16_t u)
{
  return 10 << 12;
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
