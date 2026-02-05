/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t modbus_t1_5_timeout = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */
    if (modbus_uart5_pending) {
        modbus_uart5_pending = 0;
        if (RxBufferUART5[0] == SLAVE_ID) {
            TransmitFuncPtr = &UART5_Transmit_DMA_Blocking;
            RxBuffer = RxBufferUART5;
            TxBuffer = TxBufferUART5;
            HandleModbusRequest(RxBufferUART5);
        }
        DMA1_Stream0->NDTR = UART5_RX_BUF_SIZE;
        DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
        DMA1_Stream0->CR |= DMA_SxCR_EN;
        while (!(DMA1_Stream0->CR & DMA_SxCR_EN))
            ;
    }

    if (modbus_uart4_pending && modbus_state == MODBUS_FRAME_COMPLETE) {
        modbus_uart4_pending = 0;
        TransmitFuncPtr = &UART4_Transmit_DMA_Blocking;
        RxBuffer = RxBufferUART4;
        TxBuffer = TxBufferUART4;
        HandleModbusRequest(RxBufferUART4);
        SizeRxBufUART4 = 0;
        modbus_state = MODBUS_IDLE;
    }
    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void UART5_IRQHandler(void)
{
    if (UART5->SR & USART_SR_IDLE) {
        UART5->DR;
        UART5->SR;
        SizeRxBuf = UART5_RX_BUF_SIZE - DMA1_Stream0->NDTR;
        if (SizeRxBuf > 0) {
            DMA1_Stream0->CR &= ~DMA_SxCR_EN;
            while (DMA1_Stream0->CR & DMA_SxCR_EN)
                ;
            modbus_uart5_pending = 1;
            SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
        }
    }
}

void UART4_IRQHandler(void)
{
    if (UART4->SR & USART_SR_RXNE) {
        uint8_t data = UART4->DR;
        UART4->SR;
        TIM3->CNT = 0;
        if (modbus_state == MODBUS_FRAME_ERROR) {
            return;
        } else if (modbus_state == MODBUS_IDLE) {
            modbus_state = MODBUS_RECEIVING;
            RxBufferUART4[SizeRxBufUART4++] = data;
            TIM3->CR1 |= TIM_CR1_CEN;
        } else if (modbus_state == MODBUS_RECEIVING) {
            if (!modbus_t1_5_timeout) {
                if (SizeRxBufUART4 < UART4_RX_BUF_SIZE)
                    RxBufferUART4[SizeRxBufUART4++] = data;
                else
                    modbus_state = MODBUS_FRAME_ERROR;
            } else
                modbus_state = MODBUS_FRAME_ERROR;
        }
    }
}
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_CC1IF) && (TIM3->DIER & TIM_DIER_CC1IE)) {
        TIM3->SR &= ~TIM_SR_CC1IF;
        modbus_t1_5_timeout = 1;
    }
    if ((TIM3->SR & TIM_SR_CC2IF) && (TIM3->DIER & TIM_DIER_CC2IE)) {
        TIM3->SR &= ~TIM_SR_CC2IF;

        if (modbus_state == MODBUS_FRAME_ERROR) {
            modbus_state = MODBUS_IDLE;
            SizeRxBufUART4 = 0;
        } else if (modbus_state == MODBUS_RECEIVING) {
            if (SizeRxBufUART4 >= 4) {
                modbus_state = MODBUS_FRAME_COMPLETE;
                modbus_uart4_pending = 1;
                SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
            } else {
                SizeRxBufUART4 = 0;
                modbus_state = MODBUS_IDLE;
            }
        }
        modbus_t1_5_timeout = 0;
    }
}
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    if (EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR = EXTI_PR_PR13; // Сброс флага
        Calibration();
    }
    /* USER CODE END EXTI15_10_IRQn 0 */
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    /* USER CODE END EXTI15_10_IRQn 1 */
}

void TIM6_DAC_IRQHandler(void)
{
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF;
        if (CHECK_CALIB_IN_PR())
            DAC_ChangeVoltage();
        else
            Change_I();
    }
    /* USER CODE END TIM6_DAC_IRQn 0 */
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
    /* USER CODE END TIM6_DAC_IRQn 1 */
}

// В дальнейшем попробовать отказаться от флага dma_busy в пользу статусныъ регистров DMA
void DMA1_Stream7_IRQHandler(void)
{
    if (DMA1->HISR & DMA_HISR_TCIF7) {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
        uart5_tx_dma_busy = 0;
    }
}

void DMA1_Stream4_IRQHandler(void)
{
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
        uart4_tx_dma_busy = 0;
    }
}

/* USER CODE END 1 */
