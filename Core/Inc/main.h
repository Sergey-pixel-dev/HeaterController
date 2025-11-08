/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbusSlave.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define UART4_RX_BUF_SIZE 256
#define UART4_TX_BUF_SIZE 256
#define UART5_RX_BUF_SIZE 256
#define UART5_TX_BUF_SIZE 256

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */
  extern void UART5_Transmit_DMA_Blocking(uint8_t *data, uint16_t size);
  /* USER CODE END EFP */

  /* Private defines -----------------------------------------------------------*/

  /* USER CODE BEGIN Private defines */
  extern uint8_t RxBufferUART5[UART5_RX_BUF_SIZE];
  extern uint8_t TxBufferUART5[UART5_TX_BUF_SIZE];
  extern uint16_t SizeRxBuf;
  extern uint8_t uart5_event_data_ready;
  extern uint8_t uart4_event_data_ready;
  extern uint8_t uart5_tx_dma_busy;
  extern uint8_t uart4_tx_dma_busy;

  extern volatile ModbusState_t modbus_state;
  extern volatile uint16_t SizeRxBufUART4;
  extern uint8_t RxBufferUART4[UART4_RX_BUF_SIZE];
  extern uint8_t TxBufferUART4[UART4_TX_BUF_SIZE];
  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */