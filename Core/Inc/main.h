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
extern "C" {
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

#define N_SEGMENTS 10 // кол-во коэф (частей, на которые мы делим ось U графика I(U))
#define MIN_I 0
#define MAX_I 25000 // мА
#define MIN_U 0
#define MAX_U 3000
#define MAX_DAC_VOLTAGE 3300 // VDDA в мВ

#define N_TIMERS 7 // Кол-во интервалов в 1 сек

// ============================================================================
// GPIO пины для управления (порт B)
// ============================================================================
#define SOURCE_PIN 14    // PB14 - Включение источника (выход)
#define CONVERTER_PIN 15 // PB15 - Включение преобразователя (выход)
#define JUMPER_PIN 0     // PB0 - Статус джампера (вход)
#define BUTTON_PIN 13    // PB13 - Кнопка калибровки (уже настроена)

#define SOURCE_STATUS_PIN 1    // PB1 - Фактическое состояние источника (вход)
#define CONVERTER_STATUS_PIN 2 // PB2 - Фактическое состояние преобразователя (вход)

// ============================================================================
// Q4.12 Fixed Point арифметика
// ============================================================================
#define Q12_SHIFT 12
#define Q12_ONE (1U << Q12_SHIFT)        // 4096
#define Q12_HALF (1U << (Q12_SHIFT - 1)) // 2048 для округления

// ============================================================================
// Биты Discrete Inputs (usDiscreteBuf[0])
// ============================================================================
#define DISCRETE_BIT_CUR_SETTING 0
#define DISCRETE_BIT_SOURCE_STATUS 1
#define DISCRETE_BIT_CONVERTER_STATUS 2
#define DISCRETE_BIT_CALIB_IN_PR 3
#define DISCRETE_BIT_JMPR_STATUS 4
#define DISCRETE_BIT_ERROR_27V 5
#define DISCRETE_BIT_ERROR_12V 6
#define DISCRETE_BIT_ERROR_M5V 7

// Проверка битов
#define CHECK_CUR_SETTING() ((usDiscreteBuf[0] >> DISCRETE_BIT_CUR_SETTING) & 0x01)
#define CHECK_SOURCE_STATUS() ((usDiscreteBuf[0] >> DISCRETE_BIT_SOURCE_STATUS) & 0x01)
#define CHECK_CONVERTER_STATUS() ((usDiscreteBuf[0] >> DISCRETE_BIT_CONVERTER_STATUS) & 0x01)
#define CHECK_CALIB_IN_PR() ((usDiscreteBuf[0] >> DISCRETE_BIT_CALIB_IN_PR) & 0x01)
#define CHECK_JMPR_STATUS() ((usDiscreteBuf[0] >> DISCRETE_BIT_JMPR_STATUS) & 0x01)

// Установка/сброс битов
#define SET_CUR_SETTING() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_CUR_SETTING))
#define CLR_CUR_SETTING() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_CUR_SETTING))

#define SET_SOURCE_STATUS() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_SOURCE_STATUS))
#define CLR_SOURCE_STATUS() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_SOURCE_STATUS))

#define SET_CONVERTER_STATUS() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_CONVERTER_STATUS))
#define CLR_CONVERTER_STATUS() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_CONVERTER_STATUS))

#define SET_CALIB_MODE_IN_PR() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_CALIB_IN_PR))
#define CLR_CALIB_MODE_IN_PR() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_CALIB_IN_PR))

#define SET_JMPR_STATUS() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_JMPR_STATUS))
#define CLR_JMPR_STATUS() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_JMPR_STATUS))

#define SET_ERROR_27V() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_ERROR_27V))
#define CLR_ERROR_27V() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_ERROR_27V))

#define SET_ERROR_12V() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_ERROR_12V))
#define CLR_ERROR_12V() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_ERROR_12V))

#define SET_ERROR_M5V() (usDiscreteBuf[0] |= (1U << DISCRETE_BIT_ERROR_M5V))
#define CLR_ERROR_M5V() (usDiscreteBuf[0] &= ~(1U << DISCRETE_BIT_ERROR_M5V))

// ============================================================================
// Coils (usCoilsBuf[0])
// ============================================================================
#define COIL_BIT_ENABLE_SOURCE 0
#define COIL_BIT_ENABLE_CONVERTER 1
#define COIL_BIT_SET_OPERATING_MODE 2
#define COIL_BIT_SET_STANDBY_MODE 3
#define COIL_BIT_SET_CALIB_MODE 4

#define CHECK_ENABLE_SOURCE() ((usDiscreteBuf[0] >> DISCRETE_BIT_SOURCE_STATUS) & 0x01)
#define CHECK_ENABLE_CONVERTER() ((usDiscreteBuf[0] >> DISCRETE_BIT_CONVERTER_STATUS) & 0x01)
#define CHECK_SET_OPERATING_MODE() ((usCoilsBuf[0] >> COIL_BIT_SET_OPERATING_MODE) & 0x01)
#define CHECK_SET_STANDBY_MODE() ((usCoilsBuf[0] >> COIL_BIT_SET_STANDBY_MODE) & 0x01)
#define CHECK_SET_CALIB_MODE() ((usCoilsBuf[0] >> COIL_BIT_SET_CALIB_MODE) & 0x01)

#define SET_ENABLE_SOURCE() (usCoilsBuf[0] |= (1U << COIL_BIT_ENABLE_SOURCE))
#define CLR_ENABLE_SOURCE() (usCoilsBuf[0] &= ~(1U << COIL_BIT_ENABLE_SOURCE))

#define SET_ENABLE_CONVERTER() (usCoilsBuf[0] |= (1U << COIL_BIT_ENABLE_CONVERTER))
#define CLR_ENABLE_CONVERTER() (usCoilsBuf[0] &= ~(1U << COIL_BIT_ENABLE_CONVERTER))

#define SET_OPERATING_MODE() (usCoilsBuf[0] |= (1U << COIL_BIT_SET_OPERATING_MODE))
#define CLR_OPERATING_MODE() (usCoilsBuf[0] &= ~(1U << COIL_BIT_SET_OPERATING_MODE))

#define SET_STANDBY_MODE() (usCoilsBuf[0] |= (1U << COIL_BIT_SET_STANDBY_MODE))
#define CLR_STANDBY_MODE() (usCoilsBuf[0] &= ~(1U << COIL_BIT_SET_STANDBY_MODE))

#define SET_CALIB_MODE() (usCoilsBuf[0] |= (1U << COIL_BIT_SET_CALIB_MODE))
#define CLR_CALIB_MODE() (usCoilsBuf[0] &= ~(1U << COIL_BIT_SET_CALIB_MODE))

// ============================================================================
// Holding Registers
// ============================================================================
#define HREG_OPERATING_I 0
#define HREG_STANDBY_I 1
#define HREG_CURRENT_SPEED 2
#define HREG_ACCURACY 3
#define HREG_CALIB_VALUE 4
#define HREG_27V 5
#define HREG_12V 6
#define HREG_M5V 7

// ============================================================================
// Input Registers
// ============================================================================
#define IREG_I_CURRENT 0
#define IREG_U_SET 1
#define IREG_U_CURRENT_MES 2
#define IREG_U_MES 3
#define IREG_COMPARATOR_U 4
#define IREG_27V 5
#define IREG_12V 6
#define IREG_M5V 7
#define IREG_CALIB_STEP 8
#define IREG_OSCILLOSCOPE_I 9
#define IREG_CALIB_CA 10
#define IREG_CALIB_CB 11

// ============================================================================
// Значения по умолчанию
// ============================================================================
#define CALIB_INITIAL_I 1000 // Начальный ток для калибровки
#define COEF_A_DEFAULT 6     // c_a по умолчанию (без сдвига)
#define COEF_B_DEFAULT 7     // c_b по умолчанию (без сдвига)
#define LOOP_UPDATE_PERIOD_MS 150
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void UART5_Transmit_DMA_Blocking(uint8_t *data, uint16_t size);
extern void Calibration(void);
extern void Change_I(void);
extern void DAC_ChangeVoltage(void);
extern void DAC_StartChangingV(void);
extern void Set_I(uint16_t i);
extern void StopChange_I(void);
extern void Coils_ApplyToPins(void);
extern void Update_Input_States(void);
extern void SetOperatingMode(void);
extern void SetStandbyMode(void);
extern void SetCalibrationMode(void);
extern uint16_t Calculate_I_from_U(uint16_t u_mes);
extern uint16_t Get_c_a_by_I(uint16_t i);
extern uint16_t Get_c_b_by_U_mes(uint16_t u);
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

extern uint16_t c_a[N_SEGMENTS];
extern uint16_t c_b[N_SEGMENTS];
extern uint16_t c_c[N_SEGMENTS];
extern uint16_t c_d;
extern uint16_t i_e;
extern uint16_t c_index;
extern uint16_t delta_u;

extern uint16_t i_set_cur;

extern uint32_t last_tick;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
