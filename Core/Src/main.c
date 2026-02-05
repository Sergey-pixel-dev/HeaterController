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
#include "eeprom.h"
#include "i2c.h"
#include "init.h"
#include "modbusSlave.h"
#include "stm32f4xx_hal.h"
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
uint16_t vdda = 3300;
uint8_t RxBufferUART5[UART5_TX_BUF_SIZE];
uint8_t TxBufferUART5[UART5_TX_BUF_SIZE];
uint16_t SizeRxBuf = 0;
uint8_t uart5_tx_dma_busy = 0;
uint8_t uart4_tx_dma_busy = 0;

volatile uint8_t modbus_uart5_pending = 0;
volatile uint8_t modbus_uart4_pending = 0;
volatile uint8_t eeprom_write_pending = 0;

volatile ModbusState_t modbus_state = MODBUS_IDLE;
volatile uint16_t SizeRxBufUART4 = 0;
uint8_t RxBufferUART4[UART4_RX_BUF_SIZE];
uint8_t TxBufferUART4[UART4_TX_BUF_SIZE];

uint16_t c_a[N_SEGMENTS];
uint16_t c_b[N_SEGMENTS];
uint16_t c_d[N_SEGMENTS];
uint16_t c_e[N_SEGMENTS];
uint16_t c_index = 0;
uint16_t delta_u;
uint16_t i_set_cur;

uint32_t last_tick = 0;
uint32_t last_tick_second = 0;

uint8_t buf1[256];
uint8_t buf2[256];

// Статистика
uint32_t modbus_req_count = 0;
uint32_t work_time = 0;
uint8_t log_count = 0;
uint8_t log_i = 0;

EEPROM_t eeprom = {0x50, 256, 8, 1, 32, I2C_MEMADD_SIZE_8BIT};

uint8_t NormalOrKZ = 0; // 0 - 1 Ом, 1 - КЗ

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void DAC_Init(void);
void DAC_SetVoltage(uint16_t voltage);
void DAC_StartChangingV(void);
void ADC_Init(void);
void MeasureVref(void);
uint16_t ADC_ReadChannel(uint8_t channel);

void DMA_Init(void);

void UART_Init(void);

void MODBUS_Timer_Init(void);

void Calibration(void);
void Set_I(uint16_t i);
void Change_I(void);

void TIM6_Init(void);
uint16_t Get_c_a_by_I(uint16_t i);
uint16_t Get_c_b_by_U_mes(uint16_t u);
uint16_t Get_c_d_by_U(uint16_t u);
uint16_t Get_c_e_by_I(uint16_t i);
void WriteLog(LogType_t type, uint8_t subtype, uint8_t code);
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
    I2C1_Init(I2C_SPEED_STANDARD);
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    MeasureVref();

    usRegHoldingBuf[HREG_OPERATING_I] = 6000;  // Рабочий ток
    usRegHoldingBuf[HREG_STANDBY_I] = 3000;    // Ток ожидания
    usRegHoldingBuf[HREG_CURRENT_SPEED] = 200; // Скорость нарастания
    usRegHoldingBuf[HREG_ACCURACY] = 50;       // Точность

    usCoilsBuf[0] = 0;
    SET_ENABLE_SOURCE();
    SET_ENABLE_CONVERTER();
    SET_STANDBY_MODE();
    SET_JMPR_STATUS();

    usDiscreteBuf[0] = 0;

    Coils_ApplyToPins();
    last_tick = HAL_GetTick();
    last_tick_second = HAL_GetTick();
    DAC_SetVoltage(1);
    for (uint8_t i = 0; i < N_SEGMENTS; i++) {
        c_d[i] = COEF_D_DEFAULT << Q11_SHIFT;
        c_e[i] = COEF_E_DEFAULT;
    }
    c_d[0] = 0x8DA6;
    c_d[1] = 0x8C0B;
    c_e[0] = 0x01C7;
    c_e[1] = 0x03AA;
    // ДОБАВИЛИСЬ НОВЫЕ КОЭФ. ИСПРАВИТЬ!
    // Структура хранения. [] - это 1 байт. Наименее значимый байт хранится по меньшему адресу.
    // [кол-во коэф.] (сначала c_a) [Младший байт коэф. 1][Старший байт коэф. 1][Младший байт коэф. 2][Старший
    // байт коэф. 2]...
    // ...[Младший байт коэф. 15][Старший байт коэф. 15](потом c_b) [Младший байт коэф. 1][Старший байт коэф.
    // 1][Младший байт коэф. 2][Старший байт коэф. 2]...
    // ... [Младший байт коэф. 15][Старший байт коэф. 15] [][][] - 3 байта для выравнивания по странице
    // --- 64 БАЙТА ЗАПИСАНЫ ---
    // Далее идут 64 байта статистики, пока запишем так:
    // [время работы][время работы][время работы][время работы](4 байта времени работы в сек.)
    // [кол-во модбас обращений][кол-во модбас обращений][кол-во модбас обращений][кол-во модбас обращений]
    // Далее идут 56 пока пустых байтов
    // --- 128 БАЙТОВ ЗАПИСАНЫ ---
    // Сначала 1 байт хранит кол-во записей в данной сессии (ВСЕ ЗАПИСИ ОЧИЩАЮТСЯ ПЕРЕД НОВЫМ ЗАПУСКОМ)
    // [кол-во записей]
    // Далее идут байты записи (5 байт), структура записи:
    // [байт тик][байт тик][байт тик][байт тик](32-битный тик с момента запуска контролллера)
    // [AA, BB, CCCC], где AA 01 - INFO, 10 - WARNING, 11 - ERROR. BB - код раздела (типа для разделения), CCCC
    // - код записи. для 256 x 8 eeprom кол-во записей макс - 25.
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    // Очищаем записи

    /* EEPROM_Write(&eeprom, 128, buf1, 128, 100);
    // Читаем коэф. с памяти
    uint8_t c_c = 0;
    EEPROM_Read(&eeprom, 0, &c_c, 1, 10);
    if (c_c == N_SEGMENTS) {
        uint8_t c_a_8[2 * N_SEGMENTS];
        uint8_t c_b_8[2 * N_SEGMENTS];
        EEPROM_Read(&eeprom, 1, c_a_8, N_SEGMENTS, 100);
        EEPROM_Read(&eeprom, 31, c_b_8, N_SEGMENTS, 100);
        for (uint8_t i = 0; i < N_SEGMENTS; i++) {
            c_a[i] = c_a_8[i] | c_a_8[i + 1] << 8;
            if (c_a[i] < 6 || c_a[i] > 12) // пределы уточнить
                c_a[i] = 0;
            c_b[i] = c_b_8[i] | c_b_8[i + 1] << 8;
            if (c_b[i] < 6 || c_b[i] > 12) // пределы уточнить
                c_b[i] = 0;
        }
    }

    // Читаем статистику с памяти
    uint8_t work_time_8[4];
    EEPROM_Read(&eeprom, 64, work_time_8, 4, 100);
    work_time = work_time_8[0] | work_time_8[1] << 8 | work_time_8[2] << 16 | work_time_8[3] << 24;

    uint8_t modbus_req_count_8[4];
    EEPROM_Read(&eeprom, 65, modbus_req_count_8, 4, 100);
    modbus_req_count =
        modbus_req_count_8[0] | modbus_req_count_8[1] << 8 | modbus_req_count_8[2] << 16 |
    modbus_req_count_8[3] << 24;
 */
    while (1) {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // Deferred EEPROM write for MODBUS statistics
        /* if (eeprom_write_pending) {
            eeprom_write_pending = 0;
            uint8_t modbus_req_count_8[4] = {modbus_req_count & 0x000000FF, (modbus_req_count >> 8) & 0xFF,
                                             (modbus_req_count >> 16) & 0xFF, (modbus_req_count >> 24) & 0xFF};
            EEPROM_Write(&eeprom, 65, modbus_req_count_8, 4, 100);
        } */

        uint32_t current_tick = HAL_GetTick();
        if (current_tick - last_tick >= LOOP_UPDATE_PERIOD_MS) {
            last_tick = current_tick;

            Update_Input_States();

            /* uint8_t jumper_state = (GPIOB->IDR & (1 << JUMPER_PIN)) ? 1 : 0;
            if (jumper_state != CHECK_JMPR_STATUS()) {
                if (jumper_state)
                    SET_JMPR_STATUS();
                else
                    CLR_JMPR_STATUS();
            } */

            uint16_t u_current_mes = ADC_ReadChannel(0);
            uint16_t I_cur = Calculate_I_from_U(u_current_mes);

            usRegInputBuf[IREG_I_CURRENT] = I_cur;
            usRegInputBuf[IREG_U_CURRENT_MES] = u_current_mes;

            // U_выхода = u_adc * K_u - I * K_i
            uint16_t u_adc_raw = ADC_ReadChannel(1);
            uint16_t K_u = Get_c_d_by_U(u_adc_raw);
            uint16_t K_i = Get_c_e_by_I(I_cur);

            uint32_t term1 = ((uint32_t)u_adc_raw * K_u + Q11_HALF) >> Q11_SHIFT;
            uint32_t term2 = ((uint32_t)I_cur * K_i + Q11_HALF) >> Q11_SHIFT;

            usRegInputBuf[IREG_U_MES] = (term1 >= term2) ? (uint16_t)(term1 - term2) : 0;
            usRegInputBuf[IREG_COMPARATOR_U] = ADC_ReadChannel(5);

            uint16_t u_27v = ADC_ReadChannel(2);
            uint16_t u_12v = ADC_ReadChannel(3);
            uint16_t u_m5v = ADC_ReadChannel(8); // PB0 = ADC1_IN8

            usRegInputBuf[IREG_27V] = u_27v;
            usRegInputBuf[IREG_12V] = u_12v;
            usRegInputBuf[IREG_M5V] = u_m5v;

            uint16_t ref_27v = usRegHoldingBuf[HREG_27V];
            uint16_t ref_12v = usRegHoldingBuf[HREG_12V];
            uint16_t ref_m5v = usRegHoldingBuf[HREG_M5V];

            /*             if (ABS_DIFF(u_27v, ref_27v) > 20) {
                            if (!CHECK_ERROR_27V())
                                __NOP();

                            // WriteLog(ERR, 1, 1);
                            SET_ERROR_27V();

                        } else
                            CLR_ERROR_27V();

                        if (ABS_DIFF(u_12v, ref_12v) > 20) {
                            if (!CHECK_ERROR_12V())
                                __NOP();
                            // WriteLog(ERR, 1, 2);
                            SET_ERROR_12V();

                        } else
                            CLR_ERROR_12V();

                        if (ABS_DIFF(u_m5v, ref_m5v) > 20) {
                            if (!CHECK_ERROR_M5V())
                                __NOP();

                            // WriteLog(ERR, 1, 3);
                            SET_ERROR_M5V();

                        } else
                            CLR_ERROR_M5V();
             */
            if (!CHECK_CUR_SETTING() && !CHECK_CALIB_IN_PR()) {
                uint16_t I_setpoint;
                if (CHECK_SET_OPERATING_MODE())
                    I_setpoint = usRegHoldingBuf[HREG_OPERATING_I];
                else
                    I_setpoint = usRegHoldingBuf[HREG_STANDBY_I];
                if (ABS_DIFF(I_cur, I_setpoint) > usRegHoldingBuf[HREG_ACCURACY]) {
                    Set_I(I_setpoint);
                }
            }
        }
        if (current_tick - last_tick_second >= 1000) {
            work_time++;
            uint8_t work_time_8[4] = {work_time & 0x000000FF, work_time & 0x0000FF00, work_time & 0x00FF0000,
                                      work_time & 0xFF000000};
            // EEPROM_Write(&eeprom, 64, work_time_8, 4, 100);
            last_tick_second = current_tick;
            // MeasureVref();
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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
    GPIO_Init();
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DAC_SetVoltage(uint16_t voltage)
{
    DAC->DHR12R1 = (((uint32_t)voltage << 12) / vdda) & 0xFFF;
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}
void DAC_ChangeVoltage(void)
{
    if (DAC->DHR12R1 > usRegHoldingBuf[HREG_CALIB_VALUE])
        DAC->DHR12R1 -= 1;
    else if (DAC->DHR12R1 < usRegHoldingBuf[HREG_CALIB_VALUE])
        DAC->DHR12R1 += 1;
    else {
        CLR_CUR_SETTING();
        // Остановка TIM6
        TIM6->CR1 &= ~TIM_CR1_CEN;
        TIM6->CNT = 0;
        return;
    }
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}
void DAC_StartChangingV(void) //! вызывается просто при смене регистра, может вызваться не в режиме калибровки.
{                             // даже если уже устаналвивается, все равно вызовется
    SET_CUR_SETTING();
    // Запуск TIM6
    TIM6->CNT = 0;
    TIM6->SR &= ~TIM_SR_UIF;
    TIM6->CR1 |= TIM_CR1_CEN;
}

uint16_t ADC_ReadChannel(uint8_t channel)
{
    // Static buffers for sliding filter (channels 0 and 1)
    static struct {
        uint16_t buffer[5];
        uint8_t index;
        uint8_t count; // 0-5: number of valid elements
    } ch_filters[2] = {0};

    uint32_t sum = 0;
    ADC1->SQR3 = channel;

    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC))
        ;
    (void)ADC1->DR;
    ADC1->SR = 0;

    for (uint8_t i = 0; i < 10; i++) {
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC))
            ;
        sum += ADC1->DR;
        ADC1->SR = 0;
    }

    uint16_t raw_avg = sum / 10;
    uint16_t mv_value = ((uint32_t)raw_avg * vdda) >> 12;

    if (channel == 0 || channel == 1) {
        uint8_t filter_idx = channel;

        ch_filters[filter_idx].buffer[ch_filters[filter_idx].index] = mv_value;
        ch_filters[filter_idx].index++;

        if (ch_filters[filter_idx].index >= 5) {
            ch_filters[filter_idx].index = 0;
        }

        if (ch_filters[filter_idx].count < 5) {
            ch_filters[filter_idx].count++;
        }

        uint32_t filter_sum = 0;
        for (uint8_t i = 0; i < ch_filters[filter_idx].count; i++) {
            filter_sum += ch_filters[filter_idx].buffer[i];
        }

        return (uint16_t)(filter_sum / ch_filters[filter_idx].count);
    }

    // For channels 2-5, 17 - no filter
    return mv_value;
}
void MeasureVref(void)
{
    uint32_t sum = 0;

    ADC1->DR;
    ADC1->SR = 0;
    ADC1->SQR3 = 17;

    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC))
        ;
    (void)ADC1->DR;
    ADC1->SR = 0;

    for (uint8_t i = 0; i < 8; i++) {
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC))
            ;
        sum += ADC1->DR;
        ADC1->SR = 0;
    }

    uint16_t raw_vrefint = sum / 8;
    vdda = (3300UL * (*(uint16_t *)VREFINT_CAL_ADDR)) / raw_vrefint;
}

void UART4_Transmit(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
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
    static uint16_t u_cur_mes;
    static uint16_t u_mes_kz;
    static uint16_t u_mes;
    if (CHECK_CALIB_IN_PR() && (c_index <= N_SEGMENTS)) { //&& !CHECK_CUR_SETTING()) {
        if (!NormalOrKZ) {
            u_cur_mes = ADC_ReadChannel(0);
            u_mes = ADC_ReadChannel(1);
            c_a[c_index] = (uint16_t)(((uint32_t)usRegInputBuf[IREG_OSCILLOSCOPE_I] << Q11_SHIFT) /
                                      (((uint32_t)vdda * usRegHoldingBuf[HREG_CALIB_VALUE]) >> 11));
            c_b[c_index] = (uint16_t)(((uint32_t)usRegInputBuf[IREG_OSCILLOSCOPE_I] << Q11_SHIFT) / u_cur_mes);

            uint16_t scaled_c_a = (uint16_t)((((uint32_t)c_a[c_index] * 1000) + Q11_HALF) >> Q11_SHIFT);
            uint16_t scaled_c_b = (uint16_t)((((uint32_t)c_b[c_index] * 1000) + Q11_HALF) >> Q11_SHIFT);

            usRegInputBuf[IREG_CALIB_STEP] = c_index + 1;
            usRegInputBuf[IREG_CALIB_CA] = scaled_c_a;
            usRegInputBuf[IREG_CALIB_CB] = scaled_c_b;

            SET_NormalOrKZ();
            NormalOrKZ = 1;

        } else {
            u_mes_kz = ADC_ReadChannel(1);

            uint16_t I_osc = usRegInputBuf[IREG_OSCILLOSCOPE_I];
            uint16_t I_kz = usRegHoldingBuf[HREG_CALIB_I_KZ];
            uint16_t U_calib = usRegHoldingBuf[HREG_CALIB_U];

            if (I_kz == 0)
                I_kz = 1;

            // c_d = U_calib / (u_mes - I_osc/I_kz * u_mes_kz)
            uint32_t ratio_q11 = ((uint32_t)I_osc << Q11_SHIFT) / I_kz;
            uint32_t subtract_term = (ratio_q11 * u_mes_kz) >> Q11_SHIFT;

            int32_t denom = (int32_t)u_mes - (int32_t)subtract_term;
            if (denom <= 0)
                denom = 1;

            c_d[c_index] = (uint16_t)(((uint32_t)U_calib << Q11_SHIFT) / (uint32_t)denom);

            // c_e = (u_mes * c_d - U_calib) / I_osc
            if (I_osc == 0)
                I_osc = 1;

            uint32_t u_mes_times_cd = (uint32_t)u_mes * c_d[c_index];
            uint32_t u_calib_q11 = (uint32_t)U_calib << Q11_SHIFT;

            if (u_mes_times_cd >= u_calib_q11) {
                c_e[c_index] = (uint16_t)((u_mes_times_cd - u_calib_q11) / I_osc);
            } else {
                c_e[c_index] = 0;
            }

            CLR_NormalOrKZ();
            NormalOrKZ = 0;
            usRegHoldingBuf[HREG_CALIB_U] = 0;
            usRegHoldingBuf[HREG_CALIB_I_KZ] = 0;
            usRegHoldingBuf[HREG_CALIB_VALUE] = 0;
            c_index++;
            usRegInputBuf[IREG_OSCILLOSCOPE_I] = MIN_I + c_index * ((MAX_I - MIN_I) / N_SEGMENTS);
        }
    }
    if (c_index == N_SEGMENTS + 1 ||
        !CHECK_SET_CALIB_MODE()) { // зайдет сюда, даже если не был в режиме калибровки а просто нажал кнопку
        NormalOrKZ = 0;
        TIM6->ARR = 500;
        c_index = 0;
        CLR_CALIB_MODE();
        CLR_CALIB_MODE_IN_PR();
        SET_STANDBY_MODE();
        usRegInputBuf[IREG_OSCILLOSCOPE_I] = 0;
        usRegHoldingBuf[HREG_CALIB_VALUE] = 0;
        usRegInputBuf[IREG_CALIB_STEP] = 0;
        usRegInputBuf[IREG_CALIB_CA] = 0;
        usRegInputBuf[IREG_CALIB_CB] = 0;
        uint8_t c_c = N_SEGMENTS;
        uint8_t c_a_8[2 * N_SEGMENTS];
        uint8_t c_b_8[2 * N_SEGMENTS];
        for (uint8_t i = 0; i < N_SEGMENTS; i++) {
            c_a_8[i] = c_a[i] & 0xFF;
            c_a_8[2 * i + 1] = (c_a[i] >> 8) & 0xFF;
            c_b_8[i] = c_b[i] & 0xFF;
            c_b_8[2 * i + 1] = (c_b[i] >> 8) & 0xFF;
        }
        // EEPROM_Write(&eeprom, 0, &c_c, 1, 100);
        // EEPROM_Write(&eeprom, 1, c_a_8, 2 * N_SEGMENTS, 100);
        // EEPROM_Write(&eeprom, 1, c_b_8, 2 * N_SEGMENTS, 100);
    }
}
void Set_I(uint16_t i)
{
    i_set_cur = i;
    SET_CUR_SETTING();
    // Запуск TIM6
    TIM6->CNT = 0;
    TIM6->SR &= ~TIM_SR_UIF;
    TIM6->CR1 |= TIM_CR1_CEN;
}
void StopChange_I(void)
{
    CLR_CUR_SETTING();
    // Остановка TIM6
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->CNT = 0;
}
void Change_I(void)
{
    // if (!(CHECK_CONVERTER_STATUS() && CHECK_SOURCE_STATUS()))
    //    return;

    uint16_t U_mes = ADC_ReadChannel(0);
    uint16_t coef_b = Get_c_b_by_U_mes(U_mes);
    uint16_t I_cur = (uint16_t)(((uint32_t)U_mes * coef_b) >> Q11_SHIFT);
    uint16_t coef_a = Get_c_a_by_I(I_cur);

    uint16_t I_set_cur_local;
    if (CHECK_SET_OPERATING_MODE())
        I_set_cur_local = usRegHoldingBuf[HREG_OPERATING_I];
    else if (CHECK_SET_STANDBY_MODE())
        I_set_cur_local = usRegHoldingBuf[HREG_STANDBY_I];
    else
        I_set_cur_local = usRegHoldingBuf[HREG_STANDBY_I]; // По умолчанию

    uint16_t diff_I = (I_cur > I_set_cur_local) ? (I_cur - I_set_cur_local) : (I_set_cur_local - I_cur);

    if (diff_I < usRegHoldingBuf[HREG_ACCURACY]) {
        StopChange_I();
        return;
    }

    uint16_t delta_U;
    if (diff_I < 500) {
        // Мелкое управление - шаг = точность
        delta_U = (uint16_t)(((uint32_t)usRegHoldingBuf[HREG_ACCURACY] << Q11_SHIFT) / (uint32_t)coef_a);
    } else {
        // Быстрое управление - шаг = скорость / N_TIMERS
        delta_U =
            (uint16_t)(((uint32_t)usRegHoldingBuf[HREG_CURRENT_SPEED] << Q11_SHIFT) / ((uint32_t)N_TIMERS * coef_a));
    }

    uint16_t new_voltage;
    if (I_cur < I_set_cur_local) {
        new_voltage = usRegInputBuf[IREG_U_SET] + delta_U;
    } else {
        new_voltage = usRegInputBuf[IREG_U_SET] - delta_U;
    }

    if (new_voltage > MAX_U)
        new_voltage = MAX_U;

    DAC_SetVoltage(new_voltage);
    usRegInputBuf[IREG_U_SET] = new_voltage;
}
uint16_t Get_c_a_by_I(uint16_t i)
{
    if (CHECK_CALIB_IN_PR())
        return COEF_A_DEFAULT << Q11_SHIFT;

    const uint16_t default_val = (uint16_t)(COEF_A_DEFAULT << Q11_SHIFT);

    if (i <= MIN_I)
        return (c_a[0] != 0) ? c_a[0] : default_val;
    if (i >= MAX_I)
        return (c_a[N_SEGMENTS - 1] != 0) ? c_a[N_SEGMENTS - 1] : default_val;

    uint32_t idx = ((uint32_t)(i - MIN_I) * (uint32_t)N_SEGMENTS) / (MAX_I - MIN_I + 1);
    if (idx >= (uint32_t)N_SEGMENTS)
        idx = N_SEGMENTS - 1;

    return (c_a[idx] == 0) ? default_val : c_a[idx];
}

uint16_t Get_c_b_by_U_mes(uint16_t u)
{
    if (CHECK_CALIB_IN_PR())
        return COEF_B_DEFAULT << Q11_SHIFT;

    const uint16_t default_val = (uint16_t)(COEF_B_DEFAULT << Q11_SHIFT);

    if (u <= MIN_U)
        return (c_b[0] != 0) ? c_b[0] : default_val;
    if (u >= MAX_U)
        return (c_b[N_SEGMENTS - 1] != 0) ? c_b[N_SEGMENTS - 1] : default_val;

    uint32_t denom = (uint32_t)MAX_U - (uint32_t)MIN_U + 1U;
    if (denom == 0)
        return default_val;

    uint32_t idx = ((uint32_t)(u - MIN_U) * (uint32_t)N_SEGMENTS) / (MAX_U - MIN_U + 1);
    if (idx >= (uint32_t)N_SEGMENTS)
        idx = N_SEGMENTS - 1;

    return (c_b[idx] == 0) ? default_val : c_b[idx];
}

uint16_t Calculate_I_from_U(uint16_t u_mes)
{
    uint16_t coef_b = Get_c_b_by_U_mes(u_mes);
    return (uint16_t)(((uint32_t)u_mes * coef_b) >> Q11_SHIFT);
}

uint16_t Get_c_d_by_U(uint16_t u)
{
    if (CHECK_CALIB_IN_PR())
        return COEF_D_DEFAULT << Q11_SHIFT;

    const uint16_t default_val = (uint16_t)(COEF_D_DEFAULT << Q11_SHIFT);

    if (u <= MIN_U)
        return (c_d[0] != 0) ? c_d[0] : default_val;
    if (u >= MAX_U)
        return (c_d[N_SEGMENTS - 1] != 0) ? c_d[N_SEGMENTS - 1] : default_val;
    uint32_t idx = ((uint32_t)(u - MIN_U) * (uint32_t)N_SEGMENTS) / (MAX_U - MIN_U + 1);
    if (idx >= (uint32_t)N_SEGMENTS)
        idx = N_SEGMENTS - 1;

    return (c_d[idx] == 0) ? default_val : c_d[idx];
}

uint16_t Get_c_e_by_I(uint16_t i)
{
    if (CHECK_CALIB_IN_PR())
        return COEF_E_DEFAULT;

    if (i <= MIN_I)
        return c_e[0];
    if (i >= MAX_I)
        return c_e[N_SEGMENTS - 1];

    uint32_t idx = ((uint32_t)(i - MIN_I) * (uint32_t)N_SEGMENTS) / (MAX_I - MIN_I + 1);
    if (idx >= (uint32_t)N_SEGMENTS)
        idx = N_SEGMENTS - 1;

    return c_e[idx];
}

void Update_Input_States(void)
{
    uint8_t source_state = (GPIOB->IDR & (1 << SOURCE_STATUS_PIN)) ? 1 : 0;
    if (source_state != CHECK_SOURCE_STATUS()) {
        if (source_state)
            SET_SOURCE_STATUS();
        else
            CLR_SOURCE_STATUS();
    }

    uint8_t converter_state = (GPIOB->IDR & (1 << CONVERTER_STATUS_PIN)) ? 1 : 0;
    if (converter_state != CHECK_CONVERTER_STATUS()) {
        if (converter_state)
            SET_CONVERTER_STATUS();
        else
            CLR_CONVERTER_STATUS();
    }
}

void SetOperatingMode(void)
{
    if (CHECK_CALIB_IN_PR()) {
        CLR_CALIB_MODE();
        Calibration(); // Завершить калибровку
    } else if (CHECK_SET_STANDBY_MODE()) {
        CLR_STANDBY_MODE();
    }
}

void SetStandbyMode(void)
{
    if (CHECK_CALIB_IN_PR()) {
        CLR_CALIB_MODE();
        Calibration(); // Завершить калибровку
    } else if (CHECK_SET_OPERATING_MODE()) {
        CLR_OPERATING_MODE();
    }
}

void SetCalibrationMode(void)
{
    // Проверка джампера - калибровка запрещена если джампер установлен
    if (CHECK_JMPR_STATUS()) {
        CLR_CALIB_MODE();
        return;
    }

    // Выход из текущего режима
    if (CHECK_SET_STANDBY_MODE()) {
        CLR_STANDBY_MODE();
    } else if (CHECK_SET_OPERATING_MODE()) {
        CLR_OPERATING_MODE();
    }

    // Вход в режим калибровки
    if (!CHECK_JMPR_STATUS()) {
        usRegInputBuf[IREG_OSCILLOSCOPE_I] = CALIB_INITIAL_I;
        TIM6->ARR = 500;
        SET_CALIB_MODE_IN_PR();
    }
}

void Coils_ApplyToPins(void)
{
    if (CHECK_ENABLE_SOURCE())
        GPIOB->BSRR = (1 << SOURCE_PIN);
    else
        GPIOB->BSRR = (1 << (SOURCE_PIN + 16));

    if (CHECK_ENABLE_CONVERTER())
        GPIOB->BSRR = (1 << CONVERTER_PIN);
    else
        GPIOB->BSRR = (1 << (CONVERTER_PIN + 16));
}
void HandleModbusRequest(uint8_t *RxBuf)
{
    if (RxBuf[0] == SLAVE_ID) {
        switch (RxBuf[1]) {
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
        modbus_req_count++;
        eeprom_write_pending = 1; // Deferred write
    }
}

void WriteLog(LogType_t type, uint8_t subtype, uint8_t code)
{
    uint32_t tick = HAL_GetTick();
    uint8_t full_code = type << 6 | subtype << 4 | code;
    uint8_t log[5] = {tick & 0x000000FF, tick & 0x0000FF00, tick & 0x00FF0000, tick & 0xFF000000, full_code};
    EEPROM_Write(&eeprom, 129 + 5 * log_i, log, 5, 100);
    if (log_i == 24)
        log_i = 0;
    else
        log_i++;
    if (log_count != 255) {
        log_count++;
        EEPROM_Write(&eeprom, 128, &log_count, 1, 100);
    }
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
    while (1) {
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
