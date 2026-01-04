#ifndef INIT_H
#define INIT_H
#include "stm32f4xx_hal.h"
void DAC_Init(void);
void ADC_Init(void);
void DMA_Init(void);
void UART_Init(void);
void MODBUS_Timer_Init(void);
void TIM6_Init(void);
void GPIO_Init(void);
#endif
