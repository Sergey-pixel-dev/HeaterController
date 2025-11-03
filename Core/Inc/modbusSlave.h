#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f4xx_hal.h"

#define SLAVE_ID 10

#define ILLEGAL_FUNCTION 0x01
#define ILLEGAL_DATA_ADDRESS 0x02
#define ILLEGAL_DATA_VALUE 0x03

#define REG_INPUT_START 31001
#define REG_HOLDING_START 41001
#define COILS_START 00001
#define DISCRETE_START 10001
#define COILS_N 2
#define DISCRETE_N 6
#define REG_INPUT_NREGS 9
#define REG_HOLDING_NREGS 6

extern uint16_t usRegInputBuf[REG_INPUT_NREGS];
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint16_t usCoilsBuf[1];
extern uint16_t usDiscreteBuf[1];

uint8_t readHoldingRegs(void);
uint8_t readInputRegs(void);
uint8_t readCoils(void);
uint8_t readInputs(void);

uint8_t writeSingleReg(void);
uint8_t writeHoldingRegs(void);
uint8_t writeSingleCoil(void);
uint8_t writeMultiCoils(void);

void modbusException(uint8_t exceptioncode);

#endif /* INC_MODBUSSLAVE_H_ */
