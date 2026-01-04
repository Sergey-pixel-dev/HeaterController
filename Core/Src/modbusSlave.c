#include "modbusSlave.h"
#include "main.h"
#include <string.h>
uint8_t *RxBuffer;
uint8_t *TxBuffer;

uint16_t usRegInputBuf[REG_INPUT_NREGS];
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
uint16_t usCoilsBuf[1];
uint16_t usDiscreteBuf[1];

void (*TransmitFuncPtr)(uint8_t *data, uint16_t size);

void sendData(uint8_t *data, int size)
{
    // we will calculate the CRC in this function itself
    uint16_t crc = crc16(data, size);
    data[size] = crc & 0xFF;            // CRC LOW
    data[size + 1] = (crc >> 8) & 0xFF; // CRC HIGH
    TransmitFuncPtr(data, size + 2);
}

void modbusException(uint8_t exceptioncode)
{
    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1] | 0x80;
    TxBuffer[2] = exceptioncode;
    sendData(TxBuffer, 3);
}

uint8_t readHoldingRegs(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t numRegs = ((RxBuffer[4] << 8) | RxBuffer[5]);

    if ((numRegs < 1) || (numRegs > 125)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    if (startAddr < REG_HOLDING_START || startAddr + numRegs > REG_HOLDING_START + REG_HOLDING_NREGS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = numRegs * 2;

    uint16_t start_idx = startAddr - REG_HOLDING_START;
    int indx = 3;
    for (uint16_t i = 0; i < numRegs; i++) {
        TxBuffer[indx++] = (usRegHoldingBuf[start_idx + i] >> 8) & 0xFF;
        TxBuffer[indx++] = usRegHoldingBuf[start_idx + i] & 0xFF;
    }

    sendData(TxBuffer, indx);
    return 0;
}

uint8_t readInputRegs(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t numRegs = ((RxBuffer[4] << 8) | RxBuffer[5]);

    if ((numRegs < 1) || (numRegs > 125)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    if (startAddr < REG_INPUT_START || startAddr + numRegs > REG_INPUT_START + REG_INPUT_NREGS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = numRegs * 2;

    uint16_t start_idx = startAddr - REG_INPUT_START;
    int indx = 3;
    for (uint16_t i = 0; i < numRegs; i++) {
        TxBuffer[indx++] = (usRegInputBuf[start_idx + i] >> 8) & 0xFF;
        TxBuffer[indx++] = usRegInputBuf[start_idx + i] & 0xFF;
    }

    sendData(TxBuffer, indx);
    return 0;
}

uint8_t readCoils(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t numCoils = ((RxBuffer[4] << 8) | RxBuffer[5]);

    if ((numCoils < 1) || (numCoils > 2000)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    if (startAddr < COILS_START || startAddr + numCoils > COILS_START + COILS_N) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    uint8_t byte_count = (numCoils + 7) / 8;
    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = byte_count;

    for (uint8_t i = 0; i < byte_count; i++) {
        TxBuffer[3 + i] = 0;
    }

    for (uint16_t i = 0; i < numCoils; i++) {
        uint32_t bit_index = (uint32_t)(startAddr - COILS_START) + i;
        uint16_t word_index = bit_index >> 4;
        uint16_t bit_in_word = bit_index & 0x0F;

        if (usCoilsBuf[word_index] & (1 << bit_in_word)) {
            TxBuffer[3 + (i / 8)] |= (1 << (i % 8));
        }
    }

    sendData(TxBuffer, 3 + byte_count);
    return 0;
}

uint8_t readInputs(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t numInputs = ((RxBuffer[4] << 8) | RxBuffer[5]);

    if ((numInputs < 1) || (numInputs > 2000)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    if (startAddr < DISCRETE_START || startAddr + numInputs > DISCRETE_START + DISCRETE_N) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    uint8_t byte_count = (numInputs + 7) / 8;
    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = byte_count;

    for (uint8_t i = 0; i < byte_count; i++) {
        TxBuffer[3 + i] = 0;
    }

    for (uint16_t i = 0; i < numInputs; i++) {
        uint32_t bit_index = (uint32_t)(startAddr - DISCRETE_START) + i;
        uint16_t word_index = bit_index >> 4;
        uint16_t bit_in_word = bit_index & 0x0F;

        if (usDiscreteBuf[word_index] & (1 << bit_in_word)) {
            TxBuffer[3 + (i / 8)] |= (1 << (i % 8));
        }
    }

    sendData(TxBuffer, 3 + byte_count);
    return 0;
}

uint8_t writeHoldingRegs(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t numRegs = ((RxBuffer[4] << 8) | RxBuffer[5]);

    if ((numRegs < 1) || (numRegs > 123)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    if (startAddr < REG_HOLDING_START || startAddr + numRegs > REG_HOLDING_START + REG_HOLDING_NREGS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    int iRegIndex = (int)(startAddr - REG_HOLDING_START);
    int indx = 7;
    while (numRegs > 0) {
        usRegHoldingBuf[iRegIndex++] = (RxBuffer[indx] << 8) | RxBuffer[indx + 1];
        indx += 2;
        numRegs--;
    }

    if (iRegIndex > 0 && (startAddr - REG_HOLDING_START) == 0) {
        Set_I(usRegHoldingBuf[0]);
    }

    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = RxBuffer[2];
    TxBuffer[3] = RxBuffer[3];
    TxBuffer[4] = RxBuffer[4];
    TxBuffer[5] = RxBuffer[5];
    sendData(TxBuffer, 6);
    return 0;
}

uint8_t writeSingleReg(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);

    if (startAddr < REG_HOLDING_START || startAddr >= REG_HOLDING_START + REG_HOLDING_NREGS) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    uint16_t reg_idx = startAddr - REG_HOLDING_START;
    uint16_t old_val = usRegHoldingBuf[reg_idx];
    usRegHoldingBuf[reg_idx] = (RxBuffer[4] << 8) | RxBuffer[5];

    if (reg_idx == 0 && old_val != usRegHoldingBuf[reg_idx]) {
        Set_I(usRegHoldingBuf[0]);
    }
    if (reg_idx == 4)
        DAC_StartChangingV();

    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = RxBuffer[2];
    TxBuffer[3] = RxBuffer[3];
    TxBuffer[4] = RxBuffer[4];
    TxBuffer[5] = RxBuffer[5];
    sendData(TxBuffer, 6);
    return 0;
}

uint8_t writeSingleCoil(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t value_field = (RxBuffer[4] << 8) | RxBuffer[5];

    if (startAddr < COILS_START || startAddr >= COILS_START + COILS_N) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    if (value_field != 0xFF00 && value_field != 0x0000) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    uint16_t coil_num = startAddr - COILS_START;
    uint16_t word_index = coil_num >> 4;
    uint16_t bit = coil_num & 0x0F;

    if (value_field == 0xFF00)
        usCoilsBuf[word_index] |= (1 << bit);
    else
        usCoilsBuf[word_index] &= ~(1 << bit);

    if (coil_num == COIL_BIT_ENABLE_SOURCE || coil_num == COIL_BIT_ENABLE_CONVERTER) {
        Coils_ApplyToPins();
    } else if (coil_num == COIL_BIT_SET_OPERATING_MODE) {
        SetOperatingMode();
    } else if (coil_num == COIL_BIT_SET_STANDBY_MODE) {
        SetStandbyMode();
    } else if (coil_num == COIL_BIT_SET_CALIB_MODE) {
        SetCalibrationMode();
    }

    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = RxBuffer[2];
    TxBuffer[3] = RxBuffer[3];
    TxBuffer[4] = RxBuffer[4];
    TxBuffer[5] = RxBuffer[5];
    sendData(TxBuffer, 6);
    return 0;
}

uint8_t writeMultiCoils(void)
{
    uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]);
    uint16_t numCoils = ((RxBuffer[4] << 8) | RxBuffer[5]);

    if ((numCoils < 1) || (numCoils > 2000)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 1;
    }

    if (startAddr < COILS_START || startAddr + numCoils > COILS_START + COILS_N) {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 1;
    }

    int startByte = (startAddr - COILS_START) / 16;
    uint16_t bitPosition = (startAddr - COILS_START) % 16;
    int indxPosition = 0;
    int indx = 7;

    uint8_t need_apply_pins = 0;
    uint8_t need_operating = 0;
    uint8_t need_standby = 0;
    uint8_t need_calib = 0;

    for (int i = 0; i < numCoils; i++) {
        if (((RxBuffer[indx] >> indxPosition) & 0x01) == 1) {
            usCoilsBuf[startByte] |= 1 << bitPosition;
        } else {
            usCoilsBuf[startByte] &= ~(1 << bitPosition);
        }

        uint16_t coil_num = (startAddr - COILS_START) + i;
        if (coil_num == COIL_BIT_ENABLE_SOURCE || coil_num == COIL_BIT_ENABLE_CONVERTER) {
            need_apply_pins = 1;
        } else if (coil_num == COIL_BIT_SET_OPERATING_MODE) {
            uint8_t new_value = ((usCoilsBuf[startByte] >> bitPosition) & 0x01);
            if (new_value == 1)
                need_operating = 1;
        } else if (coil_num == COIL_BIT_SET_STANDBY_MODE) {
            uint8_t new_value = ((usCoilsBuf[startByte] >> bitPosition) & 0x01);
            if (new_value == 1)
                need_standby = 1;
        } else if (coil_num == COIL_BIT_SET_CALIB_MODE) {
            uint8_t new_value = ((usCoilsBuf[startByte] >> bitPosition) & 0x01);
            if (new_value == 1)
                need_calib = 1;
        }

        bitPosition++;
        indxPosition++;

        if (indxPosition > 7) {
            indxPosition = 0;
            indx++;
        }
        if (bitPosition > 15) {
            bitPosition = 0;
            startByte++;
        }
    }

    if (need_apply_pins)
        Coils_ApplyToPins();
    if (need_operating)
        SetOperatingMode();
    if (need_standby)
        SetStandbyMode();
    if (need_calib)
        SetCalibrationMode();

    TxBuffer[0] = SLAVE_ID;
    TxBuffer[1] = RxBuffer[1];
    TxBuffer[2] = RxBuffer[2];
    TxBuffer[3] = RxBuffer[3];
    TxBuffer[4] = RxBuffer[4];
    TxBuffer[5] = RxBuffer[5];
    sendData(TxBuffer, 6);
    return 0;
}
