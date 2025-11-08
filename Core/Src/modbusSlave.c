#include "modbusSlave.h"
#include "string.h"
#include "main.h"
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
	data[size] = crc & 0xFF;			// CRC LOW
	data[size + 1] = (crc >> 8) & 0xFF; // CRC HIGH
	TransmitFuncPtr(data, size + 2);
}

void modbusException(uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxBuffer[0] = RxBuffer[0];		  // slave ID
	TxBuffer[1] = RxBuffer[1] | 0x80; // adding 1 to the MSB of the function code
	TxBuffer[2] = exceptioncode;	  // Load the Exception code
	sendData(TxBuffer, 3);			  // send Data... CRC will be calculated in the function
}

uint8_t readHoldingRegs(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Register Address

	uint16_t numRegs = ((RxBuffer[4] << 8) | RxBuffer[5]); // number to registers master has requested
	if ((numRegs < 1) || (numRegs > 125))				   // maximum no. of Registers as per the PDF
	{
		modbusException(ILLEGAL_DATA_VALUE); // send an exception
		return 1;
	}
	// Prepare TxBuffer buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |
	TxBuffer[0] = SLAVE_ID;	   // slave ID
	TxBuffer[1] = RxBuffer[1]; // function code
	TxBuffer[2] = numRegs * 2; // Byte count
							   // we need to keep track of how many bytes has been stored in TxBuffer Buffer
	int indx = 3;
	if ((startAddr >= REG_HOLDING_START) &&
		(startAddr + numRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{

		int iRegIndex = (int)(startAddr - REG_HOLDING_START);
		while (numRegs > 0)
		{
			TxBuffer[indx++] = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
			TxBuffer[indx++] = (unsigned char)(usRegHoldingBuf[iRegIndex] & 0xFF);
			iRegIndex++;
			numRegs--;
		}
		sendData(TxBuffer, indx); // send data... CRC will be calculated in the function itself
		return 0;				  // success
	}
	else
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}
}

uint8_t readInputRegs(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Register Address

	uint16_t numRegs = ((RxBuffer[4] << 8) | RxBuffer[5]); // number to registers master has requested
	if ((numRegs < 1) || (numRegs > 125))				   // maximum no. of Registers as per the PDF
	{
		modbusException(ILLEGAL_DATA_VALUE); // send an exception
		return 1;
	}

	// Prepare TxBuffer buffer
	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |
	TxBuffer[0] = SLAVE_ID;	   // slave ID
	TxBuffer[1] = RxBuffer[1]; // function code
	TxBuffer[2] = numRegs * 2; // Byte count
	int indx = 3;			   // we need to keep track of how many bytes has been stored in TxBuffer Buffer
	if ((startAddr >= REG_INPUT_START) &&
		(startAddr + numRegs <= REG_INPUT_START + REG_INPUT_NREGS))
	{
		int iRegIndex = (int)(startAddr - REG_INPUT_START);

		while (numRegs > 0)
		{
			TxBuffer[indx++] = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
			TxBuffer[indx++] = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
			iRegIndex++;
			numRegs--;
		}
		sendData(TxBuffer, indx); // send data... CRC will be calculated in the function itself
		return 0;				  // success
	}
	else
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}
}

uint8_t readCoils(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Coil Address

	uint16_t numCoils = ((RxBuffer[4] << 8) | RxBuffer[5]); // number to coils master has requested
	if ((numCoils < 1) || (numCoils > COILS_N))				// maximum no. of coils as per the PDF
	{
		modbusException(ILLEGAL_DATA_VALUE); // send an exception
		return 1;
	}

	if (!(startAddr >= COILS_START) ||
		!(startAddr + numCoils <= COILS_START + COILS_N))
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}

	// reset TxBuffer buffer
	memset(TxBuffer, '\0', 256);

	// Prepare TxBuffer buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxBuffer[0] = SLAVE_ID;										 // slave ID
	TxBuffer[1] = RxBuffer[1];									 // function code
	TxBuffer[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0); // Byte count
	int indx = 3;												 // we need to keep track of how many bytes has been stored in TxBuffer Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the TxBuffer buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxBuffer[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxBuffer
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr / 8;				// which byte we have to start extracting the data from
	uint16_t bitPosition = (startAddr - 1) % 8; // The shift position in the first byte
	int indxPosition = 0;						// The shift position in the current indx of the TxBuffer buffer

	// Load the actual data into TxBuffer buffer
	for (int i = 0; i < numCoils; i++)
	{
		TxBuffer[indx] |= ((usCoilsBuf[startByte] >> bitPosition) & 0x01) << indxPosition;
		indxPosition++;
		bitPosition++;
		if (indxPosition > 7) // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7) // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition = 0;
			startByte++;
		}
	}

	if (numCoils % 8 != 0)
		indx++;				  // increment the indx variable, only if the numcoils is not a multiple of 8
	sendData(TxBuffer, indx); // send data... CRC will be calculated in the function itself
	return 0;				  // success
}

uint8_t readInputs(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Register Address

	uint16_t numCoils = ((RxBuffer[4] << 8) | RxBuffer[5]); // number to coils master has requested
	if ((numCoils < 1) || (numCoils > DISCRETE_N))			// maximum no. of coils as per the PDF
	{
		modbusException(ILLEGAL_DATA_VALUE); // send an exception
		return 1;
	}

	if (!(startAddr >= DISCRETE_START) ||
		!(startAddr + numCoils <= DISCRETE_START + DISCRETE_N))
	{ // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total

		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}

	// reset TxBuffer buffer
	memset(TxBuffer, '\0', 256);

	// Prepare TxBuffer buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxBuffer[0] = SLAVE_ID;										 // slave ID
	TxBuffer[1] = RxBuffer[1];									 // function code
	TxBuffer[2] = (numCoils / 8) + ((numCoils % 8) > 0 ? 1 : 0); // Byte count
	int indx = 3;												 // we need to keep track of how many bytes has been stored in TxBuffer Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the TxBuffer buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxBuffer[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxBuffer
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = (startAddr - DISCRETE_START) / 8;		 // which byte we have to start extracting the data from
	uint16_t bitPosition = (startAddr - DISCRETE_START) % 8; // The shift position in the first byte
	int indxPosition = 0;									 // The shift position in the current indx of the TxBuffer buffer

	// Load the actual data into TxBuffer buffer
	for (int i = 0; i < numCoils; i++)
	{
		TxBuffer[indx] |= ((usDiscreteBuf[startByte] >> bitPosition) & 0x01) << indxPosition;
		indxPosition++;
		bitPosition++;
		if (indxPosition > 7) // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7) // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition = 0;
			startByte++;
		}
	}

	if (numCoils % 8 != 0)
		indx++;				  // increment the indx variable, only if the numcoils is not a multiple of 8
	sendData(TxBuffer, indx); // send data... CRC will be calculated in the function itself
	return 0;				  // success
}

uint8_t writeHoldingRegs(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Register Address

	uint16_t numRegs = ((RxBuffer[4] << 8) | RxBuffer[5]); // number to registers master has requested
	if ((numRegs < 1) || (numRegs > 123))				   // maximum no. of Registers as per the PDF
	{
		modbusException(ILLEGAL_DATA_VALUE); // send an exception
		return 1;
	}

	if (!(startAddr >= REG_HOLDING_START) ||
		!(startAddr + numRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}
	int iRegIndex = (int)(startAddr - REG_HOLDING_START);
	/* start saving 16 bit data
	 * Data starts from RxBuffer[7] and we need to combine 2 bytes together
	 * 16 bit Data = firstByte<<8|secondByte
	 */
	int indx = 7; // we need to keep track of index in RxBuffer
	while (numRegs > 0)
	{
		usRegHoldingBuf[iRegIndex++] = (RxBuffer[indx++] << 8) | RxBuffer[indx++];
		numRegs--;
	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | num of Regs    | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES      | 2 BYTES |

	TxBuffer[0] = SLAVE_ID;	   // slave ID
	TxBuffer[1] = RxBuffer[1]; // function code
	TxBuffer[2] = RxBuffer[2]; // Start Addr HIGH Byte
	TxBuffer[3] = RxBuffer[3]; // Start Addr LOW Byte
	TxBuffer[4] = RxBuffer[4]; // num of Regs HIGH Byte
	TxBuffer[5] = RxBuffer[5]; // num of Regs LOW Byte
	sendData(TxBuffer, 6);	   // send data... CRC will be calculated in the function itself
	return 0;				   // success
}

uint8_t writeSingleReg(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Register Address

	if (!(startAddr >= REG_HOLDING_START) ||
		!(startAddr <= REG_HOLDING_START + REG_HOLDING_NREGS))
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}

	/* Save the 16 bit data
	 * Data is the combination of 2 bytes, RxBuffer[4] and RxBuffer[5]
	 */

	usRegHoldingBuf[startAddr] = (RxBuffer[4] << 8) | RxBuffer[5];

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxBuffer[0] = SLAVE_ID;	   // slave ID
	TxBuffer[1] = RxBuffer[1]; // function code
	TxBuffer[2] = RxBuffer[2]; // Start Addr HIGH Byte
	TxBuffer[3] = RxBuffer[3]; // Start Addr LOW Byte
	TxBuffer[4] = RxBuffer[4]; // Reg Data HIGH Byte
	TxBuffer[5] = RxBuffer[5]; // Reg Data LOW  Byte
	sendData(TxBuffer, 6);	   // send data... CRC will be calculated in the function itself
	return 0;				   // success
}

uint8_t writeSingleCoil(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Coil Address

	if (!(startAddr >= COILS_START) ||
		!(startAddr <= COILS_START + COILS_N))
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}

	/* Calculation for the bit in the database, where the modification will be done */
	int startByte = (startAddr - COILS_START) / 8;		  // which byte we have to start writing the data into
	uint16_t bitPosition = (startAddr - COILS_START) % 8; // The shift position in the first byte

	/* The next 2 bytes in the RxBuffer determines the state of the coil
	 * A value of FF 00 hex requests the coil to be ON.
	 * A value of 00 00 requests it to be OFF.
	 * All other values are illegal and will not affect the coil.
	 */

	if ((RxBuffer[4] == 0xFF) && (RxBuffer[5] == 0x00))
	{
		usCoilsBuf[startByte] |= 1 << bitPosition; // Replace that bit with 1
	}

	else if ((RxBuffer[4] == 0x00) && (RxBuffer[5] == 0x00))
	{
		usCoilsBuf[startByte] &= ~(1 << bitPosition); // Replace that bit with 0
	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxBuffer[0] = SLAVE_ID;	   // slave ID
	TxBuffer[1] = RxBuffer[1]; // function code
	TxBuffer[2] = RxBuffer[2]; // Start Addr HIGH Byte
	TxBuffer[3] = RxBuffer[3]; // Start Addr LOW Byte
	TxBuffer[4] = RxBuffer[4]; // Coil Data HIGH Byte
	TxBuffer[5] = RxBuffer[5]; // Coil Data LOW  Byte

	sendData(TxBuffer, 6); // send data... CRC will be calculated in the function itself
	return 0;			   // success
}

uint8_t writeMultiCoils(void)
{
	uint16_t startAddr = ((RxBuffer[2] << 8) | RxBuffer[3]); // start Coil Address

	uint16_t numCoils = ((RxBuffer[4] << 8) | RxBuffer[5]); // number to coils master has requested
	if ((numCoils < 1) || (numCoils > COILS_N))				// maximum no. of coils as per the PDF
	{
		modbusException(ILLEGAL_DATA_VALUE); // send an exception
		return 1;
	}

	if (!(startAddr >= COILS_START) ||
		!(startAddr + numCoils <= COILS_START + COILS_N))
	{
		modbusException(ILLEGAL_DATA_ADDRESS); // send an exception
		return 1;
	}

	/* Calculation for the bit in the database, where the modification will be done */
	int startByte = (startAddr - COILS_START) / 8;		  // which byte we have to start writing the data into
	uint16_t bitPosition = (startAddr - COILS_START) % 8; // The shift position in the first byte
	int indxPosition = 0;								  // The shift position in the current indx of the RxBuffer buffer

	int indx = 7; // we need to keep track of index in RxBuffer

	/* The approach is simple. We will read 1 bit (starting from the very first bit in the RxBuffer Buffer)
	 * at a time and store them in the Database.
	 * First find the offset in the first byte we write into, for eg- if the start coil is 13,
	 * we will Write into database[1] with an offset of 5. This bit is read from the RxBuffer[indx] at 0th indxposition.
	 * Then we will keep shifting the RxBuffer[indx] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte and start modifying the next byte in the database
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy from the next byte of the RxBuffer
	 * This keeps going until the number of coils required have been modified
	 */

	// Modify the bits as per the Byte received
	for (int i = 0; i < numCoils; i++)
	{
		if (((RxBuffer[indx] >> indxPosition) & 0x01) == 1)
		{
			usCoilsBuf[startByte] |= 1 << bitPosition; // replace that bit with 1
		}
		else
		{
			usCoilsBuf[startByte] &= ~(1 << bitPosition); // replace that bit with 0
		}

		bitPosition++;
		indxPosition++;

		if (indxPosition > 7) // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition > 7) // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition = 0;
			startByte++;
		}
	}
	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxBuffer[0] = SLAVE_ID;	   // slave ID
	TxBuffer[1] = RxBuffer[1]; // function code
	TxBuffer[2] = RxBuffer[2]; // Start Addr HIGH Byte
	TxBuffer[3] = RxBuffer[3]; // Start Addr LOW Byte
	TxBuffer[4] = RxBuffer[4]; // num of coils HIGH Byte
	TxBuffer[5] = RxBuffer[5]; // num of coils LOW  Byte

	sendData(TxBuffer, 6); // send data... CRC will be calculated in the function itself
	return 0;			   // success
}
