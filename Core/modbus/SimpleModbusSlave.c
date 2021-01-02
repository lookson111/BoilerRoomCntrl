#include "SimpleModbusSlave.h"
#include "main.h"
#include "cmsis_os.h"

//UART_HandleTypeDef huart3;


void modbus_configure(ModBusTypeDef *modBusData,
											UART_HandleTypeDef *_uart,
											uint8_t _slaveID,
											//GPIO_TypeDef *_TxEnable_Port,
											//uint16_t _TxEnablePin,
											uint16_t _holdingRegsSize,
											uint16_t*  _regs)
{
	modBusData->uart = _uart;
	modbus_update_comms(modBusData, _uart->Init.BaudRate);
	modBusData->holdingRegsSize = _holdingRegsSize;
	modBusData->regs = _regs;
	//modBusData->TxEnable_Port = _TxEnable_Port;
	//modBusData->TxEnablePin = _TxEnablePin;
	modBusData->slaveID = _slaveID;
  //pinMode(TxEnablePin, OUTPUT);
  //digitalWrite(TxEnablePin, LOW);
	modBusData->errorCount = 0; // initialize errorCount
	modBusData->available = 0;
}

void modbus_update_comms(ModBusTypeDef *modBusData, uint32_t baud)
{
	//(*ModbusPort).begin(baud, byteFormat);


	// Modbus states that a baud rate higher than 19200 must use a fixed 750 us
  // for inter character time out and 1.75 ms for a frame delay for baud rates
  // below 19200 the timing is more critical and has to be calculated.
  // E.g. 9600 baud in a 10 bit packet is 960 characters per second
  // In milliseconds this will be 960characters per 1000ms. So for 1 character
  // 1000ms/960characters is 1.04167ms per character and finally modbus states
  // an inter-character must be 1.5T or 1.5 times longer than a character. Thus
  // 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.

	if (baud > 19200)
	{
		modBusData->T1_5 = 750;
		modBusData->T3_5 = 1750;
	}
	else
	{
		modBusData->T1_5 = 15000000/baud; // 1T * 1.5 = T1.5
		modBusData->T3_5 = 35000000/baud; // 1T * 3.5 = T3.5
	}
}

uint16_t modbus_update(ModBusTypeDef *modBusData)
{
  if (modBusData->available)
  {
  	modBusData->available = 0;

	  uint8_t overflow = 0;
//
//	  while ((*ModbusPort).available())
//	  {
//
//		  // If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the
//		  // serial buffer will be red untill all the data is cleared from the receive buffer.
//		  if (overflow)
//		  	HAL_UART_Receive(&huart3, frame, 1, 1);
//			  //(*ModbusPort).read();
//		  else
//		  {
//			  if (buffer == BUFFER_SIZE)
//				  overflow = 1;
//			  HAL_UART_Receive(&huart3, &frame[buffer], 1, 1000);
//			  //frame[buffer] = (*ModbusPort).read();
//			  buffer++;
//		  }
//		  delayMicroseconds(T1_5); // inter character time out
//	  }

//	Serial.println();
//	for (int i = 0; i < buffer; i++)
//	{
//		Serial.print("Frame1[");
//		Serial.print(i);
//		Serial.print("] = ");
//		Serial.println(frame[i]);
//	}
	  // If an overflow occurred increment the errorCount
	  // variable and return to the main sketch without
	  // responding to the request i.e. force a timeout
	  if (overflow)
		  return modBusData->errorCount++;

	  // The minimum request packet is 8 bytes for function 3 & 16
    if (modBusData->buffer > 7)
	  {
		  uint8_t id = modBusData->frame[0];
			// широковещательные комады
		  modBusData->broadcastFlag = 0;

		  if (id == 0)
		  	modBusData->broadcastFlag = 1;

      if (id == modBusData->slaveID || modBusData->broadcastFlag) // if the recieved ID matches the slaveID or broadcasting id (0), continue
      {
			  // сrc контролня сумма пакета
        uint16_t crc = ((modBusData->frame[modBusData->buffer - 2] << 8) | modBusData->frame[modBusData->buffer - 1]); // combine the crc Low & High bytes
        if (calculateCRC(modBusData, modBusData->buffer - 2) == crc) // if the calculated crc matches the recieved crc continue
        {
        	modBusData->function = modBusData->frame[1];
				  uint16_t startingAddress = ((modBusData->frame[2] << 8) | modBusData->frame[3]); // combine the starting address bytes
				  uint16_t no_of_registers = ((modBusData->frame[4] << 8) | modBusData->frame[5]); // combine the number of register bytes
				  uint16_t maxData = startingAddress + no_of_registers;
				  uint8_t index;
				  uint8_t address;
				  uint16_t crc16;

				  // broadcasting is not supported for function 3
				  if (!modBusData->broadcastFlag && (modBusData->function == 3))
				  {
					  if (startingAddress < modBusData->holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
					  {
						  if (maxData <= modBusData->holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
						  {
							  uint8_t noOfBytes = no_of_registers * 2;
                // ID, function, noOfBytes, (dataLo + dataHi)*number of registers,
                //  crcLo, crcHi
							  uint8_t responseFrameSize = 5 + noOfBytes;
							  modBusData->frame[0] = modBusData->slaveID;
							  modBusData->frame[1] = modBusData->function;
							  modBusData->frame[2] = noOfBytes;
							  address = 3; // PDU starts at the 4th byte
							  uint16_t temp;

							  for (index = startingAddress; index < maxData; index++)
						  	{
								  temp = modBusData->regs[index];
								  modBusData->frame[address] = temp >> 8; // split the register into 2 bytes
								  address++;
								  modBusData->frame[address] = temp & 0xFF;
								  address++;
							  }

							  crc16 = calculateCRC(modBusData, responseFrameSize - 2);
							  modBusData->frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
							  modBusData->frame[responseFrameSize - 1] = crc16 & 0xFF;

							  // Serial.println();
								// for (int i = 0; i < responseFrameSize; i++)
								// {
									// Serial.print("Frame2[");
									// Serial.print(i);
									// Serial.print("] = ");
									// Serial.println(frame[i]);
								// }

							  sendPacket(modBusData, responseFrameSize);
						  }
						  else
							  exceptionResponse(modBusData, 3); // exception 3 ILLEGAL DATA VALUE
					  }
					  else
						  exceptionResponse(modBusData, 2); // exception 2 ILLEGAL DATA ADDRESS
				  }
				  else if (modBusData->function == 16)
				  {
					  // Check if the recieved number of bytes matches the calculated bytes
            // minus the request bytes.
					  // id + function + (2 * address bytes) + (2 * no of register bytes) +
            // byte count + (2 * CRC bytes) = 9 bytes
					  if (modBusData->frame[6] == (modBusData->buffer - 9))
					  {
						  if (startingAddress < modBusData->holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						  {
							  if (maxData <= modBusData->holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
							  {
								  address = 7; // start at the 8th byte in the frame

								  for (index = startingAddress; index < maxData; index++)
							  	{
								  	modBusData->regs[index] = ((modBusData->frame[address] << 8) | modBusData->frame[address + 1]);
									  address += 2;
								  }

								  // only the first 6 bytes are used for CRC calculation
								  crc16 = calculateCRC(modBusData, 6);
								  modBusData->frame[6] = crc16 >> 8; // split crc into 2 bytes
								  modBusData->frame[7] = crc16 & 0xFF;

								  // a function 16 response is an echo of the first 6 bytes from
                  // the request + 2 crc bytes
								  if (!modBusData->broadcastFlag) // don't respond if it's a broadcast message
									  sendPacket(modBusData, 8);
							  }
							  else
								  exceptionResponse(modBusData, 3); // exception 3 ILLEGAL DATA VALUE
						  }
						  else
							  exceptionResponse(modBusData, 2); // exception 2 ILLEGAL DATA ADDRESS
					  }
					  else
					  	modBusData->errorCount++; // corrupted packet
          }
				  else
					  exceptionResponse(modBusData, 1); // exception 1 ILLEGAL FUNCTION
        }
			  else // checksum failed
			  	modBusData->errorCount++;
      } // incorrect id
    }
	  else if (modBusData->buffer > 0 && modBusData->buffer < 8)
	  	modBusData->errorCount++; // corrupted packet
  }
	return modBusData->errorCount;
}

void exceptionResponse(ModBusTypeDef *modBusData, uint8_t exception)
{
  // each call to exceptionResponse() will increment the errorCount
	modBusData->errorCount++;
	if (!modBusData->broadcastFlag) // don't respond if its a broadcast message
	{
		modBusData->frame[0] = modBusData->slaveID;
		modBusData->frame[1] = (modBusData->function | 0x80); // set MSB bit high, informs the master of an exception
		modBusData->frame[2] = exception;
		uint16_t crc16 = calculateCRC(modBusData, 3); // ID, function|0x80, exception code
		modBusData->frame[3] = crc16 >> 8;
		modBusData->frame[4] = crc16 & 0xFF;
    // exception response is always 5 bytes
    // ID, function + 0x80, exception code, 2 bytes crc
		sendPacket(modBusData, 5);
	}
}

uint16_t calculateCRC(ModBusTypeDef *modBusData, uint8_t bufferSize)
{
  uint16_t temp, temp2, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < bufferSize; i++)
  {
    temp = temp ^ modBusData->frame[i];
    for (uint8_t j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

void sendPacket(ModBusTypeDef *modBusData, uint8_t bufferSize)
{
	// Serial.print("bufferSize = ");
	// Serial.println(bufferSize);
  //digitalWrite(TxEnablePin, HIGH);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);

//  modBusData->buffer = 0;
//  modBusData->bufferSize = bufferSize;
//  USART3->DR = (uint8_t*)modBusData->frame[modBusData->buffer];
////	for (int i = 0; i < bufferSize; i++)
////	 {
////		 Serial.print("Frameout = ");
////		 Serial.println(frame[i]);
////	 }
//	// delay(2000);
//  //for (uint8_t i = 0; i < bufferSize; i++)
//      //(*ModbusPort).write(frame[i]);
	HAL_UART_Transmit(modBusData->uart, (uint8_t*)modBusData->frame, bufferSize, 10);//bufferSize);
	fl_transmit_485 = 1;
	TIM4->ARR = modBusData->T1_5;
	TIM4->CNT = 0;
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->CR1 |= TIM_CR1_CEN;
//
//	//(*ModbusPort).flush();
//
//	// allow a frame delay to indicate end of transmission
//	delayMicroseconds(T3_5);
//
//	HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
  //digitalWrite(TxEnablePin, LOW);
}
