#ifndef SIMPLE_MODBUS_SLAVE_H
#define SIMPLE_MODBUS_SLAVE_H

// SimpleModbusSlaveV9

/*
 SimpleModbusSlave allows you to communicate
 to any slave using the Modbus RTU protocol.

 This implementation DOES NOT fully comply with the Modbus specifications.

 Specifically the frame time out have not been implemented according
 to Modbus standards. The code does however combine the check for
 inter character time out and frame time out by incorporating a maximum
 time out allowable when reading from the message stream.

 SimpleModbusSlave implements an unsigned int return value on a call to modbus_update().
 This value is the total error count since the slave started. It's useful for fault finding.

 This code is for a Modbus slave implementing functions 3 and 16
 function 3: Reads the binary contents of holding registers (4X references)
 function 16: Presets values into a sequence of holding registers (4X references)

 All the functions share the same register array.

 Note:
 The Arduino serial ring buffer changed in V1.05 it is now 64 bytes or 32 unsigned int registers.
 Most of the time you will connect the arduino to a master via serial
 using a MAX485 or similar.

 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 58 bytes or 29 registers.

 In a function 16 request the master will attempt to write to your
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS,
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 54 bytes or 27 registers.

 Using a USB to Serial converter the maximum bytes you can send is
 limited to its internal buffer which differs between manufactures.

 The functions included here have been derived from the
 Modbus Specifications and Implementation Guides

 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 http://www.modbus.org/docs/PI_MBUS_300.pdf

*/


#include "stm32f1xx_hal.h"
#include "stdint.h"


#define BUFFER_SIZE_485 64

typedef struct {
	uint8_t frame[BUFFER_SIZE_485];
	uint16_t holdingRegsSize; // size of the register array
	uint16_t* regs; // user array address
	uint8_t broadcastFlag;
	uint8_t slaveID;
	uint8_t function;
	uint8_t available;
	uint8_t buffer;
	uint8_t bufferSize;
	//GPIO_TypeDef* TxEnable_Port;
	//uint16_t TxEnablePin;
	uint16_t errorCount;
	uint16_t T1_5; // inter character time out
	uint16_t T3_5; // frame delay
	uint32_t baud;
	UART_HandleTypeDef *uart;
} ModBusTypeDef;

// function definitions

void modbus_configure(ModBusTypeDef *modBusData,
											UART_HandleTypeDef *_uart,
											uint8_t _slaveID,
											//GPIO_TypeDef *_TxEnable_Port,
											//uint16_t _TxEnablePin,
											uint16_t _holdingRegsSize,
											uint16_t*  _regs);
void modbus_update_comms(ModBusTypeDef *modBusData, uint32_t baud);
uint16_t modbus_update(ModBusTypeDef *modBusData);
// function definitions
void exceptionResponse(ModBusTypeDef *modBusData, uint8_t exception);
uint16_t calculateCRC(ModBusTypeDef *modBusData, uint8_t bufferSize);
void sendPacket(ModBusTypeDef *modBusData, uint8_t bufferSize);

#endif
