#include "SimpleModbusSlave.h"
#include "cmsis_os.h"
#include "main.h"
#include "../Inc/constants.h"

//UART_HandleTypeDef huart3;


void modbus_configure(ModBusTypeDef* modBusData, UART_HandleTypeDef* _uart,
                      uint8_t _slaveID, uint16_t _holdingRegsSize,
                      uint16_t* _regs)
{
    modBusData->uart = _uart;
    modbus_update_comms(modBusData, _uart->Init.BaudRate);
    modBusData->holdingRegsSize = _holdingRegsSize;
    modBusData->regs = _regs;
    modBusData->slaveID = _slaveID;
    modBusData->errorCount = 0;
    modBusData->available = 0;
}

void modbus_update_comms(ModBusTypeDef* modBusData, uint32_t baud)
{
    //(*ModbusPort).begin(baud, byteFormat);


    // Modbus states that a baud rate higher than MODBUS_BAUD_THRESHOLD must use a fixed MODBUS_T15_HIGH_BAUD us
    // for inter character time out and MODBUS_T35_HIGH_BAUD for a frame delay for baud rates
    // below MODBUS_BAUD_THRESHOLD the timing is more critical and has to be calculated.
    // E.g. 9600 baud in a 10 bit packet is 960 characters per second
    // In milliseconds this will be 960characters per 1000ms. So for 1 character
    // 1000ms/960characters is 1.04167ms per character and finally modbus states
    // an inter-character must be 1.5T or 1.5 times longer than a character. Thus
    // 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.

    if (baud > MODBUS_BAUD_THRESHOLD) {
        modBusData->T1_5 = MODBUS_T15_HIGH_BAUD;
        modBusData->T3_5 = MODBUS_T35_HIGH_BAUD;
    } else {
        modBusData->T1_5 = MODBUS_T15_MULTIPLIER / baud; // 1T * 1.5 = T1.5
        modBusData->T3_5 = MODBUS_T35_MULTIPLIER / baud; // 1T * 3.5 = T3.5
    }
}

uint16_t modbus_update(ModBusTypeDef* modBusData)
{
    if (modBusData->available) {
        modBusData->available = 0;

        uint8_t overflow = 0;

        if (overflow)
            return modBusData->errorCount++;

        // The minimum request packet is MODBUS_MIN_REQUEST_LEN bytes for function 3 & 16
        if (modBusData->buffer > MODBUS_MIN_REQUEST_LEN - 1) {
            uint8_t id = modBusData->frame[0];
            // широковещательные комады
            modBusData->broadcastFlag = 0;

            if (id == 0)
                modBusData->broadcastFlag = 1;

            if (id == modBusData->slaveID ||
                modBusData
                    ->broadcastFlag) // if the recieved ID matches the slaveID or broadcasting id (0), continue
            {
                // сrc контролня сумма пакета
                uint16_t crc =
                    ((modBusData->frame[modBusData->buffer - 2] << 8) |
                     modBusData->frame[modBusData->buffer -
                                       1]); // combine the crc Low & High bytes
                if (calculateCRC(modBusData, modBusData->buffer - 2) ==
                    crc) // if the calculated crc matches the recieved crc continue
                {
                    modBusData->function = modBusData->frame[1];
                    uint16_t startingAddress =
                        ((modBusData->frame[2] << 8) |
                         modBusData
                             ->frame[3]); // combine the starting address bytes
                    uint16_t no_of_registers =
                        ((modBusData->frame[4] << 8) |
                         modBusData->frame
                             [5]); // combine the number of register bytes
                    uint16_t maxData = startingAddress + no_of_registers;
                    uint8_t index;
                    uint8_t address;
                    uint16_t crc16;

                    // broadcasting is not supported for function 3
                    if (!modBusData->broadcastFlag &&
                        (modBusData->function == 3)) {
                        if (startingAddress <
                            modBusData
                                ->holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
                        {
                            if (maxData <=
                                modBusData
                                    ->holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
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

                                for (index = startingAddress; index < maxData;
                                     index++) {
                                    temp = modBusData->regs[index];
                                    modBusData->frame[address] =
                                        temp >>
                                        8; // split the register into 2 bytes
                                    address++;
                                    modBusData->frame[address] = temp & MODBUS_CRC_MASK;
                                    address++;
                                }

                                crc16 = calculateCRC(modBusData,
                                                     responseFrameSize - 2);
                                modBusData->frame[responseFrameSize - 2] =
                                    crc16 >> 8;
                                modBusData->frame[responseFrameSize - 1] =
                                    crc16 & MODBUS_CRC_MASK;

                                sendPacket(modBusData, responseFrameSize);
                            } else
                                exceptionResponse(
                                    modBusData,
                                    3); // exception 3 ILLEGAL DATA VALUE
                        } else
                            exceptionResponse(
                                modBusData,
                                2); // exception 2 ILLEGAL DATA ADDRESS
                    } else if (modBusData->function == 16) {
                        // Check if the recieved number of bytes matches the calculated bytes
                        // minus the request bytes.
                        // id + function + (2 * address bytes) + (2 * no of register bytes) +
                        // byte count + (2 * CRC bytes) = MODBUS_MIN_REQUEST_LEN + 1 bytes
                        if (modBusData->frame[6] == (modBusData->buffer - (MODBUS_MIN_REQUEST_LEN + 1))) {
                            if (startingAddress <
                                modBusData
                                    ->holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
                            {
                                if (maxData <=
                                    modBusData
                                        ->holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
                                {
                                    address =
                                        MODBUS_MIN_REQUEST_LEN - 1; // start at the MODBUS_MIN_REQUEST_LENth byte in the frame

                                    for (index = startingAddress;
                                         index < maxData; index++) {
                                        modBusData->regs[index] =
                                            ((modBusData->frame[address] << 8) |
                                             modBusData->frame[address + 1]);
                                        address += 2;
                                    }

                                    // only the first MODBUS_MIN_REQUEST_LEN - 2 bytes are used for CRC calculation
                                    crc16 = calculateCRC(modBusData, MODBUS_MIN_REQUEST_LEN - 2);
                                    modBusData->frame[6] =
                                        crc16 >> 8; // split crc into 2 bytes
                                    modBusData->frame[7] = crc16 & MODBUS_CRC_MASK;

                                    // a function 16 response is an echo of the first MODBUS_MIN_REQUEST_LEN - 2 bytes from
                                    // the request + 2 crc bytes
                                    if (!modBusData
                                             ->broadcastFlag) // don't respond if it's a broadcast message
                                        sendPacket(modBusData, MODBUS_MIN_REQUEST_LEN);
                                } else
                                    exceptionResponse(
                                        modBusData,
                                        3); // exception 3 ILLEGAL DATA VALUE
                            } else
                                exceptionResponse(
                                    modBusData,
                                    2); // exception 2 ILLEGAL DATA ADDRESS
                        } else
                            modBusData->errorCount++; // corrupted packet
                    } else
                        exceptionResponse(modBusData,
                                          1); // exception 1 ILLEGAL FUNCTION
                } else                        // checksum failed
                    modBusData->errorCount++;
            } // incorrect id
        } else if (modBusData->buffer > 0 && modBusData->buffer < MODBUS_MIN_REQUEST_LEN)
            modBusData->errorCount++; // corrupted packet
    }
    return modBusData->errorCount;
}

void exceptionResponse(ModBusTypeDef* modBusData, uint8_t exception)
{
    // each call to exceptionResponse() will increment the errorCount
    modBusData->errorCount++;
    if (!modBusData->broadcastFlag) // don't respond if its a broadcast message
    {
        modBusData->frame[0] = modBusData->slaveID;
        modBusData->frame[1] =
            (modBusData->function |
             MODBUS_EXCEPT_BIT); // set MSB bit high, informs the master of an exception
        modBusData->frame[2] = exception;
        uint16_t crc16 =
            calculateCRC(modBusData, MODBUS_EXCEPT_RESPONSE_SIZE); // ID, function|0x80, exception code
        modBusData->frame[3] = crc16 >> 8;
        modBusData->frame[4] = crc16 & MODBUS_CRC_MASK;
        // exception response is always MODBUS_EXCEPT_RESPONSE_SIZE bytes
        // ID, function + 0x80, exception code, 2 bytes crc
        sendPacket(modBusData, MODBUS_EXCEPT_RESPONSE_SIZE);
    }
}

uint16_t calculateCRC(ModBusTypeDef* modBusData, uint8_t bufferSize)
{
    uint16_t temp, temp2, flag;
    temp = MODBUS_CRC_INIT;
    for (uint8_t i = 0; i < bufferSize; i++) {
        temp = temp ^ modBusData->frame[i];
        for (uint8_t j = 1; j <= 8; j++) {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= MODBUS_CRC_POLY;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= MODBUS_CRC_MASK;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}

void sendPacket(ModBusTypeDef* modBusData, uint8_t bufferSize)
{
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);

    HAL_UART_Transmit(modBusData->uart, (uint8_t*)modBusData->frame, bufferSize,
                      UART_TRANSMIT_TIMEOUT);
    fl_transmit_485 = 1;
    TIM4->ARR = modBusData->T1_5;
    TIM4->CNT = 0;
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
}
