/* DHT library

MIT license
written by Adafruit Industries
*/

#include "DHT.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "../Inc/constants.h"

static volatile uint32_t sysTickCount = 0;

void pinMode(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, char fl)
{
    GPIO_InitTypeDef GPIO_InitStruct_DHT = {0};
    if (fl == 0) {
        /*Configure GPIO pin : DHT22_2_Pin */
        GPIO_InitStruct_DHT.Pin = DHT22_2_Pin;
        GPIO_InitStruct_DHT.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct_DHT.Pull = GPIO_NOPULL;
        GPIO_InitStruct_DHT.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(DHT22_2_GPIO_Port, &GPIO_InitStruct_DHT);
    } else {
        /*Configure GPIO pins :  */
        GPIO_InitStruct_DHT.Pin = DHT22_2_Pin;
        GPIO_InitStruct_DHT.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct_DHT.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(DHT22_2_GPIO_Port, &GPIO_InitStruct_DHT);
    }
}

void begin(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    // set up the pins!
    pinMode(GPIO_Port, GPIO_Pin, OUTPUT);
    HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
    _lastreadtime = 0;
}

float readTemperature(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, char S)
{
    float f;
    _type = DHT22; // надо б это переписать

    if (read(GPIO_Port, GPIO_Pin)) {
        switch (_type) {
            case DHT11:
                f = data[2];
                if (S)
                    f = convertCtoF(f);

                return f;
            case DHT22:
            case DHT21:
                f = data[2] & 0x7F;
                f *= 256;
                f += data[3];
                f /= 10;
                if (data[2] & 0x80)
                    f *= -1;
                if (S)
                    f = convertCtoF(f);

                return f;
        }
    }
    return NAN;
}

float convertCtoF(float c)
{
    return c * 9 / 5 + 32;
}

float readHumidity(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    float f;
    _type = DHT22;

    if (read(GPIO_Port, GPIO_Pin)) {
        switch (_type) {
            case DHT11:
                f = data[0];
                return f;
            case DHT22:
            case DHT21:
                f = data[0];
                f *= 256;
                f += data[1];
                f /= 10;
                return f;
        }
    }
    return NAN;
}


char read(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    _count = DHT22_BIT_THRESHOLD;
    unsigned long currenttime;

    // pull the pin high and wait DHT22_INIT_DELAY_MS milliseconds
    HAL_GPIO_WritePin(DHT22_2_GPIO_Port, DHT22_2_Pin, GPIO_PIN_SET);
    osDelay(DHT22_INIT_DELAY_MS);

    currenttime = dwt_timer::millis();
    if (currenttime < _lastreadtime) {
        // ie there was a rollover
        _lastreadtime = 0;
    }
    if (!firstreading && ((currenttime - _lastreadtime) < DHT22_READ_INTERVAL_MS)) {
    }
    firstreading = FALSE;
    /*
    Serial.print("Currtime: "); Serial.print(currenttime);
    Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
  */
    _lastreadtime = dwt_timer::millis();

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // now pull it low for ~DHT22_PULL_LOW_DELAY_MS milliseconds
    pinMode(GPIO_Port, GPIO_Pin, OUTPUT);
    HAL_GPIO_WritePin(DHT22_2_GPIO_Port, DHT22_2_Pin, GPIO_PIN_RESET);
    osDelay(DHT22_PULL_LOW_DELAY_MS);
    //cli();
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(DHT22_2_GPIO_Port, DHT22_2_Pin, GPIO_PIN_SET);
    dwt_timer::delay_us(DHT22_PULL_HIGH_DELAY_US);
    pinMode(GPIO_Port, GPIO_Pin, INPUT);
    dwt_timer::delay_us(DHT22_INPUT_SETUP_US);
    // read in timings
    for (i = 0; i < MAXTIMINGS; i++) {
        counter = 0;
        while (HAL_GPIO_ReadPin(DHT22_2_GPIO_Port, DHT22_2_Pin) == laststate) {
            counter++;
            dwt_timer::delay_us(1);
            if (counter == DHT22_TIMEOUT_COUNT) {
                break;
            }
        }
        laststate = HAL_GPIO_ReadPin(DHT22_2_GPIO_Port, DHT22_2_Pin);

        if (counter == DHT22_TIMEOUT_COUNT)
            break;

        // ignore first 3 transitions
        if ((i >= 4) && (i % 2 == 0)) {
            // shove each bit into the storage bytes
            data[j / 8] <<= 1;
            if (counter > _count)
                data[j / 8] |= 1;
            j++;
        }
    }
    taskEXIT_CRITICAL();
    //sei();

    /*
  Serial.println(j, DEC);
  Serial.print(data[0], HEX); Serial.print(", ");
  Serial.print(data[1], HEX); Serial.print(", ");
  Serial.print(data[2], HEX); Serial.print(", ");
  Serial.print(data[3], HEX); Serial.print(", ");
  Serial.print(data[4], HEX); Serial.print(" =? ");
  Serial.println(data[0] + data[1] + data[2] + data[3], HEX);
  */

    // check we read DHT22_BIT_COUNT bits and that the checksum matches
    if ((j >= DHT22_BIT_COUNT) &&
        (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
        return TRUE;
    }


    return FALSE;
}
