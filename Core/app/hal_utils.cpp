/*
 * hal_utils.c
 *
 *  Created on: 5 мая 2020 г.
 *      Author: Rinat
 */
#include "hal_utils.h"
#include "constants.h"

void fltochar(char* tmpl, float fltdata)
{
    float fltData = fltdata;
    int intData;
    int medData;
    int k;
    int done = 0;
    for (int i = 0; i < STR_BUF_SIZE_6; i++) {
        tmpl[i] = ' ';
    }
    tmpl[STR_BUF_SIZE_6] = STR_NULL_TERMINATOR;

    if ((fltData > FLOAT_TEMP_MAX) || (fltData < FLOAT_TEMP_MIN)) {
        return;
    }
    if ((fltData < FLOAT_TEMP_MAX) && (fltData > 1)) {
        intData = 0;
    }
    intData = fltData * FLOAT_SCALE_FACTOR;
    tmpl[3] = '.';
    for (int i = 4; i >= 0; i--) {
        medData = intData % 10;
        intData = intData / 10;
        if (done)
            break;
        if (intData == 0)
            done = 1;
        if (i > 2) {
            k = i + 1;
        } else {
            k = i;
        }
        switch (medData) {
            case 0:
                tmpl[k] = '0';
                continue;
            case 1:
                tmpl[k] = '1';
                continue;
            case 2:
                tmpl[k] = '2';
                continue;
            case 3:
                tmpl[k] = '3';
                continue;
            case 4:
                tmpl[k] = '4';
                continue;
            case 5:
                tmpl[k] = '5';
                continue;
            case 6:
                tmpl[k] = '6';
                continue;
            case 7:
                tmpl[k] = '7';
                continue;
            case 8:
                tmpl[k] = '8';
                continue;
            case 9:
                tmpl[k] = '9';
                continue;
        }
    }
}

void inttochar(char* tmpl, uint32_t intdata)
{
    uint32_t inData = intdata;
    int medData;
    int k;
    int done = 0;
    for (int i = 0; i < STR_BUF_SIZE_6; i++) {
        tmpl[i] = ' ';
    }
    tmpl[STR_BUF_SIZE_6] = STR_NULL_TERMINATOR;
    for (int i = 5; i >= 0; i--) {
        medData = inData % 10;
        inData = inData / 10;
        if (done)
            break;
        if (inData == 0)
            done = 1;
        k = i;
        switch (medData) {
            case 0:
                tmpl[k] = '0';
                continue;
            case 1:
                tmpl[k] = '1';
                continue;
            case 2:
                tmpl[k] = '2';
                continue;
            case 3:
                tmpl[k] = '3';
                continue;
            case 4:
                tmpl[k] = '4';
                continue;
            case 5:
                tmpl[k] = '5';
                continue;
            case 6:
                tmpl[k] = '6';
                continue;
            case 7:
                tmpl[k] = '7';
                continue;
            case 8:
                tmpl[k] = '8';
                continue;
            case 9:
                tmpl[k] = '9';
                continue;
        }
    }
}

uint32_t ADC_Result(ADC_HandleTypeDef* hadc, uint32_t ch)
{
    ADC_ChannelConfTypeDef sConfig;
    uint32_t adcResult = 0;
    int count = ADC_SAMPLE_COUNT_TR;

    sConfig.Channel = ch;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    for (int i = 0; i < count; i++) {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, ADC_TIMEOUT_MS);
        adcResult += HAL_ADC_GetValue(hadc);
    }
    adcResult = adcResult / count;
    HAL_ADC_Stop(hadc);
    return adcResult;
}


uint32_t ADC_Avg_result(AdcAverage* adcavg)
{
    uint32_t avg = 0;
    for (uint8_t i = 0; i < adcavg->quantity; i++) {
        avg += adcavg->arAdc[i];
    }
    return avg / adcavg->quantity;
}

void ADC_Avg_init(AdcAverage* adcavg, uint8_t qty, uint8_t qty_of_el)
{
    for (uint8_t i = 0; i < qty; i++) {
        adcavg[i].arAdc = static_cast<float*>(pvPortMalloc(qty_of_el * 4));
    }
}

void ADC_Avg_add(AdcAverage* adcavg, uint32_t add_var, uint8_t qty)
{
    adcavg->arAdc[adcavg->count] = add_var;
    if (adcavg->quantity < (qty - 1)) {
        adcavg->quantity += 1;
    }
    if (adcavg->count < adcavg->quantity) {
        adcavg->count += 1;
    } else {
        adcavg->count = 0;
    }
}

// ManagePressHeatingSys constructor
ManagePressHeatingSys::ManagePressHeatingSys()
    : currentVolt(0), previousPress(0), dVolt(0), barPerSecond(0),
      minPressPoint(1.0f), maxPressPoint(2.0f), minVolt(0.5f), maxVolt(4.5f),
      minBarPerSecond(0.05f), timePreviousPress(0),
      lagMinBerPerSecondError(1000), timeOnPomp(0), GPIO_Port(nullptr),
      GPIO_Pin(0), pomp_on(0), error(0)
{
}

void ManagePressHeatingSys::init(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin) {
    this->GPIO_Port = GPIO_Port;
    this->GPIO_Pin = GPIO_Pin;
    this->previousPress = 0;
    this->timePreviousPress = 0;
    this->minBarPerSecond = Pressure::MIN_BAR_PER_SEC;
    this->lagMinBerPerSecondError = Pressure::ERROR_LAG_MS;
    this->maxPressPoint = Pressure::MAX_POINT;
    this->minPressPoint = Pressure::MIN_POINT;
    this->maxVolt = Pressure::VOLT_MAX;
    this->minVolt = Pressure::VOLT_MIN;
    this->dVolt = Pressure::VOLT_DELTA;
    this->error = 0;
    this->pomp_on = 0;
}

void ManagePressHeatingSys::work(uint32_t adc_volt, uint32_t time) {
    this->meas(adc_volt, time);
    if (this->error) {
        this->pomp_on = 0;
        HAL_GPIO_WritePin(this->GPIO_Port, this->GPIO_Pin, GPIO_PIN_RESET);
        return;
    }
    if ((this->previousPress < this->minPressPoint) & !this->pomp_on) {
        this->pomp_on = 1;
        this->timeOnPomp = time;
        HAL_GPIO_WritePin(this->GPIO_Port, this->GPIO_Pin, GPIO_PIN_SET);
    }
    if ((this->previousPress > this->maxPressPoint) & this->pomp_on) {
        this->pomp_on = 0;
        HAL_GPIO_WritePin(this->GPIO_Port, this->GPIO_Pin, GPIO_PIN_RESET);
    }
}

void ManagePressHeatingSys::meas(uint32_t adc_volt, uint32_t time) {
    float Rt = adc_volt;
    float Ut;
    float pmavg;
    uint32_t dt;
    Ut = Rt / VoltDiv::FACTOR_4095 * Pressure::REF_VOLT * Pressure::SCALE;
    if (Ut > (this->maxVolt + this->dVolt)) {
        this->error |= Error::PRESS_MET_OUT_OF_VOLT;
        return;
    }
    if (Ut < (this->minVolt - this->dVolt)) {
        this->error |= Error::WIRE_BREAK;
        return;
    }
    this->error &= (~Error::PRESS_MET_OUT_OF_VOLT & ~Error::WIRE_BREAK);

    pmavg = (Ut * Pressure::MULT - Pressure::OFFSET) / Pressure::DIV;
    if (this->pomp_on) {
        if ((time < this->timePreviousPress)) {
            dt = (0xFFFFFFFF - this->timePreviousPress) + time;
        }
        this->barPerSecond = (pmavg - this->previousPress) / (dt / (float)RTOS::TICK_RATE_HZ);
        if (this->barPerSecond < this->minBarPerSecond) {
            if (time > (this->timeOnPomp + this->lagMinBerPerSecondError)) {
                this->error |= Error::PRESS_MET_OUT_OF_BAR;
                return;
            }
        }
    }
    this->previousPress = pmavg;
    this->timePreviousPress = time;
}

void initManagePressHeatingSys(StManagePressHeatingSys* st,
                               GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    st->GPIO_Port = GPIO_Port;
    st->GPIO_Pin = GPIO_Pin;
    st->previousPress = 0;
    st->timePreviousPress = 0;
    st->minBarPerSecond = PRESSURE_MIN_BAR_PER_SEC;
    st->lagMinBerPerSecondError = PRESSURE_ERROR_LAG_MS; // one second (in ms)
    st->maxPressPoint = PRESSURE_MAX_POINT;
    st->minPressPoint = PRESSURE_MIN_POINT;
    st->maxVolt = PRESSURE_VOLT_MAX;
    st->minVolt = PRESSURE_VOLT_MIN;
    st->dVolt = PRESSURE_VOLT_DELTA;
    st->error = 0;
    st->pomp_on = 0;
}

void workManagePressHeatingSys(StManagePressHeatingSys* st, uint32_t adc_volt,
                               uint32_t time)
{
    measManagePressHeatingSys(st, adc_volt, time);
    if (st->error) {
        st->pomp_on = 0;
        HAL_GPIO_WritePin(st->GPIO_Port, st->GPIO_Pin, GPIO_PIN_RESET);
        return;
    }
    if ((st->previousPress < st->minPressPoint) & !st->pomp_on) {
        st->pomp_on = 1;
        st->timeOnPomp = time;
        HAL_GPIO_WritePin(st->GPIO_Port, st->GPIO_Pin, GPIO_PIN_SET);
    }
    if ((st->previousPress > st->maxPressPoint) & st->pomp_on) {
        st->pomp_on = 0;
        HAL_GPIO_WritePin(st->GPIO_Port, st->GPIO_Pin, GPIO_PIN_RESET);
    }
}

void measManagePressHeatingSys(StManagePressHeatingSys* st, uint32_t adc_volt,
                               uint32_t time)
{
    float Rt = adc_volt;
    float Ut;
    float pmavg;
    uint32_t dt;
    Ut = Rt / VOLT_DIV_4095_DIV * PRESSURE_REF_VOLT * PRESSURE_SCALE;
    if (Ut > (st->maxVolt + st->dVolt)) {
        st->error |= ERROR_PRESS_MET_OUT_OF_VOLT;
        return;
    }
    if (Ut < (st->minVolt - st->dVolt)) {
        st->error |= ERROR_WIRE_BREAK;
        return;
    }
    st->error &= (~ERROR_PRESS_MET_OUT_OF_VOLT & ~ERROR_WIRE_BREAK);

    pmavg = (Ut * PRESSURE_MULT - PRESSURE_OFFSET) / PRESSURE_DIV;
    if (st->pomp_on) {
        if ((time < st->timePreviousPress)) {
            dt = (0xFFFFFFFF - st->timePreviousPress) + time;
        }
        st->barPerSecond = (pmavg - st->previousPress) / (dt / (float)FREERTOS_TICK_RATE_HZ);
        if (st->barPerSecond < st->minBarPerSecond) {
            if (time > (st->timeOnPomp + st->lagMinBerPerSecondError)) {
                st->error |= ERROR_PRESS_MET_OUT_OF_BAR_PER_SECOND;
                return;
            }
        }
    }
    st->previousPress = pmavg;
    st->timePreviousPress = time;
}
