/*
 * hal_utils.h
 *
 *  Created on: 5 мая 2020 г.
 *      Author: Rinat
 */

#ifndef INC_HAL_UTILS_H_
#define INC_HAL_UTILS_H_

#include <stdint.h>
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

class AdcAverage {
public:
    AdcAverage();
    ~AdcAverage();
    
    void init(uint8_t qty, uint8_t qty_of_el);
    uint32_t result() const;
    void add(uint32_t add_var, uint8_t qty);
    
private:
    uint8_t quantity;
    float* arAdc;
    uint8_t count;
    
    friend uint32_t ADC_Avg_result(AdcAverage* adcavg);
    friend void ADC_Avg_init(AdcAverage* adcavg, uint8_t qty, uint8_t qty_of_el);
    friend void ADC_Avg_add(AdcAverage* adcavg, uint32_t add_var, uint8_t qty);
};

struct StRelManage {
    GPIO_TypeDef* gpio_port;
    uint16_t pin;
    uint8_t line;
    uint8_t fl_on_off;
};

// Legacy alias for backward compatibility
using St_Rel_manage = StRelManage;

class ManagePressHeatingSys {
public:
    ManagePressHeatingSys();

    void init(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
    void work(uint32_t adc_volt, uint32_t time);
    void meas(uint32_t adc_volt, uint32_t time);

    // Getters
    float getCurrentVolt() const { return currentVolt; }
    float getPreviousPress() const { return previousPress; }
    float getBarPerSecond() const { return barPerSecond; }
    bool isPompOn() const { return pomp_on != 0; }
    uint32_t getError() const { return error; }

private:
    float currentVolt;
    float previousPress;
    float dVolt; //дельта погрешности по напряжению
    float barPerSecond;
    float minPressPoint;   //p
    float maxPressPoint;   //p
    float minVolt;         //p
    float maxVolt;         //p
    float minBarPerSecond; //p
    uint32_t timePreviousPress;
    uint32_t lagMinBerPerSecondError; //p
    uint32_t timeOnPomp;
    GPIO_TypeDef* GPIO_Port;
    uint16_t GPIO_Pin;
    char pomp_on;
    uint32_t error;

    friend void initManagePressHeatingSys(ManagePressHeatingSys* st, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
    friend void workManagePressHeatingSys(ManagePressHeatingSys* st, uint32_t adc_volt, uint32_t time);
    friend void measManagePressHeatingSys(ManagePressHeatingSys* st, uint32_t adc_volt, uint32_t time);
};

// Legacy alias for backward compatibility
using StManagePressHeatingSys = ManagePressHeatingSys;

// Legacy C-style API for backward compatibility
enum { en_tr1, en_tr2, en_tr4, en_tr3, enTrChanEnd };
extern int enChannelsTr;

enum {
    en_pm1, // датчик давления в системе отопления
    en_pm2,
    enPmChanEnd
};
extern int enChannelsPm;

// C-style functions (call class methods internally)
uint32_t ADC_Avg_result(const AdcAverage* adcavg);
void ADC_Avg_init(AdcAverage* adcavg, uint8_t qty, uint8_t qty_of_el);
void fltochar(char* tmpl, float fltdata);
uint32_t ADC_Result(ADC_HandleTypeDef* hadc, uint32_t ch);
void inttochar(char* tmpl, uint32_t intdata);
void ADC_Avg_add(AdcAverage* adcavg, uint32_t add_var, uint8_t qty);

void initManagePressHeatingSys(ManagePressHeatingSys* st, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
void workManagePressHeatingSys(ManagePressHeatingSys* st, uint32_t adc_volt, uint32_t time);
void measManagePressHeatingSys(ManagePressHeatingSys* st, uint32_t adc_volt, uint32_t time);


#endif /* INC_HAL_UTILS_H_ */
