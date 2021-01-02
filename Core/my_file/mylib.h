/*
 * mylib.h
 *
 *  Created on: 5 мая 2020 г.
 *      Author: Rinat
 */

#ifndef INC_MYLIB_H_
#define INC_MYLIB_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define ADC_AVG_QTY_TR 10
#define ADC_AVG_QTY_PM 3

#define ERROR_NULL 															0x00000000
#define ERROR_PRESS_MET_OUT_OF_VOLT							0x00000001
#define ERROR_PRESS_MET_OUT_OF_BAR_PER_SECOND		0x00000002
#define ERROR_WIRE_BREAK												0x00000004


typedef struct {
	uint8_t quantity;
	float *arAdc;
	uint8_t count;
} AdcAverage;

typedef struct {
	GPIO_TypeDef *gpio_port;
	uint16_t pin;
	uint8_t line;
	uint8_t fl_on_off;
} St_Rel_manage;

typedef struct {
	float currentVolt;
	float previousPress;
	float dVolt;											//дельта погрешности по напряжению
	float barPerSecond;
	float minPressPoint; 							//p
	float maxPressPoint; 							//p
	float minVolt; 										//p
	float maxVolt;										//p
	float minBarPerSecond;						//p
	uint32_t timePreviousPress;
	uint32_t lagMinBerPerSecondError;	//p
	uint32_t timeOnPomp;
	GPIO_TypeDef* GPIO_Port;
	uint16_t GPIO_Pin;
	char pomp_on;
	uint32_t error;
} StManagePressHeatingSys;

enum {
	en_tr1,
	en_tr2,
	en_tr4,
	en_tr3,
	enTrChanEnd
} enChannelsTr;

enum {
	en_pm1,		// датчик давления в системе отопления
	en_pm2,
	enPmChanEnd
} enChannelsPm;

uint32_t ADC_Avg_result(AdcAverage *adcavg);
void ADC_Avg_init(AdcAverage *adcavg, uint8_t qty, uint8_t qty_of_el);
void fltochar(char* tmpl, float fltdata);
uint32_t ADC_Result(ADC_HandleTypeDef *hadc, uint32_t ch);
void inttochar(char* tmpl, uint32_t intdata);
void ADC_Avg_add(AdcAverage *adcavg, uint32_t add_var, uint8_t qty);

void initManagePressHeatingSys(StManagePressHeatingSys *st, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
void workManagePressHeatingSys(StManagePressHeatingSys *st, uint32_t adc_volt, uint32_t time);
void measManagePressHeatingSys(StManagePressHeatingSys *st, uint32_t adc_volt, uint32_t time);


#endif /* INC_MYLIB_H_ */
