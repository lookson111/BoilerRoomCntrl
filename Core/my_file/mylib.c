/*
 * mylib.c
 *
 *  Created on: 5 мая 2020 г.
 *      Author: Rinat
 */
#include "mylib.h"

void fltochar(char* tmpl, float fltdata) {
	float fltData = fltdata;
	int intData;
	int medData;
	int k;
	int bool = 0;
	//char chData[6];
	for(int i = 0; i < 6; i++) {
		tmpl[i] = ' ';
	}
	tmpl[6] = 0x00;

	if ((fltData > 100) || (fltData < -50)) {
		return;
	}
	if ((fltData < 100) && (fltData > 1)) {
		intData = 0;
	}
	intData = fltData * 100;
	tmpl[3] = '.';
	for(int i = 4; i >= 0; i--) {
		medData = intData % 10;
		intData = intData / 10;
		if (bool)
			break;
		if (intData == 0)
			bool = 1;
		if (i > 2) {
			k = i + 1;
		} else {
			k = i;
		}
		switch(medData) {
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

void inttochar(char* tmpl, uint32_t intdata) {
	uint32_t inData = intdata;
	//int intData;
	int medData;
	int k;
	int bool = 0;
	//char chData[6];
	for(int i = 0; i < 6; i++) {
		tmpl[i] = ' ';
	}
	tmpl[6] = 0x00;

//	if ((inData > 10000) || (inData < 0)) {
//		return;
//	}
//	if ((inData < 100) && (inData >= 0)) {
//		inData = 0;
//	}
	for(int i = 5; i >= 0; i--) {
		medData = inData % 10;
		inData = inData / 10;
		if (bool)
			break;
		if (inData == 0)
			bool = 1;
		k = i;
		switch(medData) {
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

uint32_t ADC_Result(ADC_HandleTypeDef *hadc, uint32_t ch){
       ADC_ChannelConfTypeDef sConfig;
       uint32_t adcResult = 0;
       int count = 1000;

       sConfig.Channel = ch;
       sConfig.Rank = ADC_REGULAR_RANK_1;
       sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;// ADC_SAMPLETIME_13CYCLES_5;
       HAL_ADC_ConfigChannel(hadc, &sConfig);

       for(int i = 0; i < count; i ++) {
      	 HAL_ADC_Start(hadc);
         HAL_ADC_PollForConversion(hadc, 100);
         adcResult += HAL_ADC_GetValue(hadc);
       }
       adcResult = adcResult / count;
       HAL_ADC_Stop(hadc);
       return adcResult;
}


uint32_t ADC_Avg_result(AdcAverage *adcavg) {
	uint32_t avg = 0;
	for (uint8_t i = 0; i < adcavg->quantity; i++) {
		avg += adcavg->arAdc[i];
	}
	return avg/adcavg->quantity;
}

void ADC_Avg_init(AdcAverage *adcavg, uint8_t qty, uint8_t qty_of_el) {
	for (uint8_t i = 0; i < qty; i++) {
		adcavg[i].arAdc = pvPortMalloc(qty_of_el * 4);
	}
}

void ADC_Avg_add(AdcAverage *adcavg, uint32_t add_var, uint8_t qty) {
	adcavg->arAdc[adcavg->count] = add_var;
	if (adcavg->quantity < (qty-1)){
		adcavg->quantity +=1;
	}
	if (adcavg->count < adcavg->quantity) {
		adcavg->count += 1;
	} else {
		adcavg->count = 0;
	}

}

void initManagePressHeatingSys(StManagePressHeatingSys *st, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin) {
	st->GPIO_Port = GPIO_Port;
	st->GPIO_Pin = GPIO_Pin;
	st->previousPress = 0;
	st->timePreviousPress = 0;
	st->minBarPerSecond = 0.05;
	st->lagMinBerPerSecondError = 72000000; // одна секунда
	st->maxPressPoint = 2;
	st->minPressPoint = 1;
	st->maxVolt = 4.5;
	st->minVolt = 0.5;
	st->dVolt = 0.05;
	st->error = 0;
	st->pomp_on = 0;
}

void workManagePressHeatingSys(StManagePressHeatingSys *st, uint32_t adc_volt, uint32_t time) {
	measManagePressHeatingSys(st, adc_volt, time);
	if (st->error) {
		st->pomp_on = 0;
		HAL_GPIO_WritePin(st->GPIO_Port, st->GPIO_Pin, RESET);
		return;
	}
	if ((st->previousPress < st->minPressPoint) & !st->pomp_on ) {
		st->pomp_on = 1;
		st->timeOnPomp = time;
		HAL_GPIO_WritePin(st->GPIO_Port, st->GPIO_Pin, SET);
	}
	if ((st->previousPress > st->maxPressPoint) & st->pomp_on ) {
		st->pomp_on = 0;
		HAL_GPIO_WritePin(st->GPIO_Port, st->GPIO_Pin, RESET);
	}
}

void measManagePressHeatingSys(StManagePressHeatingSys *st, uint32_t adc_volt, uint32_t time) {
	float Rt = adc_volt;
	float Ut;
	float pmavg;
	uint32_t dt;
	Ut = Rt/4095*3.3*1.4751;
	if  (Ut > (st->maxVolt + st->dVolt)) {
		st->error |= ERROR_PRESS_MET_OUT_OF_VOLT;
		return;
	}
	if (Ut < (st->minVolt - st->dVolt)) {
		st->error |= ERROR_WIRE_BREAK;
		return;
	}
	st->error &= (~ERROR_PRESS_MET_OUT_OF_VOLT & ~ERROR_WIRE_BREAK);

	pmavg = (Ut*50-25) / 14.5038;
	if (st->pomp_on) {
		if ((time < st->timePreviousPress)) {
			dt = (0xFFFFFFFF - st->timePreviousPress) + time;
		}
		st->barPerSecond = (pmavg - st->previousPress) / (dt / 72000000.0);
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


























