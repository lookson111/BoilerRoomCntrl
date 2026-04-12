/*
 * dwt_timer.h
 *
 *  Created on: Jan 17, 2021
 *      Author: Rinat
 */

#ifndef INC_DWT_TIMER_H_
#define INC_DWT_TIMER_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#define DWT_CYCCNT  *(volatile unsigned long*)0xE0001004
#define DWT_CONTROL *(volatile unsigned long*)0xE0001000
#define SCB_DEMCR   *(volatile unsigned long*)0xE000EDFC

void DWT_Init(void);
void delay_us(uint32_t us);
uint32_t millis(void);

#endif /* INC_DWT_TIMER_H_ */
