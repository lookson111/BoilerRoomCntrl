/*
 * dwt_timer.h
 *
 *  Created on: Jan 17, 2021
 *      Author: Rinat
 */

#ifndef INC_DWT_TIMER_H_
#define INC_DWT_TIMER_H_

#ifdef __cplusplus

#include "main.h"

namespace dwt_timer {

// Initialize DWT cycle counter
void init();

// Busy-wait delay in microseconds
void delay_us(uint32_t us);

// Get elapsed milliseconds since DWT counter started
uint32_t millis();

}  // namespace dwt_timer

#endif // __cplusplus

#endif /* INC_DWT_TIMER_H_ */
