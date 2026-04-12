/*
 * dwt_timer.cpp
 *
 *  Created on: Jan 17, 2021
 *      Author: Rinat
 */

#include "dwt_timer.h"

namespace dwt_timer {

void init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t us_count_tic = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - t0) < us_count_tic)
        ;
}

uint32_t millis()
{
    return DWT->CYCCNT / (SystemCoreClock / 1000);
}

}  // namespace dwt_timer
