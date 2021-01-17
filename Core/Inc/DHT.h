#ifndef DHT_H
#define DHT_H


#include "main.h"
#include "../my_file/mytime.h"
/* DHT library 

MIT license
written by Adafruit Industries
*/

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21


#define NAN 0x00
#define HIGH 0x01

#define TRUE 0x01
#define FALSE 0x00

#define INPUT 	0x01
#define OUTPUT  0x00
/*
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
*/

uint8_t data[6];
uint8_t _pin, _type, _count;
void pinMode(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, char fl);
char read(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
unsigned long _lastreadtime;
char firstreading;


void begin(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
float convertCtoF(float);
float readHumidity(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin);
float readTemperature(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, char S);

char  dataRead(float* tmp, float* hum);
/*
void DWT_Init(void);
void delay_us(uint32_t us);
uint32_t millis(void);
*/

#endif
