/*
 * disp_spi.h
 *
 *  Created on: Nov 16, 2019
 *      Author: Rinat
 */

#ifndef INC_DISP_SPI_H_
#define INC_DISP_SPI_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi2;
//-------------------------------------------------------------------
#define RESET_ACTIVE() 	HAL_GPIO_WritePin(DISP_RES_GPIO_Port, DISP_RES_Pin, GPIO_PIN_RESET)
#define RESET_IDLE() 	HAL_GPIO_WritePin(DISP_RES_GPIO_Port, DISP_RES_Pin, GPIO_PIN_SET)
#define CS_ACTIVE() 	HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, GPIO_PIN_RESET)
#define CS_IDLE() 		HAL_GPIO_WritePin(DISP_CS_GPIO_Port, DISP_CS_Pin, GPIO_PIN_SET)
#define DC_COMMAND() 	HAL_GPIO_WritePin(DISP_DC_GPIO_Port, DISP_DC_Pin, GPIO_PIN_RESET)
#define DC_DATA() 		HAL_GPIO_WritePin(DISP_DC_GPIO_Port, DISP_DC_Pin, GPIO_PIN_SET)
//-------------------------------------------------------------------
#define ST7789VW_MADCTL_MY  0x80
#define ST7789VW_MADCTL_MX  0x40
#define ST7789VW_MADCTL_MV  0x20
#define ST7789VW_MADCTL_ML  0x10
#define ST7789VW_MADCTL_RGB 0x00
#define ST7789VW_MADCTL_BGR 0x08
#define ST7789VW_MADCTL_MH  0x04
#define ST7789VW_ROTATION (ST7789VW_MADCTL_MX | ST7789VW_MADCTL_BGR)
#define	ST7789VW_BLACK   0x0000
#define	ST7789VW_BLUE    0x001F
#define	ST7789VW_RED     0xF800
#define	ST7789VW_GREEN   0x07E0
#define ST7789VW_CYAN    0x07FF
#define ST7789VW_MAGENTA 0xF81F
#define ST7789VW_YELLOW  0xFFE0
#define ST7789VW_WHITE   0xFFFF
//-------------------------------------------------------------------
#define swap(a,b) {int16_t t=a;a=b;b=t;}
//-------------------------------------------------------------------

void ST7789VW_ini(uint16_t w_size, uint16_t h_size);
void ST7789VW_reset(void);
void ST7789VW_SendCommand(uint8_t cmd);
void ST7789VW_SendData(uint8_t dt);
void ST7789VW_WriteData(uint8_t* buff, size_t buff_size);
void ST7789VW_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789VW_FillScreen(uint16_t color);

#endif /* INC_DISP_SPI_H_ */
