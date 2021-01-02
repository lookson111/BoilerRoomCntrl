/*
 * disp_spi.c
 *
 *  Created on: Nov 16, 2019
 *      Author: Rinat
 */

#include "disp_spi.h"


//extern RNG_HandleTypeDef hrng;
uint16_t ST7789VW_WIDTH;
uint16_t ST7789VW_HEIGHT;

void ST7789VW_ini(uint16_t w_size, uint16_t h_size)
{
	uint8_t data[15];
	CS_ACTIVE();
	ST7789VW_reset();

	//Software Reset Програмная перезагрузка дисплея
	ST7789VW_SendCommand(0x01);
	HAL_Delay(1000);
	//Power Control A
	data[0] = 0x39;
	data[1] = 0x2C;
	data[2] = 0x00;
	data[3] = 0x34;
	data[4] = 0x02;
	ST7789VW_SendCommand(0xCB);
	ST7789VW_WriteData(data, 5);
	//Power Control B
	data[0] = 0x00;
	data[1] = 0xC1;
	data[2] = 0x30;
	ST7789VW_SendCommand(0xCF);
	ST7789VW_WriteData(data, 3);
	//Driver timing control A
	data[0] = 0x85;
	data[1] = 0x00;
	data[2] = 0x78;
	ST7789VW_SendCommand(0xE8);
	ST7789VW_WriteData(data, 3);
	//Driver timing control B
	data[0] = 0x00;
	data[1] = 0x00;
	ST7789VW_SendCommand(0xEA);
	ST7789VW_WriteData(data, 2);
	//Power on Sequence control
	data[0] = 0x64;
	data[1] = 0x03;
	data[2] = 0x12;
	data[3] = 0x81;
	ST7789VW_SendCommand(0xED);
	ST7789VW_WriteData(data, 4);
	//Pump ratio control
	data[0] = 0x20;
	ST7789VW_SendCommand(0xF7);
	ST7789VW_WriteData(data, 1);
	//Power Control,VRH[5:0]
	data[0] = 0x10;
	ST7789VW_SendCommand(0xC0);
	ST7789VW_WriteData(data, 1);
	//Power Control,SAP[2:0];BT[3:0]
	data[0] = 0x10;
	ST7789VW_SendCommand(0xC1);
	ST7789VW_WriteData(data, 1);
	//VCOM Control 1
	data[0] = 0x3E;
	data[1] = 0x28;
	ST7789VW_SendCommand(0xC5);
	ST7789VW_WriteData(data, 2);
	//VCOM Control 2
	data[0] = 0x86;
	ST7789VW_SendCommand(0xC7);
	ST7789VW_WriteData(data, 1);
	//Memory Acsess Control
	data[0] = 0x48;
	ST7789VW_SendCommand(0x36);
	ST7789VW_WriteData(data, 1);
	//Pixel Format Set
	data[0] = 0x55;//16bit
	ST7789VW_SendCommand(0x3A);
	ST7789VW_WriteData(data, 1);
	//Frame Rratio Control, Standard RGB Color
	data[0] = 0x00;
	data[1] = 0x18;
	ST7789VW_SendCommand(0xB1);
	ST7789VW_WriteData(data, 2);
	//Display Function Control
	data[0] = 0x08;
	data[1] = 0x82;
	data[2] = 0x27;//320 строк
	ST7789VW_SendCommand(0xB6);
	ST7789VW_WriteData(data, 3);
	//Enable 3G (пока не знаю что это за режим)
	data[0] = 0x00;//не включаем
	ST7789VW_SendCommand(0xF2);
	ST7789VW_WriteData(data, 1);
	//Gamma set
	data[0] = 0x01;//Gamma Curve (G2.2) (Кривая цветовой гаммы)
	ST7789VW_SendCommand(0x26);
	ST7789VW_WriteData(data, 1);
	//Positive Gamma  Correction
	data[0] = 0x0F;
	data[1] = 0x31;
	data[2] = 0x2B;
	data[3] = 0x0C;
	data[4] = 0x0E;
	data[5] = 0x08;
	data[6] = 0x4E;
	data[7] = 0xF1;
	data[8] = 0x37;
	data[9] = 0x07;
	data[10] = 0x10;
	data[11] = 0x03;
	data[12] = 0x0E;
	data[13] = 0x09;
	data[14] = 0x00;
	ST7789VW_SendCommand(0xE0);
	ST7789VW_WriteData(data, 15);
	//Negative Gamma  Correction
	data[0] = 0x00;
	data[1] = 0x0E;
	data[2] = 0x14;
	data[3] = 0x03;
	data[4] = 0x11;
	data[5] = 0x07;
	data[6] = 0x31;
	data[7] = 0xC1;
	data[8] = 0x48;
	data[9] = 0x08;
	data[10] = 0x0F;
	data[11] = 0x0C;
	data[12] = 0x31;
	data[13] = 0x36;
	data[14] = 0x0F;
	ST7789VW_SendCommand(0xE1);
	ST7789VW_WriteData(data, 15);
	ST7789VW_SendCommand(0x11);//Выйдем из спящего режима

	HAL_Delay(120);

	//Display ON
	data[0] = ST7789VW_ROTATION;
	ST7789VW_SendCommand(0x29);
	ST7789VW_WriteData(data, 1);

	ST7789VW_WriteData(data, 1);
	ST7789VW_WIDTH = w_size;
	ST7789VW_HEIGHT = h_size;
}

void ST7789VW_reset(void)
{
	RESET_ACTIVE();
	HAL_Delay(5);
	RESET_IDLE();
}

// Функция отправки команды на дисплей по шине SPI
void ST7789VW_SendCommand(uint8_t cmd)
{
	DC_COMMAND();
	HAL_SPI_Transmit (&hspi2, &cmd, 1, 5000);
}

// функция отправки данных на дисплей по шине SPI
void ST7789VW_SendData(uint8_t dt)
{
	DC_DATA();
	HAL_SPI_Transmit (&hspi2, &dt, 1, 5000);
}

// Функция отправки данных сразу пакетом в определенном количестве
void ST7789VW_WriteData(uint8_t* buff, size_t buff_size)
{
	DC_DATA();
	while(buff_size > 0) {
		uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
		HAL_SPI_Transmit(&hspi2, buff, chunk_size, HAL_MAX_DELAY);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}

//-функцию заливки прямоугольника
static void ST7789VW_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  // column address set
  ST7789VW_SendCommand(0x2A); // CASET
  {
    uint8_t data[] = { (x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF };
    ST7789VW_WriteData(data, sizeof(data));
  }

  // row address set
  ST7789VW_SendCommand(0x2B); // RASET
  {
    uint8_t data[] = { (y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF };
    ST7789VW_WriteData(data, sizeof(data));
  }

  // write to RAM
  ST7789VW_SendCommand(0x2C); // RAMWR
}//-------------------------------------------------------------------
void ST7789VW_FillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  if((x1 >= ST7789VW_WIDTH) || (y1 >= ST7789VW_HEIGHT) || (x2 >= ST7789VW_WIDTH) || (y2 >= ST7789VW_HEIGHT)) return;
	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);
  ST7789VW_SetAddrWindow(x1, y1, x2, y2);
  uint8_t data[] = { color >> 8, color & 0xFF };
  DC_DATA();
  for(uint32_t i = 0; i < (x2-x1+1)*(y2-y1+1); i++)
  {
      HAL_SPI_Transmit(&hspi2, data, 2, HAL_MAX_DELAY);
  }
}
//-------------------------------------------------------------------

//-Ниже добавим функцию заливки всего экрана определённым цветом
void ST7789VW_FillScreen(uint16_t color)
{
	ST7789VW_FillRect(0, 0, ST7789VW_WIDTH-1, ST7789VW_HEIGHT-1, color);
}
//-------------------------------------------------------------------


//




