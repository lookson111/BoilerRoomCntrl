/*
 * my_disp_lib.c
 *
 *  Created on: May 31, 2020
 *      Author: Rinat
 */

#include "my_disp_lib.h"

#include "..\ili9341\ili9341.h"
#include "mylib.h"
#include "stdint.h"
#include <string.h>

St_Rel_manage rel_manage[] = {{Water_Heat_Home_GPIO_Port, Water_Heat_Home_Pin, 	menuRelHeatingHome, 0},
															{wtr_hm_in_GPIO_Port, 			wtr_hm_in_Pin, 				menuRelWtrPompHome, 0},
															{CPW_HEAT_HOME_GPIO_Port, 	CPW_HEAT_HOME_Pin, 		menuRelCPHeatSys, 	0},
															{CP_HOT_WATER_GPIO_Port, 		CP_HOT_WATER_Pin, 		menuRelCPHotWater, 	0},
															{WtoHS_GPIO_Port,				 		WtoHS_Pin, 						menuRelPompHeatSys, 0},
															{WATER_VALVE_GPIO_Port,			WATER_VALVE_Pin, 			menuRelValveWater, 	0}
														 };

const char title_line = 1;		// первая строка титульная
const char lines_max  = 14;		// максимальное количество отображаемых строк
const char disp_znak[] = {'>' , 0x00, ' ', 0x00};

void disp_init (stDispMenu *dm) {
	dm->redrawDispMenu = 1;
	dm->diap_min = 0;
	dm->diap_max = lines_max;
	dm->menu_data = 1;
	dm->pwr_on = 1;
	dm->menuCountEl = menuCountElements;
	dm->b_enter_line = 0;
	dm->count = 1;
	dm->pwm_tmp = 0;
	
  ILI9341_Init();
  ILI9341_SetRotation(1);
  ILI9341_FillScreen(ILI9341_WHITE);


}

void disp_time_view(stDispMenu *dm, RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate)
{
	char humad[7];
	char time[10];
	// обновление времени
	HAL_RTC_GetTime(hrtc, sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, sDate, RTC_FORMAT_BIN);
	inttochar(humad, (uint32_t)sTime->Hours);
	time[0] = humad[4];
	time[1] = humad[5];
	time[2] = ':';
	inttochar(humad, (uint32_t)sTime->Minutes);
	time[3] = humad[4];
	time[4] = humad[5];
	time[5] = ':';
	inttochar(humad, (uint32_t)sTime->Seconds);
	time[6] = humad[4];
	time[7] = humad[5];
	time[8] = 0x00;
	ILI9341_WriteString(12 * 19, (240-15), time, Font_12x15, ILI9341_BLACK, ILI9341_WHITE);
	// конец обновления времени
}

short disp_curs_view(stDispMenu *dm, FontDef font) {
	if (count_lt == 0)
		return 0;
	// сначала удалаяем указатель в меню
  //disp_znak[0] = ' ';
	if (dm->count == 0 ) {
		if (dm->menu_data) {
			ILI9341_WriteString(0, font.height*dm->count, &disp_znak[2], font, ILI9341_WHITE, ILI9341_BLACK);
		} else {
			ILI9341_WriteString(160, font.height*dm->count, &disp_znak[2], font, ILI9341_WHITE, ILI9341_BLACK);
		}
	} else {
		ILI9341_WriteString(0, font.height*dm->count, &disp_znak[2], font, ILI9341_BLACK, ILI9341_WHITE);
	}
	// счетик изменеия положения курсора
	if (count_lt > 0) {
		dm->count--;
		count_lt = 0;
	} else if (count_lt < 0) {
		dm->count++;
		count_lt = 0;
	}
	// логика работы курсора
	if (dm->count > lines_max) {
		if (dm->diap_max >= dm->menuCountEl) {
			dm->diap_min = 0;
			dm->count = title_line;
		} else {
			dm->count = lines_max;
			dm->diap_min +=1;
		}
		dm->diap_max = dm->diap_min + lines_max;
		dm->redrawDispMenu = 1;
	} else if (dm->count <= 0) {
		if (dm->diap_min <= 0) {
			if (dm->count < 0) {
				dm->diap_max = dm->menuCountEl;
				dm->diap_min = dm->diap_max - lines_max;
				dm->count = lines_max;
				dm->redrawDispMenu = 1;
			}
		} else {
			dm->diap_min -= 1;
			dm->count = title_line;
			dm->diap_max = dm->diap_min + lines_max;
			dm->redrawDispMenu = 1;
		}
	}
	// показываем курсор
	//disp_znak[0] = '>';
	if (dm->count == 0 ) {
		if (dm->menu_data) {
			ILI9341_WriteString(0, font.height*dm->count, &disp_znak[0], font, ILI9341_WHITE, ILI9341_BLACK);
		} else {
			ILI9341_WriteString(160, font.height*dm->count, &disp_znak[0], font, ILI9341_WHITE, ILI9341_BLACK);
		}
	} else {
		ILI9341_WriteString(0, font.height*dm->count, &disp_znak[0], font, ILI9341_BLACK, ILI9341_WHITE);
	}
	return 0;
}

short disp_out_lines(stDispMenu *dm, FontDef font) {
	// Запись данных в дисплей
	const char *menu;
	if (dm->redrawDispMenu) {
		if (dm->menu_data) {
			dm->clrRectLeft = ILI9341_BLACK;
			dm->clrRectRight = ILI9341_RED;
			dm->clrWordsLf = ILI9341_WHITE;
			dm->clrWordsRt = ILI9341_BLACK;
		} else {
			dm->clrRectLeft = ILI9341_RED;
			dm->clrRectRight = ILI9341_BLACK;
			dm->clrWordsLf = ILI9341_BLACK;
			dm->clrWordsRt = ILI9341_WHITE;
		}
		// Рисуем заголовки
		ILI9341_FillRectangle(0, 0, 160, font.height, dm->clrRectLeft);
		ILI9341_FillRectangle(160, 0, 160, font.height, dm->clrRectRight);
		ILI9341_WriteString_DMA(12, 0, &str_title[0][0], font, dm->clrWordsLf, dm->clrRectLeft);
		ILI9341_WriteString_DMA(12+160, 0, &str_title[1][0], font, dm->clrWordsRt, dm->clrRectRight);
		// отображаем пункты меню
		for (uint16_t i = dm->diap_min; i < dm->diap_max; i++) {
			if (dm->menu_data)
				menu = &strMenuNameData[i][0];
			else
				menu = &strMenuNamePoint[i][0];
		  ILI9341_WriteString_DMA(12, font.height * (i - dm->diap_min + title_line), menu, font, ILI9341_BLACK, ILI9341_WHITE);
		}
		dm->redrawDispMenu = 0;
	}
	// отображаем занчения параметов
	for (uint16_t i = dm->diap_min; i < dm->diap_max; i++) {
		if (dm->menu_data)
			menu = &strMenuValsData[i][0];
		else
			menu = &strMenuValsPoint[i][0];
		ILI9341_WriteString_DMA(12 * 18, font.height * (i - dm->diap_min + title_line), menu, font, ILI9341_BLACK, ILI9341_WHITE);
	}
	// Конец записи данных в дисплей
	return 0;
}

void disp_point_edit (stDispMenu *dm) {
	// изменение уставок
	if (((dm->diap_min + dm->count - title_line) == menuPWMTermRez) ) {
		if (enterButton) {
			if ((dm->diap_min + dm->count - title_line) == dm->b_enter_line) {
				dm->b_enter_line = 0;
			} else {
				dm->b_enter_line = menuPWMTermRez;
			}
			enterButton = 0;
		}
		if ((dm->diap_min + dm->count - title_line) == dm->b_enter_line) {
			if (count_lt < 0) {
				dm->pwm_tmp = dm->pwm_tmp + 500;
				if (dm->pwm_tmp > 65500)
					dm->pwm_tmp = 0;
			} else if (count_lt > 0) {
				if (dm->pwm_tmp < 1000) {
					dm->pwm_tmp = 0;
				} else {
					dm->pwm_tmp = dm->pwm_tmp - 500;
				}
			}
			count_lt = 0;
		}
	}
	// КОНЕЦ изменение уставок
}

// включение устройства
void disp_poweron(stDispMenu *dm) {
	if (dm->pwr_on) {
		for(int i = 0; i < 6; i++) {
			rel_manage[i].fl_on_off = GPIO_PIN_RESET;
			memcpy(&strMenuValsPoint[rel_manage[i].line][0], str_off, CNTVSYMINSTR);
			HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin, rel_manage[i].fl_on_off);
		}
		dm->pwr_on = 0;
	}
}

void disp_button_press(stDispMenu *dm) {
	// Оработка наждатия
	if (enterButton) {
		// если кнопка нажата на шапке то изменить тип отображаемого меню
		enterButton = 0;
		if (dm->count == 0) {
			dm->menu_data = (~dm->menu_data) & 0x01;
			if (dm->menu_data) {
				dm->menuCountEl = menuCountElements;
			} else {
				dm->menuCountEl = menuPCountElements;
			}
			dm->redrawDispMenu = 1;
			//enterButton = 0;
			return;
		}
		//dm->line = dm->diap_min + dm->count - title_line;
		//enterButton = 0;
		for(int i = 0; i < 6; i++) {
			if (dm->count == rel_manage[i].line) {
				if (rel_manage[i].fl_on_off) {
					rel_manage[i].fl_on_off = GPIO_PIN_RESET;
					memcpy(&strMenuValsPoint[rel_manage[i].line][0], str_off, CNTVSYMINSTR);
					HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin, rel_manage[i].fl_on_off);
				} else {
					rel_manage[i].fl_on_off = GPIO_PIN_SET;
					memcpy(&strMenuValsPoint[rel_manage[i].line][0], str_on, CNTVSYMINSTR);
					HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin, rel_manage[i].fl_on_off);

				}
			}
		}
	}
	// КОНЕЦ обработки нажатия
}
























