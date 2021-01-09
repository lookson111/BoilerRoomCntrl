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


void disp_init (stDispMenu *dm) {
	dm->disp_znak[0] = '>';
	dm->disp_znak[1] = 0x00;
	dm->fl_res_menu = 1;
	dm->title_line = 1;
	dm->diap_min = 0;
	dm->lines_max = 14;
	dm->diap_max = dm->lines_max;
	dm->menu_data = 1;
	dm->pwr_on = 1;
	dm->menuCountEl = menuCountElements;
	dm->line = 0;
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

  dm->disp_znak[0] = ' ';
	if (dm->count == 0 ) {
		if (dm->menu_data) {
			ILI9341_WriteString(0, font.height*dm->count, dm->disp_znak, font, ILI9341_WHITE, ILI9341_BLACK);
		} else {
			ILI9341_WriteString(160, font.height*dm->count, dm->disp_znak, font, ILI9341_WHITE, ILI9341_BLACK);
		}
	} else {
		ILI9341_WriteString(0, font.height*dm->count, dm->disp_znak, font, ILI9341_BLACK, ILI9341_WHITE);
	}
	if (count_lt > 0) {
		dm->count--;
		count_lt = 0;
	} else if (count_lt < 0) {
		dm->count++;
		count_lt = 0;
	}
	if (dm->count > dm->lines_max) {
		if (dm->diap_max >= dm->menuCountEl) {
			dm->diap_min = 0;
			dm->count = dm->title_line;
		} else {
			dm->count = dm->lines_max;
			dm->diap_min +=1;
		}
		dm->diap_max = dm->diap_min + dm->lines_max;
		dm->fl_res_menu = 1;
	} else if (dm->count <= 0) {
		if (dm->diap_min <= 0) {
			if (dm->count < 0) {
				dm->diap_max = dm->menuCountEl;
				dm->diap_min = dm->diap_max - dm->lines_max;
				dm->count = dm->lines_max;
				dm->fl_res_menu = 1;
			}
		} else {
			dm->diap_min -= 1;
			dm->count = dm->title_line;
			dm->diap_max = dm->diap_min + dm->lines_max;
			dm->fl_res_menu = 1;
		}
	}
	dm->disp_znak[0] = '>';
	if (dm->count == 0 ) {
		if (dm->menu_data) {
			ILI9341_WriteString(0, font.height*dm->count, dm->disp_znak, font, ILI9341_WHITE, ILI9341_BLACK);
		} else {
			ILI9341_WriteString(160, font.height*dm->count, dm->disp_znak, font, ILI9341_WHITE, ILI9341_BLACK);
		}
	} else {
		ILI9341_WriteString(0, font.height*dm->count, dm->disp_znak, font, ILI9341_BLACK, ILI9341_WHITE);
	}
	return 0;
}

short disp_out_lines(stDispMenu *dm, FontDef font) {
	// Запись данных в дисплей
	const char *menu;
	if (dm->fl_res_menu) {
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
				menu = &str_menu[i][0];
			else
				menu = &str_menu_point[i][0];
		  ILI9341_WriteString_DMA(12, font.height * (i - dm->diap_min + dm->title_line), menu, font, ILI9341_BLACK, ILI9341_WHITE);
		}
		dm->fl_res_menu = 0;
	}
	// отображаем занчения параметов
	for (uint16_t i = dm->diap_min; i < dm->diap_max; i++) {
		if (dm->menu_data)
			menu = &str_menu_values[i][0];
		else
			menu = &str_menu_values_point[i][0];
		ILI9341_WriteString_DMA(12 * 18, font.height * (i - dm->diap_min + dm->title_line), menu, font, ILI9341_BLACK, ILI9341_WHITE);
	}
	// Конец записи данных в дисплей
	return 0;
}

void disp_point_edit (stDispMenu *dm) {
	// изменение уставок
	if (((dm->diap_min + dm->count - dm->title_line) == menuPWMTermRez) ) {
		if (enterButton) {
			if ((dm->diap_min + dm->count - dm->title_line) == dm->b_enter_line) {
				dm->b_enter_line = 0;
			} else {
				dm->b_enter_line = menuPWMTermRez;
			}
			enterButton = 0;
		}
		if ((dm->diap_min + dm->count - dm->title_line) == dm->b_enter_line) {
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

void disp_button_press(stDispMenu *dm) {
	// Оработка наждатия
	if (dm->pwr_on) {
		for(int i = 0; i < 6; i++) {
			memcpy(&str_menu_values[rel_manage[i].line][0], str_off, 7);
			HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin, GPIO_PIN_RESET);
		}
		dm->pwr_on = 0;
	}
	if (enterButton) {
		dm->line = dm->diap_min + dm->count - dm->title_line;
		enterButton = 0;
		for(int i = 0; i < 6; i++) {
			if (dm->line == rel_manage[i].line) {
				if (rel_manage[i].fl_on_off) {
					memcpy(&str_menu_values[rel_manage[i].line][0], str_off, 7);
					HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin, GPIO_PIN_RESET);
					rel_manage[i].fl_on_off = 0;
				} else {
					memcpy(&str_menu_values[rel_manage[i].line][0], str_on, 7);
					HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin, GPIO_PIN_SET);
					rel_manage[i].fl_on_off = 1;
				}
			}
		}
		// если кнопка нажата на шапке то изменить тип отображаемого меню
		if (dm->count == 0) {
			dm->menu_data = (~dm->menu_data) & 0x01;
			if (dm->menu_data) {
				dm->menuCountEl = menuCountElements;
			} else {
				dm->menuCountEl = menuPCountElements;
			}
			dm->fl_res_menu = 1;
		}
	}
	// КОНЕЦ обработки нажатия
}
























