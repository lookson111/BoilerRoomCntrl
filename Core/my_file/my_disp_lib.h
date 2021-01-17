/*
 * my_disp_lib.h
 *
 *  Created on: May 31, 2020
 *      Author: Rinat
 */

#ifndef MY_FILE_MY_DISP_LIB_H_
#define MY_FILE_MY_DISP_LIB_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "..\ili9341\fonts.h"
#include "mymenu.h"
#include "mytime.h"

//char strMenuValsData[menuCountElements][7];
//char strMenuValsPoint[menuPCountElements][7];

typedef struct {
	char disp_znak[2];
	char humad[7];
	char time[10];
	char redrawDispMenu; // перерисовать названия значений на экране
	char diap_min;
	char diap_max;
	char menu_data;
	char menuCountEl;
	char pwr_on;
	short clrRectLeft;
	short clrRectRight;
	short clrWordsLf;
	short clrWordsRt;
	int line;						// указатель на текущий выбранный элемент
	char b_enter_line;
	int count;						// указатель на текущую линию в листе!! от 0 от lines_max текущий элемент на который указывает указатель
	uint32_t pwm_tmp;
} stDispMenu;

stDispMenu dm;
void disp_init (stDispMenu *dm);
void disp_time_view(stDispMenu *dm, RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate);
short disp_out_lines(stDispMenu *dm, FontDef font);
short disp_curs_view(stDispMenu *dm, FontDef font);
void disp_poweron(stDispMenu *dm);
void disp_button_press(stDispMenu *dm);
void disp_point_edit (stDispMenu *dm);




#endif /* MY_FILE_MY_DISP_LIB_H_ */
