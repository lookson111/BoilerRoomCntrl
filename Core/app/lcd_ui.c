/*
 * lcd_ui.c
 *
 *  Created on: May 31, 2020
 *      Author: Rinat
 */

#include "lcd_ui.h"

#include <string.h>
#include "../ili9341/ili9341.h"
#include "hal_utils.h"
#include "stdint.h"

St_Rel_manage rel_manage[] = {
    {Water_Heat_Home_GPIO_Port, Water_Heat_Home_Pin, menuRelHeatingHome, 0},
    {wtr_hm_in_GPIO_Port, wtr_hm_in_Pin, menuRelWtrPompHome, 0},
    {CPW_HEAT_HOME_GPIO_Port, CPW_HEAT_HOME_Pin, menuRelCPHeatSys, 0},
    {CP_HOT_WATER_GPIO_Port, CP_HOT_WATER_Pin, menuRelCPHotWater, 0},
    {WtoHS_GPIO_Port, WtoHS_Pin, menuRelPompHeatSys, 0},
    {WATER_VALVE_GPIO_Port, WATER_VALVE_Pin, menuRelValveWater, 0}};

const char title_line = 1; // первая строка титульная
const char lines_max = 14; // максимальное количество отображаемых строк
const char disp_znak[] = {'>', 0x00, ' ', 0x00};

void disp_init(stDispMenu* dm)
{
    dm->redrawDispMenu = 1;
    dm->menuCountEl = menuCountElements;
    dm->diap_min = 0;
    dm->diap_max = dm->menuCountEl;
    dm->menu_data = 1;
    dm->pwr_on = 1;
    dm->b_enter_line = 0;
    dm->count = 1;
    dm->pwm_tmp = 0;
    dm->time_edit_mode = 0;
    dm->time_tmp_hour = 0;
    dm->time_tmp_minute = 0;
    dm->time_tmp_second = 0;
    dm->time_tmp_day = 1;
    dm->time_tmp_month = 1;
    dm->time_tmp_year = 25;

    ILI9341_Init();
    ILI9341_SetRotation(1);
    ILI9341_FillScreen(ILI9341_WHITE);
}

void disp_time_view(stDispMenu* dm, RTC_HandleTypeDef* hrtc,
                    RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate)
{
    char time[9];
    // обновление времени
    HAL_RTC_GetTime(hrtc, sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, sDate, RTC_FORMAT_BIN);
    time[0] = '0' + (sTime->Hours / 10);
    time[1] = '0' + (sTime->Hours % 10);
    time[2] = ':';
    time[3] = '0' + (sTime->Minutes / 10);
    time[4] = '0' + (sTime->Minutes % 10);
    time[5] = ':';
    time[6] = '0' + (sTime->Seconds / 10);
    time[7] = '0' + (sTime->Seconds % 10);
    time[8] = 0x00;
    ILI9341_WriteString(12 * 19, (240 - 15), time, Font_12x15, ILI9341_BLACK,
                        ILI9341_WHITE);
    // конец обновления времени
}

short disp_curs_view(stDispMenu* dm, FontDef font)
{
    if (count_lt == 0)
        return 0;
    if (dm->count == 0) {
        if (dm->menu_data) {
            ILI9341_WriteString(0, font.height * dm->count, &disp_znak[2], font,
                                ILI9341_WHITE, ILI9341_BLACK);
        } else {
            ILI9341_WriteString(160, font.height * dm->count, &disp_znak[2],
                                font, ILI9341_WHITE, ILI9341_BLACK);
        }
    } else {
        ILI9341_WriteString(0, font.height * dm->count, &disp_znak[2], font,
                            ILI9341_BLACK, ILI9341_WHITE);
    }
    // счетик изменеия положения курсора
    if (count_lt > 0) {
        dm->count--;
        dm->line--;
        count_lt = 0;
    } else if (count_lt < 0) {
        dm->count++;
        dm->line++;
        count_lt = 0;
    }
    // логика работы курсора
    if (dm->count > dm->menuCountEl) {
        if (dm->diap_max >= dm->menuCountEl) {
            // если мы превысили счетчиком максимальное количество элеметнов
            // то переводим курсор на 1 элемент
            dm->diap_min = 0;
            dm->count = title_line;
            dm->line = dm->diap_min;
        } else {
            // если не превысили то сдвигаем список на один вниз
            dm->count = lines_max;
            dm->diap_min += 1;
            dm->line += 1;
        }
        dm->diap_max = dm->diap_min + dm->menuCountEl; //lines_max;
        dm->redrawDispMenu = 1;
    } else if (dm->count <= 0) {
        if (dm->diap_min <= 0) { // ==0
            if (dm->count < 0) {
                // если выкрутили наверх доконца то перемещаем курсор в самый конец списка
                dm->diap_max = dm->menuCountEl;
                if (dm->diap_max > lines_max) {
                    dm->diap_min = dm->diap_max - lines_max;
                    dm->count = lines_max;
                } else {
                    dm->diap_min = 0;
                    dm->count = dm->diap_max;
                }
                dm->line = dm->diap_max - 1;
                dm->redrawDispMenu = 1;
            }
        } else {
            dm->diap_min -= 1;
            dm->line -= 1;
            dm->count = title_line;
            dm->diap_max = dm->diap_min + dm->menuCountEl;
            dm->redrawDispMenu = 1;
        }
    }
    if (dm->count == 0) {
        if (dm->menu_data) {
            ILI9341_WriteString(0, font.height * dm->count, &disp_znak[0], font,
                                ILI9341_WHITE, ILI9341_BLACK);
        } else {
            ILI9341_WriteString(160, font.height * dm->count, &disp_znak[0],
                                font, ILI9341_WHITE, ILI9341_BLACK);
        }
    } else {
        ILI9341_WriteString(0, font.height * dm->count, &disp_znak[0], font,
                            ILI9341_BLACK, ILI9341_WHITE);
    }
    return 0;
}

short disp_out_lines(stDispMenu* dm, FontDef font)
{
    // Запись данных в дисплей
    const char* menu;
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
        ILI9341_WriteString(12, 0, &str_title[0][0], font, dm->clrWordsLf,
                            dm->clrRectLeft);
        ILI9341_WriteString(12 + 160, 0, &str_title[1][0], font, dm->clrWordsRt,
                            dm->clrRectRight);
        // отображаем пункты меню
        for (uint16_t i = dm->diap_min; i < dm->diap_max; i++) {
            if (dm->menu_data)
                menu = &strMenuNameData[i][0];
            else
                menu = &strMenuNamePoint[i][0];
            ILI9341_WriteString(12,
                                font.height * (i - dm->diap_min + title_line),
                                menu, font, ILI9341_BLACK, ILI9341_WHITE);
        }
        for (uint16_t i = dm->diap_max; i < lines_max; i++) {
            ILI9341_WriteString(
                12, font.height * (i - dm->diap_min + title_line), strClearName,
                font, ILI9341_BLACK, ILI9341_WHITE);
        }

        dm->redrawDispMenu = 0;
    }
    // отображаем занчения параметов
    for (uint16_t i = dm->diap_min; i < dm->diap_max; i++) {
        if (dm->menu_data)
            menu = &strMenuValsData[i][0];
        else
            menu = &strMenuValsPoint[i][0];
        ILI9341_WriteString(12 * 18,
                            font.height * (i - dm->diap_min + title_line), menu,
                            font, ILI9341_BLACK, ILI9341_WHITE);
    }
    for (uint16_t i = dm->diap_max; i < lines_max; i++) {
        ILI9341_WriteString(12 * 18,
                            font.height * (i - dm->diap_min + title_line),
                            str_clear, font, ILI9341_BLACK, ILI9341_WHITE);
    }
    // Конец записи данных в дисплей
    return 0;
}

void disp_point_edit(stDispMenu* dm)
{
    // изменение уставок

    // КОНЕЦ изменение уставок
}

// Write time values from dm temp vars to RTC
void disp_set_time(stDispMenu* dm, RTC_HandleTypeDef* hrtc)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = dm->time_tmp_hour;
    sTime.Minutes = dm->time_tmp_minute;
    sTime.Seconds = dm->time_tmp_second;

    sDate.WeekDay = 1; // Monday (could be calculated)
    sDate.Month = dm->time_tmp_month;
    sDate.Date = dm->time_tmp_day;
    sDate.Year = dm->time_tmp_year;

    HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BIN);
}

// включение устройства
void disp_poweron(stDispMenu* dm)
{
    if (dm->pwr_on) {
        for (int i = 0; i < 6; i++) {
            rel_manage[i].fl_on_off = GPIO_PIN_RESET;
            memcpy(&strMenuValsPoint[rel_manage[i].line][0], str_off,
                   CNTVSYMINSTR);
            HAL_GPIO_WritePin(rel_manage[i].gpio_port, rel_manage[i].pin,
                              rel_manage[i].fl_on_off);
        }
        dm->pwr_on = 0;
    }
}

void disp_button_press(stDispMenu* dm, RTC_HandleTypeDef* hrtc)
{
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
            dm->diap_max = dm->menuCountEl;
            dm->redrawDispMenu = 1;
            return;
        }
        char flexit = true;

        if (!dm->menu_data)
            switch (strMenuTypeValPoint[dm->line]) {
                case ITNONE:
                    break;
                case ITONOFF:
                    for (int i = 0; i < 6; i++) {
                        if (dm->line == rel_manage[i].line) {
                            rel_manage[i].fl_on_off = ~rel_manage[i].fl_on_off;
                            memcpy(&strMenuValsPoint[rel_manage[i].line][0],
                                   rel_manage[i].fl_on_off ? str_on : str_off,
                                   CNTVSYMINSTR);
                            HAL_GPIO_WritePin(rel_manage[i].gpio_port,
                                              rel_manage[i].pin,
                                              rel_manage[i].fl_on_off);
                        }
                    }
                    break;
                case ITTIME:
                    // Enter/exit time editing mode
                    if (dm->time_edit_mode == 0) {
                        // Load current time from RTC
                        RTC_TimeTypeDef sTime = {0};
                        RTC_DateTypeDef sDate = {0};
                        HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN);
                        HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN);
                        dm->time_tmp_hour = sTime.Hours;
                        dm->time_tmp_minute = sTime.Minutes;
                        dm->time_tmp_second = sTime.Seconds;
                        dm->time_tmp_day = sDate.Date;
                        dm->time_tmp_month = sDate.Month;
                        dm->time_tmp_year = sDate.Year;
                        dm->time_edit_mode = 1;
                        dm->redrawDispMenu = 1;
                    } else {
                        // Apply time to RTC
                        dm->time_edit_mode = 0;
                        disp_set_time(dm, hrtc);
                        dm->redrawDispMenu = 1;
                    }
                    break;
                case ITINT:
                    // для редактирования значений захватываем поток
                    do {
                        // счетик изменеия положения курсора
                        if (count_lt > 0) {
                            // Increment value based on current menu item
                            if (dm->line == menuTimeHour) {
                                dm->time_tmp_hour =
                                    (dm->time_tmp_hour + 1) % 24;
                            } else if (dm->line == menuTimeMinute) {
                                dm->time_tmp_minute =
                                    (dm->time_tmp_minute + 1) % 60;
                            } else if (dm->line == menuTimeSecond) {
                                dm->time_tmp_second =
                                    (dm->time_tmp_second + 1) % 60;
                            } else if (dm->line == menuTimeDay) {
                                if (dm->time_tmp_day < 31)
                                    dm->time_tmp_day++;
                                else
                                    dm->time_tmp_day = 1;
                            } else if (dm->line == menuTimeMonth) {
                                if (dm->time_tmp_month < 12)
                                    dm->time_tmp_month++;
                                else
                                    dm->time_tmp_month = 1;
                            } else if (dm->line == menuTimeYear) {
                                if (dm->time_tmp_year < 99)
                                    dm->time_tmp_year++;
                                else
                                    dm->time_tmp_year = 0;
                            }
                            dm->redrawDispMenu = 1;
                            count_lt = 0;
                        } else if (count_lt < 0) {
                            // Decrement value based on current menu item
                            if (dm->line == menuTimeHour) {
                                if (dm->time_tmp_hour > 0)
                                    dm->time_tmp_hour--;
                                else
                                    dm->time_tmp_hour = 23;
                            } else if (dm->line == menuTimeMinute) {
                                if (dm->time_tmp_minute > 0)
                                    dm->time_tmp_minute--;
                                else
                                    dm->time_tmp_minute = 59;
                            } else if (dm->line == menuTimeSecond) {
                                if (dm->time_tmp_second > 0)
                                    dm->time_tmp_second--;
                                else
                                    dm->time_tmp_second = 59;
                            } else if (dm->line == menuTimeDay) {
                                if (dm->time_tmp_day > 1)
                                    dm->time_tmp_day--;
                                else
                                    dm->time_tmp_day = 31;
                            } else if (dm->line == menuTimeMonth) {
                                if (dm->time_tmp_month > 1)
                                    dm->time_tmp_month--;
                                else
                                    dm->time_tmp_month = 12;
                            } else if (dm->line == menuTimeYear) {
                                if (dm->time_tmp_year > 0)
                                    dm->time_tmp_year--;
                                else
                                    dm->time_tmp_year = 99;
                            }
                            dm->redrawDispMenu = 1;
                            count_lt = 0;
                        }

                        // необходима задержка
                        osDelay(100);
                        if (enterButton) {
                            flexit = false;
                            osDelay(200);
                            enterButton = 0;
                        }
                    } while (flexit);
                    break;
                case ITFLOAT:
                    // для редактирования значений захватываем поток
                    do {
                        // счетик изменеия положения курсора
                        if (count_lt > 0) {
                            count_lt = 0;
                        } else if (count_lt < 0) {
                            count_lt = 0;
                        }

                        // необходима задержка
                        osDelay(100);
                        if (enterButton) {
                            flexit = false;
                            osDelay(200);
                            enterButton = 0;
                        }
                    } while (flexit);
                    break;
                default:
                    break;
            }
    }
    // КОНЕЦ обработки нажатия
}
