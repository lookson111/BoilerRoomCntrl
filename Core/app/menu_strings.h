/*
 * menu_strings.h
 *
 *  Created on: Jan 17, 2021
 *      Author: Rinat
 */

#ifndef INC_MENU_STRINGS_H_
#define INC_MENU_STRINGS_H_


#define MENUCOUNTPARAM 11  //8+3
#define CNTMSYMINSTR 17 // символов в строке в названиях
#define CNTVSYMINSTR 7  // символов в строке значений

#define ITNONE 0
#define ITONOFF 1
#define ITINT   2
#define ITFLOAT 3
#define ITTIME  4

enum {
	menuDHT22_1_temp,
	menuDHT22_1_humd,
	menuDHT22_2_temp,
	menuDHT22_2_humd,
	menuNull1,
	menuTRez_1,
	menuTRez_2,
	menuTRez_3,
	menuTRez_4,
	menuNull2,
	menuPres_1,
	menuPres_2,
	menuWtrCounter, // счетчик воды
	menuPWMTermRez,
	menuCountElements
};

enum {
	// насос отопления
	menuRelPompHeatSys,
	mPntDriveHeatMinPres, // минимальное давление
	mPntPNull2,
	mPntPNull3,
	mPntPNull4,
	mPntPNull5,
	mPntPNull6,
	mPntPNull7,
	menuRelHeatingHome,
	menuRelWtrPompHome,
	menuRelCPHotWater,
	menuRelCPHeatSys,
	menuRelValveWater,
	menuTimeHour,       // Часы
	menuTimeMinute,     // Минуты
	menuTimeSecond,     // Секунды
	menuTimeDay,        // День
	menuTimeMonth,      // Месяц
	menuTimeYear,       // Год
	menuPCountElements
};

enum {
	titleData,
	titlePiont,
	titleCountElements
};


/*
typedef struct {
	const char name[CNTMSYMINSTR];
	const char typedata;				// тип отображаемых данных справа
															// 0 - откл/вкл
															// 1 - int
															// 2 - float
	const char val[CNTVSYMINSTR];
} dispMenu;
*/
static const char str_title[titleCountElements][13] = {
		{0xc4, 0xe0, 0xed, 0xed, 0xfb, 0xe5, 0x00}, // Данные
		{0xd3, 0xf1, 0xf2, 0xe0, 0xe2, 0xea, 0xe8, 0x00} // Уставки
};


static const char strClearName[]  = {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00}; //
static const char str_on[]				= {0xc2, 0xea, 0xeb, 0x2e, 0x20, 0x20, 0x00}; // Вкл.
static const char str_off[]				= {0xce, 0xf2, 0xea, 0xeb, 0x2e, 0x20, 0x00}; // Откл.
static const char str_clear[]			= {0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00}; //
extern const char strMenuNameData[menuCountElements][CNTMSYMINSTR];
extern const char strMenuNamePoint[menuPCountElements][CNTMSYMINSTR];
extern const char strMenuTypeValPoint[menuPCountElements];

extern char strMenuValsData[menuCountElements][7];
extern char strMenuValsPoint[menuPCountElements][7];


#endif /* INC_MENU_STRINGS_H_ */
