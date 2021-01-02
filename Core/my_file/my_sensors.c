///*
// * my_sensors.c
// *
// *  Created on: 21 мая 2020 г.
// *      Author: Rinat
// */
//
//#include "my_sensors.h"
//
//static const unsigned int tTR_temper_volt_arr[151][2] = {
//		{133500000, 32275},
//		{125672000, 32231},
//		{118350000, 32184},
//		{111498000, 32135},
//		{105084000, 32084},
//		{99077300, 32030},
//		{93446900, 31974},
//		{88175000, 31914},
//		{83229600, 31852},
//		{78590900, 31787},
//		{74238400, 31718},
//		{70152700, 31647},
//		{66316200, 31572},
//		{62712200, 31493},
//		{59325400, 31412},
//		{56141600, 31326},
//		{53147500, 31237},
//		{50330700, 31144},
//		{47679900, 31047},
//		{45184200, 30945},
//		{42883900, 30842},
//		{40619700, 30730},
//		{38533000, 30616},
//		{36565600, 30498},
//		{34710300, 30375},
//		{32960000, 30247},
//		{31308100, 30114},
//		{29748700, 29977},
//		{28276000, 29835},
//		{26884800, 29687},
//		{25570200, 29535},
//		{24327400, 29377},
//		{23152300, 29214},
//		{22040700, 29046},
//		{20968900, 28870},
//		{19993400, 28694},
//		{19050900, 28510},
//		{18158200, 28321},
//		{17312400, 28126},
//		{16510900, 27926},
//		{15751100, 27720},
//		{15030600, 27509},
//		{14347200, 27293},
//		{13698700, 27071},
//		{13083300, 26845},
//		{12499000, 26612},
//		{11944100, 26375},
//		{11416900, 26133},
//		{10916100, 25886},
//		{10440000, 25634},
//		{10000000, 25385},
//		{9558930, 25117},
//		{9147430, 24850},
//		{8757770, 24580},
//		{8386900, 24306},
//		{8033800, 24028},
//		{7697530, 23746},
//		{7377210, 23460},
//		{7072000, 23171},
//		{6781100, 22878},
//		{6503780, 22583},
//		{6239340, 22285},
//		{5987110, 21984},
//		{5746460, 21681},
//		{5516800, 21376},
//		{5297580, 21069},
//		{5088280, 20760},
//		{4888380, 20450},
//		{4697430, 20139},
//		{4514980, 19826},
//		{4340600, 19513},
//		{4173910, 19200},
//		{4014520, 18886},
//		{3862070, 18573},
//		{3716240, 18260},
//		{3588000, 17973},
//		{3443140, 17635},
//		{3315290, 17324},
//		{3192870, 17014},
//		{3075630, 16705},
//		{2963310, 16398},
//		{2855690, 16093},
//		{2752560, 15790},
//		{2653690, 15489},
//		{2558900, 15191},
//		{2467990, 14895},
//		{2380800, 14601},
//		{2297140, 14311},
//		{2216850, 14023},
//		{2139800, 13739},
//		{2065830, 13457},
//		{1994800, 13179},
//		{1926580, 12905},
//		{1861050, 12634},
//		{1798090, 12367},
//		{1737580, 12103},
//		{1679400, 11843},
//		{1623510, 11588},
//		{1569750, 11336},
//		{1518040, 11088},
//		{1473000, 10867},
//		{1420450, 10604},
//		{1374390, 10368},
//		{1330070, 10137},
//		{1287400, 9909},
//		{1246320, 9686},
//		{1206750, 9466},
//		{1168640, 9251},
//		{1131930, 9040},
//		{1096550, 8833},
//		{1062460, 8631},
//		{1029600, 8432},
//		{997924, 8237},
//		{967376, 8046},
//		{937916, 7860},
//		{909498, 7677},
//		{882083, 7498},
//		{855630, 7323},
//		{830101, 7152},
//		{805459, 6985},
//		{781570, 6820},
//		{758701, 6661},
//		{736519, 6505},
//		{715094, 6352},
//		{694397, 6203},
//		{674400, 6057},
//		{655075, 5914},
//		{636398, 5775},
//		{618345, 5639},
//		{600890, 5507},
//		{584013, 5377},
//		{567690, 5251},
//		{551902, 5128},
//		{536629, 5007},
//		{521852, 4890},
//		{507552, 4775},
//		{493712, 4663},
//		{480316, 4554},
//		{467346, 4448},
//		{454788, 4344},
//		{442627, 4243},
//		{430848, 4144},
//		{419438, 4048},
//		{408384, 3954},
//		{397674, 3862},
//		{387294, 3773},
//		{377233, 3686},
//		{367481, 3601},
//		{358026, 3518},
//		{348859, 3438},
//		{339968, 3359}
//};
//
//
