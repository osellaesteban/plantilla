/*
 * menu.c
 *
 *  Created on: 15 oct. 2018
 *      Author: esteban
 */
#include "menu.h"


const menuItem SelectMenuVariable[] =
{
	{"\n\r1. Set Acceleration thresholds\n\r", AccelMenuThreshold, flag_variable_menu},
	{"2. Set Angular Acceleration thresholds\n\r", AngVelMenuThreshold, flag_variable_menu},
	{"3. Set Filter parameters\n\r", goFilterMenu, flag_variable_menu},
	{"4. Set MPU sampling time\n\r", goTimingMenu, flag_variable_menu},
	//{"5. Set saving file frequency\n\r", config_saving, flag_variable_menu},
	{NULL , NULL}
};


const menuItem SelectAccelThreshold[] =
{
	{"\n\rEnter Maximum Acceleration thresholds in x,y and z separated by spaces:\r\n\r\n", go_back, flag_Acc_threshold_menu},
	{NULL , NULL}
};

const menuItem SelectAngVelThreshold[] =
{
	{"\n\rEnter Maximum Angular acceleration thresholds in x, y and z separated by spaces:\r\n\r\n", go_back, flag_Ang_Acc_threshold_menu},
	{NULL , NULL}
};



const menuItem FilterMenuParameters[] =
{
		{"1. Filter accelerometer parameters\r\n", go_AccParams, flag_filter_menu},
		{"2. Filter giroscope parameters\r\n", go_GiroParams, flag_filter_menu},
		{"3. Filter magnetometer parameters\r\n", go_MagParams, flag_filter_menu},
		{NULL , NULL}

};

const menuItem FilterAccParameters[] =
{
		{"\n\rEnter the 5 filter accelerometer parameters separated by spaces:\r\n\r\n", go_back, flag_filter_submenu},
		{NULL , NULL}
};

const menuItem FilterGiroParameters[] =
{
		{"\n\rEnter the 5 filter giroscope parameters separated by spaces:\r\n\r\n", go_back, flag_filter_submenu},
		{NULL , NULL}
};

const menuItem FilterMagParameters[] =
{
		{"\n\rEnter the 5 filter magnetometer parameters separated by spaces:\r\n\r\n", go_back, flag_filter_submenu},
		{NULL , NULL}
};

const menuItem TimingMenuParameters[] =
{
		{"\n\rEnter MPU sampling time in milliseconds:\r\n\r\n", go_back, flag_timing_menu},
		{NULL , NULL}
};

//const menuItem SavingMenuParameters[] =
//{
//		{"\n\rEnter file saving frequency in minutes\r\n", go_back, flag_threshold_menu},
//		{NULL , NULL}
////		{" 1: Activate register \r\n", config_saving, flag_saving_menu},
////		{" 2: Saving time (duration of register:1-24 hours) \r\n", config_saving, flag_saving_menu},
////		{" 3: file_saving_time (duration of each file in msec) \r\n", config_saving, flag_saving_menu},
////		{" 4: Go Back\n\r", go_back, flag_saving_menu},
////		{NULL , NULL}
//};
//
//static const char *ThresholdConfig = "\r\nHello \r\n"
//							"STABILITY OBSERVER \r\n"
//							"Press TEC2 to initiate configuration or \'x\' to stop it\r\n";

void * AccelMenuThreshold(void){

	return SelectAccelThreshold;
}

void * AngVelMenuThreshold(void){

	return SelectAngVelThreshold;
}

//void * AngleMenuThreshold(void){
//
//	return SelectAngleThreshold;
//}


void * goFilterMenu(void)
{
	return FilterMenuParameters;
}

void * goTimingMenu(void)
{
	return TimingMenuParameters;
}

//void * goSavingMenu(void)
//{
//	return SavingMenuParameters;
//}

void * getMainMenu(void)
{
	return SelectMenuVariable;
}


void * go_AccParams(void)
{

	return FilterAccParameters;
}

void * go_GiroParams(void)
{

	return FilterGiroParameters;
}
//
//void * filter_number(void)
//{
//
//	return FilterMenuParameters;
//}
//
//void * get_parameters(void)
//{
//
//	return FilterMenuParameters;
//}
//
//void * config_time(void)
//{
//
//	return TimingMenuParameters;
//}
//
void * go_MagParams(void)
{

	return FilterMagParameters;
}

void * go_back(void)
{
	return SelectMenuVariable;
}


