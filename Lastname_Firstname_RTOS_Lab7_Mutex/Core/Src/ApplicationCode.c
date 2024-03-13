/*
 * ApplicationCode.c
 *
 *  Created on: Nov 14, 2023
 *      Author: xcowa
 */

#include "ApplicationCode.h"



void ApplicationInit(void)
{
	LTCD__Init();
    LTCD_Layer_Init(0);
    Gyro_Init();

}

void RunDemoForLCD(void)
{
	LCD_Clear(0,LCD_COLOR_WHITE);
	QuickDemo();
}

