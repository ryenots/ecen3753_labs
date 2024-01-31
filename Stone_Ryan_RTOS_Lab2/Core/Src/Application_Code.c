/*
 * Application_Code.c
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#include "Application_Code.h"

void get_btn_state(){
	usr_btn_state = HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);
}

int read_gyro_rot(){

}
