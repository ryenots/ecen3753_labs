/*
 * Application_Code.c
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#include "Application_Code.h"

void init_app(){
	Gyro_Init();
}

void get_btn_state(){
	usr_btn_state = HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);
}

int16_t read_gyro_velocity(){
	return Gyro_Get_Velocity();
}

void drive_leds(int16_t velocity){
	if(velocity > C_FAST && usr_btn_state){
		//red led OR button
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
	}else{
		//near zero range
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
	}

	if(velocity < CC_FAST || usr_btn_state){
		//green led AND button
		HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);

	}else{
		//near zero range
		HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_RESET);
	}

}
