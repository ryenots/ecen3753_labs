/*
 * Application_Code.c
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#include "Application_Code.h"

static int systick_count = 0;

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

void HAL_SYSTICK_Callback(){
	systick_count += 1;
	if(systick_count % 100 == 0){
		systick_count = 0;
		int16_t velocity = read_gyro_velocity();
		drive_leds(velocity);
	}
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//
//
//}

void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	get_btn_state();

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

