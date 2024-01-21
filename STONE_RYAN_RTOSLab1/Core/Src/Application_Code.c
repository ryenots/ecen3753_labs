/*
 * Application_Code.c
 *
 *  Created on: Jan 20, 2024
 *      Author: ryenots
 */


#include "Application_Code.h"

void app_init(){
	GPIO_InitTypeDef gpio_init_B;
	gpio_init_B.Pin = USR_BTN_PIN;

	GPIO_InitTypeDef gpio_init_G;
	gpio_init_G.Pin = GRN_LED_PIN | RED_LED_PIN;

	HAL_GPIO_Init(USR_BTN_PORT, &gpio_init_B);
	HAL_GPIO_Init(USR_BTN_PORT, &gpio_init_G);
}

void led_toggle(){
	if(HAL_GPIO_ReadPin(LED_PORT, GRN_LED_PIN) == GPIO_PIN_SET){
		HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);
	}
	int test = get_btn_state();
	HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, test);

	HAL_Delay(1000);
}

int get_btn_state(){
	return HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);
}
