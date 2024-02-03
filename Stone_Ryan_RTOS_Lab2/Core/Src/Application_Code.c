/*
 * Application_Code.c
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#include "Application_Code.h"

static int systick_count = 0;

int usr_btn_state = 0;

/*
 * @brief Initialize application.
 */
void init_app(){
	Gyro_Init();
}

/*
 * @brief Retrieve user button state for application use.
 */
void get_btn_state(){
	usr_btn_state = HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);
}

/*
 * @brief Retrieve gryo velocity when a new value is ready in hardware.
 * @return Signed 16 bit int representing rotational velocity.
 */
int16_t read_gyro_velocity(){
	return Gyro_Get_Velocity();
}

/*
 * @brief Use input velocity to drive both leds to correct state.
 * @param velocity 16 bit input velocity
 */
void drive_leds(int16_t velocity){
	if(velocity >= CC_SLOW && usr_btn_state){
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

/*
 * @brief Callback function for system tick. By default is called every 1ms. This implementation will cause gyro velocity to be ready every 100ms.
 */
void HAL_SYSTICK_Callback(){
	systick_count += 1;
	if(systick_count % 100 == 0){
		systick_count = 0;
		int16_t velocity = read_gyro_velocity();
		drive_leds(velocity);
	}
}

/*
 * @brief External interrupt 0 request handler. User button 0 causes this interrupt, and will retrieve button state when pressed/unpressed.
 */
void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	get_btn_state();

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

