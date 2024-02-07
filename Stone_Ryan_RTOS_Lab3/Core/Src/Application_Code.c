/*
 * Application_Code.c
 *
 *  Created on: Feb 6, 2024
 *      Author: ryenots
 */


#include "Application_Code.h"

#define TIMER_MS_PERIOD 100U

//static int systick_count = 0;

int usr_btn_state = 0;

static osTimerId_t periodic_id;
static StaticTimer_t static_timer;

//osTimerAttr_t
const char* name = "app_timer";
const StaticTimer_t* timer_tcb_addr = &static_timer;
const uint32_t StaticTimer_t_size = sizeof(StaticTimer_t);

void timer_callback(void* argument);

/*
 * @brief Initialize application.
 */
void init_app(){
	Gyro_Init();

	osTimerAttr_t timer_attr;
	timer_attr.name = name;
	timer_attr.cb_mem = timer_tcb_addr;
	timer_attr.cb_size = StaticTimer_t_size;
	periodic_id = osTimerNew(timer_callback, osTimerPeriodic, (void*)0, &timer_attr);

	// Halt if timer was not created successfully.
	if(periodic_id == NULL) while(1);

	osStatus_t status = osTimerStart(periodic_id, TIMER_MS_PERIOD);

	// Halt if timer could not be started.
	if(status != osOK) while(1);
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
//void HAL_SYSTICK_Callback(){
//	systick_count += 1;
//	if(systick_count % 100 == 0){
//		systick_count = 0;
//		int16_t velocity = read_gyro_velocity();
//		drive_leds(velocity);
//	}
//}

/*
 * @brief External interrupt 0 request handler. User button 0 causes this interrupt, and will retrieve button state when pressed/unpressed.
 */
void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	get_btn_state();

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void timer_callback(void* argument){
	// remove unused variable warning
	(void) &argument;
	get_btn_state();
	int16_t velocity = read_gyro_velocity();
	drive_leds(velocity);
	HAL_Delay(100);

}






