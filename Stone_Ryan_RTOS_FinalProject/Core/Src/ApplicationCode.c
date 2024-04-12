/*
 * ApplicationCode.c
 *
 *  Created on: Dec 30, 2023
 *      Author: Xavion
 */

#include "ApplicationCode.h"
#include "game_config.h"

#define TICKRATE 100

#define DISRUPTOR_ACTIVE 0x00000001


/* Static variables */

// Task ids
static osThreadId_t disruptor_task_id;
static osThreadId_t gyro_polling_task_id;
static osThreadId_t game_comp_task_id;
static osThreadId_t lcd_display_task_id;
static osThreadId_t led_output_task_id;

// ITC defines
static osEventFlagsId_t event_group_id;
static osSemaphoreId_t disruptor_sem_id;
static osMutexId_t xy_direction_mutex_id;

// Data structures
static struct{
	float x;
	float y;
}xy_direction;

extern void initialise_monitor_handles(void);

void disruptor_task(void* arg);
void gyro_polling_task(void* arg);
void game_comp_task(void* arg);
void lcd_display_task(void* arg);
void led_output_task(void* arg);

void ApplicationInit(void){
	initialise_monitor_handles(); // Allows printf functionality
    LTCD__Init();
    LTCD_Layer_Init(0);
    LCD_Clear(0, LCD_COLOR_WHITE);

    // ITC init
    event_group_id = osEventFlagsNew(NULL);
    if(event_group_id == NULL) while(1);

    disruptor_sem_id = osSemaphoreNew(1, 0, NULL);
    if(disruptor_sem_id == NULL) while(1);

    xy_direction_mutex_id = osMutexNew(NULL);
    if(xy_direction_mutex_id == NULL) while(1);

    // task init
    disruptor_task_id = osThreadNew(disruptor_task, (void*)0, NULL);
    if(disruptor_task_id == NULL) while(1);

    gyro_polling_task_id = osThreadNew(gyro_polling_task, (void*)0, NULL);
    if(gyro_polling_task_id == NULL) while(1);

    game_comp_task_id = osThreadNew(game_comp_task, (void*)0, NULL);
    if(game_comp_task_id == NULL) while(1);

    lcd_display_task_id = osThreadNew(lcd_display_task, (void*)0, NULL);
    if(lcd_display_task_id == NULL) while(1);

    led_output_task_id = osThreadNew(led_output_task, (void*)0, NULL);
    if(led_output_task_id == NULL) while(1);
}

void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	int btn_state = HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);

	if(btn_state){
		// rising edge
		osEventFlagsSet(event_group_id, DISRUPTOR_ACTIVE);

	}else{
		// falling edge
		osEventFlagsClear(event_group_id, DISRUPTOR_ACTIVE);

	}

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void disruptor_task(void* arg){
	// start timer when button is pushed, with varying length (oneShot)
	(void) &arg;
	while(1){
		osDelay(100);
	}

}

void gyro_polling_task(void* arg){
	(void) &arg;
	int SAMPLES = 10;
	int y_velocity[SAMPLES];
	int x_velocity[SAMPLES];

	for(int i=0; i<SAMPLES; i++){
		y_velocity[i] = Gyro_Get_Y_Velocity();
		x_velocity[i] = Gyro_Get_X_Velocity();
		osDelay(100);
	}

	int position = 0;
	double T = 0.1;
	while(1){
		if(position == SAMPLES){
			position = 0;
			double x_rot_angle = 0;
			for(int i=0; i<SAMPLES; i++){
				x_rot_angle += x_velocity[i] * T;
			}
		}
		y_velocity[position] = Gyro_Get_Y_Velocity();
		x_velocity[position] = Gyro_Get_X_Velocity();


		position += 1;
		osDelay(100);
	}
}

void game_comp_task(void* arg){
	// generate maze first
	(void) &arg;
	while(1){
		osDelay(100);
	}

	// then handle game inputs and outputs

}

void lcd_display_task(void* arg){
	(void) &arg;
	LCD_Clear(0, LCD_COLOR_WHITE);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);

	int diameter = drone.diameter;
	LCD_Draw_Circle_Fill(50, 50, diameter, LCD_COLOR_BLACK);
	while(1){
		osDelay(100);
	}
}

void led_output_task(void* arg){
	(void) &arg;
	while(1){
		osDelay(100);
	}
}

//void LCD_Visual_Demo(void){
//	visualDemo();
//}
