/*
 * ApplicationCode.c
 *
 *  Created on: Dec 30, 2023
 *      Author: Xavion
 */

#include "ApplicationCode.h"
#include "game_config.h"
#include "math.h"
#include "Gyro_Driver.h"
#include "stdlib.h"

#define TICKRATE 100

#define DISRUPTOR_ACTIVE 0x00000001
#define MAZE_GENERATED   0x00000002
#define COLLISION_X		 0x00000004
#define COLLISION_Y		 0x00000008


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

RNG_HandleTypeDef* hrng_ptr;

// Data structures
static struct{
	double x;
	double y;
}xy_direction;

static struct Cell** maze;

extern void initialise_monitor_handles(void);

void disruptor_task(void* arg);
void gyro_polling_task(void* arg);
void game_comp_task(void* arg);
void lcd_display_task(void* arg);
void led_output_task(void* arg);

void ApplicationInit(RNG_HandleTypeDef* hrng){
	initialise_monitor_handles(); // Allows printf functionality
    LTCD__Init();
    LTCD_Layer_Init(0);
    LCD_Clear(0, LCD_COLOR_WHITE);
    Gyro_Init();

    hrng_ptr = hrng;

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

void read_xy_direction(double* xy){
	osMutexAcquire(xy_direction_mutex_id, osWaitForever);

	xy[0] = xy_direction.x;
	xy[1] = xy_direction.y;

	osMutexRelease(xy_direction_mutex_id);
}

void write_xy_direction(double* xy){
	osMutexAcquire(xy_direction_mutex_id, osWaitForever);

	xy_direction.x = xy[0];
	xy_direction.y = xy[1];

	osMutexRelease(xy_direction_mutex_id);
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
//	while(1){
//		osDelay(100);
//	}

	int zero_range = 40;
	double T = 0.02;
	float rads_ratio = 0.017453;
	double gyro_sensitivity = 17.50;

	double xy_pos[2];

	int16_t x_read;
	double x_rads_per_sec = 0;
	double x_rads = 0;
	double x_accel = 0;
	double x_velocity = 0;
	double x_pos = 0;

	int16_t y_read;
	double y_rads_per_sec = 0;
	double y_rads = 0;
	double y_accel = 0;
	double y_velocity = 0;
	double y_pos = 0;

	double x_prev = 0;
	double y_prev = 0;
	double x_dif = 0;
	double y_dif = 0;
	while(1){
		y_read = Gyro_Get_X_Velocity();
		x_read = Gyro_Get_Y_Velocity();

		if(x_read <= zero_range && x_read >= -zero_range) x_read = 0;
		if(y_read <= zero_range && y_read >= -zero_range) y_read = 0;

		x_rads_per_sec = (x_read * rads_ratio * gyro_sensitivity)/1000;
		y_rads_per_sec = (y_read * rads_ratio * gyro_sensitivity)/1000;

		x_rads += x_rads_per_sec * T;
		y_rads += y_rads_per_sec * T;

		x_accel = physics.gravity * sin(x_rads);
		y_accel = physics.gravity * sin(y_rads);

		// if x or y collides with an object, reset its velocity.
		uint32_t flags = osEventFlagsGet(event_group_id);
		if(flags & COLLISION_X){
			osEventFlagsClear(event_group_id, COLLISION_X);
//			modify position somehow
			x_pos = x_prev;
			x_velocity = 0;
		}

		if(flags & COLLISION_Y){
			osEventFlagsClear(event_group_id, COLLISION_Y);
			y_pos = y_prev;
			y_velocity = 0;
		}

		x_velocity += x_accel * T;
		y_velocity += y_accel * T;

		x_pos += x_velocity * T;
		y_pos += y_velocity * T;

		x_dif = x_pos - x_prev;
		y_dif = y_pos - y_prev;

		xy_pos[0] = x_dif;
		xy_pos[1] = y_dif;

		write_xy_direction(xy_pos);

		x_prev = x_pos;
		y_prev = y_pos;

		osDelay(20);
	}
}

//x_accel = g * sin(x_angle);
//y_accel = g * sin(y_agle_';')
//x_vel += x_accel * timestep;
//""
//x_pos += x_vel * timestep;
//''
//// Collision
//// Game logic busisens

void generate_maze(void){
	maze = malloc(sizeof(Cell*) * maze_config.size->width);

	for(int i=0; i<maze_config.size->width; i++) {
	    maze[i] = malloc(sizeof(Cell) * maze_config.size->height);
	}

	uint32_t rand;
	for(int i=0; i<maze_config.size->width; i++){
		for(int j=0; j<maze_config.size->height; j++){

			osStatus_t status = HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
			if(status != osOK) while(1);
			maze[i][j].top = rand % 2;
			status = HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
			if(status != osOK) while(1);
			maze[i][j].right = rand % 2;
		}
	}
}

void game_comp_task(void* arg){
	(void) &arg;
	// generate maze first

	generate_maze();

	osEventFlagsSet(event_group_id, MAZE_GENERATED);

//	while(1){
//		osDelay(100);
//	}

	double xy[2];
//	double x_pos, y_pos;
	double x_pos = LCD_PIXEL_WIDTH/2;
	double y_pos = LCD_PIXEL_HEIGHT/2;
	int prev_x_cell = -1;
	int prev_y_cell = -1;

	while(1){
		read_xy_direction(xy);

		x_pos += xy[0];
		y_pos += xy[1];

		int cur_x_cell = x_pos/maze_config.cell_size;
		int cur_y_cell = y_pos/maze_config.cell_size;

		if(prev_x_cell == -1){
			prev_x_cell = cur_x_cell;
			prev_y_cell = cur_y_cell;
			continue;
		}else{
			if(prev_x_cell != cur_x_cell){
				int dif = cur_x_cell - prev_x_cell;
				if(dif == 1){
					if(maze[prev_x_cell][cur_y_cell].right){
						//collision
						osEventFlagsSet(event_group_id, COLLISION_X);
					}
				}else if(dif == -1){
					if(maze[cur_x_cell][cur_y_cell].right){
						//collision
						osEventFlagsSet(event_group_id, COLLISION_X);
					}
				}else{
//					while(1);
				}
			}

			if(prev_y_cell != cur_y_cell){
				int dif = cur_y_cell - prev_y_cell;
				if(dif == 1){
					if(maze[cur_x_cell][prev_y_cell].top){
						//collision
						osEventFlagsSet(event_group_id, COLLISION_Y);
					}
				}else if(dif == -1){
					if(maze[cur_x_cell][cur_y_cell].top){
						//collision
						osEventFlagsSet(event_group_id, COLLISION_Y);
					}
				}else{
//					while(1);
				}
			}

		}

//		for(int i=-1; i<2; i++){
//			for(int j=-1; j<2; i++){
//				if(cur_x_cell + i >= 0 && cur_x_cell + i < maze_config.size->width){
//					// cell is within the maze. Check if walls are within collision proximity.
//				}
//				if(cur_y_cell + i >= 0 && cur_y_cell + i < maze_config.size->height){
//					// cell is within the maze. Check if walls are within collision proximity.
//				}
//			}
//		}


		osDelay(100);
	}

	// then handle game inputs and outputs

}

void lcd_draw_maze(void){
	for(int i=0; i<maze_config.size->width; i++){
		for(int j=0; j<maze_config.size->height; j++){
			if(maze[i][j].top){
				LCD_Draw_Horizontal_Line(i*maze_config.cell_size, j*maze_config.cell_size, maze_config.cell_size, LCD_COLOR_BLACK);
			}

			if(maze[i][j].right){
				LCD_Draw_Vertical_Line(i*maze_config.cell_size + maze_config.cell_size, j*maze_config.cell_size, maze_config.cell_size, LCD_COLOR_BLACK);
			}
		}
	}

	if(maze_config.hard_edged){
		LCD_Draw_Vertical_Line(0, 0, maze_config.cell_size*maze_config.size->height, LCD_COLOR_BLACK);
		LCD_Draw_Horizontal_Line(0, maze_config.cell_size*maze_config.size->height, maze_config.cell_size*maze_config.size->width, LCD_COLOR_BLACK);
	}
}

void lcd_display_task(void* arg){
	(void) &arg;
	LCD_Clear(0, LCD_COLOR_WHITE);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);

	osEventFlagsWait(event_group_id, MAZE_GENERATED, osFlagsWaitAny, osWaitForever);

//	while(1){
//		osDelay(100);
//	}

	int radius = drone.diameter/2;
	double xy_pos[2];
	double x_pos, y_pos;
	x_pos = LCD_PIXEL_WIDTH/2;
	y_pos = LCD_PIXEL_HEIGHT/2;
	while(1){
		lcd_draw_maze();
		read_xy_direction(xy_pos);
		x_pos += xy_pos[0];
		y_pos += xy_pos[1];

//		if(x_pos >= LCD_PIXEL_WIDTH - radius || x_pos < radius) x_pos = LCD_PIXEL_WIDTH/2;
//		if(y_pos >= LCD_PIXEL_HEIGHT - radius || y_pos < radius) y_pos = LCD_PIXEL_HEIGHT/2;

		LCD_Draw_Circle_Fill((int)x_pos, (int)y_pos, radius, LCD_COLOR_BLACK);
		osDelay(100);
		LCD_Clear(0, LCD_COLOR_WHITE);
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
