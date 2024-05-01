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

#define TICKRATE   100
#define TOTAL_TIME 30000

#define BTN_STATUS 		 0x00000001
#define MAZE_GENERATED   0x00000002
#define COLLISION_X		 0x00000004
#define COLLISION_Y		 0x00000008
#define DISRUPTOR_STATE  0x00000010
#define DISRUPTOR_END    0x00000020
#define GAME_OVER_LOSS	 0x00000040
#define GAME_OVER_WIN	 0x00000080
#define GAME_RESET		 0x00000100


/* Static variables */

// Task ids
static osThreadId_t disruptor_task_id;
static osThreadAttr_t disruptor_task_attr = {
		.name = "disruptorTask"
};

static osThreadId_t gyro_polling_task_id;
static osThreadAttr_t gyro_polling_task_attr = {
		.name = "gyroPollingTask"
};

static osThreadId_t game_comp_task_id;
static osThreadAttr_t game_comp_task_attr = {
		.name = "gameCompTask"
};

static osThreadId_t lcd_display_task_id;
static osThreadAttr_t lcd_display_task_attr = {
		.name = "lcdDisplayTask"
};

static osThreadId_t green_led_task_id;
static osThreadAttr_t green_led_task_attr = {
		.name = "greenLedTask"
};

static osThreadId_t red_led_task_id;
static osThreadAttr_t red_led_task_attr = {
		.name = "redLedTask"
};

// ITC defines
static osEventFlagsId_t event_group_id;
static osSemaphoreId_t disruptor_start_sem_id;
static osSemaphoreId_t disruptor_stop_sem_id;
static osMutexId_t xy_pos_dif_mutex_id;
static osMutexId_t xy_pos_mutex_id;
static osMutexId_t disruptor_eng_id;
static osMutexId_t game_time_mutex_id;

// Timer defines
static osTimerId_t disruptor_usage_timer_id;
static osTimerId_t disruptor_recharge_timer_id;
static osTimerId_t game_timer_id;

RNG_HandleTypeDef* hrng_ptr;

// Data structures
static struct{
	double x;
	double y;
	double x_vel;
	double y_vel;
}xy_pos_dif;

static struct{
	double x;
	double y;
}xy_position;

static struct Cell** maze;

static int disruptor_energy;
static int game_score;
static int game_time;

static char* game_end_status[3][16] = {
		{"YOU WIN!"},
		{"YOU LOSE!"},
		{"Total score: "}
};

extern void initialise_monitor_handles(void);

void disruptor_timer_end(void* arg);
void disruptor_recharge(void* arg);
void game_time_decrement(void* arg);
void disruptor_task(void* arg);
void gyro_polling_task(void* arg);
void game_comp_task(void* arg);
void lcd_display_task(void* arg);
void green_led_task(void* arg);
void red_led_task(void* arg);


void ApplicationInit(RNG_HandleTypeDef* hrng){
	initialise_monitor_handles(); // Allows printf functionality
    LTCD__Init();
    LTCD_Layer_Init(0);
    LCD_Clear(0, LCD_COLOR_WHITE);
    Gyro_Init();

    hrng_ptr = hrng;
    disruptor_energy = drone.energy_store->max_energy;
    game_score = 0;

    // ITC init
    event_group_id = osEventFlagsNew(NULL);
    if(event_group_id == NULL) while(1);

    disruptor_start_sem_id = osSemaphoreNew(1, 0, NULL);
    if(disruptor_start_sem_id == NULL) while(1);

    disruptor_stop_sem_id = osSemaphoreNew(1, 0, NULL);
    if(disruptor_stop_sem_id == NULL) while(1);

    xy_pos_dif_mutex_id = osMutexNew(NULL);
    if(xy_pos_dif_mutex_id == NULL) while(1);

    xy_pos_mutex_id = osMutexNew(NULL);
    if(xy_pos_mutex_id == NULL) while(1);

    disruptor_eng_id = osMutexNew(NULL);
    if(disruptor_eng_id == NULL) while(1);

    game_time_mutex_id = osMutexNew(NULL);
    if(game_time_mutex_id == NULL) while(1);

    // timer init
    disruptor_usage_timer_id = osTimerNew(disruptor_timer_end, osTimerOnce, (void*)0, NULL);
    if(disruptor_usage_timer_id == NULL) while(1);

    disruptor_recharge_timer_id = osTimerNew(disruptor_recharge, osTimerPeriodic, (void*)0, NULL);
    if(disruptor_recharge_timer_id == NULL) while(1);

    game_timer_id = osTimerNew(game_time_decrement, osTimerPeriodic, (void*)0, NULL);
    if(game_timer_id == NULL) while(1);

    // task init
    disruptor_task_id = osThreadNew(disruptor_task, (void*)0, &disruptor_task_attr);
    if(disruptor_task_id == NULL) while(1);

    gyro_polling_task_id = osThreadNew(gyro_polling_task, (void*)0, &gyro_polling_task_attr);
    if(gyro_polling_task_id == NULL) while(1);

    game_comp_task_id = osThreadNew(game_comp_task, (void*)0, &game_comp_task_attr);
    if(game_comp_task_id == NULL) while(1);

    lcd_display_task_id = osThreadNew(lcd_display_task, (void*)0, &lcd_display_task_attr);
    if(lcd_display_task_id == NULL) while(1);

    green_led_task_id = osThreadNew(green_led_task, (void*)0, &green_led_task_attr);
    if(green_led_task_id == NULL) while(1);

    red_led_task_id = osThreadNew(red_led_task, (void*)0, &red_led_task_attr);
    if(red_led_task_id == NULL) while(1);
}

void read_xy_pos_dif(double* xy){
	osMutexAcquire(xy_pos_dif_mutex_id, osWaitForever);

	xy[0] = xy_pos_dif.x;
	xy[1] = xy_pos_dif.y;
	xy[2] = xy_pos_dif.x_vel;
	xy[3] = xy_pos_dif.y_vel;

	osMutexRelease(xy_pos_dif_mutex_id);
}

void write_xy_pos_dif(double* xy){
	osMutexAcquire(xy_pos_dif_mutex_id, osWaitForever);

	xy_pos_dif.x = xy[0];
	xy_pos_dif.y = xy[1];
	xy_pos_dif.x_vel = xy[2];
	xy_pos_dif.y_vel = xy[3];

	osMutexRelease(xy_pos_dif_mutex_id);
}

void read_xy_position(double* xy){
	osMutexAcquire(xy_pos_mutex_id, osWaitForever);

	xy[0] = xy_position.x;
	xy[1] = xy_position.y;

	osMutexRelease(xy_pos_mutex_id);
}

void write_xy_position(double* xy){
	osMutexAcquire(xy_pos_mutex_id, osWaitForever);

	xy_position.x = xy[0];
	xy_position.y = xy[1];

	osMutexRelease(xy_pos_mutex_id);
}

void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	int btn_state = HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);
	if(btn_state){
		// rising edge
		osSemaphoreRelease(disruptor_start_sem_id);


	}else{
		// falling edge
		uint32_t flags = osEventFlagsGet(event_group_id);
		if(flags & DISRUPTOR_STATE) osEventFlagsSet(event_group_id, DISRUPTOR_END);
//		disruptor_stop((void*)0);
	}

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void read_disruptor_energy(int* energy){
	osMutexAcquire(disruptor_eng_id, osWaitForever);

	*energy = disruptor_energy;

	osMutexRelease(disruptor_eng_id);
}

void write_disruptor_energy(int energy){
	osMutexAcquire(disruptor_eng_id, osWaitForever);

//	disruptor_energy = energy;
	if(energy >= drone.energy_store->max_energy){
		disruptor_energy = drone.energy_store->max_energy;
	}else if(energy <= 0){
		disruptor_energy = 0;
	}else{
		disruptor_energy = energy;
	}

	osMutexRelease(disruptor_eng_id);
}

void disruptor_recharge(void* arg){
	int energy;
	uint32_t flags = osEventFlagsGet(event_group_id);
	if(!(flags & DISRUPTOR_STATE)){
		read_disruptor_energy(&energy);
		write_disruptor_energy(energy + drone.energy_store->recharge_rate/10);
	}
}

void disruptor_timer_end(void* arg){
	(void) &arg;
	osEventFlagsSet(event_group_id, DISRUPTOR_END);
}

void disruptor_task(void* arg){
	(void) &arg;
	// start timer when button is pushed, with varying length
	int energy;
	uint32_t flags, start, stop;
	write_disruptor_energy(drone.energy_store->max_energy); //mJ
	osTimerStart(disruptor_recharge_timer_id, 100);
	while(1){
		osSemaphoreAcquire(disruptor_start_sem_id, osWaitForever);
		while(1){
			flags = osEventFlagsGet(event_group_id);
			if(!(flags & DISRUPTOR_STATE)){
				// button pushed down
				read_disruptor_energy(&energy);
				int ticks_to_run = (energy / drone.energy_store->recharge_rate)*1000;
				if(ticks_to_run >= drone.disruptor->min_activation_energy){
					start = osKernelGetTickCount();
					osTimerStart(disruptor_usage_timer_id, ticks_to_run);
					osEventFlagsSet(event_group_id, DISRUPTOR_STATE);
				}else{
					break;
				}

			}else{
				// check for event flag WAIT
				if(flags & DISRUPTOR_END){
					if(osTimerIsRunning(disruptor_usage_timer_id)) osTimerStop(disruptor_usage_timer_id);
					stop = osKernelGetTickCount();
					osEventFlagsClear(event_group_id, DISRUPTOR_END);
					read_disruptor_energy(&energy);
					energy -= (stop - start); // subtract total ticks ran from disruptor energy
					write_disruptor_energy(energy);
					osEventFlagsClear(event_group_id, DISRUPTOR_STATE);
					break;
				}
			}
			osDelay(100);
		}
	}
}

void gyro_polling_task(void* arg){
	(void) &arg;

	const double EXCESSIVE_ANGLE = 1.57;
	int zero_range = 40;
	double T = 0.02;
	float rads_ratio = 0.017453;
	double gyro_sensitivity = 17.50;
	double conversion_const = (rads_ratio * gyro_sensitivity)/1000;

	double xy_dif[4];

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
		// no idea why this is functional when backwards
		uint32_t flags = osEventFlagsGet(event_group_id);
		if(flags & DISRUPTOR_STATE){
			osDelay(100);
			continue;
		}

		y_read = Gyro_Get_X_Velocity();
		x_read = Gyro_Get_Y_Velocity();

		if(x_read <= zero_range && x_read >= -zero_range) x_read = 0;
		if(y_read <= zero_range && y_read >= -zero_range) y_read = 0;

		x_rads_per_sec = x_read * conversion_const;
		y_rads_per_sec = y_read * conversion_const;

		x_rads += x_rads_per_sec * T;
		y_rads += y_rads_per_sec * T;

		if(x_rads > EXCESSIVE_ANGLE || x_rads < -EXCESSIVE_ANGLE || y_rads > EXCESSIVE_ANGLE || y_rads < -EXCESSIVE_ANGLE){
			osEventFlagsSet(event_group_id, GAME_OVER_LOSS);
		}

		x_accel = physics.gravity * sin(x_rads);
		y_accel = physics.gravity * sin(y_rads);

		// if x or y collides with an object, reset its velocity. TRY ACCELL INSTEAD?
		if(flags & COLLISION_X){
			osEventFlagsClear(event_group_id, COLLISION_X);
			x_velocity = 0;
			x_accel = 0;
		}

		if(flags & COLLISION_Y){
			osEventFlagsClear(event_group_id, COLLISION_Y);
			y_velocity = 0;
			y_accel = 0;
		}

		if(flags & GAME_RESET){
//			osEventFlagsClear(event_group_id, GAME_RESET);
			x_velocity = 0;
			x_accel = 0;
			y_velocity = 0;
			y_accel = 0;
		}

		x_velocity += x_accel * T;
		y_velocity += y_accel * T;

		x_pos += x_velocity * T;
		y_pos += y_velocity * T;

		x_dif = x_pos - x_prev;
		y_dif = y_pos - y_prev;

		xy_dif[0] = x_dif;
		xy_dif[1] = y_dif;
		xy_dif[2] = x_velocity;
		xy_dif[3] = y_velocity;

		write_xy_pos_dif(xy_dif);

		x_prev = x_pos;
		y_prev = y_pos;

		osDelay(20);
	}
}

void generate_maze(void){
	maze = malloc(sizeof(Cell*) * maze_config.size->width);

	for(int i=0; i<maze_config.size->width; i++) {
	    maze[i] = malloc(sizeof(Cell) * maze_config.size->height);
	}
//	bool hard_edged = maze_config.hard_edged;
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

void get_nearby_walls(int x, int y, bool* walls){
	if(x < 0 || y < 0 || x >= maze_config.size->width || y >= maze_config.size->height){
		walls[0] = false;
		walls[1] = false;
		walls[2] = false;
		walls[3] = false;
		return;
	}

	bool left, right, top, bottom;
	bool h_e = maze_config.hard_edged;
	//left
	if(h_e && x == 0){
		left = true;
	}else{
		left = maze[x-1][y].right;
	}

	//right
	if(h_e && x == (maze_config.size->width - 1)){
		right = true;
	}else{
		right = maze[x][y].right;
	}

	//top
	if(h_e && y == 0){
		top = true;
	}else{
		top = maze[x][y].top;
	}

	//bottom
	if(h_e && y == (maze_config.size->height - 1)){
		bottom = true;
	}else{
		bottom = maze[x][y+1].top;
	}

	walls[0] = left;
	walls[1] = right;
	walls[2] = top;
	walls[3] = bottom;
}

void generate_features(void){
	uint32_t rand;
	maze_config.waypoints->locations_list = malloc(sizeof(xy_pair) * maze_config.waypoints->number);
	for(int i=0; i<maze_config.waypoints->number;i++){
		HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
		maze_config.waypoints->locations_list[i].x = (rand % maze_config.size->width)*maze_config.cell_size + maze_config.cell_size/2;

		HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
		maze_config.waypoints->locations_list[i].y = (rand % maze_config.size->height)*maze_config.cell_size + maze_config.cell_size/2;
		maze_config.waypoints->locations_list[i].collected = false;
	}

	maze_config.holes->locations_list = malloc(sizeof(xy_pair) * maze_config.holes->number);
	for(int i=0; i<maze_config.holes->number;i++){
		HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
		maze_config.holes->locations_list[i].x = (rand % maze_config.size->width)*maze_config.cell_size + maze_config.cell_size/2;

		HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
		maze_config.holes->locations_list[i].y = (rand % maze_config.size->height)*maze_config.cell_size + maze_config.cell_size/2;
		maze_config.holes->locations_list[i].collected = false;
	}
}

void game_time_decrement(void* arg){
	osMutexAcquire(game_time_mutex_id, osWaitForever);

	game_time -= TICKRATE;
	if(game_time <= 0){
		game_time = 0;
		osTimerStop(game_timer_id);
		osEventFlagsSet(event_group_id, GAME_OVER_LOSS);
	}

	osMutexRelease(game_time_mutex_id);
}

void read_game_time(int* time){
	osMutexAcquire(game_time_mutex_id, osWaitForever);

	*time = game_time;

	osMutexRelease(game_time_mutex_id);
}

void write_game_time(int time){
	osMutexAcquire(game_time_mutex_id, osWaitForever);

	game_time = time;

	osMutexRelease(game_time_mutex_id);
}

void game_end(bool win){
	osTimerStop(game_timer_id);
	if(win){
		osEventFlagsSet(event_group_id, GAME_OVER_WIN);
	}else{
		osEventFlagsSet(event_group_id, GAME_OVER_LOSS);
	}
	osEventFlagsClear(event_group_id, MAZE_GENERATED);
	// graphic

	osDelay(3000);
	if(!win) game_score = 0;
	write_disruptor_energy(drone.energy_store->max_energy);
	osEventFlagsSet(event_group_id, GAME_RESET);
}

void game_comp_task(void* arg){
	(void) &arg;

	while(1){
		osEventFlagsClear(event_group_id, GAME_OVER_LOSS | GAME_OVER_WIN | GAME_RESET);

		// generate maze first
		generate_maze();
		generate_features();
		double xy_dif[4];
		double xy_pos[2];

		// random ball spawn location, checking for spawning on a hole
		uint32_t rand;
		bool invalid_spawn_pos = true;
		while(invalid_spawn_pos){
			invalid_spawn_pos = false;
			HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
			xy_pos[0] = (rand % maze_config.size->width)*maze_config.cell_size + maze_config.cell_size/2;
			HAL_RNG_GenerateRandomNumber(hrng_ptr, &rand);
			xy_pos[1] = (rand % maze_config.size->width)*maze_config.cell_size + maze_config.cell_size/2;

			for(int i=0; i<maze_config.holes->number; i++){
				if(maze_config.holes->locations_list[i].x == xy_pos[0] && maze_config.holes->locations_list[i].y == xy_pos[1]){
					invalid_spawn_pos = true;
				}
			}
		}
		write_xy_position(xy_pos);
		write_game_time(TOTAL_TIME);
		osTimerStart(game_timer_id, TICKRATE);
		osEventFlagsSet(event_group_id, MAZE_GENERATED);

		bool walls[4];
		int x_collision, y_collision;
		int radius = drone.diameter/2;
		int cell_size = maze_config.cell_size;
		double x_sub_pos, y_sub_pos;
		double x_prev_pos_dif = 0;
		double y_prev_pos_dif = 0;
		uint32_t flags;
		bool disruptor_activated = false;
		while(1){
			flags = osEventFlagsGet(event_group_id);
			if(flags & DISRUPTOR_STATE){
				xy_pos[0] += x_prev_pos_dif;
				xy_pos[1] += y_prev_pos_dif;

				disruptor_activated = true;
			}else{
				x_collision = 0;
				y_collision = 0;
				read_xy_pos_dif(xy_dif);
				read_xy_position(xy_pos);

				x_prev_pos_dif = xy_dif[0];
				y_prev_pos_dif = xy_dif[1];

				int cur_x_cell = xy_pos[0]/cell_size;
				int cur_y_cell = xy_pos[1]/cell_size;

				if(cur_x_cell < 0 || cur_x_cell > maze_config.size->width-1 || cur_y_cell < 0 || cur_y_cell > maze_config.size->height-1){
					osEventFlagsSet(event_group_id, GAME_OVER_LOSS);
				}

				// left, right, top, bottom
				get_nearby_walls(cur_x_cell, cur_y_cell, walls);

				// future optimization: dont do calculations for cells that dont have walls
				// OR if we are not headed in the direction of the wall (use +- velocity)
				x_sub_pos = fmod(xy_pos[0], cell_size);
				y_sub_pos = fmod(xy_pos[1], cell_size);

				if(disruptor_activated){
					// do something to teleport out of walls
					if(walls[0] && x_sub_pos < radius){
						xy_pos[0] = (int)(cur_x_cell*maze_config.cell_size) + radius;
					}else if(walls[1] && x_sub_pos > (cell_size - radius)){
						xy_pos[0] = (int)((cur_x_cell)*maze_config.cell_size) + (cell_size - radius);
					}

					if(walls[2] && y_sub_pos < radius){
						xy_pos[1] = (int)(cur_y_cell*maze_config.cell_size) + radius;
					}else if(walls[3] && y_sub_pos > (cell_size - radius)){
						xy_pos[1] = (int)((cur_y_cell)*maze_config.cell_size) + (cell_size - radius);
					}

					write_xy_position(xy_pos);
					disruptor_activated = false;
				}

				if(xy_dif[2] < 0 && walls[0]){
					// negative x velocity
					if(x_sub_pos + xy_dif[0] - radius < 0){
						// x left wall collision
						osEventFlagsSet(event_group_id, COLLISION_X);
						x_collision = 1;
					}

				}else if(xy_dif[2] > 0 && walls[1]){
					// positive x velocity
					if(x_sub_pos + xy_dif[0] + radius > cell_size){
						// x right wall collision
						// in this case, ignore any more positive position changes in x direction?
						// until something happens....?
						// until x becomes negative velocity, OR Y leaves this cell?
						osEventFlagsSet(event_group_id, COLLISION_X);
						x_collision = 1;
					}
				}

				if(xy_dif[3] < 0 && walls[2]){
					// negative y velocity
					if(y_sub_pos + xy_dif[1] - radius < 0){
						// y top wall collision
						osEventFlagsSet(event_group_id, COLLISION_Y);
						y_collision = 1;
					}

				}else if(xy_dif[3] > 0 && walls[3]){
					// positive y velocity
					if(y_sub_pos + xy_dif[1] + radius > cell_size){
						// y bottom wall collision
						osEventFlagsSet(event_group_id, COLLISION_Y);
						y_collision = 1;
					}
				}

				if(!x_collision) xy_pos[0] += xy_dif[0];
				if(!y_collision) xy_pos[1] += xy_dif[1];


			}

			write_xy_position(xy_pos);
			// euclidean distance of each waypoint to determine if collected.
			bool win = false;
			double distance;
			xy_pair* loc_list = maze_config.waypoints->locations_list;
			for(int i=0; i<maze_config.waypoints->number; i++){
				if(!loc_list[i].collected){
					distance = sqrt(pow(loc_list[i].x - xy_pos[0], 2) + pow(loc_list[i].y - xy_pos[1], 2));
					if(distance < radius + 1){
						loc_list[i].collected = true;
						game_score += 1;
					}
					break;
				}
				if(i == maze_config.waypoints->number-1 && loc_list[i].collected){
					win = true;
				}
			}

			if(win){
				game_end(true);
				break;
			}

			if(!(osEventFlagsGet(event_group_id) & DISRUPTOR_STATE)){
				bool hole_traversed = false;
				loc_list = maze_config.holes->locations_list;
				for(int i=0; i<maze_config.holes->number; i++){
					distance = sqrt(pow(loc_list[i].x - xy_pos[0], 2) + pow(loc_list[i].y - xy_pos[1], 2));
					if(distance < radius){
						hole_traversed = true;
						break;
					}
				}

				if((osEventFlagsGet(event_group_id) & GAME_OVER_LOSS) || hole_traversed){
					// game fail
					game_end(false);
					break;
				}
			}

			osDelay(100);
		}
	}
}

void lcd_draw_maze(bool disruptor_active){
	int color = disruptor_active ? LCD_COLOR_GREY : LCD_COLOR_BLACK;
	int cell_size = maze_config.cell_size;
	for(int i=0; i<maze_config.size->width; i++){
		for(int j=0; j<maze_config.size->height; j++){
			if(maze[i][j].top){
				LCD_Draw_Horizontal_Line(i*cell_size, j*cell_size, cell_size, color);
			}

			if(maze[i][j].right){
				LCD_Draw_Vertical_Line(i*cell_size + cell_size, j*cell_size, cell_size, color);
			}
		}
	}

	if(maze_config.hard_edged){
		LCD_Draw_Vertical_Line(0, 0, cell_size*maze_config.size->height, LCD_COLOR_BLACK);
		LCD_Draw_Horizontal_Line(0, cell_size*maze_config.size->height, cell_size*maze_config.size->width, LCD_COLOR_BLACK);
	}

	// draw features
	color = LCD_COLOR_RED;
	xy_pair* loc_list = maze_config.waypoints->locations_list;
	for(int i=0; i<maze_config.waypoints->number; i++){
		if(!maze_config.waypoints->locations_list[i].collected){
			LCD_Draw_Circle_Fill(loc_list[i].x, loc_list[i].y, maze_config.waypoints->diameter/2, color);
			color = LCD_COLOR_GREY;
		}
	}

	loc_list = maze_config.holes->locations_list;
	for(int i=0; i<maze_config.holes->number; i++){
		LCD_Draw_Circle_Fill(loc_list[i].x, loc_list[i].y, maze_config.holes->diameter/2, LCD_COLOR_BLACK);
	}
}

void lcd_display_task(void* arg){
	(void) &arg;
	LCD_Clear(0, LCD_COLOR_WHITE);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);

	int radius = drone.diameter/2;
	double xy_pos[2];
	char time_str[8];
	int game_time;
	int menu_x_offset = 10;
	int menu_y_offset = (LCD_PIXEL_HEIGHT/2)-20;
	while(1){
		uint32_t flags = osEventFlagsGet(event_group_id);
		bool disruptor_active = flags & DISRUPTOR_STATE;
		lcd_draw_maze(disruptor_active);

		// game timer display
		read_game_time(&game_time);
		game_time = (int)game_time/1000;
		itoa(game_time, time_str, 10);
		if(xy_pos[0] > 1 && xy_pos[0] < 35 && xy_pos[1] > 1 && xy_pos[1] < 25){
			LCD_Draw_Rectangle_Fill(1, LCD_PIXEL_HEIGHT-25, 35, 25, LCD_COLOR_WHITE);
			LCD_DisplayString(5, LCD_PIXEL_HEIGHT-20, time_str);
		}else{
			LCD_Draw_Rectangle_Fill(1, 1, 35, 25, LCD_COLOR_WHITE);
			LCD_DisplayString(5, 5, time_str);
		}

		if(flags & GAME_OVER_LOSS || flags & GAME_OVER_WIN){
			bool win = flags & GAME_OVER_WIN;
			LCD_Draw_Rectangle_Fill(menu_x_offset, menu_y_offset, 220, 69, LCD_COLOR_BLACK); // background frame
			LCD_Draw_Rectangle_Fill(menu_x_offset+2, menu_y_offset+2, 216, 65, LCD_COLOR_WHITE); // foreground white
			LCD_DisplayString(menu_x_offset+50, menu_y_offset+4, *game_end_status[!win]); // win/lose
			LCD_DisplayString(menu_x_offset+14, menu_y_offset+24, *game_end_status[2]); // score
			LCD_DisplayNumber(menu_x_offset+194, menu_y_offset+24, game_score);
		}
		osEventFlagsWait(event_group_id, MAZE_GENERATED, osFlagsNoClear, osWaitForever);

		read_xy_position(xy_pos);
		if(!(flags & GAME_OVER_LOSS || flags & GAME_OVER_WIN)){
			if(disruptor_active){
				LCD_Draw_Circle_Fill((int)xy_pos[0], (int)xy_pos[1], radius, LCD_COLOR_MAGENTA);
			}else{
				LCD_Draw_Circle_Fill((int)xy_pos[0], (int)xy_pos[1], radius, LCD_COLOR_BLUE);
			}
		}

		osDelay(100);
		LCD_Clear(0, LCD_COLOR_WHITE);
	}
}

void green_led_task(void* arg){
	(void) &arg;
	int max_energy = drone.energy_store->max_energy;
	int on_time_ms, energy;
	const int TOTAL_PERIOD = 20;
	while(1){
        read_disruptor_energy(&energy);

        if(energy < max_energy){
			// Calculate the ON time based on the energy percentage
			on_time_ms = (int)((float)energy / max_energy * TOTAL_PERIOD); // 20ms is the total period (ON + OFF)

			// Turn ON the LED
			HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);
			osDelay(on_time_ms);

			// Turn OFF the LED
			HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_RESET);
			osDelay(TOTAL_PERIOD - on_time_ms); // Subtract ON time from total period to get OFF time
        }else{
        	HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);
        	osDelay(100);
        }
	}
}

void red_led_task(void* arg){
	(void) &arg;
	int min_energy = drone.disruptor->min_activation_energy;
	int half_period, energy;
	while(1){
        read_disruptor_energy(&energy);

        if(energy < min_energy){
        	half_period = (min_energy - energy)/20;
        	if(half_period < 2) half_period = 2;
			// Turn ON the LED
			HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
			osDelay(half_period);

			// Turn OFF the LED
			HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
			osDelay(half_period); // Subtract ON time from total period to get OFF time
        }else{
        	HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
        	osDelay(100);
        }
	}
}
