/*
 * ApplicationCode.c
 *
 *  Created on: Nov 14, 2023
 *      Author: xcowa
 */

#include "ApplicationCode.h"
#include "Gyro_Driver.h"
#include "LCD_Driver.h"

#define TASK_STACK_LEN 64
#define TASK_DELAY_PERIOD 100U
#define BTN_TIMEOUT_MS 1000
#define DECREMENT 0
#define INCREMENT 1
#define SPEED_CHANGE_UNIT 5
#define SPEED_LIMIT 75
#define SPEED_LIMIT_TURN 45
#define COLLISION_WARN_PERIOD_MS 5000

#define PRESS_EVENT 		       0x00000001
#define HOLD_EVENT  		       0x00000002
#define DIRECTION_CHANGE_EVENT     0x00000004
#define SPEED_DATA_READY	       0x00000008
#define DIRECTION_DATA_READY       0x00000010
#define SPEED_VIOLATION		       0x00000020
#define DIRECTION_VIOLATION        0x00000040
#define DIRECTION_VIOLATION_CLEAR  0x00000080


typedef enum{
	decrement = 0,
	increment
}speed_change_t;

typedef enum{
	hard_left = 0,
	left,
	straight,
	right,
	hard_right
}vehicle_direction_enum;

static char* direction_str[5][12] = {
		"Hard left",
		"Left",
		"Straight",
		"Right",
		"Hard right"
};

typedef enum{
	CC_FAST = -1000,
	CC_SLOW = -100,
	ZERO = 0,
	C_SLOW = 100,
	C_FAST = 1000
}Gyro_Range_Enum;

static struct{
	int speed;
	int increments;
	int decrements;
}speed_setpoint_data;

static struct{
	vehicle_direction_enum v_direction;
	int left_turns;
	int right_turns;
}vehicle_direction_data;

static StaticSemaphore_t ss_sem_cb;
static osSemaphoreId_t ss_sem_id;
static osSemaphoreAttr_t s_attr = {
		.cb_mem = &ss_sem_cb,
		.cb_size = sizeof(ss_sem_cb),
		.name = "ss_sem"
};

static osEventFlagsId_t vehicle_event_group;
static StaticEventGroup_t event_cb;
static osEventFlagsAttr_t e_attr = {
//		.cb_mem = &event_cb,
//		.cb_size = sizeof(event_cb),
		.name = "vehicle_event_group"
};


// Speed setpoint resources
static int btn_timeout = 0;

static osTimerId_t ss_timer_id;
static StaticTimer_t ss_static_timer;
const osTimerAttr_t timer_attr = {
	.name =  "ss_timer",
	.cb_mem = &ss_static_timer,
	.cb_size = sizeof(ss_static_timer)
};

static StaticTask_t speed_setpoint_task_cb;
static uint32_t speed_setpoint_stack[TASK_STACK_LEN];
static osThreadId_t speed_setpoint_task_id;
const osThreadAttr_t speed_setpoint_attr = {
		.cb_mem = &speed_setpoint_task_cb,
		.cb_size = sizeof(speed_setpoint_task_cb),
		.stack_mem = &speed_setpoint_stack[0],
		.stack_size = sizeof(speed_setpoint_stack),
		.name = "speed_setpoint_task"
};

static osMutexId_t speed_setpoint_mutex_id;
static osMutexAttr_t speed_setpoint_m_attr = {
		.cb_mem = NULL,
		.cb_size = 0U
};


// Vehicle direction resources
static StaticTask_t vehicle_direction_task_cb;
static uint32_t vehicle_direction_stack[TASK_STACK_LEN];
static osThreadId_t vehicle_direction_task_id;
const osThreadAttr_t vehicle_direction_attr = {
		.cb_mem = &vehicle_direction_task_cb,
		.cb_size = sizeof(vehicle_direction_task_cb),
		.stack_mem = &vehicle_direction_stack[0],
		.stack_size = sizeof(vehicle_direction_stack),
		.name = "vehicle_direction_task"
};

static osMutexId_t vehicle_direction_mutex_id;
static osMutexAttr_t vehicle_direction_m_attr = {
		.cb_mem = NULL,
		.cb_size = 0U
};

// Vehicle monitor resources
static StaticTask_t vehicle_monitor_task_cb;
static uint32_t vehicle_monitor_stack[TASK_STACK_LEN];
static osThreadId_t vehicle_monitor_task_id;
const osThreadAttr_t vehicle_monitor_attr = {
		.cb_mem = &vehicle_monitor_task_cb,
		.cb_size = sizeof(vehicle_monitor_task_cb),
		.stack_mem = &vehicle_monitor_stack[0],
		.stack_size = sizeof(vehicle_monitor_stack),
		.name = "vehicle_monitor_task"
};

static osTimerId_t vm_timer_id;
static StaticTimer_t vm_static_timer;
const osTimerAttr_t vm_timer_attr = {
	.name =  "vm_timer",
	.cb_mem = &vm_static_timer,
	.cb_size = sizeof(vm_static_timer)
};

static StaticTask_t led_output_task_cb;
static uint32_t led_output_stack[TASK_STACK_LEN];
static osThreadId_t led_output_task_id;
const osThreadAttr_t led_output_attr = {
		.cb_mem = &led_output_task_cb,
		.cb_size = sizeof(led_output_task_cb),
		.stack_mem = &led_output_stack[0],
		.stack_size = sizeof(led_output_stack),
		.name = "led_output_task"
};

static StaticTask_t lcd_display_task_cb;
static uint32_t lcd_display_stack[TASK_STACK_LEN];
static osThreadId_t lcd_display_task_id;
const osThreadAttr_t lcd_display_attr = {
		.cb_mem = &lcd_display_task_cb,
		.cb_size = sizeof(lcd_display_task_cb),
		.stack_mem = &lcd_display_stack[0],
		.stack_size = sizeof(lcd_display_stack),
		.name = "lcd_display_task"
};

void press_hold_timeout(void* arg);
void speed_setpoint_task(void* arg);
void vehicle_direction_task(void* arg);
void vehicle_monitor_task(void* arg);
void dir_violation_timeout(void* arg);
void led_output_task(void* arg);
void lcd_display_task(void* arg);

void ApplicationInit(void)
{
	LTCD__Init();
    LTCD_Layer_Init(0);
    Gyro_Init();

    //write default data structure values

    // Global event group
    vehicle_event_group = osEventFlagsNew(&e_attr);
	if(vehicle_event_group == NULL) while(1);

	//Setpoint resources
    ss_timer_id = osTimerNew(press_hold_timeout, osTimerOnce, (void*)0, &timer_attr);
	if(ss_timer_id == NULL) while(1);

	speed_setpoint_mutex_id = osMutexNew(&speed_setpoint_m_attr);
	if(speed_setpoint_mutex_id == NULL) while(1);

	ss_sem_id = osSemaphoreNew(1, 0, &s_attr);
	if(ss_sem_id == NULL) while(1);

	speed_setpoint_task_id = osThreadNew(speed_setpoint_task, (void*)0, &speed_setpoint_attr);
	if(speed_setpoint_task_id == NULL) while(1);

	//Vehicle direction resources
	vehicle_direction_mutex_id = osMutexNew(&vehicle_direction_m_attr);
	if(vehicle_direction_mutex_id == NULL) while(1);

	vehicle_direction_task_id = osThreadNew(vehicle_direction_task, (void*)0, &vehicle_direction_attr);
	if(vehicle_direction_task_id == NULL) while(1);

	//Vehicle monitor resources
	vm_timer_id = osTimerNew(dir_violation_timeout, osTimerOnce, (void*)0, &vm_timer_attr);
	if(vm_timer_id == NULL) while(1);

	vehicle_monitor_task_id = osThreadNew(vehicle_monitor_task, (void*)0, &vehicle_monitor_attr);
	if(vehicle_monitor_task_id == NULL) while(1);

	//Led output resources
	led_output_task_id = osThreadNew(led_output_task, (void*)0, &led_output_attr);
	if(led_output_task_id == NULL) while(1);

	//Lcd display resources
	lcd_display_task_id = osThreadNew(lcd_display_task, (void*)0, &lcd_display_attr);
	if(lcd_display_task_id == NULL) while(1);
}



/*
 * ========================= Speed Setpoint Task =========================
 */

int get_btn_state(){
	return HAL_GPIO_ReadPin(USR_BTN_PORT, USR_BTN_PIN);
}

void write_speed_setpoint(speed_change_t type){
	osMutexAcquire(speed_setpoint_mutex_id, osWaitForever);

	if(type == decrement){
		speed_setpoint_data.speed -= SPEED_CHANGE_UNIT;
		speed_setpoint_data.decrements += 1;
	}else{
		speed_setpoint_data.speed += SPEED_CHANGE_UNIT;
		speed_setpoint_data.increments += 1;
	}

	osMutexRelease(speed_setpoint_mutex_id);
}

int read_speed_setpoint(void){
	osMutexAcquire(speed_setpoint_mutex_id, osWaitForever);

	int speed = speed_setpoint_data.speed;

	osMutexRelease(speed_setpoint_mutex_id);
	return speed;
}

void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	int btn_state = get_btn_state();
	// falling vs. rising edge
	if(btn_state){
		// start timer
		osStatus_t status = osTimerStart(ss_timer_id, BTN_TIMEOUT_MS);
		if(status != osOK) while(1);

	}else{
		// stop timer, write timeout value
		osStatus_t status = osTimerStop(ss_timer_id);
		if(status != osOK) while(1);

		// write correct event to event group
		if(btn_timeout)
			osEventFlagsSet(vehicle_event_group, HOLD_EVENT);
		else
			osEventFlagsSet(vehicle_event_group, PRESS_EVENT);

		btn_timeout = 0;

		status = osSemaphoreRelease(ss_sem_id);
		if(status != osOK) while(1);
	}

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void press_hold_timeout(void* arg){
	(void) &arg;
	btn_timeout = 1;
}

void speed_setpoint_task(void* arg){
	(void) &arg;
	while(1){
		osStatus_t status = osSemaphoreAcquire(ss_sem_id, osWaitForever);
		if(status != osOK) while(1);

		// check for either press or hold event, write new value to setpoint data
		uint32_t flags = osEventFlagsGet(vehicle_event_group);
		if(flags & PRESS_EVENT){
			osEventFlagsClear(vehicle_event_group, PRESS_EVENT);
			write_speed_setpoint(increment);
		}

		if(flags & HOLD_EVENT){
			osEventFlagsClear(vehicle_event_group, HOLD_EVENT);
			write_speed_setpoint(decrement);
		}

		// set event flag for new value for vehicle mon task
		osEventFlagsSet(vehicle_event_group, SPEED_DATA_READY);
	}
}



/*
 * ========================= Vehicle Direction Task =========================
 */

void write_v_direction(vehicle_direction_enum direction){
	osMutexAcquire(vehicle_direction_mutex_id, osWaitForever);

	vehicle_direction_data.v_direction = direction;
	if(direction == left || direction == hard_left){
		vehicle_direction_data.left_turns += 1;
	}else{
		vehicle_direction_data.right_turns += 1;
	}

	osMutexRelease(vehicle_direction_mutex_id);
}

vehicle_direction_enum read_v_direction(void){
	osMutexAcquire(vehicle_direction_mutex_id, osWaitForever);

	vehicle_direction_enum direction = vehicle_direction_data.v_direction;

	osMutexRelease(vehicle_direction_mutex_id);
	return direction;
}

int16_t read_gyro_velocity(){
	return Gyro_Get_Velocity();
}

void vehicle_direction_task(void* arg){
	(void) &arg;
	while(1){
		int16_t velocity = read_gyro_velocity();
		vehicle_direction_enum d;

		if(velocity > C_FAST){
			d = hard_right;
		}else if(velocity <= C_FAST && velocity > ZERO){
			d = right;
		}else if(velocity <= ZERO && velocity > CC_FAST){
			d = left;
		}else if(velocity <= CC_FAST){
			d = hard_left;
		}

		write_v_direction(d);
		osEventFlagsSet(vehicle_event_group, DIRECTION_DATA_READY);

		osDelay(TASK_DELAY_PERIOD);
	}
}



/*
 * ========================= Vehicle Monitor Task =========================
 */

void dir_violation_timeout(void* arg){
	(void) &arg;
	osEventFlagsSet(vehicle_event_group, DIRECTION_VIOLATION);
//	HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);
}

void vehicle_monitor_task(void* arg){
	(void) &arg;
	int speed = 0;
	vehicle_direction_enum direction = left;
	vehicle_direction_enum prev_direction = left;

	while(1){
		uint32_t flags = osEventFlagsWait(vehicle_event_group, (SPEED_DATA_READY | DIRECTION_DATA_READY), osFlagsWaitAny, osWaitForever);

//		uint32_t flags = osEventFlagsGet(vehicle_event_group);

		if(flags & DIRECTION_DATA_READY){
			direction = read_v_direction();
//			osEventFlagsClear(vehicle_event_group, DIRECTION_DATA_READY);

			// check for direction violation, when new direction data is ready
			if(osTimerIsRunning(vm_timer_id) && direction != prev_direction){
					osTimerStop(vm_timer_id);
					osEventFlagsSet(vehicle_event_group, DIRECTION_VIOLATION_CLEAR);
//					HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_RESET);
			}else if(direction == prev_direction){
				osTimerStart(vehicle_event_group, COLLISION_WARN_PERIOD_MS);
			}
		}

		if(flags & SPEED_DATA_READY){
			speed = read_speed_setpoint();
//			osEventFlagsClear(vehicle_event_group, SPEED_DATA_READY);

			// check for speed violation, when new speed data is ready
			if(direction == left || direction == right){
				if(speed > SPEED_LIMIT)
					osEventFlagsSet(vehicle_event_group, SPEED_VIOLATION);
			}else{
				if(speed > SPEED_LIMIT_TURN)
					osEventFlagsSet(vehicle_event_group, SPEED_VIOLATION);
			}
		}

		prev_direction = direction;
	}
}



/*
 * ========================= LED Output Task =========================
 */

void led_output_task(void* arg){
	(void) &arg;
	while(1){
		uint32_t flags = osEventFlagsWait(vehicle_event_group, (DIRECTION_VIOLATION | DIRECTION_VIOLATION_CLEAR), osFlagsNoClear, osWaitForever);

//		osEventFlagsGet(vehicle_event_group);

		if(flags & DIRECTION_VIOLATION){
//			osEventFlagsClear(vehicle_event_group, DIRECTION_VIOLATION);
			HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);
		}

		if(flags & DIRECTION_VIOLATION_CLEAR){
//			osEventFlagsClear(vehicle_event_group, DIRECTION_VIOLATION_CLEAR);
			HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_RESET);
		}
	}
}



/*
 * ========================= LED Output Task =========================
 */

void lcd_display_task(void* arg){
	(void) &arg;
	LCD_Clear(0, LCD_COLOR_WHITE);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);
	while(1){
		int speed = read_speed_setpoint();
		vehicle_direction_enum direction = read_v_direction();

		char* test_str = "test";
//		LCD_DisplayString(170, 120, );
		LCD_DisplayString(170, 120, test_str);
		LCD_DisplayNumber(170, 180, (uint16_t)speed);
//		LCD_Clear(0, LCD_COLOR_WHITE);
		osDelay(TASK_DELAY_PERIOD);
	}
}

void RunDemoForLCD(void)
{
	LCD_Clear(0,LCD_COLOR_WHITE);
	QuickDemo();
}

