/*
 * Application_Code.c
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#include "Application_Code.h"

#define Q_SIZE 16
#define BTN_INPUT 0
#define GYRO_INPUT 1
#define USR_BTN_FLAG 0x1
#define TASK_STACK_LEN 128
#define TIMER_MS_PERIOD 100U
//static int systick_count = 0;

int usr_btn_state = 0;

static osSemaphoreId_t sId;
static StaticSemaphore_t sem_cb;
static osSemaphoreAttr_t s_attr = {
		.cb_mem = &sem_cb,
		.cb_size = sizeof(sem_cb),
		.name = "sem_gyro_input"
};

static osEventFlagsId_t eId;
static StaticEventGroup_t event_cb;
static osEventFlagsAttr_t e_attr = {
		.cb_mem = &event_cb,
		.cb_size = sizeof(event_cb),
		.name = "btn_event_group"
};

static osMessageQueueId_t qId;
static StaticMessageBuffer_t msg_buf;
static osMessageQueueAttr_t q_attr = {
		.cb_mem = &msg_buf,
		.cb_size = sizeof(msg_buf),
		.name = "msg_q"
};

static osMemoryPoolId_t mId;

struct message{
	int msg_type;
	int16_t velocity;
	int btn_state;
	struct message* self;
};

static osTimerId_t periodic_id;
static StaticTimer_t static_timer;
const osTimerAttr_t timer_attr = {
	.name =  "app_timer",
	.cb_mem = &static_timer,
	.cb_size = sizeof(static_timer)
};

static StaticTask_t gyro_input_cb;
static uint32_t gyro_input_stack[TASK_STACK_LEN];
static osThreadId_t gyro_input_task_id;
const osThreadAttr_t gyro_thread_attr = {
		.cb_mem = &gyro_input_cb,
		.cb_size = sizeof(gyro_input_cb),
		.stack_mem = &gyro_input_stack[0],
		.stack_size = sizeof(gyro_input_stack),
		.name = "gyro_input_task"
};

static StaticTask_t btn_input_cb;
static uint32_t btn_input_stack[TASK_STACK_LEN];
static osThreadId_t btn_input_task_id;
const osThreadAttr_t btn_thread_attr = {
		.cb_mem = &btn_input_cb,
		.cb_size = sizeof(btn_input_cb),
		.stack_mem = &btn_input_stack[0],
		.stack_size = sizeof(btn_input_stack),
		.name = "btn_input_task"
};

static StaticTask_t led_output_cb;
static uint32_t led_output_stack[TASK_STACK_LEN];
static osThreadId_t led_output_task_id;
const osThreadAttr_t led_thread_attr = {
		.cb_mem = &led_output_cb,
		.cb_size = sizeof(led_output_cb),
		.stack_mem = &led_output_stack[0],
		.stack_size = sizeof(led_output_stack),
		.name = "btn_input_task"
};

void timer_callback(void* arg);
void gyro_input_task(void* arg);
void btn_input_task(void* arg);
void green_led_task(void* arg);

/*
 * @brief Initialize application.
 */
void init_app(){
	Gyro_Init();

	//init message queue
	qId = osMessageQueueNew(Q_SIZE, sizeof(struct message*), NULL);
	if(qId == NULL) while(1);

	mId = osMemoryPoolNew(Q_SIZE, sizeof(struct message), NULL);
	if(mId == NULL) while(1);

	//init semaphore
	sId = osSemaphoreNew(1, 0, &s_attr);
	if(sId == NULL) while(1);

	//init event group
	eId = osEventFlagsNew(&e_attr);
	if(eId == NULL) while(1);

	//init gyro input task
	gyro_input_task_id = osThreadNew(gyro_input_task, (void*)0, &gyro_thread_attr);
	if(gyro_input_task_id == NULL) while(1);

	//init button input task
	btn_input_task_id = osThreadNew(btn_input_task, (void*)0, &btn_thread_attr);
	if(btn_input_task_id == NULL) while(1);

	//init led output task
	led_output_task_id = osThreadNew(green_led_task, (void*)0, &led_thread_attr);
	if(led_output_task_id == NULL) while(1);

	//init timer
	periodic_id = osTimerNew(timer_callback, osTimerPeriodic, (void*)0, &timer_attr);
	if(periodic_id == NULL) while(1);

	//start timer
	osStatus_t status = osTimerStart(periodic_id, TIMER_MS_PERIOD);
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
void drive_leds(int16_t velocity, int btn_state){
	if(velocity >= CC_SLOW && btn_state){
		//red led OR button
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
	}else{
		//near zero range
		HAL_GPIO_WritePin(LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
	}

	if(velocity < CC_FAST || btn_state){
		//green led AND button
		HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_SET);

	}else{
		//near zero range
		HAL_GPIO_WritePin(LED_PORT, GRN_LED_PIN, GPIO_PIN_RESET);
	}

}

/*
 * @brief External interrupt 0 request handler. User button 0 causes this interrupt, and will retrieve button state when pressed/unpressed.
 */
void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	get_btn_state();
	osEventFlagsSet(eId, USR_BTN_FLAG);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void btn_input_task(void* arg){
	(void) &arg;
	while(1){
		osEventFlagsWait(eId, USR_BTN_FLAG, osFlagsWaitAny, osWaitForever);

		struct message* pMem = (struct message *) osMemoryPoolAlloc(mId, osWaitForever);
		pMem->msg_type = BTN_INPUT;
		pMem->btn_state = usr_btn_state;
		pMem->self = pMem;

		osStatus_t status = osMessageQueuePut(qId, &pMem, 0, osWaitForever);
		if(status != osOK) while(1);
	}
}

void timer_callback(void* arg){
	(void) &arg;

	osStatus_t status = osSemaphoreRelease(sId);
	if(status != osOK) while(1);
}

void gyro_input_task(void* arg){
	(void) &arg;
	while(1){
		osStatus_t status = osSemaphoreAcquire(sId, osWaitForever);
		if(status != osOK) while(1);

		//use one message-sized pool for each message? Add pool ptr to message queue?
		struct message* pMem = (struct message *) osMemoryPoolAlloc(mId, osWaitForever);
		pMem->msg_type = GYRO_INPUT;
		pMem->velocity = read_gyro_velocity();
		pMem->self = pMem;

		status = osMessageQueuePut(qId, &pMem, 0, osWaitForever);
		if(status != osOK) while(1);
	}
}

void green_led_task(void* arg){
	(void) &arg;
	struct message* msg;
	int16_t velocity = 0;
	int btn_state = 0;
	while(1){
		osStatus_t status = osMessageQueueGet(qId, &msg, NULL, osWaitForever);
		if(status != osOK) while(1);

		if(msg->msg_type == BTN_INPUT){
			btn_state = msg->btn_state;
		}else{
			//gyro input
			velocity = msg->velocity;
		}

		drive_leds(velocity, btn_state);

		status = osMemoryPoolFree(mId, msg);
		if(status != osOK) while(1);
	}
}

