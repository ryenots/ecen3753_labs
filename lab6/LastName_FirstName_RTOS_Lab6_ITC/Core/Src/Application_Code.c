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
//static int systick_count = 0;

int usr_btn_state = 0;

static osSemaphoreAttr_t sId = NULL;
static StaticSemaphore_t sem_cb;
static osSemaphoreAttr_t s_attr = {
		.cb_mem = &sem_cb,
		.cb_size = sizeof(sem_cb)
		.name = "sem_gyro_input",
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
		.cb_size = sizeof(msg_buf)
		.name = "msg_q",
};

struct message{
	int msg_type;
	int16_t velocity;
	int btn_state;
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
static uint32_t btn_input_stack[STACK_LEN];
static osThreadId_t btn_input_task_id;
const osThreadAttr_t btn_thread_attr = {
		.cb_mem = &btn_input_cb,
		.cb_size = sizeof(btn_input_cb),
		.stack_mem = &btn_input_stack[0],
		.stack_size = sizeof(btn_input_stack),
		.name = "btn_input_task"
};

void timer_callback(void* arg);
void gyro_input_task(void* arg);
void btn_input_task(void* arg);
/*
 * @brief Initialize application.
 */
void init_app(){
	Gyro_Init();

	//init message queue
	qId = osMessageQueueNew(Q_SIZE, sizeof(message), &q_attr);
	if(qId == NULL) while(1);

	//init semaphore
	sId = osSemaphoreNew(1, 0, &s_attr);
	if(sId == NULL) while(1);

	//init event group
	eId = osEventFlagsNew(&e_attr);
	if(eId == NULL) while(1);

	gyro_input_task_id = osThreadNew(gyro_input_task, (void*)0, &gyro_thread_attr);
	if(gyro_input_task_id == NULL) while(1);

	btn_input_task_id = osThreadNew(btn_input_task, (void*)0, &btn_thread_attr);
	if(btn_input_task_id == NULL) while(1);

	//init timer
	periodic_id = osTimerNew(timer_callback, osTimerPeriodic, (void*)0, &timer_attr);
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
 * @brief External interrupt 0 request handler. User button 0 causes this interrupt, and will retrieve button state when pressed/unpressed.
 */
void EXTI0_IRQHandler(){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	get_btn_state();

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void timer_callback(void* arg){
	// remove unused variable warning
	(void) &arg;

	osStatus_t status = osSemaphoreRelease(sId);
	if(status != osOK) while(1);
}

void gyro_input_task(void* arg){
	(void) &arg;
	osStatus_t status = osSemaphoreAcquire(sId, osWaitForever);
	if(status != osOK) while(1);

	struct message newMessage;
	newMessage.msg_type = GYRO_INPUT;
	newMessage.velocity = read_gyro_velocity();

	//use one message-sized pool for each message? Add pool ptr to message queue?
	osMemoryPoolId_t mempool_id;
	osMemoryPoolNew(block_count, block_size, attr)
	osMemoryPoolAlloc(mp_id, timeout)

	osMessageQueuePut(qId, msg_ptr, 0, osWaitForever);
//	drive_leds(velocity);

}





