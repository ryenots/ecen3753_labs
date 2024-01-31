/*
 * Application_Code.h
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#ifndef INC_APPLICATION_CODE_H_
#define INC_APPLICATION_CODE_H_

#include "stm32f4xx_hal.h"
#include "Gyro_Driver.h"

#define USR_BTN_PORT GPIOA
#define USR_BTN_PIN GPIO_PIN_0

#define LED_PORT GPIOG
#define GRN_LED_PIN GPIO_PIN_13
#define RED_LED_PIN GPIO_PIN_14

static int usr_btn_state;

typedef enum{
	CC_FAST = -1000,
	CC_SLOW = -100,
	ZERO = 0,
	C_SLOW = 100,
	C_FAST = 1000
};
void init_app();
void get_btn_state();
int16_t read_gyro_velocity();
void drive_leds(int16_t velocity);

#endif /* INC_APPLICATION_CODE_H_ */
