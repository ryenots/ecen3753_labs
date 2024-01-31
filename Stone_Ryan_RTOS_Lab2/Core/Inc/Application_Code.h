/*
 * Application_Code.h
 *
 *  Created on: Jan 29, 2024
 *      Author: ryenots
 */

#ifndef INC_APPLICATION_CODE_H_
#define INC_APPLICATION_CODE_H_

#include "stm32f4xx_hal.h"
#define USR_BTN_PORT GPIOA
#define USR_BTN_PIN GPIO_PIN_0

#define LED_PORT GPIOG
#define GRN_LED_PIN GPIO_PIN_13
#define RED_LED_PIN GPIO_PIN_14

static int usr_btn_state;

typedef enum{
	CC_SLOW = -2,
	CC_FAST,
	ZERO,
	C_SLOW,
	C_FAST
};

int get_btn_state();
int read_gyro_rot();
void drive_leds();

#endif /* INC_APPLICATION_CODE_H_ */
