/*
 * Application_Code.h
 *
 *  Created on: Jan 20, 2024
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

void app_init();

void led_toggle();

int get_btn_state();

#endif /* INC_APPLICATION_CODE_H_ */
