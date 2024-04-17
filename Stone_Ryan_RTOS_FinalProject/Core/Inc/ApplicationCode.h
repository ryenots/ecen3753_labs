/*
 * ApplicationCode.h
 *
 *  Created on: Dec 30, 2023
 *      Author: Xavion
 */

#include "LCD_Driver.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>


#ifndef INC_APPLICATIONCODE_H_
#define INC_APPLICATIONCODE_H_

#define USR_BTN_PORT GPIOA
#define USR_BTN_PIN GPIO_PIN_0

#define LED_PORT GPIOG
#define GRN_LED_PIN GPIO_PIN_13
#define RED_LED_PIN GPIO_PIN_14

void ApplicationInit(RNG_HandleTypeDef*);
void LCD_Visual_Demo(void);

#if (COMPILE_TOUCH_FUNCTIONS == 1) && (COMPILE_TOUCH_INTERRUPT_SUPPORT == 0)
void LCD_Touch_Polling_Demo(void);
#endif // (COMPILE_TOUCH_FUNCTIONS == 1) && (COMPILE_TOUCH_INTERRUPT_SUPPORT == 0)

#endif /* INC_APPLICATIONCODE_H_ */
