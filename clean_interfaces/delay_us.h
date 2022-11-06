#ifndef DELAY_US_H
#define DELAY_US_H

#include "stm32l1xx_hal.h"

extern TIM_HandleTypeDef htim2;

void delay_us (uint16_t delay) {
	__HAL_TIM_SET_COUNTER (&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2)<delay);
}

#endif /* DELAY_US */
