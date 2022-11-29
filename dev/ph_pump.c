
#ifndef SRC_PH_PUMP_H_
#define SRC_PH_PUMP_H_

#include "ph_pump.h"
#include <stdint.h>
#include "stm32l1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
/*
 * @brief   Pumps 10 ml of base pH solution into the aquarium
 */
void ph_pump_base(){
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	  HAL_Delay(1100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	  HAL_Delay(1100);
}

/*
 * @brief   Pumps 10 ml of acid pH solution into the aquarium
 */
void ph_pump_acid(){
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	  HAL_Delay(1100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	  HAL_Delay(1100);
}