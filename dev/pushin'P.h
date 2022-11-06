/*!
 * @file    pushin'P.h
 * @brief   Interface code for pH balancing system on the Jellyfish Labs Smart Aquarium
 * @note	The interface involves using an H-Bridge to drive the peristaltic pumps used
 * 			to balance the pH of the smart aquarium. On the H-Bridge, EN1 and EN2 are
 * 			tied high, while 2A and 4A are tied low. Thus, we can control the motors using
 * 			one GPIO pin.
 *          PINOUT  LABEL           PORT/PIN
 *          --------------------------------
 *          PH_UP	1A				PC_8
 *          PH_DOWN	3A				PC_6
 *
 *
 * @author	Christian Foreman (cjforema)
 * @author  Miles Hanbury (mhanbury)
 * @author  James Kelly (jkellymi)
 * @author	Andrew Lyandar (alyandar)
 * @author  Joshua Nye (nyej)
 */

/* ----------------------------------------- Includes ------------------------------------------ */
#include "stm32l1xx_hal.h"                                                                          // provides definitions for SPI/GPIO types
#include <string.h>                                                                                 // provides string functions
#include <stdlib.h>
#include <stdio.h>

/* ---------------------------------------- Functions ------------------------------------------ */

typedef struct Motor {
	GPIO_TypeDef* motor_port;
	uint16_t motor_pin;
} MOTOR_Struct;

void dispense(MOTOR_Struct motor, uint16_t time) {
	  HAL_GPIO_WritePin(motor.motor_port, motor.motor_pin, GPIO_PIN_SET);
	  HAL_Delay(time);
	  HAL_GPIO_WritePin(motor.motor_port, motor.motor_pin, GPIO_PIN_RESET);
}
