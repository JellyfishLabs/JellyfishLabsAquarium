/*!
 * @file    Stepper.h
 * @brief   Interface code for pH stepper motor on the Jellyfish Labs Smart Aquarium
 * @note	The stepper motor interface involves using an H-Bridge to drive the
 * 			peristaltic pumps.
 * 			one GPIO pin.
 *          PINOUT  LABEL           PORT/PIN
 *          --------------------------------
 *          PH_UP	X				PX_Y
 *          PH_DOWN	X				PX_Y
 *
 *
 */

#ifndef SRC_STEPPER_H_
#define SRC_STEPPER_H_

#include <stdint.h>
#include "stm32l1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct STEPPER_STRUCT {

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in us, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on

    // motor pins:
    uint16_t motor_pin_1;
    uint16_t motor_pin_2;
    uint16_t motor_pin_3;
    uint16_t motor_pin_4;

    unsigned long last_step_time; // time stamp in us of when the last step was taken

} Stepper;



/*
 * @brief   Initializes stepper struct
 * @param	number_of_steps		number of steps for stepper motor
 * @param	s					pointer to Stepper object
 * @param	motor_port			[0] = pin1 port, [1] = pin2 port, ... [3] = pin4 port
 * @param	motor_pin			[0] = pin1 number, [1] = pin2 number, ... [3] = pin4 number
 */
void init_stepper(int number_of_steps, Stepper *s, uint16_t* motor_pin, GPIO_TypeDef* motor_port_in[4]);

/*
 * @brief   Sets speed for stepper motor
 * @param	whatSpeed	Speed set for stepper motor
 */
void setSpeed(long whatSpeed, Stepper *s);

/*
 * @brief	Sets the speed revs per minute
 * @param   step				number of steps for stepper motor
 * @param	s					pointer to stepper motor
*/
void step(int number_of_steps, Stepper *s);

/*
 * @brief Returns version of library
 */
int version(void);

/*
 * @brief   step				Steps the motor to step number
 * @param	this_step			Step number
*/
void stepMotor(Stepper* s, int thisStep);



#endif /* SRC_STEPPER_H_ */
