/*
 * Stepper.c
 *
 *  Created on: Nov 5, 2022
 *      Author: andrewjohnlyandar
 */


#include "Stepper.h"

/*
 * @brief   Initializes stepper struct
 * @param	number_of_steps		number of steps for stepper motor
 * @param	s					pointer to Stepper object
 * @param	motor_p1			1A H-bridge pin
 * @param	motor_p1			2A H-bridge pin
 * @param	motor_p1			3A H-bridge pin
 * @param	motor_p1			4A H-bridge pin
 */
void init_stepper(int number_of_steps, Stepper *s , char motor_port[], int motor_pin[])
{
	  s->step_number = 0;    // which step the motor is on
	  s->direction = 0;      // motor direction
	  s->last_step_time = 0; // timestamp in us of the last step taken
	  s->number_of_steps = number_of_steps; // total number of steps for s motor

	  // STM Ports for pins:
	  s->motor_port_1 = motor_port[0];
	  s->motor_port_2 = motor_port[1];
	  s->motor_port_3 = motor_port[2];
	  s->motor_port_4 = motor_port[3];

	  // STM pins for the motor control connection:
	  s->motor_pin_1 = motor_pin[0];
	  s->motor_pin_2 = motor_pin[1];
	  s->motor_pin_3 = motor_pin[2];
	  s->motor_pin_4 = motor_pin[3];

	  // setup the pins on the microcontroller:
	  //none done here
	  //outputs declared on the .ioc diagram


	  // pin_count is used by the stepMotor() method:
	  s->pin_count = 4;
}

/*
 * @brief   Sets speed for stepper motor
 * @param	whatSpeed	Speed set for stepper motor
 */
void setSpeed(long whatSpeed, Stepper *s){
	s->step_delay = 60L * 1000L * 1000L / s->number_of_steps / whatSpeed;
}

/*
 * @brief   step				number of steps for stepper motor
 * @param	number_of_steps		Speed set for stepper motor
 */
void step(int steps_to_move, Stepper *s)
{
  int steps_left = abs(steps_to_move);  // how many steps to take

  // determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0) { s->direction = 1; }
  if (steps_to_move < 0) { s->direction = 0; }


  // decrement the number of steps, moving one step each time:
  while (steps_left > 0){
	  // increment or decrement the step number,
	  // depending on direction:
	  if (s->direction == 1)
	  {
		s->step_number++;
		if (s->step_number == s->number_of_steps) {
		  s->step_number = 0;
		}
	  }
	  else
	  {
		if (s->step_number == 0) {
		  s->step_number = s->number_of_steps;
		}
		s->step_number--;
	  }
	  // decrement the steps left:
	  steps_left--;
	  // step the motor to step number 0, 1, ..., {3 or 10}
	  if (s->pin_count == 5)
		stepMotor(s->step_number % 10);
	  else
		stepMotor(s->step_number % 4);
  }

}

void stepMotor(int thisStep)
{

  if (this->pin_count == 4) {
    switch (thisStep) {
      case 0:  // 1010

        HAL_GPIO_WritePin(motor_port_1, motor_pin_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_port_2, motor_pin_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_port_3, motor_pin_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_port_4, motor_pin_4, GPIO_PIN_RESET);
      break;
      case 1:  // 0110
        HAL_GPIO_WritePin(motor_port_1, motor_pin_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_port_2, motor_pin_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_port_3, motor_pin_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_port_4, motor_pin_4, GPIO_PIN_RESET);
      break;
      case 2:  //0101
        HAL_GPIO_WritePin(motor_port_1, motor_pin_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_port_2, motor_pin_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_port_3, motor_pin_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_port_4, motor_pin_4, GPIO_PIN_SET);
      break;
      case 3:  //1001
        HAL_GPIO_WritePin(motor_port_1, motor_pin_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_port_2, motor_pin_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_port_3, motor_pin_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_port_4, motor_pin_4, GPIO_PIN_SET);
      break;
    }
  }


}
/*
 * @brief Returns version of library
 */
int version(void){
	return 5;
}

