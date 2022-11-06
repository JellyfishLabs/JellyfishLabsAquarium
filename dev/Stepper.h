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

struct Stepper {

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in us, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on

    // array for motor ports:
    char motor_port_1[6];
    char motor_port_2[6];
    char motor_port_3[6];
    char motor_port_4[6];

    // motor pins:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;

    unsigned long last_step_time; // time stamp in us of when the last step was taken

};



/*
 * @brief   Initializes stepper struct
 * @param	number_of_steps		number of steps for stepper motor
 * @param	s					pointer to Stepper object
 * @param	motor_port			[0] = pin1 port, [1] = pin2 port, ... [3] = pin4 port
 * @param	motor_pin			[0] = pin1 number, [1] = pin2 number, ... [3] = pin4 number
 */
void init_stepper(int number_of_steps, Stepper *s, int motor_pin[]);

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
void stepMotor(int this_step);



#endif /* SRC_STEPPER_H_ */
