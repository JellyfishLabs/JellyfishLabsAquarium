#ifndef PH_H
#define PH_H

#include "stm32l1xx_hal.h"

// Used for tuning the PH val
extern float OFFSET;

struct Ph {
	float vals[8];
	int pos;
	ADC_HandleTypeDef hadc;
};

// initialize pH object, connect the hadc with it
void pH_init(struct Ph *self, ADC_HandleTypeDef handle);

// Read a value of the pH sensor, average it, and return value
float pH_read(struct Ph *self);

// compute runtime average pH value
float average_pH(struct Ph *self, int adc_val);

#endif /* PH_H */

