/*!
 * @file    Ph.h
 * @brief   Driver code for the ph meter v1.1
 * @note    Used to read the pH value in analog from the ph meter v1.1 by DFRobot
 *
 *          PINOUT  LABEL           PORT/PIN
 *          --------------------------------
 *          A1      Analog          A1
 *
 * @author  Christian Foreman (cjforema
 * @author  Miles Hanbury (mhanbury)
 * @author  James Kelly (jkellymi)
 * @author  Andrew Lyandar (alyandar)
 * @author  Joshua Nye (nyej)
 *
 */

#include "Ph.h"

// Offset value made for specific
float OFFSET = 0.48;

void pH_init(struct Ph *self, ADC_HandleTypeDef handle) {
	for(int i = 0; i < 8; ++i) {
		self->vals[0] = 0;
	}
	self->pos = 0;
	self->hadc = handle;
}

float pH_read(struct Ph *self) {
	HAL_ADC_Start(&self->hadc);//start conversion
	HAL_ADC_PollForConversion(&self->hadc, 0xFFFFFFFF);//wait for conversion to finish
	int adc_val = HAL_ADC_GetValue(&self->hadc);//retrieve value
	return average_pH(self, adc_val);
}

float average_pH(struct Ph *self, int adc_val) {
	self->vals[self->pos] = adc_val / 4096.0 * 3.3 * 3.5 + OFFSET;
	self->pos = (self->pos + 1) % 8;
	float running_avg = 0;
	for(int i = 0; i < 8; ++i) {
		running_avg += self->vals[i];
	}
	return running_avg / 8.0;
}
