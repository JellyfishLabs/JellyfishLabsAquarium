#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "stm32l1xx_hal.h"
#include "stdbool.h"

#define GPIO_TEMP_Base GPIOA
#define GPIO_TEMP_TX GPIO_PIN_5
#define GPIO_TEMP_RX GPIO_PIN_6

struct Temperature {
	double vals[8];
	int pos;
};

void temperature_init(struct Temperature *self);

double temperature_read(struct Temperature *self);

double average_temperature();

bool DS18B20_Init();

void DS18B20_Write(uint8_t data);

uint8_t DS18B20_Read();

#endif /* TEMPERATURE_H */
