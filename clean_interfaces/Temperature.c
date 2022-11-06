#include "Temperature.h"

#include "delay_us.h"

void temperature_init(struct Temperature *self) {
	for(int i = 0; i < 8; ++i) {
		self->vals[0] = 0;
	}
	self->pos = 0;
}

double temperature_read(struct Temperature *self) {
	if (!DS18B20_Init()) return 0;
	DS18B20_Write (0xCC); // Send skip ROM command
	DS18B20_Write (0x44); // Send reading start conversion command
	if (!DS18B20_Init()) return 0;
	DS18B20_Write (0xCC); // Send skip ROM command
	DS18B20_Write (0xBE); // Read the register, a total of nine bytes, the first two bytes are the conversion value
	int temp = DS18B20_Read (); // Low byte
	temp |= DS18B20_Read () << 8; // High byte
	if(temp == 0) {
		return 0;
	}
	return average_temperature(self, temp);
}

double average_temperature(struct Temperature *self, int temp_raw) {
	self->vals[self->pos] = temp_raw * 0.1125 + 32; // convert to fahrenheit
	self->pos = (self->pos + 1) % 8;
	double running_avg = 0;
	for(int i = 0; i < 8; ++i) {
		running_avg += self->vals[i];
	}
	return running_avg / 8.0;
}

bool DS18B20_Init() {
	HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
	delay_us(5);
	HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
	delay_us(750);//480-960
	HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
	int t = 0;
	while (HAL_GPIO_ReadPin(GPIO_TEMP_Base,GPIO_TEMP_RX)) {
		t++;
		if (t > 60) return false;
		delay_us(1);
	}
	t = 480 - t;
	delay_us(t);
	HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
	return true;
}

void DS18B20_Write(uint8_t data) {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
		delay_us(10);
		if (data & 1) HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
		data >>= 1;
		delay_us(50);
		HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
	}
}

uint8_t DS18B20_Read() {
	HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX, GPIO_PIN_SET);
	delay_us(2);
	uint8_t data = 0;
	for (int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
		delay_us(1);
		HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
		delay_us(5);
		data >>= 1;
		if (HAL_GPIO_ReadPin(GPIO_TEMP_Base,GPIO_TEMP_RX)) data |= 0x80;
		delay_us(55);
		HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
	}
	return data;
}
