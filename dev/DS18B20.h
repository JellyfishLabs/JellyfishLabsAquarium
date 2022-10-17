/*
  * @file           : DS18B20.h
  * @brief          : Interface code for temperature sensor on the Jellyfish Labs Smart Aquarium
  * @note	This code interfaces with the DS18B20 temperature sensor, using the Dallas 1-Wire Protocol, adapted from this tutorial https://how2electronics.com/interfacing-ds18b20-temperature-sensor-with-stm32/
  * 		This code requires one of the hardware timers for the delay_us function.
  *			Timer must be configured as follows:
  *			  - Clock Source: Internal Clock (32 MHz)
  *			  - Prescaler: 23 - 1
  *			  - Counter Period: 65535 - 1
  *  		This code also requires two GPIO pins, one configured as an input, one as an output, referred to here as GPIO_TEMP_TX and GPIO_TEMP_RX
  *
  *			Connect temp sensor data wire, GPIO_TEMP_TX, and GPIO_TEMP_RX all together. Add resistor between data and +5V.
  *
  *			Note that TempRead() returns a raw value, suggested usage for Celsius and Fahrenheit conversion:
  *				- double temp_raw = TempRead();
  *				- double temp_C  = temp_raw * 0.0625; // conversion accuracy is 0.0625 / LSB
  *	            - double temp_F = temp_raw * 0.1125 + 32;
  *
  *          PINOUT              LABEL           PORT/PIN
  *          ---------------------------------------------
  *          GPIO_TEMP_TX	    D13				PA_5
  *          GPIO_TEMP_RX	    D12				PA_6
  * @author	 Christian Foreman (cjforema)
  * @author  Miles Hanbury (mhanbury)
  * @author  James Kelly (jkellymi)
  * @author	 Andrew Lyandar (alyandar)
  * @author  Joshua Nye (nyej)
*/
/* -----------------------------------------------------Includes ------------------------------------------------------------------*/

#include "stdbool.h"
#include "stdint.h"

/* -----------------------------------------------------Defines ------------------------------------------------------------------*/

#define GPIO_TEMP_Base GPIOA
#define GPIO_TEMP_TX GPIO_PIN_5
#define GPIO_TEMP_RX GPIO_PIN_6
#define us_timer htim2

/* -----------------------------------------------------Functions ------------------------------------------------------------------*/

void delay_us (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER (&us_timer, 0);
	while (__HAL_TIM_GET_COUNTER(&us_timer)<delay);
}

bool DS18B20_Init()
{
  //pinMode(DSPIN, OUTPUT);
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  delay_us(5);
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
  delay_us(750);//480-960
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  //pinMode(DSPIN, INPUT);
  int t = 0;
  while (HAL_GPIO_ReadPin(GPIO_TEMP_Base,GPIO_TEMP_RX))
  {
    t++;
    if (t > 60) return false;
    delay_us(1);
  }
  t = 480 - t;
  //pinMode(DSPIN, OUTPUT);
  delay_us(t);
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  return true;
}

void DS18B20_Write(uint8_t data)
{
  //pinMode(DSPIN, OUTPUT);
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
    delay_us(10);
    if (data & 1) HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
    else HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
    data >>= 1;
    delay_us(50);
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  }
}

uint8_t DS18B20_Read()
{
  //pinMode(DSPIN, OUTPUT);
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX, GPIO_PIN_SET);
  delay_us(2);
  uint8_t data = 0;
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  //  pinMode(DSPIN, INPUT);
    delay_us(5);
    data >>= 1;
    if (HAL_GPIO_ReadPin(GPIO_TEMP_Base,GPIO_TEMP_RX)) data |= 0x80;
    delay_us(55);
    //pinMode(DSPIN, OUTPUT);
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  }
  return data;
}

volatile int TempRead()
{
  if (!DS18B20_Init()) return 0;
  DS18B20_Write (0xCC); // Send skip ROM command
  DS18B20_Write (0x44); // Send reading start conversion command
  if (!DS18B20_Init()) return 0;
  DS18B20_Write (0xCC); // Send skip ROM command
  DS18B20_Write (0xBE); // Read the register, a total of nine bytes, the first two bytes are the conversion value
  int temp = DS18B20_Read (); // Low byte
  temp |= DS18B20_Read () << 8; // High byte
  return temp;
}
