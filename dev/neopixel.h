/*!
 * @file    neopixel.h
 * @brief   Interface code for LED Strip on the Jellyfish Labs Smart Aquarium
 * @note	This interface allows for communication with Adafruit Neopixel LEDs.
 * 			This requires one PWM pin.
 *          PINOUT  LABEL           PORT/PIN
 *          --------------------------------
 *          LED		LED_CTRL		PA_0
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

#define NEOPIXEL_ZERO	13
#define NEOPIXEL_ONE	27
#define NUM_PIXELS		29
#define DMA_BUFF_SIZE 	(NUM_PIXELS * 24) + 1

uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};

typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

void neopixel_set(TIM_HandleTypeDef *htim, uint8_t power, uint8_t r, uint8_t g, uint8_t b) {
	if(power == 0) {
		r=0;
		g=0;
		b=0;
	}
	PixelRGB_t pixel = {0};
	uint32_t *pBuff;
	pBuff = dmaBuffer;
	pixel.color.b = b;
	pixel.color.r = r;
	pixel.color.g = g;
	for (int i = 0; i < NUM_PIXELS; i++) {
		for (int j = 23; j >= 0; j--) {
			if ((pixel.data >> j) & 0x01) {
				*pBuff = NEOPIXEL_ONE;
			}
			else {
				*pBuff = NEOPIXEL_ZERO;
			}
			pBuff++;
		}
	}
	dmaBuffer[DMA_BUFF_SIZE - 1] = 0;
	HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
	HAL_Delay(10);
}
