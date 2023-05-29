/*
 * audio_processing.h
 *
 *  Created on: May 7, 2023
 *      Author: Psychronic Audio
 */

#ifndef INC_AUDIO_PROCESSING_H_
#define INC_AUDIO_PROCESSING_H_

#include "stm32h7xx_hal.h"
#include "main.h"

#define INT24_TO_FLOAT (1.0f/8388607.0f)
#define FLOAT_TO_INT24 (8388607.0f)

void process_audio(volatile int32_t *inputSamples, volatile int32_t *outputSamples);

#endif /* INC_AUDIO_PROCESSING_H_ */
