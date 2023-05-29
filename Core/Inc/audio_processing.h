/*
 * audio_processing.h
 *
 *  Created on: May 7, 2023
 *      Author: mbeck
 */

#ifndef INC_AUDIO_PROCESSING_H_
#define INC_AUDIO_PROCESSING_H_

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include "main.h"

#define INT24_TO_FLOAT (1.0f/8388607.0f)
#define FLOAT_TO_INT24 (8388607.0f)
#define FILTER_IMPULSE_LENGTH 128

typedef struct {
	arm_fir_instance_f32 fir_instance;
	uint32_t impulse_length;
	float *taps;
} fir_filter;

extern arm_fir_instance_f32 firSettingsBand1, firSettingsBand2, firSettingsBand3, firSettingsBand4, firSettingsBand5, firSettingsBand6;
extern float32_t impulse1[FILTER_IMPULSE_LENGTH],
							impulse2[FILTER_IMPULSE_LENGTH],
							impulse3[FILTER_IMPULSE_LENGTH],
							impulse4[FILTER_IMPULSE_LENGTH],
							impulse5[FILTER_IMPULSE_LENGTH],
							impulse6[FILTER_IMPULSE_LENGTH];

extern float32_t fir_state_1[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_2[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_3[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_4[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_5[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1],
	fir_state_6[FILTER_IMPULSE_LENGTH+BUFFER_SIZE/2 - 1];

extern float32_t fir_out_1[BUFFER_SIZE/2],
	fir_out_2[BUFFER_SIZE/2],
	fir_out_3[BUFFER_SIZE/2],
	fir_out_4[BUFFER_SIZE/2],
	fir_out_5[BUFFER_SIZE/2],
	fir_out_6[BUFFER_SIZE/2];

void initialize_filters();
void process_audio(volatile int32_t *inputSamples, volatile int32_t *outputSamples);
void convolve(volatile float* x, volatile float* h, volatile float* y, volatile uint32_t xLength, volatile uint32_t hLength);

#endif /* INC_AUDIO_PROCESSING_H_ */
