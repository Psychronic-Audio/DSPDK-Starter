/*
 * audio_processing.c
 *
 *  Created on: May 12, 2023
 *      Author: Psychronic Audio
 */

#include "main.h"
#include "audio_processing.h"

void process_audio(volatile int32_t *inputSamples, volatile int32_t *outputSamples){
	int32_t leftChannel[BUFFER_SIZE];
	float leftChannelFloat[BUFFER_SIZE];

	//The data collected from the ADC and DAC are still stereo. We need to separate the left channel out of the mix
	for(int i = 0; i < BUFFER_SIZE*2; i += 2){
		int index = 0;
		leftChannel[index] = inputSamples[i];
		index += 1;
	}

	//The data collected is in 24-bit signed format on a 32 bit frame, to do DSP we want the data to be in float format from [-1.0, +1.0]
	for(int i = 0; i < BUFFER_SIZE; i++){
		leftChannelFloat[i] = INT24_TO_FLOAT * leftChannel[i];
	}

	//Audio passthrough
	for(int i = 0; i < BUFFER_SIZE*2; i += 2){
			outputSamples[i] = (int32_t)(FLOAT_TO_INT24 * leftChannelFloat[i/2]);
			outputSamples[i+1] = 0;
	}
}
