/*
 * audio_processing.c
 *
 *  Created on: May 12, 2023
 *      Author: Psychronic Audio
 */

#include "main.h"
#include "audio_processing.h"

void process_audio(volatile int32_t *inputSamples, volatile int32_t *outputSamples){

	//The data collected from the ADC and DAC are still stereo. We need to separate the left channel out of the mix
	for(int i = 0; i < BUFFER_SIZE*2; i += 2){
		outputSamples[i] = inputSamples[i];
		outputSamples[i+1] = 0;
	}

}
