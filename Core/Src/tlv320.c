/*
 * tlv320.c
 *
 *  Created on: May 6, 2023
 *      Author: Psychronic Audio
 *
 *
 */

#include "tlv320.h"
#include "stm32h7xx_hal.h"

int SETUP_TLV320(I2C_HandleTypeDef *i2c_handle){

	//Perform hardware reset and wait a bit for chip to come back online
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

	uint8_t data[2];
	HAL_StatusTypeDef ret;

	//Perform a software reset. Set reset bit, and wait for it to clear itself.
	data[0] = DAC_RESET_REG;
	data[1] = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}else{
		while(data[1] != 0){
			ret = HAL_I2C_Mem_Read(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
			if(ret != HAL_OK){
					return 1;
			}
		}
	}

	/*
	PLL SETUP
	STM32 MCLK OUT = 47.743kHz * 256 = 12.222208MHz
	NDAC Defaults to 1
	DAC Sample Rate = 1 * Fs_ref/NDAC (DUAL RATE MODE DISABLED) = 47.743kHz
	Fs_ref = (MCLK_IN * K * R)/(2048 * P) = 5697.875Hz * (K * R / P) = 47.743kHz
	(K*R/P) = 47743/5697.875 = 8
	K=J.D = 8.0000
	R = 1
	P = 1
	*/

	//Enable PLL, set P to 1
	data[0] = PLL_PGRM_REG_A;
	data[1] = (uint8_t)0b10010001;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Set PLL J to 8
	data[0] = PLL_PGRM_REG_B;
	data[1] = (uint8_t)0b00100000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Set PLL R to 1
	data[0] = DAC_OF_FLAG_REG;
	data[1] = (uint8_t)0b00000001;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Set PLL Clock Div (N) to 8
	data[0] = CLK_GEN_CTRL_REG;
	data[1] = (uint8_t)0b00001000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Enable Dual Rate Mode, Setup Left DAC Datapath
	data[0] = DAC_DATAPATH_REG;
	data[1] = (uint8_t)0b00001000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Choose Left-Justified I2S mode, with 32-bit word length
	data[0] = ASDI_CTRL_REG_B;
	data[1] = (uint8_t)0b11110000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Choose AC-Coupled Driver Config, and Fully-differential Config
	data[0] = HSET_BTTN_PRESS_DET_REG_B;
	data[1] = (uint8_t)0b11000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Set output common mode voltage to 1.65V
	data[0] = HPOS_CTRL_REG;
	data[1] = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Enable Left DAC Direct Path
	data[0] = DAC_OUT_SWTCH_CTRL_REG;
	data[1] = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Route DAC_L to HPLOUT
	data[0] = DAC_L_HPLOUT_VOL_CTRL_REG;
	data[1] = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Choose PLLDIV_OUT Clock for DAC_CLKIN
	data[0] = ADDL_CTRL_REG_B;
	data[1] = (uint8_t)0b00000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Power up Left DAC
	data[0] = DAC_PWR_CTRL_REG;
	data[1] = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Unmute and power up HPLOUT
	data[0] = HPLOUT_OUTPUT_LVL_CTRL_REG;
	data[1] = (uint8_t)0b00001101;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Unmute and power up HPLCOM
	data[0] = HPLCOM_OUTPUT_LVL_CTRL_REG;
	data[1] = (uint8_t)0b00001101;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, data[0], 1, &data[1], 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	return 0;
}
