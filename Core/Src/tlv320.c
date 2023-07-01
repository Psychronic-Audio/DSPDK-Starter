
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
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(100);

	HAL_StatusTypeDef ret;

	//Perform a software reset. Set reset bit, and wait for it to clear itself.
	uint8_t reg_addr;
	uint8_t data;

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
	reg_addr = PLL_PGRM_REG_A;
	data = (uint8_t)0b10010001;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 2;
	}

	//Set PLL J to 8
	reg_addr = PLL_PGRM_REG_B;
	data = (uint8_t)0b00100000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 3;
	}

	//Set PLL R to 1
	reg_addr = DAC_OF_FLAG_REG;
	data = (uint8_t)0b00000001;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 4;
	}

	//Set PLL Clock Div (N) to 8
	reg_addr = CLK_GEN_CTRL_REG;
	data = (uint8_t)0b00001000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Enable Dual Rate Mode, Setup Left DAC Datapath
	reg_addr = DAC_DATAPATH_REG;
	data = (uint8_t)0b00001000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Choose Left-Justified I2S mode, with 32-bit word length
	reg_addr = ASDI_CTRL_REG_B;
	data = (uint8_t)0b11110000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Choose AC-Coupled Driver Config, and Fully-differential Config
	reg_addr = HSET_BTTN_PRESS_DET_REG_B;
	data = (uint8_t)0b11000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Set output common mode voltage to 1.65V
	reg_addr = HPOS_CTRL_REG;
	data = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Enable Left DAC Direct Path
	reg_addr = DAC_OUT_SWTCH_CTRL_REG;
	data = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Route DAC_L to HPLOUT
	reg_addr = DAC_L_HPLOUT_VOL_CTRL_REG;
	data = (uint8_t)0b00000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Choose PLLDIV_OUT Clock for DAC_CLKIN
	reg_addr = ADDL_CTRL_REG_B;
	data = (uint8_t)0b00000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Power up Left DAC
	reg_addr = DAC_PWR_CTRL_REG;
	data = (uint8_t)0b10000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Unmute Left DAC, no attenuation
	reg_addr = LDAC_DVOL_CTRL_REG;
	data = (uint8_t)0b00000000;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, 1, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}


	//Unmute and power up HPLOUT
	reg_addr = HPLOUT_OUTPUT_LVL_CTRL_REG;
	data = (uint8_t)0b00001101;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	//Unmute and power up HPLCOM
	reg_addr = HPLCOM_OUTPUT_LVL_CTRL_REG;
	data = (uint8_t)0b00001101;
	ret = HAL_I2C_Mem_Write(i2c_handle, TLV320_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 1;
	}

	return 0;
}
