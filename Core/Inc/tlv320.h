/*
 * tlv320.h
 *
 *  Created on: May 6, 2023
 *      Author: Psychronic Audio
 */

#ifndef INC_TLV320_H_
#define INC_TLV320_H_
#include "stm32h7xx_hal.h"
#define TLV320_ADDR (((uint8_t) 0b00011000) << 1) /*TLV320DAC32IRHBR I2C Address*/

//Page 0
#define DAC_PAGE_SEL_REG ((uint8_t) 0x00) /*Page Select Register*/
#define DAC_RESET_REG ((uint8_t) 0x01) /*Software Reset Register*/
#define DAC_RATE_SEL_REG ((uint8_t) 0x02) /*DAC Sample Rate Select Register*/
#define PLL_PGRM_REG_A ((uint8_t) 0x03) /*PLL Programming Register A*/
#define PLL_PGRM_REG_B ((uint8_t) 0x04) /*PLL Programming Register B*/
#define PLL_PGRM_REG_C ((uint8_t) 0x05) /*PLL Programming Register C*/
#define PLL_PGRM_REG_D ((uint8_t) 0x06) /*PLL Programming Register D*/
#define DAC_DATAPATH_REG ((uint8_t) 0x07) /*DAC Datapath Setup Register*/
#define ASDI_CTRL_REG_A ((uint8_t) 0x08) /*Audio Serial Data Interface Control Register A*/
#define ASDI_CTRL_REG_B ((uint8_t) 0x09) /*Audio Serial Data Interface Control Register B*/
#define DAC_OF_FLAG_REG ((uint8_t) 0x0B) /* Audio DAC Overflow Flag Register*/
#define HSET_BTTN_PRESS_DET_REG_B ((uint8_t) 0x0E) /* Headset / Button Press Detection Register B */
#define DAC_PWR_CTRL_REG ((uint8_t) 0x25) /*DAC Power and Output Driver Control Register*/
#define HPOD_CTRL_REG ((uint8_t) 0x26) /*High Power Output Driver Control Register*/
#define HPOS_CTRL_REG ((uint8_t) 0x28) /*High Power Output Stage Control Register*/
#define DAC_OUT_SWTCH_CTRL_REG ((uint8_t) 0x29) /*DAC Output Switching Control Register*/
#define LDAC_DVOL_CTRL_REG ((uint8_t) 0x2B) /*Left DAC Digital Volume Control Register*/
#define RDAC_DVOL_CTRL_REG ((uint8_t) 0x2C) /*Right DAC Digital Volume Control Register*/
#define DAC_L_HPLOUT_VOL_CTRL_REG ((uint8_t) 0x2F) /*DAC_L to HPLOUT Volume Control Register*/
#define HPLOUT_OUTPUT_LVL_CTRL_REG ((uint8_t) 0x33) /*HPLOUT Output Level Control Register*/
#define HPLCOM_OUTPUT_LVL_CTRL_REG ((uint8_t) 0x3A) /*HPLCOM Output Level Control Register*/
#define MDL_PWR_STATUS_REG ((uint8_t) 0x5E) /*Module Power Status Register*/
#define ADDL_CTRL_REG_B ((uint8_t) 0x65) /*Additional Control Register B*/
#define CLK_GEN_CTRL_REG ((uint8_t) 0x66) /*Clock Generation Control Register*/

int SETUP_TLV320(I2C_HandleTypeDef *i2c_handle);

#endif /* INC_TLV320_H_ */
