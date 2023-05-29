/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef  enum {
	TRUE,
	FALSE
} bool;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POT6_Pin GPIO_PIN_0
#define POT6_GPIO_Port GPIOC
#define POT7_Pin GPIO_PIN_1
#define POT7_GPIO_Port GPIOC
#define POT4_Pin GPIO_PIN_5
#define POT4_GPIO_Port GPIOC
#define POT5_Pin GPIO_PIN_0
#define POT5_GPIO_Port GPIOB
#define POT2_Pin GPIO_PIN_1
#define POT2_GPIO_Port GPIOB
#define POT1_Pin GPIO_PIN_11
#define POT1_GPIO_Port GPIOF
#define POT3_Pin GPIO_PIN_12
#define POT3_GPIO_Port GPIOF
#define DAC_RESETn_Pin GPIO_PIN_11
#define DAC_RESETn_GPIO_Port GPIOB
#define FSWITCH_2_INT_Pin GPIO_PIN_8
#define FSWITCH_2_INT_GPIO_Port GPIOA
#define FSWITCH_2_INT_EXTI_IRQn EXTI9_5_IRQn
#define FSWITCH_1_INT_Pin GPIO_PIN_9
#define FSWITCH_1_INT_GPIO_Port GPIOA
#define FSWITCH_1_INT_EXTI_IRQn EXTI9_5_IRQn
#define LED_1_Pin GPIO_PIN_10
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_11
#define LED_2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 512

extern volatile int32_t adcData[BUFFER_SIZE*2];
extern volatile int32_t dacData[BUFFER_SIZE*2];
extern volatile float processedData[BUFFER_SIZE/2];
extern volatile int32_t *inBufPtr;
extern volatile int32_t *outBufPtr;

extern volatile uint8_t dataReadyFlag;

extern volatile uint16_t adcControlData[7];
extern volatile float gains[7];
extern volatile bool EQ_Enable;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
