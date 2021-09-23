/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "meme.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#include <string.h>
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define BUFFER_SIZE_SAMPLES (1024)
#define SAMPLE_SIZE (4)

#define DMA_BUFFER_SIZE ((BUFFER_SIZE_SAMPLES*2U) * 2U) // First double due to being in stereo, second due to being "two half-buffers"
#define DMA_BUFFER_SIZE_BYTES (DMA_BUFFER_SIZE * SAMPLE_SIZE)


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void fillProcessingBuffer();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_RESET_Pin GPIO_PIN_15
#define ADC_RESET_GPIO_Port GPIOC
#define USERBUTTON1_Pin GPIO_PIN_10
#define USERBUTTON1_GPIO_Port GPIOC
#define USERBUTTON2_Pin GPIO_PIN_11
#define USERBUTTON2_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_4
#define LCD_E_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_5
#define LCD_RW_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_6
#define LCD_RS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

typedef struct {
	uint8_t button_1;
	uint8_t button_2;
	uint32_t encoder_1;
	uint32_t encoder_2;
	uint8_t program;

	union
	{
	 uint8_t     changed;      /* Allows us to refer to the flags 'en masse' */
	 struct
	 {
	  uint8_t button1_changed : 1,        /* Explanation of foo */
	          button2_changed : 1,        /* Explanation of bar */
	          encoder1_changed : 1,     /* Unused */
	          encoder2_changed : 1,     /* Unused */
	          program_changed : 1,     /* Unused */
	          spare2 : 1,     /* Unused */
	          spare1 : 1,     /* Unused */
	          spare0 : 1;     /* Unused */
	 };
	};
} input_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/