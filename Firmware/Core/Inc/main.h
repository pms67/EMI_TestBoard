/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TL_Pin GPIO_PIN_0
#define TL_GPIO_Port GPIOA
#define TL_LOOP_Pin GPIO_PIN_1
#define TL_LOOP_GPIO_Port GPIOA
#define TL_SPLIT_Pin GPIO_PIN_2
#define TL_SPLIT_GPIO_Port GPIOA
#define TL_VIA_Pin GPIO_PIN_3
#define TL_VIA_GPIO_Port GPIOA
#define TL_VIA_T_Pin GPIO_PIN_4
#define TL_VIA_T_GPIO_Port GPIOA
#define TL_R_Pin GPIO_PIN_5
#define TL_R_GPIO_Port GPIOA
#define TL_RC_B_Pin GPIO_PIN_6
#define TL_RC_B_GPIO_Port GPIOA
#define TL_RC_A_Pin GPIO_PIN_7
#define TL_RC_A_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define TL_LCL_Pin GPIO_PIN_9
#define TL_LCL_GPIO_Port GPIOA
#define TL_EM_Pin GPIO_PIN_10
#define TL_EM_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
