/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BNO_CS1_Pin GPIO_PIN_4
#define BNO_CS1_GPIO_Port GPIOA
#define BNO_INT1_Pin GPIO_PIN_0
#define BNO_INT1_GPIO_Port GPIOB
#define BNO_INT1_EXTI_IRQn EXTI0_IRQn
#define BNO_RST1_Pin GPIO_PIN_1
#define BNO_RST1_GPIO_Port GPIOB
#define BNO_WAKE1_Pin GPIO_PIN_2
#define BNO_WAKE1_GPIO_Port GPIOB
#define BNO_CS2_Pin GPIO_PIN_15
#define BNO_CS2_GPIO_Port GPIOA
#define BNO_INT2_Pin GPIO_PIN_3
#define BNO_INT2_GPIO_Port GPIOB
#define BNO_INT2_EXTI_IRQn EXTI3_IRQn
#define BNO_RST2_Pin GPIO_PIN_4
#define BNO_RST2_GPIO_Port GPIOB
#define BNO_WAKE2_Pin GPIO_PIN_5
#define BNO_WAKE2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
