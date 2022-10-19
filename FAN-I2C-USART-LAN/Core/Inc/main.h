/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAN_A2_SENSER_Pin GPIO_PIN_2
#define FAN_A2_SENSER_GPIO_Port GPIOE
#define STATE_LED0_Pin GPIO_PIN_13
#define STATE_LED0_GPIO_Port GPIOC
#define STATE_LED1_Pin GPIO_PIN_14
#define STATE_LED1_GPIO_Port GPIOC
#define STATE_LED2_Pin GPIO_PIN_15
#define STATE_LED2_GPIO_Port GPIOC
#define FAN_B1_PWM_Pin GPIO_PIN_8
#define FAN_B1_PWM_GPIO_Port GPIOF
#define FAN_B2_PWM_Pin GPIO_PIN_9
#define FAN_B2_PWM_GPIO_Port GPIOF
#define STM32_GPS_PPS_Pin GPIO_PIN_0
#define STM32_GPS_PPS_GPIO_Port GPIOA
#define STM32_INS_PPS_Pin GPIO_PIN_1
#define STM32_INS_PPS_GPIO_Port GPIOA
#define EXA6_Pin GPIO_PIN_12
#define EXA6_GPIO_Port GPIOF
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define FAN1_ONOFF_Pin GPIO_PIN_11
#define FAN1_ONOFF_GPIO_Port GPIOA
#define FAN2_ONOFF_Pin GPIO_PIN_12
#define FAN2_ONOFF_GPIO_Port GPIOA
#define FAN_A1_SENSER_Pin GPIO_PIN_3
#define FAN_A1_SENSER_GPIO_Port GPIOD
#define WIZNET_RESET_Pin GPIO_PIN_11
#define WIZNET_RESET_GPIO_Port GPIOG
#define FAN_A1_PWM_Pin GPIO_PIN_8
#define FAN_A1_PWM_GPIO_Port GPIOB
#define FAN_A2_PWM_Pin GPIO_PIN_9
#define FAN_A2_PWM_GPIO_Port GPIOB
#define NBL0_Pin GPIO_PIN_0
#define NBL0_GPIO_Port GPIOE
#define NBL1_Pin GPIO_PIN_1
#define NBL1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
