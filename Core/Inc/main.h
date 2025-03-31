/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define GPIO3_Pin GPIO_PIN_13
#define GPIO3_GPIO_Port GPIOC
#define GPIO4_Pin GPIO_PIN_14
#define GPIO4_GPIO_Port GPIOC
#define GPIO5_Pin GPIO_PIN_15
#define GPIO5_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_2
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_3
#define PWM4_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define RF2_IRQ_Pin GPIO_PIN_0
#define RF2_IRQ_GPIO_Port GPIOB
#define RF2_CE_Pin GPIO_PIN_1
#define RF2_CE_GPIO_Port GPIOB
#define RF2_CS_Pin GPIO_PIN_2
#define RF2_CS_GPIO_Port GPIOB
#define RF1_CS_Pin GPIO_PIN_12
#define RF1_CS_GPIO_Port GPIOB
#define RF_SCK_Pin GPIO_PIN_13
#define RF_SCK_GPIO_Port GPIOB
#define RF_MISO_Pin GPIO_PIN_14
#define RF_MISO_GPIO_Port GPIOB
#define RF_MOSI_Pin GPIO_PIN_15
#define RF_MOSI_GPIO_Port GPIOB
#define RF1_CE_Pin GPIO_PIN_6
#define RF1_CE_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_10
#define GPIO1_GPIO_Port GPIOC
#define GPIO2_Pin GPIO_PIN_11
#define GPIO2_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_3
#define PWM2_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_4
#define PWM5_GPIO_Port GPIOB
#define PWM6_Pin GPIO_PIN_5
#define PWM6_GPIO_Port GPIOB
#define PWM7_Pin GPIO_PIN_6
#define PWM7_GPIO_Port GPIOB
#define PWM8_Pin GPIO_PIN_7
#define PWM8_GPIO_Port GPIOB
#define RF1_IRQ_Pin GPIO_PIN_9
#define RF1_IRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
