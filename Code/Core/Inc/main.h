/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define RFM95_DIO5_Pin GPIO_PIN_3
#define RFM95_DIO5_GPIO_Port GPIOA
#define RFM95_DIO5_EXTI_IRQn EXTI3_IRQn
#define RFM95_NSS_SPI_Pin GPIO_PIN_0
#define RFM95_NSS_SPI_GPIO_Port GPIOB
#define RFM95_NRST_Pin GPIO_PIN_1
#define RFM95_NRST_GPIO_Port GPIOB
#define RFM95_DIO0_Pin GPIO_PIN_2
#define RFM95_DIO0_GPIO_Port GPIOB
#define RFM95_DIO0_EXTI_IRQn EXTI2_IRQn
#define RFM95_DIO1_Pin GPIO_PIN_10
#define RFM95_DIO1_GPIO_Port GPIOB
#define RFM95_DIO1_EXTI_IRQn EXTI15_10_IRQn
#define RFM95_DIO2_Pin GPIO_PIN_11
#define RFM95_DIO2_GPIO_Port GPIOB
#define RFM95_DIO2_EXTI_IRQn EXTI15_10_IRQn
#define status_led_Pin GPIO_PIN_12
#define status_led_GPIO_Port GPIOB
#define config_sw_Pin GPIO_PIN_13
#define config_sw_GPIO_Port GPIOB
#define VL53L3CX_GPIO1_INT_Pin GPIO_PIN_15
#define VL53L3CX_GPIO1_INT_GPIO_Port GPIOB
#define VL53L3CX_GPIO1_INT_EXTI_IRQn EXTI15_10_IRQn
#define VL53L3CX_XSHUT_Pin GPIO_PIN_8
#define VL53L3CX_XSHUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
