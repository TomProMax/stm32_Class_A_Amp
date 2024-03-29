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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void My_Error_Handle(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_GOOD_Pin GPIO_PIN_0
#define PWR_GOOD_GPIO_Port GPIOF
#define MUTE_Pin GPIO_PIN_1
#define MUTE_GPIO_Port GPIOF
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_3
#define LED_RED_GPIO_Port GPIOA
#define Headphone_in_Pin GPIO_PIN_6
#define Headphone_in_GPIO_Port GPIOA
#define DCDC_EN_Pin GPIO_PIN_7
#define DCDC_EN_GPIO_Port GPIOA
#define CFG1_Pin GPIO_PIN_1
#define CFG1_GPIO_Port GPIOB
#define CFG3_Pin GPIO_PIN_9
#define CFG3_GPIO_Port GPIOA
#define CFG2_Pin GPIO_PIN_10
#define CFG2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define Aux_in_Pin GPIO_PIN_14
#define Aux_in_GPIO_Port GPIOA

#define PWR_CON_Pin GPIO_PIN_13
#define PWR_CON_GPIO_Port GPIOA

#define Max_Time_out 100//休眠超时时间
extern uint16_t ADC_data[4]; //定义ADC数组
extern uint16_t sys_sleep_req;//系统睡眠请求
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
