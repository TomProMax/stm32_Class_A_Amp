/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
__IO uint8_t pwm_duty = 98;//占空比设置
__IO uint8_t pwm_cnt = 0;//中断计数器
__IO uint8_t pwm_dir = 0;//占空比变换方向
__IO uint8_t sleep_time_cnt = 0;//休眠时间计数器
__IO uint8_t aux_in_cnt = 0;//音频无设备输入时间计数器
__IO uint8_t headphone_in_cnt = 0;//无耳机插入时间计数器
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

  //直流检测
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */
if (((ADC_data[2] < 1650) || (ADC_data[2] > 2400)  ) || ((ADC_data[3] < 1650) || ((ADC_data[3] > 2400)) ))//检测直流分量
  {
    My_Error_Handle();//如果有直流分量1.285-1.935
  }
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  //软件PWM呼吸灯实现
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(pwm_cnt++ == 100)//每进入100次中断更改一次占空比 占空比控制精确度1%
    {
      pwm_cnt = 0;//复位
      if(pwm_dir)//判断目前灯光占空比变化方向
     	  pwm_duty--;
      else
     	  pwm_duty++;
      if(pwm_duty == 99 || pwm_duty == 5)//占空比变化范围
     	  pwm_dir = !pwm_dir;//转换方向
	  LED_GREEN_GPIO_Port-> BSRR = LED_GREEN_Pin;//同时拉高电平开启下一个周期
     }
  if(pwm_cnt == pwm_duty)//如果目前进入中断的次数等于占空比
      LED_GREEN_GPIO_Port-> BRR = LED_GREEN_Pin;//拉低电平
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
  //1s中断
  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
  //零输入超时检测
  if((ADC_data[0] == 0) && (ADC_data[1] == 0)) //检测到没有声音输入
  {
    if(sleep_time_cnt++ >= Max_Time_out)//超时
    {
       HAL_TIM_Base_Stop_IT(&htim14);//把定时器关掉
       sleep_time_cnt = 0;//清空计数值
       sys_sleep_req = 1;//发出系统睡眠请求
    }
  }
  else
	  sleep_time_cnt = 0;
  //耳机插入检测
  if(HAL_GPIO_ReadPin(Headphone_in_GPIO_Port,Headphone_in_Pin) == GPIO_PIN_RESET)//如果检测到没有耳机插入
  {
    if(headphone_in_cnt++ >= 2)//连续3s都没有检测到耳机插入
    { 
      headphone_in_cnt = 0;//清空计数值
      sys_sleep_req = 1;//发出系统睡眠请求
    }
  }
  else
	  headphone_in_cnt = 0;//如果有耳机插入 那么计数器一直保持为0
  //AUX插入检测
  if(HAL_GPIO_ReadPin(Aux_in_GPIO_Port,Aux_in_Pin) == GPIO_PIN_RESET)//如果检测到没有耳机插入
  {
    if(aux_in_cnt++ >= 2)//连续3s都没有检测到AUX插入
    { 
      aux_in_cnt = 0;//清空计数值
      sys_sleep_req = 1;//发出系统睡眠请求
    }
  }
  else
	  aux_in_cnt = 0;//如果有AUX插入 那么计数器一直保持为0
  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */
  //10s超时定时器
  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */
  My_Error_Handle();
  
  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
