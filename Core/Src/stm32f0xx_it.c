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
__IO uint8_t pwm_duty = 98;//ռ�ձ�����
__IO uint8_t pwm_cnt = 0;//�жϼ�����
__IO uint8_t pwm_dir = 0;//ռ�ձȱ任����
__IO uint8_t sleep_time_cnt = 0;//����ʱ�������
__IO uint8_t aux_in_cnt = 0;//��Ƶ���豸����ʱ�������
__IO uint8_t headphone_in_cnt = 0;//�޶�������ʱ�������
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

  //ֱ�����
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */
if (((ADC_data[2] < 1650) || (ADC_data[2] > 2400)  ) || ((ADC_data[3] < 1650) || ((ADC_data[3] > 2400)) ))//���ֱ������
  {
    My_Error_Handle();//�����ֱ������1.285-1.935
  }
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  //���PWM������ʵ��
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if(pwm_cnt++ == 100)//ÿ����100���жϸ���һ��ռ�ձ� ռ�ձȿ��ƾ�ȷ��1%
    {
      pwm_cnt = 0;//��λ
      if(pwm_dir)//�ж�Ŀǰ�ƹ�ռ�ձȱ仯����
     	  pwm_duty--;
      else
     	  pwm_duty++;
      if(pwm_duty == 99 || pwm_duty == 5)//ռ�ձȱ仯��Χ
     	  pwm_dir = !pwm_dir;//ת������
	  LED_GREEN_GPIO_Port-> BSRR = LED_GREEN_Pin;//ͬʱ���ߵ�ƽ������һ������
     }
  if(pwm_cnt == pwm_duty)//���Ŀǰ�����жϵĴ�������ռ�ձ�
      LED_GREEN_GPIO_Port-> BRR = LED_GREEN_Pin;//���͵�ƽ
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
  //1s�ж�
  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
  //�����볬ʱ���
  if((ADC_data[0] == 0) && (ADC_data[1] == 0)) //��⵽û����������
  {
    if(sleep_time_cnt++ >= Max_Time_out)//��ʱ
    {
       HAL_TIM_Base_Stop_IT(&htim14);//�Ѷ�ʱ���ص�
       sleep_time_cnt = 0;//��ռ���ֵ
       sys_sleep_req = 1;//����ϵͳ˯������
    }
  }
  else
	  sleep_time_cnt = 0;
  //����������
  if(HAL_GPIO_ReadPin(Headphone_in_GPIO_Port,Headphone_in_Pin) == GPIO_PIN_RESET)//�����⵽û�ж�������
  {
    if(headphone_in_cnt++ >= 2)//����3s��û�м�⵽��������
    { 
      headphone_in_cnt = 0;//��ռ���ֵ
      sys_sleep_req = 1;//����ϵͳ˯������
    }
  }
  else
	  headphone_in_cnt = 0;//����ж������� ��ô������һֱ����Ϊ0
  //AUX������
  if(HAL_GPIO_ReadPin(Aux_in_GPIO_Port,Aux_in_Pin) == GPIO_PIN_RESET)//�����⵽û�ж�������
  {
    if(aux_in_cnt++ >= 2)//����3s��û�м�⵽AUX����
    { 
      aux_in_cnt = 0;//��ռ���ֵ
      sys_sleep_req = 1;//����ϵͳ˯������
    }
  }
  else
	  aux_in_cnt = 0;//�����AUX���� ��ô������һֱ����Ϊ0
  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */
  //10s��ʱ��ʱ��
  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */
  My_Error_Handle();
  
  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
