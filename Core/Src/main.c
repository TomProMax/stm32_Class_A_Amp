/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
enum State //����״̬
{
  IDLE,//��ʼ״̬
  INIT,//�ϵ��ʼ��״̬
  WORKING,//��������״̬
  SLEEP,//����״̬
  SHUTDOWN,//�ر�״̬
};
uint8_t  sys_state = IDLE;//״̬
//ADC����
uint16_t ADC_data[4] = {0}; //����ADC����
uint16_t sys_sleep_req = 0;//ϵͳ˯������
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @description: PD��ƭ������
 * @event: 
 * @return {*}
 */ 
void USB_PD_Init(void)
{
  //����9V��ѹ
  HAL_GPIO_WritePin(CFG1_GPIO_Port,CFG1_Pin,GPIO_PIN_RESET);//����CFG1
  HAL_GPIO_WritePin(CFG2_GPIO_Port,CFG2_Pin,GPIO_PIN_RESET);//����cfg2
  HAL_GPIO_WritePin(CFG3_GPIO_Port,CFG3_Pin,GPIO_PIN_RESET);//����cfg3
}

/**
 * @description: ���PD��ƭ����
 * @event: 
 * @return {*}
 */ 
void USB_PD_DeInit(void)
{
  HAL_GPIO_WritePin(CFG1_GPIO_Port,CFG1_Pin,GPIO_PIN_SET);//����CFG1 ����5V��ѹ
}

/**
 * @description: ��������
 * @event: ֱ�������ִ���ʱ���� ���̹رյ�Դ���б���
 * @return {*}
 */  
void My_Error_Handle(void)
{
  __IO uint16_t time = 0xffff;
  __disable_irq();//�ر������ж�
  MUTE_GPIO_Port -> BRR = MUTE_Pin;//�رռ̵��� ����
  DCDC_EN_GPIO_Port -> BRR = DCDC_EN_Pin;//�رչ��ŵ�Դ
  LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//�ر�work��
  while (1)
  {
    //LED�����˸��ʾ
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);//�����˸
    for (uint8_t i = 0; i < 20; i++)
    {
      while (time--);
      time = 0xffff;
    }
    //����ʹ��systick������ʱ���������ж��г���ʱ����
  }
}

/**
 * @description: SWD�ӿڸ��� ���³�ʼ������
 * @event: ��SWD�ӿڸ��ó���ͨGPIOʹ��
 * @return {*}
 */  
void SWD_GPIO_ReInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(Aux_in_GPIO_Port, Aux_in_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWR_CON_GPIO_Port, PWR_CON_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin : PtPin */
  //��ʼ����Դ������˿�
  GPIO_InitStruct.Pin = Aux_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;//�������� ʹ���ⲿ1M����������
  HAL_GPIO_Init(Aux_in_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : PtPin */
  //��ʼ����Դ��������IO
  GPIO_InitStruct.Pin = PWR_CON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//��������ģʽ
  HAL_GPIO_Init(PWR_CON_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);//��si
  SWD_GPIO_ReInit();//���³�ʼ��SWD�ӿ�
  LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//Ϩ����ɫLED
  LED_RED_GPIO_Port -> BSRR = LED_RED_Pin;//Ϩ���ɫLED
  if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_SET)
      sys_state = INIT;//��ת����ʼ��״̬
  else
  {
	  USB_PD_DeInit();//�ر���ƭ ʡ��
      sys_state = SHUTDOWN;
  }
  while (1)
  {
    switch(sys_state)//�ж�Ŀǰ״̬
    {   
    case INIT:
      HAL_TIM_Base_Start_IT(&htim3);//��LED������
      __HAL_TIM_CLEAR_IT(&htim16,TIM_IT_UPDATE);//����жϱ�־λ����ֹʹ�ܶ�ʱ���ͽ����ж�
      HAL_TIM_Base_Start_IT(&htim16);//�򿪶�ʱ����ʱ10s ���10s��û����ƭ�ɹ� ��err
      while(HAL_GPIO_ReadPin(PWR_GOOD_GPIO_Port,PWR_GOOD_Pin) != GPIO_PIN_RESET)
		  HAL_Delay(10);//�ȴ�PWRGOOD����͵�ƽ ��ƭ�ɹ�
      HAL_TIM_Base_Stop_IT(&htim16);//��ƭ�ɹ� �رն�ʱ��
      LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//Ϩ����ɫLED
      LED_RED_GPIO_Port -> BSRR = LED_RED_Pin;//Ϩ���ɫLED
      HAL_ADC_Start_DMA(&hadc,(uint32_t *)&ADC_data, 4);//����ADC
//	  if((!HAL_GPIO_ReadPin(Headphone_in_GPIO_Port,Headphone_in_Pin)) || (!HAL_GPIO_ReadPin(Aux_in_GPIO_Port,Aux_in_Pin)))//�����⵽û�в���
//	  {
//        sys_state = SLEEP;//�л���SLEEP״̬
//		break;
//	  }
      DCDC_EN_GPIO_Port -> BSRR = DCDC_EN_Pin;//�򿪹��ŵ�Դ
	    HAL_Delay(1000);//����ӿ���
	    HAL_TIM_Base_Stop_IT (&htim3 );//�ر�LED������
	    HAL_TIM_Base_Start_IT(&htim1 );//����ֱ��������ⶨʱ��
	    HAL_TIM_Base_Start_IT(&htim14);//���������ⳬʱʱ�䶨ʱ��
      MUTE_GPIO_Port      -> BSRR = MUTE_Pin;//�򿪼̵��� �����Ƶ
      LED_GREEN_GPIO_Port -> BRR  = LED_GREEN_Pin;//���빤��״̬, ����ɫLEDָʾ��
      sys_state = WORKING;//��ʼ�����,�л�������״̬
      break;
  
   case WORKING:
     if(sys_sleep_req)//�ж��Ƿ���ϵͳ˯������
     {
       sys_sleep_req = 0;//�����¼���־λ
       HAL_TIM_Base_Stop_IT(&htim1);//�ر�ֱ��������ⶨʱ�� ��Ȼ�ϵ�˲���⵽ֱ���쳣�����errorhandle
       HAL_TIM_Base_Stop_IT(&htim14);//�����ⳬʱʱ�䶨ʱ��
       MUTE_GPIO_Port -> BRR = MUTE_Pin;//�رռ̵��� ����
       HAL_Delay(100);//��ʱ�رյ�Դ��ֹPOP��
       DCDC_EN_GPIO_Port -> BRR = DCDC_EN_Pin;//�رչ��ŵ�Դ
       HAL_TIM_Base_Start_IT(&htim3);//��LED������
       sys_state = SLEEP;//�л���SLEEP״̬
     }
     if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_RESET)//�����ȡ�����ر��ص��� ��ôֱ���л���SHUTDOWN
     {		  
       HAL_TIM_Base_Stop_IT(&htim1);//�ر�ֱ��������ⶨʱ�� ��Ȼ�ϵ�˲���⵽ֱ���쳣�����errorhandle
       HAL_TIM_Base_Stop_IT(&htim14);//�����ⳬʱʱ�䶨ʱ��
       MUTE_GPIO_Port -> BRR = MUTE_Pin;//�رռ̵��� ����
       HAL_Delay(100);//��ʱ�رյ�Դ��ֹPOP��
       DCDC_EN_GPIO_Port -> BRR = DCDC_EN_Pin;//�رչ��ŵ�Դ
		    LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//�ر�LED
       USB_PD_DeInit();//�ر���ƭ ʡ��
       HAL_ADC_Stop_DMA(&hadc);//Shutdown״̬����Ҫ������ ���Թر�adc
       sys_state = SHUTDOWN;//�л���SHUTDOWN״̬
     }
      break;

   case SLEEP:
	  if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_RESET)//�����ȡ�����ر��ص��� ��ôֱ���л���SHUTDOWN
     {
       HAL_TIM_Base_Stop_IT(&htim3);//�ر�LED������
       LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//�ر���ɫLED
       USB_PD_DeInit();//�ر���ƭ ʡ��
       HAL_ADC_Stop_DMA(&hadc);//Shutdown״̬����Ҫ������ ���Թر�adc
       sys_state = SHUTDOWN;//�л���SHUTDOWN״̬
     }
     if((ADC_data[0] != 0) && (ADC_data[1] != 0) && (HAL_GPIO_ReadPin(Headphone_in_GPIO_Port,Headphone_in_Pin)))//����
		    sys_state = INIT;//�л�����ʼ��״̬ ���³�ʼ��ϵͳ
        
     break;

   case SHUTDOWN:
     if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_RESET)//��ȡ�����Ƿ񱻴�
	 {
	   USB_PD_Init();//������ƭ9V��ѹ
       sys_state = INIT;//�л�����ʼ��״̬ ���³�ʼ��ϵͳ
	 }
     break;
   
    default:
      My_Error_Handle();//�����������
      break;
    }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();//�ر������ж�
  while (1)
  {
	  HAL_NVIC_SystemReset();//ϵͳ����
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
