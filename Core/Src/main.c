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
enum State //功放状态
{
  IDLE,//初始状态
  INIT,//上电初始化状态
  WORKING,//正常工作状态
  SLEEP,//休眠状态
  SHUTDOWN,//关闭状态
};
uint8_t  sys_state = IDLE;//状态
//ADC数组
uint16_t ADC_data[4] = {0}; //定义ADC数组
uint16_t sys_sleep_req = 0;//系统睡眠请求
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @description: PD诱骗器函数
 * @event: 
 * @return {*}
 */ 
void USB_PD_Init(void)
{
  //请求9V电压
  HAL_GPIO_WritePin(CFG1_GPIO_Port,CFG1_Pin,GPIO_PIN_RESET);//拉低CFG1
  HAL_GPIO_WritePin(CFG2_GPIO_Port,CFG2_Pin,GPIO_PIN_RESET);//拉低cfg2
  HAL_GPIO_WritePin(CFG3_GPIO_Port,CFG3_Pin,GPIO_PIN_RESET);//拉低cfg3
}

/**
 * @description: 解除PD诱骗函数
 * @event: 
 * @return {*}
 */ 
void USB_PD_DeInit(void)
{
  HAL_GPIO_WritePin(CFG1_GPIO_Port,CFG1_Pin,GPIO_PIN_SET);//拉高CFG1 请求5V电压
}

/**
 * @description: 错误处理函数
 * @event: 直流检测出现错误时进入 立刻关闭电源进行保护
 * @return {*}
 */  
void My_Error_Handle(void)
{
  __IO uint16_t time = 0xffff;
  __disable_irq();//关闭所有中断
  MUTE_GPIO_Port -> BRR = MUTE_Pin;//关闭继电器 静音
  DCDC_EN_GPIO_Port -> BRR = DCDC_EN_Pin;//关闭功放电源
  LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//关闭work灯
  while (1)
  {
    //LED红灯闪烁提示
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);//红灯闪烁
    for (uint8_t i = 0; i < 20; i++)
    {
      while (time--);
      time = 0xffff;
    }
    //不能使用systick进行延时，以免在中断中出错时卡死
  }
}

/**
 * @description: SWD接口复用 重新初始化函数
 * @event: 把SWD接口复用成普通GPIO使用
 * @return {*}
 */  
void SWD_GPIO_ReInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(Aux_in_GPIO_Port, Aux_in_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWR_CON_GPIO_Port, PWR_CON_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin : PtPin */
  //初始化信源插入检测端口
  GPIO_InitStruct.Pin = Aux_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;//浮空输入 使用外部1MΩ上拉电阻
  HAL_GPIO_Init(Aux_in_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : PtPin */
  //初始化电源开关输入IO
  GPIO_InitStruct.Pin = PWR_CON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//上拉输入模式
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
  HAL_Delay(1000);//防si
  SWD_GPIO_ReInit();//重新初始化SWD接口
  LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//熄灭绿色LED
  LED_RED_GPIO_Port -> BSRR = LED_RED_Pin;//熄灭红色LED
  if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_SET)
      sys_state = INIT;//跳转到初始化状态
  else
  {
	  USB_PD_DeInit();//关闭诱骗 省电
      sys_state = SHUTDOWN;
  }
  while (1)
  {
    switch(sys_state)//判断目前状态
    {   
    case INIT:
      HAL_TIM_Base_Start_IT(&htim3);//打开LED呼吸灯
      __HAL_TIM_CLEAR_IT(&htim16,TIM_IT_UPDATE);//清除中断标志位，防止使能定时器就进入中断
      HAL_TIM_Base_Start_IT(&htim16);//打开定时器计时10s 如果10s后还没有诱骗成功 就err
      while(HAL_GPIO_ReadPin(PWR_GOOD_GPIO_Port,PWR_GOOD_Pin) != GPIO_PIN_RESET)
		  HAL_Delay(10);//等待PWRGOOD输出低电平 诱骗成功
      HAL_TIM_Base_Stop_IT(&htim16);//诱骗成功 关闭定时器
      LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//熄灭绿色LED
      LED_RED_GPIO_Port -> BSRR = LED_RED_Pin;//熄灭红色LED
      HAL_ADC_Start_DMA(&hadc,(uint32_t *)&ADC_data, 4);//开启ADC
//	  if((!HAL_GPIO_ReadPin(Headphone_in_GPIO_Port,Headphone_in_Pin)) || (!HAL_GPIO_ReadPin(Aux_in_GPIO_Port,Aux_in_Pin)))//如果检测到没有插入
//	  {
//        sys_state = SLEEP;//切换到SLEEP状态
//		break;
//	  }
      DCDC_EN_GPIO_Port -> BSRR = DCDC_EN_Pin;//打开功放电源
	    HAL_Delay(1000);//防浪涌冲击
	    HAL_TIM_Base_Stop_IT (&htim3 );//关闭LED呼吸灯
	    HAL_TIM_Base_Start_IT(&htim1 );//启动直流分量检测定时器
	    HAL_TIM_Base_Start_IT(&htim14);//启动输入检测超时时间定时器
      MUTE_GPIO_Port      -> BSRR = MUTE_Pin;//打开继电器 输出音频
      LED_GREEN_GPIO_Port -> BRR  = LED_GREEN_Pin;//进入工作状态, 打开绿色LED指示灯
      sys_state = WORKING;//初始化完成,切换到工作状态
      break;
  
   case WORKING:
     if(sys_sleep_req)//判断是否有系统睡眠请求
     {
       sys_sleep_req = 0;//清零事件标志位
       HAL_TIM_Base_Stop_IT(&htim1);//关闭直流分量检测定时器 不然断电瞬间检测到直流异常会进入errorhandle
       HAL_TIM_Base_Stop_IT(&htim14);//输入检测超时时间定时器
       MUTE_GPIO_Port -> BRR = MUTE_Pin;//关闭继电器 静音
       HAL_Delay(100);//延时关闭电源防止POP音
       DCDC_EN_GPIO_Port -> BRR = DCDC_EN_Pin;//关闭功放电源
       HAL_TIM_Base_Start_IT(&htim3);//打开LED呼吸灯
       sys_state = SLEEP;//切换到SLEEP状态
     }
     if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_RESET)//如果读取到开关被关掉了 那么直接切换到SHUTDOWN
     {		  
       HAL_TIM_Base_Stop_IT(&htim1);//关闭直流分量检测定时器 不然断电瞬间检测到直流异常会进入errorhandle
       HAL_TIM_Base_Stop_IT(&htim14);//输入检测超时时间定时器
       MUTE_GPIO_Port -> BRR = MUTE_Pin;//关闭继电器 静音
       HAL_Delay(100);//延时关闭电源防止POP音
       DCDC_EN_GPIO_Port -> BRR = DCDC_EN_Pin;//关闭功放电源
		    LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//关闭LED
       USB_PD_DeInit();//关闭诱骗 省电
       HAL_ADC_Stop_DMA(&hadc);//Shutdown状态不需要输入检测 所以关闭adc
       sys_state = SHUTDOWN;//切换到SHUTDOWN状态
     }
      break;

   case SLEEP:
	  if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_RESET)//如果读取到开关被关掉了 那么直接切换到SHUTDOWN
     {
       HAL_TIM_Base_Stop_IT(&htim3);//关闭LED呼吸灯
       LED_GREEN_GPIO_Port -> BSRR = LED_GREEN_Pin;//关闭绿色LED
       USB_PD_DeInit();//关闭诱骗 省电
       HAL_ADC_Stop_DMA(&hadc);//Shutdown状态不需要输入检测 所以关闭adc
       sys_state = SHUTDOWN;//切换到SHUTDOWN状态
     }
     if((ADC_data[0] != 0) && (ADC_data[1] != 0) && (HAL_GPIO_ReadPin(Headphone_in_GPIO_Port,Headphone_in_Pin)))//唤醒
		    sys_state = INIT;//切换到初始化状态 重新初始化系统
        
     break;

   case SHUTDOWN:
     if(HAL_GPIO_ReadPin(PWR_CON_GPIO_Port,PWR_CON_Pin) == GPIO_PIN_RESET)//读取开关是否被打开
	 {
	   USB_PD_Init();//请求诱骗9V电压
       sys_state = INIT;//切换到初始化状态 重新初始化系统
	 }
     break;
   
    default:
      My_Error_Handle();//进入错误处理函数
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
  __disable_irq();//关闭所有中断
  while (1)
  {
	  HAL_NVIC_SystemReset();//系统重启
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
