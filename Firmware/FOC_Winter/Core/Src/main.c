/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "Usrt.h"
#include "delay_JR.h"
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
#include "GUI.h"

#include "Base_Motor_Control.h"
#include "InlineCurrentSense.h"
#include "Inline_ADC.h"
#include "FOC_Motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_Reg_Buffer 1
#define ADC_REF_V                   (float)(3.3f)
#define R_HIGH  20.0F
#define R_LOW		1.0F
#define SAMPLE_VOL_CON_FACTOR 	 ADC_REF_V /4095.0F / (R_LOW/(R_LOW+R_HIGH))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Target = 0;
char Mode = 0;
uint16_t angle_Row;
float angle,vel;

//uint16_t ADC_Reg_Value[ADC_Reg_Buffer] = {0};
float ADC_Reg_Value;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  delay_init(168);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,PWM_PERIOD-50);
  
  AS5600_lnit();             //编码器初始化
  Read_InlineCurrentSense(0.01,50,Channel_0,Channel_1,Channel_2);                //设定采样电阻的阻值，运放倍数，ABC�?
  InlineCurrentSense_Init();                                                    //ADC初始化和偏置电压校准
  LPF_init();             
  PID_init();               
  Base_parameter_init();
   Controller = Type_torque;
  PID_Current_d.P=0.6;       
	PID_Current_d.I=0;         
	PID_Current_q.P=0.6;
	PID_Current_q.I=0;
	PID_Velocity.P=0.55;      
	PID_Velocity.I=5;
	PID_Angle.P=20;              //位置环参数，只需P参数
	PID_Velocity.output_ramp=0;
	LPF_velocity.Tf=0.01;      //Tf设置小一点，配合爬升斜率设置，�?�度切换更平稳；如果没有爬升模式的斜率限制，Tf太小电机容易抖动

  Motor_Init();
  Motor_FOC_Init(0,UNKNOWN);
  printf("Motor ready.\r\n");
  
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);       //打开IDLE中断
  HAL_UART_Receive_DMA(&huart1,rx1_buffer,BUFFER_SIZE);    //�?启DMA接受
  

  LCD_Init();//LCD初始
  LCD_Fill(0,0,LCD_W,LCD_H,LGRAY);

//  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  angle_Row = AS5600_ReadTwoByte(_raw_ang_hi, _raw_ang_lo);
////	  angle_Row = AS5600_Get_Raw_Angle();
//	  angle = AS5600_Get_Angle();
//	  vel = Get_Velocity();
//	  printf("angle_raw = %d,angle = %f,vel = %f\r\n",angle_Row,angle,vel);

	if(rec1_end_flag)
	{
		Usart1_Handle();
	}
//	Move(Target);
//	LoopFOC();
	GUI_display();
//	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
//	  HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
    {
//		if(rec1_end_flag)
//	{
//		Usart1_Handle();
//	}
	Move(Target);
	LoopFOC();
	}
    if(htim->Instance == TIM3)
    {
//		if(rec1_end_flag)
//	{
//		Usart1_Handle();
//	}
	HAL_ADC_Start(&hadc1);
	ADC_Reg_Value = HAL_ADC_GetValue(&hadc1);
	ADC_Reg_Value = ADC_Reg_Value * SAMPLE_VOL_CON_FACTOR;
	Move(Target);
	LoopFOC();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
