/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES_MIN     -32768
#define SAMPLES_MAX     32767
#define RANGE_MAX       65535
#define RANGE_MIN       0
#define MARGIN          2728
#define NO_OF_SAMPLES   1000
#define MAX_VAL         2*((RANGE_MAX+1)/2 + MARGIN)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

//1000 samples for single cycle of sine wave
int16_t samples[1000] = {0, 206, 412, 618, 823, 1029, 1235, 1441, 1646, 1852, 2057, 2263, 2468, 2673, 2879, 3084, 3289, 3493, 3698, 3902, 4107, 4311, 4515, 4719, 4922, 5126, 5329, 5532, 5735, 5938, 6140, 6342, 6544, 6746, 6947, 7148, 7349, 7549, 7749, 7949, 8149, 8348, 8547, 8746, 8944, 9142, 9339, 9536, 9733, 9930, 10126, 10321, 10516, 10711, 10905, 11099, 11293, 11486, 11679, 11871, 12062, 12254, 12444, 12634, 12824, 13013, 13202, 13390, 13578, 13765, 13952, 14138, 14323, 14508, 14692, 14876, 15059, 15242, 15424, 15605, 15786, 15966, 16145, 16324, 16502, 16680, 16857, 17033, 17208, 17383, 17557, 17731, 17904, 18076, 18247, 18418, 18588, 18757, 18925, 19093, 19260, 19426, 19592, 19756, 19920, 20083, 20245, 20407, 20568, 20727, 20886, 21045, 21202, 21359, 21514, 21669, 21823, 21976, 22129, 22280, 22431, 22580, 22729, 22877, 23024, 23170, 23315, 23459, 23602, 23745, 23886, 24027, 24166, 24305, 24442, 24579, 24715, 24849, 24983, 25116, 25247, 25378, 25508, 25637, 25764, 25891, 26017, 26141, 26265, 26388, 26509, 26630, 26749, 26867, 26985, 27101, 27216, 27330, 27443, 27555, 27666, 27776, 27885, 27992, 28099, 28204, 28308, 28411, 28513, 28614, 28714, 28813, 28910, 29006, 29102, 29196, 29289, 29380, 29471, 29560, 29648, 29736, 29821, 29906, 29990, 30072, 30153, 30233, 30312, 30390, 30466, 30541, 30615, 30688, 30759, 30830, 30899, 30967, 31034, 31099, 31163, 31226, 31288, 31349, 31408, 31466, 31523, 31578, 31633, 31686, 31738, 31788, 31837, 31886, 31932, 31978, 32022, 32065, 32107, 32147, 32187, 32225, 32261, 32297, 32331, 32364, 32395, 32425, 32454, 32482, 32509, 32534, 32558, 32580, 32602, 32622, 32640, 32658, 32674, 32689, 32702, 32715, 32726, 32735, 32744, 32751, 32757, 32761, 32764, 32766, 32767, 32766, 32764, 32761, 32757, 32751, 32744, 32735, 32726, 32715, 32702, 32689, 32674, 32658, 32640, 32622, 32602, 32580, 32558, 32534, 32509, 32482, 32454, 32425, 32395, 32364, 32331, 32297, 32261, 32225, 32187, 32147, 32107, 32065, 32022, 31978, 31932, 31886, 31837, 31788, 31738, 31686, 31633, 31578, 31523, 31466, 31408, 31349, 31288, 31226, 31163, 31099, 31034, 30967, 30899, 30830, 30759, 30688, 30615, 30541, 30466, 30390, 30312, 30233, 30153, 30072, 29990, 29906, 29821, 29736, 29648, 29560, 29471, 29380, 29289, 29196, 29102, 29006, 28910, 28813, 28714, 28614, 28513, 28411, 28308, 28204, 28099, 27992, 27885, 27776, 27666, 27555, 27443, 27330, 27216, 27101, 26985, 26867, 26749, 26630, 26509, 26388, 26265, 26141, 26017, 25891, 25764, 25637, 25508, 25378, 25247, 25116, 24983, 24849, 24715, 24579, 24442, 24305, 24166, 24027, 23886, 23745, 23602, 23459, 23315, 23170, 23024, 22877, 22729, 22580, 22431, 22280, 22129, 21976, 21823, 21669, 21514, 21359, 21202, 21045, 20886, 20727, 20568, 20407, 20245, 20083, 19920, 19756, 19592, 19426, 19260, 19093, 18925, 18757, 18588, 18418, 18247, 18076, 17904, 17731, 17557, 17383, 17208, 17033, 16857, 16680, 16502, 16324, 16145, 15966, 15786, 15605, 15424, 15242, 15059, 14876, 14692, 14508, 14323, 14138, 13952, 13765, 13578, 13390, 13202, 13013, 12824, 12634, 12444, 12254, 12062, 11871, 11679, 11486, 11293, 11099, 10905, 10711, 10516, 10321, 10126, 9930, 9733, 9536, 9339, 9142, 8944, 8746, 8547, 8348, 8149, 7949, 7749, 7549, 7349, 7148, 6947, 6746, 6544, 6342, 6140, 5938, 5735, 5532, 5329, 5126, 4922, 4719, 4515, 4311, 4107, 3902, 3698, 3493, 3289, 3084, 2879, 2673, 2468, 2263, 2057, 1852, 1646, 1441, 1235, 1029, 823, 618, 412, 206, 0, -206, -412, -618, -823, -1029, -1235, -1441, -1646, -1852, -2057, -2263, -2468, -2673, -2879, -3084, -3289, -3493, -3698, -3902, -4107, -4311, -4515, -4719, -4922, -5126, -5329, -5532, -5735, -5938, -6140, -6342, -6544, -6746, -6947, -7148, -7349, -7549, -7749, -7949, -8149, -8348, -8547, -8746, -8944, -9142, -9339, -9536, -9733, -9930, -10126, -10321, -10516, -10711, -10905, -11099, -11293, -11486, -11679, -11871, -12062, -12254, -12444, -12634, -12824, -13013, -13202, -13390, -13578, -13765, -13952, -14138, -14323, -14508, -14692, -14876, -15059, -15242, -15424, -15605, -15786, -15966, -16145, -16324, -16502, -16680, -16857, -17033, -17208, -17383, -17557, -17731, -17904, -18076, -18247, -18418, -18588, -18757, -18925, -19093, -19260, -19426, -19592, -19756, -19920, -20083, -20245, -20407, -20568, -20727, -20886, -21045, -21202, -21359, -21514, -21669, -21823, -21976, -22129, -22280, -22431, -22580, -22729, -22877, -23024, -23170, -23315, -23459, -23602, -23745, -23886, -24027, -24166, -24305, -24442, -24579, -24715, -24849, -24983, -25116, -25247, -25378, -25508, -25637, -25764, -25891, -26017, -26141, -26265, -26388, -26509, -26630, -26749, -26867, -26985, -27101, -27216, -27330, -27443, -27555, -27666, -27776, -27885, -27992, -28099, -28204, -28308, -28411, -28513, -28614, -28714, -28813, -28910, -29006, -29102, -29196, -29289, -29380, -29471, -29560, -29648, -29736, -29821, -29906, -29990, -30072, -30153, -30233, -30312, -30390, -30466, -30541, -30615, -30688, -30759, -30830, -30899, -30967, -31034, -31099, -31163, -31226, -31288, -31349, -31408, -31466, -31523, -31578, -31633, -31686, -31738, -31788, -31837, -31886, -31932, -31978, -32022, -32065, -32107, -32147, -32187, -32225, -32261, -32297, -32331, -32364, -32395, -32425, -32454, -32482, -32509, -32534, -32558, -32580, -32602, -32622, -32640, -32658, -32674, -32689, -32702, -32715, -32726, -32735, -32744, -32751, -32757, -32761, -32764, -32766, -32767, -32766, -32764, -32761, -32757, -32751, -32744, -32735, -32726, -32715, -32702, -32689, -32674, -32658, -32640, -32622, -32602, -32580, -32558, -32534, -32509, -32482, -32454, -32425, -32395, -32364, -32331, -32297, -32261, -32225, -32187, -32147, -32107, -32065, -32022, -31978, -31932, -31886, -31837, -31788, -31738, -31686, -31633, -31578, -31523, -31466, -31408, -31349, -31288, -31226, -31163, -31099, -31034, -30967, -30899, -30830, -30759, -30688, -30615, -30541, -30466, -30390, -30312, -30233, -30153, -30072, -29990, -29906, -29821, -29736, -29648, -29560, -29471, -29380, -29289, -29196, -29102, -29006, -28910, -28813, -28714, -28614, -28513, -28411, -28308, -28204, -28099, -27992, -27885, -27776, -27666, -27555, -27443, -27330, -27216, -27101, -26985, -26867, -26749, -26630, -26509, -26388, -26265, -26141, -26017, -25891, -25764, -25637, -25508, -25378, -25247, -25116, -24983, -24849, -24715, -24579, -24442, -24305, -24166, -24027, -23886, -23745, -23602, -23459, -23315, -23170, -23024, -22877, -22729, -22580, -22431, -22280, -22129, -21976, -21823, -21669, -21514, -21359, -21202, -21045, -20886, -20727, -20568, -20407, -20245, -20083, -19920, -19756, -19592, -19426, -19260, -19093, -18925, -18757, -18588, -18418, -18247, -18076, -17904, -17731, -17557, -17383, -17208, -17033, -16857, -16680, -16502, -16324, -16145, -15966, -15786, -15605, -15424, -15242, -15059, -14876, -14692, -14508, -14323, -14138, -13952, -13765, -13578, -13390, -13202, -13013, -12824, -12634, -12444, -12254, -12062, -11871, -11679, -11486, -11293, -11099, -10905, -10711, -10516, -10321, -10126, -9930, -9733, -9536, -9339, -9142, -8944, -8746, -8547, -8348, -8149, -7949, -7749, -7549, -7349, -7148, -6947, -6746, -6544, -6342, -6140, -5938, -5735, -5532, -5329, -5126, -4922, -4719, -4515, -4311, -4107, -3902, -3698, -3493, -3289, -3084, -2879, -2673, -2468, -2263, -2057, -1852, -1646, -1441, -1235, -1029, -823, -618, -412, -206};
volatile int16_t counter = 0;           //Variable that stores the count (0 - 1000) for each sample of the sine wave
int16_t k = 256;                        //Variable for volume control. It's range is 128 - 511
int32_t pureSample;                     //Variable that stores the data of an indivisual sample

//Below are the variables that are used in the algorithm to produce the PDM output
int32_t s1 = 0,s2 = 0,s3 = 0,a1[2] = {0}, a2[2] = {0},y = 0,max = 0, min = 0;
signed char y_bit;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    k++;                //For emulating volume control
    HAL_Delay(20);
    if(k == 511)
      k = 128;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 12;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Clock Config
//Clock frequency:      48Mhz
//Prescalar:            79
//Period:               12
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)     //Timer interrupt that is called every 20 us (frequency of 50 kHz)
{
  
  if(htim->Instance == TIM3)
  {
      pureSample = samples[counter];
      pureSample *= k;                          //For Volume Control
      pureSample = pureSample>>9;

      a1[1] = a1[0] + s1;                       //Algorithm for calculating PDM output for each sample
      a2[1] = a2[0] + s2;
      s3 = a2[1] + pureSample;
      if(s3 > 32768)
      {
        GPIOA ->ODR |= (1<<6);                  //Positive line of PDM outpu is PA6
        y_bit = 2;
      }
      else if(s3 < -32768)
      {
        GPIOA ->ODR |= (1<<5);                  //Negative line of PDM outpu is PA5
        y_bit = -2;
      }
      else 
      {
        GPIOA ->ODR &= ~(1<<6);
        GPIOA ->ODR &= ~(1<<5);
        y_bit = 0;
      }
      y = y_bit << 15 ;                         //scale by 2^15
      
      s1 = pureSample - y;
      s2 = a1[1] + 2*pureSample - 2*y;
      a1[0] = a1[1];
      a2[0] = a2[1];
       
      counter++;                                //Incrementing the counter
      counter = counter%(NO_OF_SAMPLES);        //Counter goes from 0 to 1000 and then resets to 0
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
