/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

#include "dashboardDisplay.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/** HALF PERIODICITY */
#define BLINKER_PERIODICITY 8

#define WHEEL_RADIUS_CM 30.0f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim14;
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
static void blinker_loop(DashboardDisplay::infos_t * lcd);
static void get_adc_values(float * acc, float* break1, float * break2);
static void brake_loop(DashboardDisplay::infos_t * lcd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static DashboardDisplay::infos_t lcd_ext;
static DashboardDisplay lcd;


// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim14)
  {

    brake_loop(&lcd_ext);

    blinker_loop(&lcd_ext);
    lcd.displayInfos(lcd_ext);
    // Disabled
    /*
    lcd_ext.battery++;
    lcd_ext.battery = lcd_ext.battery % 5;
    */
  }
}
static uint32_t elapsed_tick = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static int cpt;
  static uint32_t last_tick = 0;
  static bool first = true;
  if (GPIO_Pin == GPIO_PIN_6)
  {
    if (HAL_GetTick() - last_tick < 100 && !first)
    {
      first = false;
      return;
    }
    cpt++;
    if (cpt == 1)
    {
      elapsed_tick = HAL_GetTick() - last_tick;
      lcd_ext.B_mph = true;
      cpt = 0;
      last_tick = HAL_GetTick();
    }
    first = false;
  }
}

static void blinker_loop(DashboardDisplay::infos_t *lcd)
{

  static uint8_t blinker_blink = 0;

  static int32_t cpt_countdown = 4;

  bool B_status_left = false;
  bool B_status_right = false;

  if (cpt_countdown == 0)
  {
    blinker_blink = !blinker_blink;
  }

  B_status_left = true;  //!HAL_GPIO_ReadPin (GPIOB, unkA_Pin);
  B_status_right = true; //!HAL_GPIO_ReadPin (GPIOA, unkB_Pin);

  // If cpt_countdown is set to zero, then
  lcd->B_left = blinker_blink && B_status_left;
  lcd->B_right = blinker_blink && B_status_right;

  // Disabled
/*
  HAL_GPIO_WritePin(GPIOB, outA_Pin, (GPIO_PinState)lcd->B_left);
  HAL_GPIO_WritePin(GPIOA, outB_Pin, (GPIO_PinState)lcd->B_right);
*/
  cpt_countdown = (BLINKER_PERIODICITY + (cpt_countdown - 1)) % BLINKER_PERIODICITY;
}
static void get_adc_values(float *acc, float *break1, float *break2)
{
  uint16_t raw;
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  raw = HAL_ADC_GetValue(&hadc);
  *break1 = 3.3 * (5.0 / 3.388) * raw / 4096.0;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  raw = HAL_ADC_GetValue(&hadc);
  *break2 = 3.3 * (5.0 / 3.388) * raw / 4096.0;

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  raw = HAL_ADC_GetValue(&hadc);
  *acc = 3.3 * (5.0 / 3.388) * raw / 4096.0;
}
static void brake_loop(DashboardDisplay::infos_t *lcd)
{

  float acc;
  float break1;
  float break2;
  get_adc_values(&acc, &break1, &break2);

  static int32_t cpt_half_periodicity = 5;
  static int32_t cpt_countdown = 4;
  static bool light_blink = false;
  bool no_blink = false;

  lcd->value = 2.0f * 3.14f * WHEEL_RADIUS_CM * 0.01f * 0.001 / ((elapsed_tick / 1000.0f) / 3600.0f); //acc;//3.3*(5.0/3.388)*raw/4096.0;//lcd_ext.value +0.1f;
                                                                                                      //lcd->value =lcd->value/10.0f;

    if(break1<0.9)
	{
		//No blink
		no_blink=true;
	}
	else if(break1<2.5)
	{
		cpt_half_periodicity = 3;
	}
	else if(break1<3.5)
	{
		cpt_half_periodicity = 2;
	}
	else
	{
		cpt_half_periodicity = 1;
	}

    if(cpt_countdown == 0)
	{
		light_blink = !light_blink;
	}

    if(no_blink)
    {
    	light_blink = true;
    }

	lcd->B_light = light_blink;

  // turn on/off light
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (GPIO_PinState)light_blink);

  cpt_countdown = (cpt_half_periodicity + (cpt_countdown - 1)) % cpt_half_periodicity;
}

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
  MX_CAN_Init();
  MX_ADC_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  // 5 : DIN
  // 6 : CLK
  // 7 : STB
  HAL_TIM_Base_Start_IT(&htim14);
  // set to zero by defaut

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, outA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, outB_Pin, GPIO_PIN_RESET);



  lcd.start_display();

  lcd_ext.value = 12.3f;
  lcd_ext.B_reserved = 1;
  lcd_ext.B_maint_green = 1;
  lcd_ext.battery = 1;
  lcd_ext.B_mph = 0;
  lcd_ext.B_kmh = 1;
  lcd_ext.B_left = 1;
  lcd_ext.B_light = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */
  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER>>1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 5;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|outA_Pin|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|outB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CANH_Pin CANL_Pin */
  GPIO_InitStruct.Pin = CANH_Pin|CANL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : unkB_Pin */
  GPIO_InitStruct.Pin = unkB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;// GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(unkB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 outA_Pin PB5 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|outA_Pin|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 outB_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|outB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : unkA_Pin */
  GPIO_InitStruct.Pin = unkA_Pin;
  GPIO_InitStruct.Mode =  GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(unkA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
