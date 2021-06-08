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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
  char B_kmh;
  char B_mph;
  char B_bluetooth;
  char B_left;
  char B_right;
  char B_light;
  char B_reserved;
  char B_maint_red;
  char B_maint_green;
  char battery;
  float value;
} lcd_t;

typedef struct{
  char led_array[5];
} lcd_internal_t;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
#define TM16XX_CMD_DATA_AUTO 0x40
#define TM16XX_CMD_DATA_READ 0x42			// command to read data used on two wire interfaces of TM1637
#define TM16XX_CMD_DATA_FIXED 0x44
#define TM16XX_CMD_DISPLAY 0x80
#define TM16XX_CMD_ADDRESS 0xC0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send(char data)
{
  for (int i = 0; i < 8; i++) {

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, data & 1 ? GPIO_PIN_SET: GPIO_PIN_RESET);

    data >>= 1;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  }

}

void start(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

void stop(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}
void sendCommand(char cmd)
{
	start();
  send(cmd);
  stop();
}

void sendData(char address, char data)
{
  sendCommand(TM16XX_CMD_DATA_FIXED);							// use fixed addressing for data
	start();
  send(TM16XX_CMD_ADDRESS | address);						// address command + address
  send(data);
  stop();
}



void clearDisplay()
{
  for(char nPos=0; nPos<10; nPos++)
  {
	  // all OFF
	  sendData(nPos << 1, 0);
	  sendData((nPos << 1) | 1, 0);
	  // all ON
	  //sendData(nPos << 1, 0b11111111);
	  //sendData((nPos << 1) | 1, 0b00000011);
  }
}

void setupDisplay()
{
  sendCommand(TM16XX_CMD_DISPLAY | 8  | 7);
}

char conv_seven[10] = {0x7E, 48, 109, 121, 51, 91, 95, 112, 127, 123};
void convertToSeven(int number, char* out)
{
	*out = *out & 0x80;
	*out = *out | conv_seven[number];

}

typedef struct{

	 union{
		 struct{
			char B_maint_green:1;
			char B_maint_red:1;
			char B_reserved:1;
			char B_bat_3 :1;
			char B_bat_2 :1;
			char B_bat_1 :1;
			char B_bat_0 :1;
			char B_padding:1;
		} bits;
		char word;

	} bottom;

	 union{
		 struct{
			char B_kmh:1;
			char B_mph:1;
			char B_dot:1;
			char B_right :1;
			char B_light :1;
			char B_bluetooth :1;
			char B_left :1;
			char B_padding:1;
		} bits;
		char word;

	} up;


} info_led_t;



void convertToInternal(lcd_t in, lcd_internal_t *out)
{

  int tenth = ((int)in.value/10)%10;
  convertToSeven(tenth,&(out->led_array[4]));
    int unit = ((int)in.value)%10;
  convertToSeven(unit,&(out->led_array[3]));
      int dec = ((int)(in.value*10))%10;
  convertToSeven(dec,&(out->led_array[2]));
  info_led_t info_led;
  info_led.bottom.bits.B_maint_green = in.B_maint_green;
  info_led.bottom.bits.B_maint_red = in.B_maint_red;
  info_led.bottom.bits.B_reserved = in.B_reserved;
  info_led.bottom.bits.B_bat_0 = in.battery>=1;
  info_led.bottom.bits.B_bat_1 = in.battery>=2;
  info_led.bottom.bits.B_bat_2 = in.battery>=3;
  info_led.bottom.bits.B_bat_3 = in.battery>=4;

  out->led_array[0]= info_led.bottom.word;

  info_led.up.bits.B_kmh = in.B_kmh;
  info_led.up.bits.B_mph = in.B_mph;


  info_led.up.bits.B_dot = 1;

  info_led.up.bits.B_right = in.B_right;
  info_led.up.bits.B_light = in.B_light;
  info_led.up.bits.B_left = in.B_left;
  info_led.up.bits.B_bluetooth = in.B_bluetooth;
  //info_led.up.bits.B_ = in.B_mph;

  out->led_array[1]= info_led.up.word;

}

void displayLcd(lcd_t lcd_ext)
{
  static lcd_internal_t lcd_int;
  convertToInternal(lcd_ext,&lcd_int);
  for(int i=0;i<5;i+=1)
  {
    sendData(i*2, lcd_int.led_array[i]);
  }
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
  /* USER CODE BEGIN 2 */
  // 5 : DIN
  // 6 : CLK
  // 7 : STB

  // set to zero by defaut
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);



  sendCommand(0x01);
  setupDisplay();
  clearDisplay();
  // Because 5 "digits"
  sendData(2, 0x01);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  static lcd_t lcd_ext;
  lcd_ext.value = 12.3f;
  lcd_ext.B_reserved = 1;
  lcd_ext.B_maint_green = 1;
  lcd_ext.battery = 1;
  lcd_ext.B_mph = 0;
  lcd_ext.B_kmh = 1;
  lcd_ext.B_left = 1;
  lcd_ext.B_light = 1;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  long int count = 300000;
	 	  for (long int i = 0; i < count; ++i) {
	 	      count--;
	 	  }

	 	 lcd_ext.value = lcd_ext.value +0.1f;
	  displayLcd(lcd_ext);
	  lcd_ext.battery++;
	  lcd_ext.battery = lcd_ext.battery % 5;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
