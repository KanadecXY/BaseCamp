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
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEBUG 1						// 1 - ON UART port  0 - OFF UART port
#define warningInTemp 38            //warning value of input temperature
#define warningOutTemp 29			//warning value of output temperature
#define warningVoltage 2.5			//warning value of voltage from potentiometer
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void PollinADC(void);
void Uart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t adcValue_InTemp = 0;
uint32_t adcValue_OutTemp = 0;
uint32_t adcValue_Potent = 0;

uint32_t inTemp = 0;
uint32_t outTemp = 0;


uint16_t valueInTemp = 0;
uint16_t valueOutTemp = 0;
uint16_t valuePotent = 0;



double voltage_InTemp = 0;
double voltage_OutTemp = 0;
double voltage_Potent = 0;

int warningLevel = 0;                                  		//warning mode

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */











 while(1)
 {

		PollinADC();

	 	voltage_InTemp = (double)adcValue_InTemp * 3 / 4095;    //convert ADC valueInTemp to voltage
	 	inTemp = (voltage_InTemp - 0.76) / 0.0025 + 25;         //convert voltage in temperature

	 	voltage_OutTemp = (double)adcValue_OutTemp * 5/ 4095;   //convert ADC ValueOutTemp to voltage
	 	outTemp = voltage_OutTemp * 100;						//convert voltage in temperature

	 	voltage_Potent = (double)adcValue_Potent * 3 /4095;		//convert ADC ValuePotent to voltage



	 	valuePotent = adcValue_Potent * 10;						//convert ADC ValuePotent in brightness of the LED

	 	valueOutTemp = adcValue_OutTemp * 418.36 - 63632; 		//convert output temperature (13 to ~30 C) in brightness of the LED

	 	valueInTemp = adcValue_InTemp * 524.68 - 532428;		//convert output temperature (~15 to ~40 C) in brightness of the LED


	 	checkInTemp();
	 	checkOutTemp();
	 	checkPotent();

		if (voltage_Potent > warningVoltage || inTemp > warningInTemp || outTemp > warningOutTemp)
			{
				if ((voltage_Potent > warningVoltage && inTemp > warningInTemp) || (voltage_Potent > warningVoltage && outTemp > warningOutTemp) || (inTemp > warningInTemp && outTemp > warningOutTemp))
				{
					if (voltage_Potent > warningVoltage && inTemp > warningInTemp && outTemp > warningOutTemp)
						{
							warningLevel = 3;
						}
					else
						{
							warningLevel = 2;
						}
				}
				else
					{
						warningLevel = 1;
					}

			}
		else
			{
				warningLevel = 0;
			}


		switch (warningLevel)
			{
				case 1:
				{
					TIM4->CCR3 = 40959;
					TIM4->PSC = 779;					// 1 HZ
				}
				break;

				case 2:
				{
					TIM4->CCR3 = 40959;
					TIM4->PSC = 311;					// 2.5 HZ
				}
				break;

				case 3:
				{
					TIM4->CCR3 = 40959;
					TIM4->PSC = 155;					// 5 HZ
				}
				break;

				default:
				{
					TIM4->CCR3 = 0;
					TIM4->PSC = 0;
				}
				break;
			}

		#ifdef DEBUG == 1

			Uart();

		#endif


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
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 40959;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */




void PollinADC (void)												//get ADC value
{
	volatile HAL_StatusTypeDef adcPoolResult1;
	volatile HAL_StatusTypeDef adcPoolResult2;
	volatile HAL_StatusTypeDef adcPoolResult3;

	HAL_ADC_Start(&hadc1);
	adcPoolResult1 = HAL_ADC_PollForConversion(&hadc1, 1);

	if (adcPoolResult1 == HAL_OK)
	{
	adcValue_InTemp = HAL_ADC_GetValue(&hadc1);
	}


	HAL_ADC_Start(&hadc2);
	adcPoolResult2 = HAL_ADC_PollForConversion(&hadc2, 1);

	if (adcPoolResult2 == HAL_OK)
	{
	adcValue_OutTemp = HAL_ADC_GetValue(&hadc2);
	}


	HAL_ADC_Start(&hadc3);
	adcPoolResult3 = HAL_ADC_PollForConversion(&hadc3, 1);

	if (adcPoolResult3 == HAL_OK)
	{
	adcValue_Potent = HAL_ADC_GetValue(&hadc3);
	}

	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
	HAL_ADC_Stop(&hadc3);
	HAL_Delay(250);
}

void checkPotent (void)									//check warning mode
{
	if (voltage_Potent > warningVoltage)
	{
		TIM4->CCR4 = 0;
	}
	else
	{
		TIM4->CCR4 = valuePotent;
	}
}

void checkInTemp (void)
{
	if (inTemp > warningInTemp)
	{
		TIM4->CCR2 = 0;
	}
	else
	{
		TIM4->CCR2 = valueInTemp;
	}
}

void checkOutTemp (void)
{
	if (outTemp > warningOutTemp)
	{
		TIM4->CCR1 = 0;
	}
	else
	{
		TIM4->CCR1 = valueOutTemp;
	}
}

void Uart(void)														//UART mode
{
	char a[100];
	char b[100];
	char c[100];
	char d[100];
	char f[100];
	char g[100];
	char h[100];
	char i[100];
	char j[100];
	char x[100];
	char z[100];
	char m[100];

	uint8_t str[] = "------------------------------------\r\n\0";

	sprintf(a, "adcValue_InTemp is %d\r\n", adcValue_InTemp);
	HAL_UART_Transmit(&huart3, a, strlen(a), 500);
	sprintf(b, "voltage_InTemp is %.2f\r\n", voltage_InTemp);
	HAL_UART_Transmit(&huart3, b, strlen(b), 500);
	sprintf(c, "inTemp is %d\r\n", inTemp);
	HAL_UART_Transmit(&huart3, c, strlen(c), 500);

	HAL_UART_Transmit(&huart3, str, strlen(str), 30);


	sprintf(d, "adcValue_OutTemp is %d\r\n", adcValue_OutTemp);
	HAL_UART_Transmit(&huart3, d, strlen(d), 500);
	sprintf(f, "voltage_OutTemp is %.2f\r\n", voltage_OutTemp);
	HAL_UART_Transmit(&huart3, b, strlen(f), 500);
	sprintf(g, "outTemp is %d\r\n", outTemp);
	HAL_UART_Transmit(&huart3, g, strlen(g), 500);

	HAL_UART_Transmit(&huart3, str, strlen(str), 30);

	sprintf(h, "adcValue_Potent is %d\r\n", adcValue_Potent);
	HAL_UART_Transmit(&huart3, h, strlen(h), 500);
	sprintf(i, "voltage_OutTemp is %.2f\r\n", voltage_Potent);
	HAL_UART_Transmit(&huart3, i, strlen(i), 500);
	sprintf(j, "valuPotent is %d\r\n", valuePotent);
	HAL_UART_Transmit(&huart3, j, strlen(j), 500);

	HAL_UART_Transmit(&huart3, str, strlen(str), 30);

	sprintf(m, "value Prescaler for OrangeLed  is %d\r\n", valueInTemp);
	HAL_UART_Transmit(&huart3, m, strlen(m), 500);

	sprintf(x, "value Prescaler for OrangeLed is %d\r\n", valueOutTemp);
	HAL_UART_Transmit(&huart3, x, strlen(x), 500);


	sprintf(z, "!!!WARNING LEVEL!!! %d\r\n", warningLevel);
	HAL_UART_Transmit(&huart3, z, strlen(z), 500);

	HAL_UART_Transmit(&huart3, str, strlen(str), 30);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
