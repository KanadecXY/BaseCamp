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
#include <stdint.h>
#include <stdbool.h>
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int mode = 0;						//variable for switch mode
int speed = 200;					//variable for switch speed
bool switcher = 0;					//variable for switcher LEDs
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AllTurnOff(void);								//initialization prototypes of function for default mode
void AllTurnOn(void);								//initialization prototypes of function for mode 1
void TurnOnAround(void);							//initialization prototypes of function for mode 2
void DoubleLED(void);								//initialization prototypes of function for mode 3
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);     //initialization interrupt handler for switch mode

/* USER CODE END PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
	  if (switcher)
	  {
	  switch (mode) // choose mode
	  	{
	  	case 0:
	  	{
	  		AllTurnOn();				//mode 1
	  	}
	  		break;
	  	case 1:
	  	{
	  		TurnOnAround();				//mode 2
	  	}
	  		break;
	  	case 2:
	  	{
	  		DoubleLED();				//mode 3
	  	}
	  		break;
	  	case 3:
	  	{
	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);		//mode 4
	  	}
	  		break;
	  	case 4:                   		// to loop modes: go to the first mode
		{
			mode = 0;
		}
			break;
	  	case -1:						// to loop modes: go to the last mode
		{
			mode = 3;
		}
			break;
		}
	  }
		else
		{
			AllTurnOff();
		}

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SWT4_Pin SWT5_Pin SWT3_Pin SWT1_Pin */
  GPIO_InitStruct.Pin = SWT4_Pin|SWT5_Pin|SWT3_Pin|SWT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SWT2_Pin */
  GPIO_InitStruct.Pin = SWT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void AllTurnOff(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);	//turn off all LEDs
}
void AllTurnOn(void) 					//function for mode 1
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);	//turn on all LEDs
}

void TurnOnAround(void) 				//function for mode 2
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); 					//inverting the state of the Green LED (On/Off)
	HAL_Delay(speed); 											// wait 0.5 second
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12| GPIO_PIN_13); 	//inverting the state of the Green LED (Off/On)
															//and Orange LED (On/Off)
	HAL_Delay(speed);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13| GPIO_PIN_14);		//inverting the state of the Orange LED (Off/On)
															//and Red LED (On/Off)
	HAL_Delay(speed);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14| GPIO_PIN_15);		//inverting the state of the Red LED (Off/On)
															// and Blue LED (On/Off)
	HAL_Delay(speed);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

void DoubleLED(void) //function for mode 3
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);  //turn off all LEDs
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12| GPIO_PIN_14);
	HAL_Delay(speed);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12| GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_15);
	HAL_Delay(speed);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13| GPIO_PIN_15);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 	//interrupt handler for switch mode, switch speed and turn on/off all LEDs
{
	switch (GPIO_Pin)
	{
	case SWT2_Pin:
	{
		switcher=!switcher; 					// with each click we invert the value for turn on or turn off all LEDs
	}
			break;
	case SWT1_Pin:
	{
		mode++;									// switch mode up
	}
		break;

	case SWT3_Pin:
	{
		mode--;									// switch mode up
	}
		break;

	case SWT4_Pin:
	{
		if (speed<=5000) 						//in order to be, the delay did not have a bug value
		{
		speed+=100;								// add 100 ms for delay
		}
	}
		break;

	case SWT5_Pin:
	{
		if (speed>=100) 						// in order to be, the delay did not have a minus value
		{
		speed-=100;								// minus 100 ms for delay
		}
	}
		break;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
