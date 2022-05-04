/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  * Author: 	Ruben Wilssens
  * Github:		https://github.com/ruben-wilssens
  * Date: 		April 2022
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

//#define DEVICE_MODE				0xF0 // 0xF0 = debug (LEDS), 0xF1 = debug (GPIO), 0xF2 = debug (EXTI) , 0x00 = normal operation
#define STM32_CMD_CS_HIGH		0x01
#define STM32_CMD_CS_LOW		0x02
#define STM32_CMD_NEWDEV_OK		0x03
//#define STM32_CMD_CS_STATUS		0x04
//#define STM32_CMD_IRQ_STATUS	0x05
//#define STM32_CMD_SLEEP		0x06

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t I2C_DATA_BUFFER[1];
//uint8_t CS_STATUS = 0xFF; //CS high (0xFF) => DW3220 SPI OFF | CS low (0x00) => DW3220 SPI ON
//uint8_t IRQ_STATUS = 0x00; //IRQ low (0x00) => no DW3220 IRQ | IRQ high (0xFF) => DW3220 IRQ
//uint8_t NEWDEV_STATUS = 0xFF; //NEWDEV high (0xFF) => board just started up and requests I2C confirmation by host
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *);
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef*);


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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Set LEDs initial state (LED
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // NEWDEV LED enabled
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); // CS LED disabled

  // Set NEW_DEVICE line low (open drain)
  HAL_GPIO_WritePin(NEW_DEVICE_GPIO_Port, NEW_DEVICE_Pin, GPIO_PIN_RESET);

  // Enable I2C receive interrupts
  HAL_I2C_Slave_Receive_IT(&hi2c1, &I2C_DATA_BUFFER, sizeof(I2C_DATA_BUFFER));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

//	  if(DEVICE_MODE == 0xF0){
//		  // Blink leds alternating
//		  HAL_Delay(500);
//		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//		  HAL_Delay(500);
//		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	  }
//	  else if(DEVICE_MODE == 0xF1){
//		  // Test output GPIO's (open drain)
//		  HAL_Delay(500);
//		  HAL_GPIO_WritePin(IRQ_line_GPIO_Port, IRQ_line_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(NEW_DEVICE_GPIO_Port, NEW_DEVICE_Pin, GPIO_PIN_SET);
//		  HAL_Delay(500);
//		  HAL_GPIO_WritePin(IRQ_line_GPIO_Port, IRQ_line_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(NEW_DEVICE_GPIO_Port, NEW_DEVICE_Pin, GPIO_PIN_RESET);
//	  }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 64;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IRQ_line_Pin|NEW_DEVICE_Pin|SPI_CSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IRQ_line_Pin NEW_DEVICE_Pin */
  GPIO_InitStruct.Pin = IRQ_line_Pin|NEW_DEVICE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CSn_Pin */
  GPIO_InitStruct.Pin = SPI_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CSn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* Callback external interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case IRQ_Pin:
			if(HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin) == GPIO_PIN_SET){
				// DW3220 generates interrupt ==> Pull down IRQ_line (open drain)
				HAL_GPIO_WritePin(IRQ_line_GPIO_Port, IRQ_line_Pin, GPIO_PIN_RESET);
			}
			else{
				// DW3220 stops interrupt ==> Release IRQ_line (open drain)
				HAL_GPIO_WritePin(IRQ_line_GPIO_Port, IRQ_line_Pin, GPIO_PIN_SET);
			}
			break;

//		case WAKEUP_Pin:
//			// Wake-up device
//			break;
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	switch(I2C_DATA_BUFFER[0]){
		case STM32_CMD_CS_HIGH:
			// Toggle CS HIGH & disable CS LED (LED2) => DW3220 SPI DISABLED
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SPI_CSn_GPIO_Port, SPI_CSn_Pin, GPIO_PIN_SET);

//			CS_STATUS = 0xFF;

			// Issue new receive interrupt
			HAL_I2C_Slave_Receive_IT(&hi2c1, &I2C_DATA_BUFFER, sizeof(I2C_DATA_BUFFER));
			break;

		case STM32_CMD_CS_LOW:
			// Toggle CS LOW & enable CS LED (LED2) => DW3220 SPI ENABLED
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SPI_CSn_GPIO_Port, SPI_CSn_Pin, GPIO_PIN_RESET);

//			CS_STATUS = 0x00;

			// Issue new receive interrupt
			HAL_I2C_Slave_Receive_IT(&hi2c1, &I2C_DATA_BUFFER, sizeof(I2C_DATA_BUFFER));
			break;

		case STM32_CMD_NEWDEV_OK:
			// Release NEWDEV line (open drain) & disable NEWDEV LED (LED1)=> NEWDEV confirmed by host
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(NEW_DEVICE_GPIO_Port, NEW_DEVICE_Pin, GPIO_PIN_SET);

//			NEWDEV_STATUS = 0x00;

			// Issue new receive interrupt
			HAL_I2C_Slave_Receive_IT(&hi2c1, &I2C_DATA_BUFFER, sizeof(I2C_DATA_BUFFER));

//		case STM32_CMD_CS_STATUS:
//			I2C_DATA_BUFFER[0] = CS_STATUS;
//			// Reply MASTER with CS_STATUS word (1 byte)
//			HAL_I2C_Slave_Transmit_IT(&hi2c1, &I2C_DATA_BUFFER, 1);
//
//			// Don't issue new receive interrupt yet!
//			break;
//
//		case STM32_CMD_IRQ_STATUS:
//			I2C_DATA_BUFFER[0] = IRQ_STATUS;
//			// Reply MASTER with IRQ_STATUS
//			HAL_I2C_Slave_Transmit_IT(&hi2c1, &I2C_DATA_BUFFER, 1);
//
//			// Don't issue new receive interrupt yet!
//			break;
//
//		case STM32_CMD_SLEEP:
//			// Sleep device
//			break;
	}
}


//void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
//	// Issue new receive interrupt after replying MASTER from a write
//	HAL_I2C_Slave_Receive_IT(&hi2c1, &I2C_DATA_BUFFER, sizeof(I2C_DATA_BUFFER));
//}
/*
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
		asm("nop");
	}
}
*/

//HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
//{
//  /* when
//   * hal_i2c_master_transmit_it (i2c_handletypedef * hi2c, uint16_t devaddress, uint8_t * pdata, uint16_t size);
//   * is called and after completion, this function will be called and executed.
//   * if you’d like to do something upon data transmission completion, then add this to your code in the application source file (main.c).
//	*/
//}

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
