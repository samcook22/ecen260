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
#include <stdio.h>
#include <string.h>
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
#define CE_PORT GPIOB // PB6 chip enable (aka slave select)
#define CE_PIN GPIO_PIN_6
#define DC_PORT GPIOA // PA4 data/control
#define DC_PIN GPIO_PIN_4
#define RESET_PORT GPIOA // PA1 reset
#define RESET_PIN GPIO_PIN_1
#define GLCD_WIDTH 84
#define GLCD_HEIGHT 48
#define NUM_BANKS 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const char font_table[][6] = {
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
{0xFE, 0x11, 0x11, 0x11, 0xFE, 0x00}, // 'A'
{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // 'B'
{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // 'C'
{0xFF, 0x81, 0x81, 0x42, 0x3C, 0x00}, // 'D'
{0xFF, 0x89, 0x89, 0x89, 0x89, 0x00}, // 'E'
{0xFF, 0x09, 0x09, 0x09, 0x09, 0x00}, // 'F'
{0xFC, 0x82, 0x89, 0x89, 0x71, 0x00}, // 'G'
{0xFF, 0x08, 0x08, 0x08, 0xFF, 0x00}, // 'H'
{0x81, 0x81, 0xFF, 0x81, 0x81, 0x00}, // 'I'
{0x61, 0x81, 0xFF, 0x01, 0x01, 0x00}, // 'J'
{0xFF, 0x18, 0x24, 0x42, 0x81, 0x00}, // 'K'
{0xFF, 0x80, 0x80, 0x80, 0x80, 0x00}, // 'L'
{0xFF, 0x01, 0xFE, 0x01, 0xFF, 0x00}, // 'M'
{0xFF, 0x06, 0x18, 0x60, 0xFF, 0x00}, // 'N'
{0xFF, 0x81, 0x81, 0x81, 0xFF, 0x00}, // 'O'
{0xFF, 0x09, 0x09, 0x09, 0x0F, 0x00}, // 'P'
{0x3E, 0x41, 0x51, 0x61, 0xBE, 0x00}, // 'Q'
{0xFF, 0x09, 0x19, 0x29, 0xC6, 0x00}, // 'R'
{0x86, 0x89, 0x89, 0x89, 0x71, 0x00}, // 'S'
{0x01, 0x01, 0xFF, 0x01, 0x01, 0x00}, // 'T'
{0x7F, 0x80, 0x80, 0x80, 0x7F, 0x00}, // 'U'
{0x07, 0x38, 0xC0, 0x38, 0x07, 0x00}, // 'V'
{0x7F, 0x80, 0xFF, 0x80, 0x7F, 0x00}, // 'W'
{0xC3, 0x24, 0x18, 0x24, 0xC3, 0x00}, // 'X'
{0x07, 0x08, 0xF8, 0x08, 0x07, 0x00}, // 'Y'
{0xC1, 0xA1, 0x99, 0x85, 0x83, 0x00}, // 'Z'
{0x7E, 0x81, 0x95, 0xA1, 0xA1, 0x95}, // smile pt1 *27*
{0x81, 0x7E, 0x00, 0x00, 0x00, 0x00}, // smile pt2 *28*
{0x7E, 0x81, 0x81, 0x81, 0x7E, 0x00}, //#0, 29
{0x80, 0x82, 0xFF, 0x80, 0x80, 0x00}, //#1, 30
{0xC4, 0xE2, 0x91, 0x8E, 0x80, 0x00}, //#2, 31
{0x00, 0x81, 0x89, 0x99, 0x7E, 0x00}, //#3, 32
{0x08, 0x0C, 0x0A, 0xFF, 0x08, 0x00}, //#4, 33
{0x8F, 0x89, 0x89, 0x91, 0x61, 0x00}, //#5, 34
{0x7E, 0x89, 0x89, 0x91, 0x61, 0x00}, //#6, 35
{0x81, 0x41, 0x31, 0x09, 0x07, 0x00}, //#7, 36
{0x66, 0x99, 0x99, 0x99, 0x66, 0x00}, //#8, 37
{0x06, 0x89, 0x89, 0x89, 0x7E, 0x00}, //#9, 38
{0x08, 0x14, 0x6B, 0x14, 0x08, 0x00}, // *  39
{0x14, 0x3E, 0x14, 0x3E, 0x14, 0x00}, // #  40
{0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00}, // .  41
{0x81, 0x42, 0x24, 0x18, 0x18, 0x00}  // >  42
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SPI_write(unsigned char data);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_init(void);
void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_putchar(int font_table_row);
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  GLCD_init(); // initialize the screen
  GLCD_clear(); // clear the screen

  GLCD_setCursor(44, 4);
  GLCD_putchar(22);
  GLCD_setCursor(22, 0);
  GLCD_putchar(22);
  GLCD_putchar(15);
  GLCD_putchar(12);
  GLCD_putchar(20);
  GLCD_putchar(1);
  GLCD_putchar(7);		// This is just writing VOLTAGE READER __ V
  GLCD_putchar(5);		// This is concrete and stays the whole time.
  GLCD_setCursor(26, 2);
  GLCD_putchar(18);
  GLCD_putchar(5);
  GLCD_putchar(1);
  GLCD_putchar(4);
  GLCD_putchar(5);
  GLCD_putchar(18);

  char uart_buffer[50];
  uint32_t previous_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Start ADC Conversion
	  HAL_ADC_Start(&hadc1);
	  // Wait for ADC conversion to complete
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  // Read ADC value
	  uint16_t analog_measurement = HAL_ADC_GetValue(&hadc1);



	  if (analog_measurement <= 310) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(29);	// Writes 0
	  }
	  else if (analog_measurement <= 620) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(30);	// Writes 1
	  }
	  else if (analog_measurement <= 930) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(31);	// Writes 2
	  }
	  else if (analog_measurement <= 1241) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(32);	// Writes 3
	  }
	  else if (analog_measurement <= 1551) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(33);	// Writes 4
	  }
	  else if (analog_measurement <= 1861) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(34);	// Writes 5
	  }
	  else if (analog_measurement <= 2172) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(35);	// Writes 6
	  }
	  else if (analog_measurement <= 2482) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(36);	// Writes 7
	  }
	  else if (analog_measurement <= 2792) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(37);	// Writes 8
	  }
	  else if (analog_measurement <= 3103) {
		  GLCD_setCursor(37, 4);
	      GLCD_putchar(38);	// Writes 9
	  }
	  else if (analog_measurement <= 3413) {
		  GLCD_setCursor(31, 4);	// Cursor shifts to the left from the addition of another decimal point.
	      GLCD_putchar(30);	// Writes 10
	      GLCD_putchar(29);
	  }
	  else if (analog_measurement <= 3723) {
		  GLCD_setCursor(31, 4);
		  GLCD_putchar(30);	// Writes 11
		  GLCD_putchar(30);
	  }
	  else if (analog_measurement <= 4033) {
		  GLCD_setCursor(31, 4);
	      GLCD_putchar(30);	// Writes 12
	      GLCD_putchar(31);
	  }
	  else if (analog_measurement > 4033) {
		  GLCD_setCursor(31, 4);
	      GLCD_putchar(30);	// Writes 13
	      GLCD_putchar(32);
	  }

	  uint32_t current_time = HAL_GetTick();  // Get system time in ms

	  // Update UART every 4s without blocking
	  if ((current_time - previous_time) >= 4000) {
	      previous_time = current_time;

	      // Read ADC value
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
          uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
	      float voltage = (adc_value * 13.2) / 4095.0;	// Can read up to 13.2 volts thanks to a voltage divider.

	      // Send voltage over UART
	      char uart_buffer[50];
	      sprintf(uart_buffer, "Voltage: %.2f V\r\n", voltage);	// Send over UART in this format, float option had to be enabled in MCU/MPU GCC Linker under Project Properties.
	      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	    }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SPI_write(unsigned char data){
// Chip Enable (low is asserted)
HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
// Send data over SPI1
HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, HAL_MAX_DELAY);
// Chip Disable
HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}
void GLCD_data_write(unsigned char data){
	// Switch to "data" mode (D/C pin high)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
	// Send data over SPI
	SPI_write(data);
	}
	void GLCD_command_write(unsigned char data){
	// Switch to "command" mode (D/C pin low)
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
	// Send data over SPI
	SPI_write(data);
	}
void GLCD_init(void){
	// Keep CE high when not transmitting
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
	// Reset the screen (low pulse - down & up)
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
	// Configure the screen (according to the datasheet)
	GLCD_command_write(0x21); // enter extended command mode
	GLCD_command_write(0xB0); // set LCD Vop for contrast (this may be adjusted)
	GLCD_command_write(0x04); // set temp coefficient
	GLCD_command_write(0x15); // set LCD bias mode (this may be adjusted)
	GLCD_command_write(0x20); // return to normal command mode
	GLCD_command_write(0x0C); // set display mode normal
	}
void GLCD_setCursor(unsigned char x, unsigned char y){
	GLCD_command_write(0x80 | x); // column
	GLCD_command_write(0x40 | y); // bank
	}
void GLCD_clear(void){
	int i;
	for(i = 0; i < (GLCD_WIDTH * NUM_BANKS); i++){
	GLCD_data_write(0x00); // write zeros
	}
	GLCD_setCursor(0,0); // return cursor to top left
	}
void GLCD_putchar(int font_table_row){
 int i;
 for (i=0; i<6; i++){
 GLCD_data_write(font_table[font_table_row][i]);
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
