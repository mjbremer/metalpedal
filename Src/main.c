/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile uint8_t pData[DMA_BUFFER_SIZE_BYTES]; // DMA input buffer
static volatile uint8_t oData[DMA_BUFFER_SIZE_BYTES]; // DMA output buffer

static volatile int32_t input_buffer[2][BUFFER_SIZE_SAMPLES]; // channel 0 is left, channel 1 is right
static volatile int32_t output_buffer[2][BUFFER_SIZE_SAMPLES];


static volatile uint8_t* volatile validpData; // points to valid (readable) input half-buffer
static volatile uint8_t* volatile validoData; // points to valid (writable) output half-buffer

static volatile uint8_t out1_INT,out2_INT,in1_INT,in2_INT; // Interrupt flags


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void fillInputBuffer()
{
	int i;

	for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) {
		int32_t temp = 0;
		int32_t temp2 = 0;
		temp |= validpData[(8*i) + 1] << 24;
		temp |= validpData[(8*i) + 0] << 16;
		temp |= validpData[(8*i) + 3] << 8;
		temp |= validpData[(8*i) + 2] << 0;

		temp2 |= validpData[(8*i) + 5] << 24;
		temp2 |= validpData[(8*i) + 4] << 16;
		temp2 |= validpData[(8*i) + 7] << 8;
		temp2 |= validpData[(8*i) + 6] << 0;

		input_buffer[1][i] = temp;
		input_buffer[0][i] = temp2;
	}
}
//
void fillOutputBuffer()
{
	int i;

	for (i = 0; i < BUFFER_SIZE_SAMPLES; i++) {
		int32_t temp = output_buffer[1][i]; // Right channel
		int32_t temp2 = output_buffer[0][i]; //Left channel

		validoData[(8*i)+1] = temp >> 24;
		validoData[(8*i)+0] = temp >> 16;
		validoData[(8*i)+3] = temp >> 8;
		validoData[(8*i)+2] = temp >> 0;

		validoData[(8*i)+5] = temp2 >> 24;
		validoData[(8*i)+4] = temp2 >> 16;
		validoData[(8*i)+7] = temp2 >> 8;
		validoData[(8*i)+6] = temp2 >> 0;

	}
}


void processingBlock() {

	int channel;
	int sample;

	int32_t insample,outsample;


	for (channel = 0; channel < 2; channel++) {
		for (sample = 0; sample < BUFFER_SIZE_SAMPLES; sample++) {

			//insample = input_buffer[channel][sample];
			insample = -input_buffer[0][sample]; // mono to stereo
			outsample = insample;

			output_buffer[channel][sample] = outsample;
		}
	}

}


void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	in1_INT = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	in2_INT = 1;
}


void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	out1_INT = 1;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	out2_INT = 1;
}


void ProcessingLoop()
{

		while (!in1_INT); // wait for half callback
		validpData = pData;
		fillInputBuffer();

		in1_INT = 0;

		processingBlock();

		while (!out1_INT);
		validoData = oData;
		fillOutputBuffer();

		out1_INT = 0;

		while (!in2_INT); // wait for half callback
		validpData = &pData[DMA_BUFFER_SIZE_BYTES/2];
		fillInputBuffer();

		in2_INT = 0;

		processingBlock();

		while (!out2_INT);
		validoData = &oData[DMA_BUFFER_SIZE_BYTES/2];
		fillOutputBuffer();

		out2_INT = 0;

}

void ReadInputs(input_t* inputs) {

	input_t new_inputs;
	memcpy(&new_inputs,inputs,sizeof(input_t));

	new_inputs.button_1 = HAL_GPIO_ReadPin(USERBUTTON1_GPIO_Port, USERBUTTON1_Pin);
	new_inputs.button_2 = HAL_GPIO_ReadPin(USERBUTTON2_GPIO_Port, USERBUTTON2_Pin);
	new_inputs.encoder_1 = htim1.Instance->CNT;

	if (new_inputs.button_1 != inputs->button_1)
		new_inputs.button1_changed = 1;
	if (new_inputs.button_2 != inputs->button_2)
		new_inputs.button2_changed = 1;
	if (new_inputs.encoder_1 != inputs->encoder_1)
			new_inputs.encoder1_changed = 1;



	memcpy(inputs,&new_inputs,sizeof(input_t));
}


void LCD_command(uint16_t i)
{
 //P1 = i; //put data on output Port

 GPIOD->BSRR = i | ((~i)<<16);

 //D_I =0; //D/I=LOW : send instruction

 HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);

 //R_W =0; //R/W=LOW : Write

 HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);

 //E = 1;

 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
 HAL_Delay(100); //enable pulse width >= 300ns
 //E = 0; //Clock enable: falling edge

 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}
/**********************************************************/
void LCD_write(uint16_t i)
{
	 //P1 = i; //put data on output Port

	 GPIOD->BSRR = (i&0xff) | (((~i)&0xff)<<16);

	 //D_I =0; //D/I=LOW : send instruction

	 HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);

	 //R_W =0; //R/W=LOW : Write

	 HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);

	 //E = 1;

	 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	 HAL_Delay(100); //enable pulse width >= 300ns
	 //E = 0; //Clock enable: falling edge

	 HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}
/**********************************************************/
void LCD_init()
{

HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
 HAL_Delay(100); //Wait >40 msec after power is applied
 LCD_command(0x30); //command 0x30 = Wake up
 HAL_Delay(30); //must wait 5ms, busy flag not available
 LCD_command(0x30); //command 0x30 = Wake up #2
 HAL_Delay(10); //must wait 160us, busy flag not available
 LCD_command(0x30); //command 0x30 = Wake up #3
 HAL_Delay(10); //must wait 160us, busy flag not available
 LCD_command(0x38); //Function set: 8-bit/2-line
 LCD_command(0x10); //Set cursor
 LCD_command(0x0f); //Display ON; Cursor ON
 LCD_command(0x06); //Entry mode set


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

  in1_INT = 0;
  in2_INT = 0;
  out1_INT = 0;
  out2_INT = 0;

  input_t inputs;
  char debug_msg[100];
  int debug_msg_len;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(400);


//  if (init_vars() != 0)
//	  Error_Handler();
  if (HAL_OK != HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)pData, DMA_BUFFER_SIZE))
	  Error_Handler();// size here is the TOTAL NUMBER OF WORDS IN THE BUFFER
  if (HAL_OK != HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)oData, DMA_BUFFER_SIZE))
	  Error_Handler();

//  LCD_init();
//
//  char h[] = "Hello World";
//  uint16_t len = strlen(h);
//  for (uint16_t i = 0; i < len; i++) {
//	  LCD_write(h[i]);
//  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  ProcessingLoop();
//	  ReadInputs(&inputs);

//	  if (inputs.changed) {
//		  if (inputs.button1_changed) {
//			 debug_msg_len = snprintf(debug_msg, 100, "Button 1 changed to %d\r\n", inputs.button_1);
//			 HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, debug_msg_len, 10000000U);
//		  }
//
//		  if (inputs.button2_changed) {
//			 debug_msg_len = snprintf(debug_msg, 100, "Button 2 changed to %d\r\n", inputs.button_2);
//			 HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, debug_msg_len, 10000000U);
//		  }
//		  if (inputs.encoder1_changed) {
//			 debug_msg_len = snprintf(debug_msg, 100, "Encoder 1 changed to %lu\r\n", inputs.encoder_1);
//			 HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, debug_msg_len, 10000000U);
//		  }
//
//	  }

//	  inputs.changed = 0;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  // ---Function Call Graveyard---
  //    //Delay 1500ms
  //	  HAL_Delay(1500);
  //	  // Stop DMA
  //	  HAL_I2S_DMAStop(&hi2s2);
  //    // Transmit processing buffer
  //	  HAL_UART_Transmit(&huart4, (uint8_t*)processingData, DMA_HALFBUF_SIZE_STEREO_SAMPLES * 4U, 10000000U);
  //	  // Send Current MIDI note
  //	  MIDIon(0, (uint8_t)cur_midi, 0x7F);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */
  hi2s2.hdmarx = &hdma_spi2_rx;
  HAL_I2S_MspInit(&hi2s2);
  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin|LCD_RW_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_RESET_Pin */
  GPIO_InitStruct.Pin = ADC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USERBUTTON1_Pin USERBUTTON2_Pin */
  GPIO_InitStruct.Pin = USERBUTTON1_Pin|USERBUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_RW_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_RW_Pin|LCD_RS_Pin;
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
  while(1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
