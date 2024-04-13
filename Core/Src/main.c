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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <string.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis =0;
char text[2];
int i=0;
char code1[]="1234";
char code2[]="0000";
int a1,a2;
int d3=0;
int d4=0;
int d5=0;
float Fr;
float T;
int I;
uint32_t C1;
uint32_t C2;
Lcd_HandleTypeDef lcd;
int redval;
float ve;
int adc_val1;
int adc_val2;
float Vn;
int Temp;
void ADC_Select_CH1(void){
	  ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_1;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}
void ADC_Select_CH3(void){
	  ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
  Error_Handler();
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
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	HAL_Delay(200);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);

	HAL_GPIO_WritePin(GPIOB, C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, C3_Pin, 1);

	Lcd_PortType ports[] = {D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
	Lcd_PinType pins[]= {D4_Pin, D5_Pin, D6_Pin, D7_Pin};

	lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, E_GPIO_Port, E_Pin, LCD_4_BIT_MODE);
	Lcd_cursor(&lcd, 0,2);
	Lcd_string(&lcd, "Keypad test");
	Lcd_cursor (&lcd, 1,0);
	Lcd_string(&lcd, "key:");

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	int cnt;

	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);


	//TIM1 ->CNT =1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*if(a1==1)
		    { a1=0;
			  Lcd_cursor (&lcd, 2,0);
			   Lcd_string(&lcd, "code correct");
		    }
		    if(a2==1)
		        { a2=0;
		    	  Lcd_cursor (&lcd, 2,0);
		    	   Lcd_string(&lcd, "code incorrect");
		        }
		    cnt = TIM1 -> CNT;
	    	  Lcd_cursor (&lcd, 3,0);
	    	   Lcd_int(&lcd, cnt); */
		//Lcd_cursor (&lcd, 2,0);
		//Lcd_float(&lcd, Fr);
		// TIM1 -> CCR1 =1500;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		redval= HAL_ADC_GetValue(&hadc1);
		HAL_Delay(200);
		ve=redval*(3.6/4095);
		Lcd_cursor (&lcd, 2,0);
		Lcd_float(&lcd, ve);
		TIM1 -> CCR1 =redval*0.4884;

		ADC_Select_CH1();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        adc_val1= HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		Vn=adc_val1*(3.6/4095);
		Lcd_cursor (&lcd, 2,0);
		Lcd_float(&lcd, Vn);



		ADC_Select_CH3();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        adc_val2= HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		Temp= adc_val2*(3.6/4095)*100;
		Lcd_cursor (&lcd, 3,0);
		Lcd_float(&lcd, Temp);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 25000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 25000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led5_Pin|Led6_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C1_Pin|C2_Pin|C3_Pin|Led1_Pin
                          |Led2_Pin|Led3_Pin|Led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Led5_Pin Led6_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Led5_Pin|Led6_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin Led1_Pin
                           Led2_Pin Led3_Pin Led4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|Led1_Pin
                          |Led2_Pin|Led3_Pin|Led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ currentMillis = HAL_GetTick();
if (currentMillis-previousMillis > 10)
{ /*Configure GPIO pins: R1 R2 R3 R4 to GPIO_INPUT*/
	GPIO_InitStructPrivate. Pin = R1_Pin | R2_Pin|R3_Pin|R4_Pin;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate. Pull = GPIO_NOPULL;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
	HAL_GPIO_WritePin (GPIOB, C1_Pin, 1);
	HAL_GPIO_WritePin (GPIOB, C2_Pin, 0);
	HAL_GPIO_WritePin (GPIOB, C3_Pin, 0);
	if((GPIO_Pin == R1_Pin) && (HAL_GPIO_ReadPin (GPIOB, R1_Pin)))
	{HAL_GPIO_WritePin (Led1_GPIO_Port,Led1_Pin,0); //D1 on

	strcpy(text,"1");
	code2[i]='1';
	i++;

	}

	if((GPIO_Pin == R2_Pin) && (HAL_GPIO_ReadPin (GPIOB, R2_Pin)))
	{HAL_GPIO_WritePin (Led2_GPIO_Port,Led2_Pin,1); //D2 off

	strcpy(text,"4");
	code2[i]='4';
	i++;

	}

	if((GPIO_Pin == R3_Pin) && (HAL_GPIO_ReadPin (GPIOB, R3_Pin)))
	{HAL_GPIO_WritePin (Led4_GPIO_Port,Led4_Pin,0); //D4 on

	strcpy(text,"7");
	code2[i]='7';
	i++;

	}

	if((GPIO_Pin == R4_Pin) && (HAL_GPIO_ReadPin (GPIOB, R4_Pin)))
	{HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin); //D1 change
	HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);	//D2 change
	i++;

	}
	HAL_GPIO_WritePin (GPIOB, C1_Pin, 0);
	HAL_GPIO_WritePin (GPIOB, C2_Pin, 1);
	HAL_GPIO_WritePin (GPIOB, C3_Pin, 0);
	if((GPIO_Pin == R1_Pin) && (HAL_GPIO_ReadPin(GPIOB, R1_Pin)))
	{HAL_GPIO_WritePin (Led1_GPIO_Port,Led1_Pin, 1); //D1 off

	strcpy(text,"2");
	code2[i]='2';
	i++;

	}
	if((GPIO_Pin == R2_Pin) && (HAL_GPIO_ReadPin (GPIOB, R2_Pin)))
	{HAL_GPIO_WritePin (Led3_GPIO_Port,Led3_Pin,0); //D3 on

	strcpy(text,"5");
	code2[i]='5';
	i++;

	}

	if((GPIO_Pin == R3_Pin) && (HAL_GPIO_ReadPin (GPIOB, R3_Pin)))
	{HAL_GPIO_WritePin (Led4_GPIO_Port,Led4_Pin,1); //D4 off

	strcpy(text,"8");
	code2[i]='8';
	i++;

	}

	if((GPIO_Pin == R4_Pin) && (HAL_GPIO_ReadPin (GPIOB, R4_Pin)))
	{
		HAL_GPIO_WritePin (Led1_GPIO_Port,Led1_Pin,0); // D1 on
		HAL_GPIO_WritePin (Led2_GPIO_Port,Led2_Pin,0); // D2 on
		HAL_GPIO_WritePin (Led3_GPIO_Port,Led3_Pin,0); // D3 on
		HAL_GPIO_WritePin (Led4_GPIO_Port,Led4_Pin,0); // D4 on

		strcpy(text,"0");
		code2[i]='0';
		i++;

	}

	HAL_GPIO_WritePin (GPIOB, C1_Pin, 0);
	HAL_GPIO_WritePin (GPIOB, C2_Pin, 0);
	HAL_GPIO_WritePin (GPIOB, C3_Pin, 1);
	if((GPIO_Pin == R1_Pin) && (HAL_GPIO_ReadPin(GPIOB, R1_Pin)))
	{HAL_GPIO_WritePin (Led2_GPIO_Port,Led2_Pin, 0); //D2 on

	strcpy(text,"3");
	code2[i]='3';
	i++;

	}
	if((GPIO_Pin == R2_Pin) && (HAL_GPIO_ReadPin (GPIOB, R2_Pin)))
	{HAL_GPIO_WritePin (Led3_GPIO_Port,Led3_Pin,1); //D3 off
	strcpy(text,"6");
	code2[i]='6';
	i++;

	}

	if((GPIO_Pin == R3_Pin) && (HAL_GPIO_ReadPin (GPIOB, R3_Pin)))
	{
		HAL_GPIO_WritePin (Led1_GPIO_Port,Led1_Pin,1); // D1 off
		HAL_GPIO_WritePin (Led2_GPIO_Port,Led2_Pin,1); // D2 off
		HAL_GPIO_WritePin (Led3_GPIO_Port,Led3_Pin,1); // D3 off
		HAL_GPIO_WritePin (Led4_GPIO_Port,Led4_Pin,1); // D4 off

		strcpy(text,"9");
		code2[i]='9';
		i++;

	}
	if((GPIO_Pin == R4_Pin) && (HAL_GPIO_ReadPin (GPIOB, R4_Pin)))
	{HAL_GPIO_TogglePin(Led3_GPIO_Port, Led3_Pin); //D3 change
	HAL_GPIO_TogglePin(Led4_GPIO_Port, Led4_Pin);	//D4 change
	if(strcmp(code1,code2)==0)
	{
		HAL_GPIO_WritePin(GPIOA,Led5_Pin,0);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOA,Led5_Pin,1);
		a1=1;
	}
	else{
		a2=1;
	}

	}

	HAL_GPIO_WritePin(GPIOB,C1_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, C2_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, C3_Pin, 1);

	Lcd_cursor (&lcd, 1,i+3);
	Lcd_string(&lcd, text);

	if (i==4)
	{
		i=0;}

	GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructPrivate. Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
	previousMillis = currentMillis;

}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim -> Instance == TIM1) {
		d3++;
		d4++;
		d5++;
		if (d3==2) {
			HAL_GPIO_TogglePin(Led3_GPIO_Port, Led3_Pin);
			d3=0;
		}
		if (d4==5) {
			HAL_GPIO_TogglePin(Led4_GPIO_Port, Led4_Pin);
			d4=0;
		}
		if (d5==10) {
			HAL_GPIO_TogglePin(Led5_GPIO_Port, Led5_Pin);
			d5=0;
		}
	}
	if (htim -> Instance == TIM1) {
		HAL_GPIO_TogglePin(Led3_GPIO_Port, Led3_Pin);
	}
	if (htim -> Instance == TIM2) {
		HAL_GPIO_TogglePin(Led4_GPIO_Port, Led4_Pin);
	}
	if (htim -> Instance == TIM3) {
		HAL_GPIO_TogglePin(Led5_GPIO_Port, Led5_Pin);
	}

	I++;
	if (I==1) {
		C1= TIM5 -> CCR1;
	}
	if (I==2) {
		I=0;
		C2= TIM5 -> CCR1;
		T=(C2 - C1)*0.4*(10^6);
		Fr=1/T;
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
