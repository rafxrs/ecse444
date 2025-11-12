/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "arm_math.h"
#include "stm32l4xx_hal.h"
#include <math.h>
#include "stm32l4xx_hal_dac.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {MODE_FIXED, MODE_TEMP, MODE_INIT, BETTER_SOUND} AppMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_MATH_CM4
#define WAVE_STEPS 16

// for the "Making better sounds" section
#define SAMPLE_RATE 1000.0f   // Hz
#define SIGNAL_FREQ 50.0f     // Hz output sine frequency
#define DAC_MAX 4095.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

/* USER CODE BEGIN PV */
	// vars to hold current output values
	uint32_t triangle = 0;
	uint32_t sawtooth = 0;
	uint32_t sine = 0;
	// index for stepping through wave arrays
	uint32_t i = 0;
	volatile float T;
	volatile uint8_t fixedWave = 0;              // 0=triangle,1=saw,2=sine
	AppMode appMode = MODE_INIT;       // default application mode


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
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
  MX_DAC1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t triangleWave[WAVE_STEPS];
  uint32_t sawtoothWave[WAVE_STEPS];
  uint32_t sineWave[WAVE_STEPS];

  GPIO_PinState lastButtonState = GPIO_PIN_SET; // not pressed

  // for the "better sound" section
  float phase = 0.0f;
  float phaseStep = 2.0f * PI * SIGNAL_FREQ / SAMPLE_RATE;

  // temperature -> frequency mapping
  float minFreq = 200.0f;   // lowest pitch
  float maxFreq = 1000.0f;  // highest pitch
  float tempMin = -40.0f;    // minimum temperature
  float tempMax = 120.0f;    // maximum temperature

  for (uint32_t j = 0; j < WAVE_STEPS; j++) {
      // triangle wave
      if (j < WAVE_STEPS / 2) {
          triangleWave[j] = j * (4095 / (WAVE_STEPS / 2));
      } else {
          triangleWave[j] = 4095 - (j - WAVE_STEPS / 2) * (4095 / (WAVE_STEPS / 2));
      }

      // sawtooth wave
      sawtoothWave[j] = j * (4095 / WAVE_STEPS);

      // sine wave
      float angle = 2.0f * M_PI * ((float)j / (float)WAVE_STEPS); // 0 to 2π
      sineWave[j] = (uint32_t)((sin(angle) + 1.0f) * 2047.5f); // scale to 0-4095
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // set DAC values
	  triangle = triangleWave[i];
	  sawtooth = sawtoothWave[i];
	  sine = sineWave[i];

	  // making better sound
      float betterSine = arm_sin_f32(phase);   // sine in [-1, 1]
      uint32_t betterSineDAC = (uint32_t)((betterSine + 1.0f) * 0.5f * DAC_MAX);
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, betterSineDAC);
      phase += phaseStep;
      if (phase >= 2.0f * PI) phase -= 2.0f * PI;

	  // detect button press
	  GPIO_PinState buttonState = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

	  if (lastButtonState == GPIO_PIN_SET && buttonState == GPIO_PIN_RESET) {
	      // button just pressed
	      if (appMode == MODE_INIT) {
	          appMode = MODE_TEMP;   // switch to temperature mode
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  // LED on
	      } else {
	          appMode = MODE_FIXED;  // switch back to fixed mode
	          fixedWave = (fixedWave + 1) % 3;
	          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // LED off
	      }
	  }
	  lastButtonState = buttonState;


	  ADC_ChannelConfTypeDef sConfig = {0};
	  // read vrefint
	  float VREF;
	  uint16_t refVoltage;
	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;// long enough for the min 5 us
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	  HAL_ADC_Start(&hadc1);
	  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		  refVoltage = HAL_ADC_GetValue(&hadc1);
		  int32_t VREFINT = *((uint16_t*) (0x1FFF75AAUL));
		  VREF = 3.0f * (float)VREFINT / (float)refVoltage;
	  }

	  // read temp sensor
	  uint16_t VTEMP;
	  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	  HAL_ADC_Start(&hadc1);
	  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		  VTEMP = HAL_ADC_GetValue(&hadc1);

		  int32_t TS_CAL1 = *((uint16_t*) (0x1FFF75A8UL));
		  int32_t TS_CAL2 = *((uint16_t*) (0x1FFF75CAUL));

		  float first = (130.0f - 30.0f) / ((float)(TS_CAL2 - TS_CAL1));
		  float second = (VTEMP * (VREF / 3.0f)) - (float)(TS_CAL1);
		  T = first * second + 30.0f;

	  }

	  // Un-comment this to override into the better_sound mode
	  // appMode = BETTER_SOUND;

	  uint32_t dacValue;
	  switch(appMode) {
	  	  case MODE_FIXED:
	  		  switch(fixedWave) {
				  case 0: dacValue = sawtoothWave[i]; break;
				  case 1: dacValue = triangleWave[i]; break;
				  case 2: dacValue = sineWave[i]; break;
	  		  }
	  		  break;
	  	  case MODE_TEMP:
	  		  // normalize temperature 0..1
	  		  float tempNorm = (T - tempMin) / (tempMax - tempMin);
	  		  if (tempNorm < 0) tempNorm = 0;
	  		  if (tempNorm > 1) tempNorm = 1;

	  		  // map temperature to frequency
	  		  float freq = minFreq + tempNorm * (maxFreq - minFreq);

	  		  phaseStep = 2.0f * PI * freq / SAMPLE_RATE; // SAMPLE_RATE = DAC update rate
	  		  phase += phaseStep;
	  		  if(phase >= 2.0f*PI) phase -= 2.0f*PI;
	  		  dacValue = (uint32_t)((arm_sin_f32(phase)+1.0f)*0.5f*DAC_MAX);

	  		  break;
	  	  case MODE_INIT:
	  		  dacValue = sawtoothWave[i]; break;
	  	  case BETTER_SOUND:
	  		  dacValue = betterSineDAC;
	  }

      // write to DAC channel 1 (speaker output)
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacValue);

	  // increment index, wrap around automatically
	  i = (i + 1) % WAVE_STEPS;

	  // wait to get a roughly 500 us - 1ms period
	  for (volatile int j = 0; j < 100; j++); // this gives 1.2 kHz frequency
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
	  __HAL_RCC_DAC1_CLK_ENABLE();
	  hdac1.Instance = DAC1;
	  HAL_DAC_Init(&hdac1);
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

	  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

	  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);
	  HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2);

	  // Enable DAC channels
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
