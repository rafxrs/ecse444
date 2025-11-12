/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ECSE 444 Lab 3 — Parts 1–4 combined
  ******************************************************************************
  * Behavior:
  * - Press button: record fixed-length mic sample (LED blinks).
  * - Press button again: play six notes, then the recorded sample (LED solid).
  * - Press button again: re-record; repeat.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  APP_IDLE = 0,        // LED off
  APP_RECORDING,       // LED blinking
  APP_READY,           // recorded, waiting to play (LED off)
  APP_PLAY_NOTES,      // LED on
  APP_PLAY_SAMPLE      // LED on
} AppState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---- Audio/sample-rate pairing ----
// DFSDM Divider = 117, FOSR = 64  -> ~16 kHz input Fs
// TIM2.Period = 7487 (ARR) -> ~16 kHz DAC trigger
#define FS_HZ                16000.0f

// ---- Recording buffer length (fixed-length sample) ----
#define NSAMPLES             16000       // ~1.0 s at ~16 kHz

// ---- Note synthesis ----
#define NOTE_MAX_SAMPLES     1024        // max single-period buffer for one note
#define NOTE_DURATION_MS     250         // duration per note
static const float kNotesHz[6] = {523.25f, 587.33f, 659.25f, 698.46f, 783.99f, 880.00f};

// ---- DAC dynamic range ----
#define DAC_MID              2048
#define DAC_AMP              1365        // ~2/3 full-scale peak (avoid speaker clipping)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile int32_t maxVal; // to test buffer size
// Mic (DFSDM) raw buffer (32-bit words with top 24-bit signed sample)
static int32_t  micBuffer[NSAMPLES];
// Converted samples for DAC (12-bit right-aligned stored in 16-bit)
static uint16_t dacSampleBuffer[NSAMPLES];

static volatile AppState g_state = APP_IDLE;
static volatile uint8_t  g_record_done = 0;
// Request from ISR for the main loop to run the “six notes + sample” sequence
static volatile uint8_t  g_request_playback = 0;

// Guard to let ISR stop any current DAC DMA/TIM2 before state change
static void stop_playback_if_running(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */
static inline uint16_t q24_to_dac12(int32_t s);
static void convert_recording_to_dac(void);
static void start_recording(void);
static void play_six_notes_blocking(void);
static void play_sample_once_blocking(void);
static void start_note_circular(float freq_hz, uint16_t *buf, uint32_t *len);
static void led_update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Map signed 24-bit DFSDM sample (in bits[31:8]) to unsigned 12-bit DAC
static inline uint16_t q24_to_dac12(int32_t s)
{
  int32_t v = (s >> 11);       // discard 8 LSBs + reduce gain
  v += DAC_MID;                // center around mid-scale
  if (v < 0)    v = 0;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}

static void convert_recording_to_dac(void)
{
  for (uint32_t i = 0; i < NSAMPLES; ++i)
    dacSampleBuffer[i] = q24_to_dac12(micBuffer[i]);
}

//  IDLE/READY: off
//  RECORDING: blink handled in main loop
//  PLAY_*: solid on
static void led_update(void)
{
  if (g_state == APP_PLAY_NOTES || g_state == APP_PLAY_SAMPLE)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  else if (g_state == APP_IDLE || g_state == APP_READY)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

static void stop_playback_if_running(void)
{
  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_TIM_Base_Stop(&htim2);
}

static void start_recording(void)
{
  stop_playback_if_running();
  g_record_done = 0;
  g_request_playback = 0;        // cancel any pending playback request
  g_state = APP_RECORDING;
  // Normal (one-shot) DMA: DFSDM → micBuffer; CpltCallback will fire when full
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, micBuffer, NSAMPLES);
}

// Build one full-period sine buffer for a given freq, start DAC DMA circular on it
static void start_note_circular(float freq_hz, uint16_t *buf, uint32_t *len)
{
  float samples_per_period_f = FS_HZ / freq_hz;
  uint32_t samples = (uint32_t)(samples_per_period_f + 0.5f);
  if (samples < 16) samples = 16;
  if (samples > NOTE_MAX_SAMPLES) samples = NOTE_MAX_SAMPLES;

  for (uint32_t i = 0; i < samples; ++i)
  {
    float ph = (2.0f * (float)M_PI) * ((float)i / (float)samples);
    float s  = arm_sin_f32(ph);
    int32_t v = DAC_MID + (int32_t)(DAC_AMP * s);
    if (v < 0)    v = 0;
    if (v > 4095) v = 4095;
    buf[i] = (uint16_t)v;
  }

  *len = samples;

  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)buf, *len, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);
}

static void play_six_notes_blocking(void)
{
  g_state = APP_PLAY_NOTES;
  led_update();

  static uint16_t noteBuf[NOTE_MAX_SAMPLES];
  uint32_t nlen = 0;

  for (int k = 0; k < 6; ++k)
  {
    if (g_state != APP_PLAY_NOTES) break;

    start_note_circular(kNotesHz[k], noteBuf, &nlen);
    HAL_Delay(NOTE_DURATION_MS);

    // Stop current note before starting the next one
    stop_playback_if_running();

    // Small pause between notes
    HAL_Delay(40);
  }

  stop_playback_if_running();
}

static void play_sample_once_blocking(void)
{
  convert_recording_to_dac();

  g_state = APP_PLAY_SAMPLE;
  led_update();

  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                    (uint32_t*)dacSampleBuffer, NSAMPLES, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);

  uint32_t play_ms = (uint32_t)((1000.0f * (float)NSAMPLES) / FS_HZ + 0.5f);

  // Busy-wait; button interrupt can preempt and start a new recording
  uint32_t elapsed = 0;
  while (elapsed < play_ms && g_state == APP_PLAY_SAMPLE)
  {
    HAL_Delay(10);
    elapsed += 10;
  }

  stop_playback_if_running();
  // After playing, go to IDLE so next press records again
  g_state = APP_IDLE;
  g_record_done = 0;
  led_update();
}


// Button PC13: ISR — DO NOT block here; just change state or set flags
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_13) return;

  switch (g_state)
  {
    case APP_IDLE:
    case APP_PLAY_NOTES:
    case APP_PLAY_SAMPLE:
      // Start a new recording
      start_recording();
      break;

    case APP_RECORDING:
      HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
      g_state = APP_IDLE;
      led_update();
      break;

    case APP_READY:
      // Only when recording is done and user presses again — request playback
      if (g_record_done)
      {
        g_request_playback = 1;   // handled in main loop
      }
      break;

    default:
      break;
  }
}

// DFSDM DMA complete: recording finished
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
  if (h != &hdfsdm1_filter0) return;

  // Stop the DFSDM filter cleanly
  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);

  // Recording is done; wait for *next button press* to play
  g_record_done = 1;
  g_state = APP_READY;
  led_update();
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
  g_state = APP_IDLE;
  led_update();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // LED blink while recording
    if (g_state == APP_RECORDING)
    {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      HAL_Delay(150);
    }

    // If READY and user requested playback (flag set by ISR), do it here (thread mode)
    if (g_request_playback && g_state == APP_READY && g_record_done)
    {
      g_request_playback = 0;
      maxVal = 0;
      for (int i = 0; i < NSAMPLES; i++) {
          int32_t val = micBuffer[i];
          if (abs(val) > maxVal) maxVal = abs(val);
      }
      HAL_Delay(1000);
      play_six_notes_blocking();
      if (g_state == APP_RECORDING) continue; // user interrupted to record

      play_sample_once_blocking();
      // After sample, we end in APP_READY; next press re-records
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

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
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 117;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7487;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
