/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Ph.h"
#include "Temperature.h"
#include "Adafruit_ILI9341.h"
#include "neopixel.h"
#include "Stepper.h"

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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_up;
DMA_HandleTypeDef hdma_tim3_ch4_up;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

osThreadId defaultTaskHandle;
osThreadId TempSensorTaskHandle;
osThreadId HeaterTaskHandle;
osThreadId PhSensorTaskHandle;
osThreadId PhBalanceTaskHandle;
osThreadId DataTxTaskHandle;
osThreadId ScreenTaskHandle;
osThreadId LEDTaskHandle;
osThreadId BettaFeederTaskHandle;
osThreadId FlakeFeederTaskHandle;
osSemaphoreId ledChangedSemHandle;
osSemaphoreId feedBettaSemHandle;
osSemaphoreId feedFlakeSemHandle;
/* USER CODE BEGIN PV */

extern void delay_us(uint16_t delay);

// Global variables
// ph values
struct Ph ph;
float current_pH = 7.0;
float target_pH = 7.0;
float display_current_pH = 0.0;
float display_target_pH = 0.0;

// temperature values
struct Temperature temperature;
double current_temperature = 75.0;
double target_temperature = 74.0;
double display_current_temperature = 0.0;
double display_target_temperature = 0.0;

// LED values
uint8_t LED_enable = 1;
uint8_t LED_red = 100, LED_blue = 0, LED_green = 0;
uint8_t new_LED_enable = 0;
uint8_t new_LED_red = 0, new_LED_blue = 0, new_LED_green = 0;

// Feeder Value
uint8_t serve_betta = 0, serve_flake = 0, size_betta = 0, size_flake = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void const * argument);
void TempSensorBegin(void const * argument);
void HeaterBegin(void const * argument);
void PhSensorBegin(void const * argument);
void PhBalanceBegin(void const * argument);
void DataTxBegin(void const * argument);
void ScreenTaskBegin(void const * argument);
void LEDBegin(void const * argument);
void BettaFeederBegin(void const * argument);
void FlakeFeederBegin(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us_main (uint16_t delay) {
	__HAL_TIM_SET_COUNTER (&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2)<delay);
}


// Heater GPIO and Definitions
#define GPIO_HEATER_Base GPIOA
#define GPIO_HEATER_PIN GPIO_PIN_12
#define HEATER_TOLERANCE_LOW 0.2
#define HEATER_TOLERANCE_HIGH 0.2
#define PH_TOLERANCE_LOW 0.1
#define PH_TOLERANCE_HIGH 0.1
#define STEPS_CAP 100
#define FLAKE_FEEDER_PIN GPIO_PIN_0
#define FLAKE_FEEDER_PORT GPIOC
#define INCLUDE_vTaskSuspend 1

//pH Pins
#define GPIO_PH_BASE GPIOC
#define GPIO_PH_UP_PIN GPIO_PIN_0
#define GPIO_PH_DOWN_PIN GPIO_PIN_1

void raise_pH(){
	HAL_GPIO_WritePin(GPIO_PH_BASE, GPIO_PH_UP_PIN, GPIO_PIN_SET);
	for (int i = 0; i < 1100; ++i){
		delay_us_main(300);
	}
	HAL_GPIO_WritePin(GPIO_PH_BASE, GPIO_PH_UP_PIN, GPIO_PIN_RESET);
}

void lower_pH(){
	HAL_GPIO_WritePin(GPIO_PH_BASE, GPIO_PH_DOWN_PIN, GPIO_PIN_SET);
	for (int i = 0; i < 1100; ++i){
		delay_us_main(300);
	}
	HAL_GPIO_WritePin(GPIO_PH_BASE, GPIO_PH_DOWN_PIN, GPIO_PIN_RESET);
}


//Screen Variables Begin:

#define SCREEN_HEADER_VALUE_X 0
#define SCREEN_DATA_VALUE_X 250
#define SCREEN_LINE_HEIGHT 20



SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef* ILI9341_HSPI_INST = &hspi2;

GPIO_TypeDef* ILI9341_CSX_PORT = GPIOC;
uint16_t ILI9341_CSX_PIN  = GPIO_PIN_4;

GPIO_TypeDef* ILI9341_DCX_PORT = GPIOC;
uint16_t ILI9341_DCX_PIN  = GPIO_PIN_5;

cursor_t cur;
cursor_t temp_title = {30,10};
cursor_t ph_title = {340, 10};
cursor_t current_temp_header= {0,80};
cursor_t current_temp_val = {60,100};
cursor_t target_temp_header = {10,190};
cursor_t target_temp_val = {60,210};
cursor_t current_pH_header = {290,80};
cursor_t current_pH_val = {320,100};
cursor_t target_pH_header = {290,190};
cursor_t target_pH_val = {320,210};

void ILI9341_default_print() {
	delay_us_main(100000);
	ILI9341_ResetTextBox(&cur);
	ILI9341_SetClrParam(0xFFFF, 0x8000);
	ILI9341_ResetTextBox(&cur);
	ILI9341_SetFontParam(3);
	ILI9341_PrintString(&temp_title, "Temperature");
	ILI9341_PrintString(&ph_title, "pH");
	ILI9341_SetFontParam(2);
	ILI9341_PrintString(&target_temp_header, "Target Temperature:");
	ILI9341_PrintString(&current_temp_header, "Current Temperature:");
	ILI9341_PrintString(&target_pH_header, "Target pH:");
	ILI9341_PrintString(&current_pH_header, "Current pH:");
	ILI9341_SetFontParam(4);
	ILI9341_SetClrParam(0x0F00, 0x8000);
}

//Define Stepper Struct
Stepper bettaFeeder;
uint16_t stepper_pins[4] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5};
GPIO_TypeDef* stepper_ports[4] = {GPIOA, GPIOA, GPIOA, GPIOA};

float uint8_to_float(uint8_t is_pH, uint8_t value[]) {
	float return_val = 0;
	if(is_pH) {
		return_val += (value[0] - 48.0) * 1.0;
		return_val += (value[2] - 48.0) / 10.0;
		return_val += (value[3] - 48.0) / 100.0;
	}
	else {
		return_val += (value[0] - 48.0) * 10.0;
		return_val += (value[1] - 48.0) * 1.0;
		return_val += (value[3] - 48.0) / 10.0;
		return_val += (value[4] - 48.0) / 100.0;
	}
	return return_val;
}

uint8_t convert_LED_value(uint8_t value[]) {
	uint8_t return_val = 0;
	return_val += (value[0] - 48) * 100;
	return_val += (value[1] - 48) * 10;
	return_val += (value[2] - 48);
}


uint8_t tx_data[12] = "75.00 7.49\r\n";
uint8_t rx_data[27];

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
  MX_ADC_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  delay_us_main(10000);
  ILI9341_Init();
  delay_us_main(10000);
  __disable_irq();
  ILI9341_default_print();
  __enable_irq();

  init_stepper(STEPS_CAP, &bettaFeeder, stepper_pins, stepper_ports);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ledChangedSem */
  osSemaphoreDef(ledChangedSem);
  ledChangedSemHandle = osSemaphoreCreate(osSemaphore(ledChangedSem), 1);

  /* definition and creation of feedBettaSem */
  osSemaphoreDef(feedBettaSem);
  feedBettaSemHandle = osSemaphoreCreate(osSemaphore(feedBettaSem), 1);

  /* definition and creation of feedFlakeSem */
  osSemaphoreDef(feedFlakeSem);
  feedFlakeSemHandle = osSemaphoreCreate(osSemaphore(feedFlakeSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TempSensorTask */
  osThreadDef(TempSensorTask, TempSensorBegin, osPriorityAboveNormal, 0, 128);
  TempSensorTaskHandle = osThreadCreate(osThread(TempSensorTask), NULL);

  /* definition and creation of HeaterTask */
  osThreadDef(HeaterTask, HeaterBegin, osPriorityNormal, 0, 128);
  HeaterTaskHandle = osThreadCreate(osThread(HeaterTask), NULL);

  /* definition and creation of PhSensorTask */
  osThreadDef(PhSensorTask, PhSensorBegin, osPriorityIdle, 0, 128);
  PhSensorTaskHandle = osThreadCreate(osThread(PhSensorTask), NULL);

  /* definition and creation of PhBalanceTask */
  osThreadDef(PhBalanceTask, PhBalanceBegin, osPriorityIdle, 0, 128);
  PhBalanceTaskHandle = osThreadCreate(osThread(PhBalanceTask), NULL);

  /* definition and creation of DataTxTask */
  osThreadDef(DataTxTask, DataTxBegin, osPriorityHigh, 0, 256);
  DataTxTaskHandle = osThreadCreate(osThread(DataTxTask), NULL);

  /* definition and creation of ScreenTask */
  osThreadDef(ScreenTask, ScreenTaskBegin, osPriorityHigh, 0, 256);
  ScreenTaskHandle = osThreadCreate(osThread(ScreenTask), NULL);

  /* definition and creation of LEDTask */
 // osThreadDef(LEDTask, LEDBegin, osPriorityAboveNormal, 0, 256);
 // LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of BettaFeederTask */
 // osThreadDef(BettaFeederTask, BettaFeederBegin, osPriorityIdle, 0, 128);
 // BettaFeederTaskHandle = osThreadCreate(osThread(BettaFeederTask), NULL);

  /* definition and creation of FlakeFeederTask */
 // osThreadDef(FlakeFeederTask, FlakeFeederBegin, osPriorityIdle, 0, 128);
 // FlakeFeederTaskHandle = osThreadCreate(osThread(FlakeFeederTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  neopixel_set(&htim3, 1, 255, 255, 255);
//	  HAL_Delay(500);
//	  neopixel_set(&htim3, 1, 255, 0, 0);
//	  HAL_Delay(500);
//	  neopixel_set(&htim3, 0, 255, 255, 255);
//	  HAL_Delay(500);

	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 23-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC0 PC1
                           PC4 PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA7 PA8 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB10 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TempSensorBegin */
/**
* @brief Function implementing the TempSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TempSensorBegin */
void TempSensorBegin(void const * argument)
{
  /* USER CODE BEGIN TempSensorBegin */
  temperature_init(&temperature);
  /* Infinite loop */
  for(;;)
  {
	taskENTER_CRITICAL();
	current_temperature = temperature_read(&temperature);
	taskEXIT_CRITICAL();
	osDelay(1000);
  }
  /* USER CODE END TempSensorBegin */
}

/* USER CODE BEGIN Header_HeaterBegin */
/**
* @brief Function implementing the HeaterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HeaterBegin */
void HeaterBegin(void const * argument)
{
  /* USER CODE BEGIN HeaterBegin */
  /* Infinite loop */
  for(;;) {
	// Heater is already on, check if it should be turned off
	if(HAL_GPIO_ReadPin(GPIO_HEATER_Base,GPIO_HEATER_PIN)){
		if(current_temperature >= target_temperature + HEATER_TOLERANCE_HIGH){
			// turn heater off
			HAL_GPIO_WritePin(GPIO_HEATER_Base,GPIO_HEATER_PIN,GPIO_PIN_RESET);
		}
	}
	// heater is off, check if we should turn it on
	else {
		if (current_temperature < target_temperature - HEATER_TOLERANCE_LOW){
			// turn heater on
			HAL_GPIO_WritePin(GPIO_HEATER_Base,GPIO_HEATER_PIN,GPIO_PIN_SET);
		}
	}
    osDelay(1000);
  }
  /* USER CODE END HeaterBegin */
}

/* USER CODE BEGIN Header_PhSensorBegin */
/**
* @brief Function implementing the PhSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PhSensorBegin */
void PhSensorBegin(void const * argument)
{
  /* USER CODE BEGIN PhSensorBegin */
  pH_init(&ph, hadc);
  /* Infinite loop */
  for(;;)
  {
	current_pH = pH_read(&ph);
    osDelay(1000);
  }
  /* USER CODE END PhSensorBegin */
}

/* USER CODE BEGIN Header_PhBalanceBegin */
/**
* @brief Function implementing the PhBalanceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PhBalanceBegin */
void PhBalanceBegin(void const * argument)
{
  /* USER CODE BEGIN PhBalanceBegin */
  /* Infinite loop */
  for(;;)
  {
		if (current_pH - PH_TOLERANCE_HIGH > target_pH){
			lower_pH();
		}
		else if (current_pH + PH_TOLERANCE_LOW < target_pH){
			raise_pH();
		}
	    osDelay(300000);
  }
  /* USER CODE END PhBalanceBegin */
}

/* USER CODE BEGIN Header_DataTxBegin */
/**
* @brief Function implementing the DataTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataTxBegin */
void DataTxBegin(void const * argument)
{
  /* USER CODE BEGIN DataTxBegin */
  int pos;
  /* Infinite loop */
  for(;;)
  {
	pos = 0;
	// transmit temperature and pH data to the website
	// add temperature
	uint8_t temp_data[6];
	snprintf(temp_data, 6, "%0.2f", current_temperature);
	memcpy(tx_data, temp_data, 5);
	pos += 5;

	// add a space
	memcpy(tx_data + pos, " ", 1);
	pos += 1;

	// add pH data
	uint pH_data[5];
	snprintf(pH_data, 5, "%0.2f", current_pH);
	memcpy(tx_data + pos, pH_data, 4);
	pos += 4;

	// add carriage return
	memcpy(tx_data + pos, "\r\n", 2);
	pos += 2;
	taskENTER_CRITICAL();
	HAL_UART_Transmit(&huart4, tx_data, sizeof(tx_data), 10000);
	HAL_UART_Receive(&huart4, rx_data, 27, 10000);
	taskEXIT_CRITICAL();
	uint8_t target_temp_uint8[5], target_pH_uint8[4], LED_red_uint8[3], LED_blue_uint8[3], LED_green_uint8[3];
	memcpy(target_temp_uint8, rx_data + 1, 5);
	memcpy(target_pH_uint8, rx_data + 7, 4);
	memcpy(LED_red_uint8, rx_data + 13, 4);
	memcpy(LED_green_uint8, rx_data + 16, 4);
	memcpy(LED_blue_uint8, rx_data + 19, 4);
	target_temperature = uint8_to_float(0, target_temp_uint8);
	target_pH = uint8_to_float(1, target_pH_uint8);
	new_LED_enable = rx_data[12] - 48;
	new_LED_red = convert_LED_value(LED_red_uint8);
	new_LED_green = convert_LED_value(LED_green_uint8);
	new_LED_blue = convert_LED_value(LED_blue_uint8);
	if(new_LED_enable != LED_enable || new_LED_red != LED_red || new_LED_green != LED_green || new_LED_blue != LED_blue) {
		LED_enable = new_LED_enable;
		LED_red = new_LED_red;
		LED_green = new_LED_green;
		LED_blue = new_LED_blue;
		xSemaphoreGive(ledChangedSemHandle);
	}
	serve_betta = rx_data[23] - 48;
	size_betta = rx_data[24] - 48;
	serve_flake = rx_data[25] - 48;
	size_flake = rx_data[26] - 48;
	if (serve_betta) {
		xSemaphoreGive(feedBettaSemHandle);
	}
	if (serve_flake) {
		xSemaphoreGive(feedFlakeSemHandle);
	}
    osDelay(1000);
  }
  /* USER CODE END DataTxBegin */
}

/* USER CODE BEGIN Header_ScreenTaskBegin */
/**
* @brief Function implementing the ScreenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScreenTaskBegin */
void ScreenTaskBegin(void const * argument)
{
  /* USER CODE BEGIN ScreenTaskBegin */
  /* Infinite loop */
  for(;;) {

	if (display_target_temperature != target_temperature) {
		uint8_t target_temp_buf[6];
		snprintf(target_temp_buf, 6, "%0.2f", target_temperature);
		ILI9341_PrintString(&target_temp_val, target_temp_buf);
		display_target_temperature = target_temperature;
		target_temp_val.x = SCREEN_DATA_VALUE_X;
		target_temp_val.y = 0;
	}
	if (display_current_temperature != current_temperature) {
		uint8_t current_temp_buf[6];
		snprintf(current_temp_buf, 6, "%0.2f", current_temperature);
		ILI9341_PrintString(&current_temp_val, current_temp_buf);
		display_current_temperature = current_temperature;
		current_temp_val.x = SCREEN_DATA_VALUE_X;
		current_temp_val.y = 20;
	}
	if (display_target_pH != target_pH) {
		uint8_t target_pH_buf[6];
		snprintf(target_pH_buf, 6, "%0.2f", target_pH);
		ILI9341_PrintString(&target_pH_val, target_pH_buf);
		display_target_pH = target_pH;
		target_pH_val.x = SCREEN_DATA_VALUE_X;
		target_pH_val.y = 40;
	}
	if (display_current_pH != current_pH) {
		uint8_t current_pH_buf[6];
		snprintf(current_pH_buf, 6, "%0.2f", current_pH);
		ILI9341_PrintString(&current_pH_val, current_pH_buf);
		display_current_pH = current_pH;
		current_pH_val.x = SCREEN_DATA_VALUE_X;
		current_pH_val.y = 60;
	}
    osDelay(1000);
  }
  /* USER CODE END ScreenTaskBegin */
}

/* USER CODE BEGIN Header_LEDBegin */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDBegin */
void LEDBegin(void const * argument)
{
  /* USER CODE BEGIN LEDBegin */
  /* Infinite loop */
  for(;;) {
	//xSemaphoreTake(ledChangedSemHandle, portMAX_DELAY);
	neopixel_set(&htim3, LED_enable, LED_red, LED_green, LED_blue);
  }
  osDelay(10000);
  /* USER CODE END LEDBegin */
}

/* USER CODE BEGIN Header_BettaFeederBegin */
/**
* @brief Function implementing the BettaFeederTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BettaFeederBegin */
void BettaFeederBegin(void const * argument)
{
  /* USER CODE BEGIN BettaFeederBegin */
  /* Infinite loop */
  for(;;)
  {
	  xSemaphoreTake(feedBettaSemHandle, portMAX_DELAY);
	  for (int i =0; i < size_betta; ++i)
	  {
		  taskENTER_CRITICAL();
	  	  for (int i = 0; i < STEPS_CAP; ++i){
	  		  step(1, &bettaFeeder);
	  		  delay_us_main(1000);
	  	  }
	  	  taskEXIT_CRITICAL();
	  }
  }
  /* USER CODE END BettaFeederBegin */
}

/* USER CODE BEGIN Header_FlakeFeederBegin */
/**
* @brief Function implementing the FlakeFeederTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FlakeFeederBegin */
void FlakeFeederBegin(void const * argument)
{
  /* USER CODE BEGIN FlakeFeederBegin */
  /* Infinite loop */
  for(;;)
  {
	 xSemaphoreTake(feedFlakeSemHandle, portMAX_DELAY);
	 for (int i = 0; i < size_flake; ++i){
		 taskENTER_CRITICAL();
		 HAL_GPIO_WritePin(FLAKE_FEEDER_PORT,FLAKE_FEEDER_PIN,GPIO_PIN_SET);

		 for (int i = 0; i < 100; ++i){
			 delay_us_main(10000);
		 }
		 HAL_GPIO_WritePin(FLAKE_FEEDER_PORT,FLAKE_FEEDER_PIN,GPIO_PIN_RESET);
		 taskEXIT_CRITICAL();
	 }
  }
  /* USER CODE END FlakeFeederBegin */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
