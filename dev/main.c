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

#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "Adafruit_ILI9341.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GPIO_TEMP_Base GPIOA
#define GPIO_TEMP_TX GPIO_PIN_5
#define GPIO_TEMP_RX GPIO_PIN_6
#define us_timer htim2

#define GPIO_HEATER_Base GPIOA
#define GPIO_HEATER_PIN GPIO_PIN_7


#define GPIO_PH_BASE GPIOC
#define PH_UP_PIN GPIO_PIN_8
#define PH_DOWN_PIN GPIO_PIN_6


#define DEFAULT_TEMP 77 //Default temp in degrees Fahrenheit if not specified
#define HEATER_TOLERANCE_LOW 2
#define HEATER_TOLERANCE_HIGH 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

osThreadId defaultTaskHandle;
osThreadId TempSensorTaskHandle;
osThreadId HeaterTaskHandle;
osThreadId PhSensorTaskHandle;
osThreadId PhBalanceTaskHandle;
osThreadId DataRxTaskHandle;
osThreadId DataTxTaskHandle;
osThreadId ScreenTaskHandle;
osTimerId myTimer01Handle;
osSemaphoreId myBinarySem01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void TempSensorBegin(void const * argument);
void HeaterBegin(void const * argument);
void PhSensorBegin(void const * argument);
void PhBalanceBegin(void const * argument);
void DataRxBegin(void const * argument);
void DataTxBegin(void const * argument);
void ScreenTaskBegin(void const * argument);
void Callback01(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Screen Variables Begin:
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef* ILI9341_HSPI_INST = &hspi2;

GPIO_TypeDef* ILI9341_CSX_PORT = GPIOC;
uint16_t ILI9341_CSX_PIN  = GPIO_PIN_4;

GPIO_TypeDef* ILI9341_DCX_PORT = GPIOC;
uint16_t ILI9341_DCX_PIN  = GPIO_PIN_5;

cursor_t cur;
int changedBrightness;                                                              				// boolean var if brightness has been changed
uint8_t size[2];

volatile double display_current_temp = 0;
volatile double display_current_pH = 0;
volatile double display_target_temp = 0;
volatile double display_target_pH = 0;

//struct ILI9341_locations{
	cursor_t target_temp_header = {0,0};
	cursor_t target_temp_val = {250,0};
	cursor_t current_temp_header= {0,20};
	cursor_t current_temp_val = {250,20};
	cursor_t target_pH_header = {0,40};
	cursor_t target_pH_val = {250,40};
	cursor_t current_pH_header = {0,60};
	cursor_t current_pH_val = {250,60};
//};
//Screen Variables End:

volatile double temp_raw;
volatile double temp_C;
volatile double temp_F;
volatile double current_pH =1;
volatile double target_pH = 7.0;

volatile double target_temp = DEFAULT_TEMP;
volatile double current_temp = DEFAULT_TEMP;
volatile double temp_samples[8] = {DEFAULT_TEMP, DEFAULT_TEMP,DEFAULT_TEMP, DEFAULT_TEMP,DEFAULT_TEMP, DEFAULT_TEMP,DEFAULT_TEMP, DEFAULT_TEMP};


volatile uint8_t tx_packet[80];
volatile uint8_t rx_packet[80];


void delay_us (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER (&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2)<delay); //{

}

void ILI9341_default_print()
{
	ILI9341_ResetTextBox(&cur);
	ILI9341_PrintString(&target_temp_header, "Target Temperature:");
	ILI9341_PrintString(&current_temp_header, "Current Temperature:");
	ILI9341_PrintString(&target_pH_header, "Target pH value:");
	ILI9341_PrintString(&current_pH_header, "Current pH value:");
}

void ILI9341_update()
{
	if (display_target_temp != target_temp){
		uint8_t target_temp_buf[6];
		snprintf(target_temp_buf, 6, "%0.2f", target_temp);
		ILI9341_PrintString(&target_temp_val, target_temp_buf);
		display_target_temp = target_temp;
		target_temp_val.x = 250;
		target_temp_val.y = 0;
	}
	if (display_current_temp != current_temp){
		uint8_t current_temp_buf[6];
		snprintf(current_temp_buf, 6, "%0.2f", current_temp);
		ILI9341_PrintString(&current_temp_val, current_temp_buf);
		display_current_temp = current_temp;
		current_temp_val.x = 250;
		current_temp_val.y = 20;
	}
	if (display_target_pH != target_pH){
		uint8_t target_pH_buf[6];
		snprintf(target_pH_buf, 6, "%0.2f", target_pH);
		ILI9341_PrintString(&target_pH_val, target_pH_buf);
		display_target_pH = target_pH;
		target_pH_val.x = 250;
		target_pH_val.y = 40;
	}
	if (display_current_pH != current_pH){
		uint8_t current_pH_buf[6];
		snprintf(current_pH_buf, 6, "%0.2f", current_pH);
		ILI9341_PrintString(&current_pH_val, current_pH_buf);
		display_current_pH = current_pH;
		current_pH_val.x = 250;
		current_pH_val.y = 60;
	}

}




bool DS18B20_Init()
{
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  delay_us(5);
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
  delay_us(750);//480-960
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  int t = 0;
  while (HAL_GPIO_ReadPin(GPIO_TEMP_Base,GPIO_TEMP_RX))
  {
    t++;
    if (t > 60) return false;
    delay_us(1);
  }
  t = 480 - t;
  delay_us(t);
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  return true;
}

void DS18B20_Write(uint8_t data)
{
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
    delay_us(10);
    if (data & 1) HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
    else HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
    data >>= 1;
    delay_us(50);
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  }
}

uint8_t DS18B20_Read()
{
  HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX, GPIO_PIN_SET);
  delay_us(2);
  uint8_t data = 0;
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
    delay_us(5);
    data >>= 1;
    if (HAL_GPIO_ReadPin(GPIO_TEMP_Base,GPIO_TEMP_RX)) data |= 0x80;
    delay_us(55);
    HAL_GPIO_WritePin(GPIO_TEMP_Base,GPIO_TEMP_TX,GPIO_PIN_SET);
  }
  return data;
}

volatile int TempRead()
{
  if (!DS18B20_Init()) return 0;
  DS18B20_Write (0xCC); // Send skip ROM command
  DS18B20_Write (0x44); // Send reading start conversion command
  if (!DS18B20_Init()) return 0;
  DS18B20_Write (0xCC); // Send skip ROM command
  DS18B20_Write (0xBE); // Read the register, a total of nine bytes, the first two bytes are the conversion value
  int temp = DS18B20_Read (); // Low byte
  temp |= DS18B20_Read () << 8; // High byte
  return temp;
}

volatile double TempCompute()
{
	double sum = 0;
	double temporary[8];
	for (int i = 0; i < 8; ++i){
		temporary[i] = temp_samples[i];
	}
	for (int i = 1; i < 8; ++i){
		temp_samples[i] = temporary[i-1];
		sum += temp_samples[i];
	}
	temp_samples[0] = temp_F;
	sum += temp_F;
	return (sum/8);
}



void HEATER_CONTROL_ON()
{
	HAL_GPIO_WritePin(GPIO_HEATER_Base,GPIO_HEATER_PIN,GPIO_PIN_SET);
}

void HEATER_CONTROL_OFF()
{
	HAL_GPIO_WritePin(GPIO_HEATER_Base,GPIO_HEATER_PIN,GPIO_PIN_RESET);
}

void HEATER_TOGGLE()
{
	 HAL_GPIO_TogglePin(GPIO_HEATER_Base,GPIO_HEATER_PIN);
}

void HEATER_DECIDE()
{
	if(HAL_GPIO_ReadPin(GPIO_HEATER_Base,GPIO_HEATER_PIN)){
		if(current_temp >= target_temp + HEATER_TOLERANCE_HIGH){
			HEATER_CONTROL_OFF();
		}
	}
	else{
		if (current_temp < target_temp - HEATER_TOLERANCE_LOW){
			HEATER_CONTROL_ON();
		}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  ILI9341_Init();
  ILI9341_default_print();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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

  /* definition and creation of DataRxTask */
  osThreadDef(DataRxTask, DataRxBegin, osPriorityIdle, 0, 128);
  DataRxTaskHandle = osThreadCreate(osThread(DataRxTask), NULL);

  /* definition and creation of DataTxTask */
  osThreadDef(DataTxTask, DataTxBegin, osPriorityIdle, 0, 256);
  DataTxTaskHandle = osThreadCreate(osThread(DataTxTask), NULL);

  /* definition and creation of ScreenTask */
  osThreadDef(ScreenTask, ScreenTaskBegin, osPriorityAboveNormal, 0, 256);
  ScreenTaskHandle = osThreadCreate(osThread(ScreenTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
    osDelay(1);
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

  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
	  temp_raw = TempRead();
	  taskEXIT_CRITICAL();
	  if (temp_raw != 0)
	  {
		  temp_C  = temp_raw * 0.0625; // conversion accuracy is 0.0625 / LSB
		  temp_F = temp_raw * 0.1125 + 32;
		  current_temp = TempCompute();
	  }
	  if (temp_F > 60){
	  	  HAL_GPIO_TogglePin(GPIO_TEMP_Base, GPIO_PIN_8);
	  }
	  osDelay(2000);

  }

  osThreadTerminate(NULL);
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
  for(;;)
  {
	HEATER_DECIDE();
	osDelay(3000);
  }

  osThreadTerminate(NULL);
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
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 0xFFFFFFFF);
	current_pH = (double)HAL_ADC_GetValue(&hadc) * 0.002734375; // *14 /1024 /5
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
	HAL_GPIO_TogglePin(GPIO_PH_BASE, PH_UP_PIN);
	HAL_GPIO_TogglePin(GPIO_PH_BASE, PH_DOWN_PIN);
    osDelay(1000);
  }
  /* USER CODE END PhBalanceBegin */
}

/* USER CODE BEGIN Header_DataRxBegin */
/**
* @brief Function implementing the DataRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataRxBegin */
void DataRxBegin(void const * argument)
{
  /* USER CODE BEGIN DataRxBegin */
  /* Infinite loop */
  for(;;)
  {
	for (int i = 0; i < 80; ++i){
		rx_packet[i] = 0;
	}
    HAL_UART_Receive(&huart4, rx_packet, 43, 10000);
    osDelay(10000);
  }
  /* USER CODE END DataRxBegin */
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
  /* Infinite loop */
  for(;;)
  {
	uint8_t carriage_return[] = "\r\n";
	uint8_t temp_header[] = " Current temperature: ";
	uint8_t tx_temp[6];
	snprintf(tx_temp, 6, "%0.2f", current_temp);
	uint8_t pH_header[] = " Current pH value: ";
	uint8_t tx_pH[sizeof(current_pH)];
	snprintf(tx_pH, 4, "%0.2f", current_pH);
	unsigned int size=0;
	memcpy(tx_packet + size, temp_header, sizeof(temp_header));
	size+= sizeof(temp_header);
	memcpy(tx_packet + size, tx_temp, sizeof(tx_temp));
	size += sizeof(tx_temp);
	memcpy(tx_packet + size, pH_header, sizeof(pH_header));
	size += sizeof(pH_header);
	memcpy(tx_packet + size, tx_pH, 3);
	size += 3;
	memcpy(tx_packet + size, carriage_return, sizeof(carriage_return));
	size += sizeof(carriage_return);

	taskENTER_CRITICAL();
	HAL_UART_Transmit(&huart4, tx_packet, size, 10000);
	taskEXIT_CRITICAL();
    osDelay(10000);
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
  for(;;)
  {
	ILI9341_update();
    osDelay(10000);
  }
  /* USER CODE END ScreenTaskBegin */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
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

