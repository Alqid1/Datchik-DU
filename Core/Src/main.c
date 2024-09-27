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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include <stdio.h>
#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MIN_LEN 1
#define CIRCLE_LEN 131071
#define TIMER_CLOCK_TICK 0.000205
#define MICROCONTROLLER_FREQ 72000000
//BUFFERS 
uint8_t bufferSpd[12];
char bufTime[2];

//Status Flag
uint8_t StopFlag=1;
uint8_t TopFlag=0;
uint8_t BotFlag=1;
uint32_t EncodSpd=0;
uint8_t TimFlag=1;
uint32_t CurCycle=0;
uint8_t CycleFlag=1;
uint32_t SpeedFlag=0;
uint8_t MoveFlag=0;
uint8_t PulseFlag=0;
uint8_t Tim2Flag=0;

//Base Mode Settings
uint8_t Mode;
uint16_t time;
//Mode 1
uint32_t spd;
char bufSpd[4];

//Mode 2
uint32_t maxSpd;
uint32_t minSpd;
uint8_t cycle;
char bufMinSpd[5];
char bufMaxSpd[4];
char bufCycle[2];

//SPI
uint8_t TX_Data[2];
extern uint32_t lastLir_angle;
extern uint32_t sumCircle;
extern uint32_t distance;
uint32_t lir_angle;
uint8_t dir;

//USB
uint32_t len=1;
char buffer[18];

//Rotate Sensor
uint32_t Timer_TICK = 0;
float SensorSpd = 0.0;
uint32_t Pulse_Period=0;

//Engine
uint32_t TPS;
uint32_t CountPer;
uint16_t SpdMode;
uint32_t SpdTest=1;
float StepSpd;
uint32_t StepTime;
uint32_t CurSpd;
uint32_t MovementMode;
uint32_t LastSpd=0;
uint32_t LastSpd1=0;
uint32_t CheckSpd=0;

extern float motorSpeed;
extern uint8_t GoodFlag;
float FactSpeed;

uint16_t massive_avrg_values[4]={0,0,0,0};
uint16_t avrage_sum = 0;
int32_t el_mas[500];
uint16_t Avarage_value =0;

int32_t Counter = 0;

uint8_t errorPHASE=0;
extern uint8_t errorDU;
uint8_t var=0;
uint32_t OurSpeed=0;
uint8_t errorInit=1;
uint8_t ErrorInitFlag=0;
uint32_t Schet=0;
uint8_t AccelerationFlag=0;
uint16_t SaveTime=0;
uint16_t TimeFlag=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void Move(void);
void RecieveData_Usb(void);
void Transmite_EncoderSpeed(void);
uint8_t ErrorInit(void);
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM5_Init(uint32_t CountPer,uint16_t SpdMode);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	int aet = 0;
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
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  MX_TIM5_Init(CountPer,SpdMode);
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (buffer[0]=='u')
			{
				RecieveData_Usb();	
				if (dir=='0'){
					HAL_GPIO_WritePin(Direction_GPIO_Port,Direction_Pin,GPIO_PIN_RESET);
				}	
				if (dir=='1'){
					HAL_GPIO_WritePin(Direction_GPIO_Port,Direction_Pin,GPIO_PIN_SET);
				}					
			}	
			
//There is no God next. i don't know what the following lines of code do 
	TimFlag=1;
	if(BotFlag == 1 && TopFlag==0 && MoveFlag==0 && StopFlag==1 && CycleFlag==1 && Mode=='2' && AccelerationFlag==1){         
			CurCycle++;
			CycleFlag=0;
		}
	if (Mode=='1'){
			Move();
		}
	if (Mode=='2')
		{
			if (AccelerationFlag==0){
				if (TimeFlag==0){
				SaveTime=time;
				spd=maxSpd;
				TimeFlag=1;
				}
				if (maxSpd<=1000)                
					{
						time=3;
					}
				if (maxSpd>1000 && maxSpd<=2000) 
					{
						time=8;
					}
				if (maxSpd>2000)                
					{
						time=15;
					}
				Move();
					if (CurSpd==spd){
						AccelerationFlag=1;
						time=SaveTime;
					}
			}
			else{
		if (CurCycle<cycle)
			{
				if (StopFlag==1 && TopFlag==1)
					{
						spd=minSpd;
						Move();
					}
				else if (StopFlag==1 && BotFlag==1)
					{
						spd=maxSpd;
						Move();
					}
			}
		else
			{
				spd=0;
				if (minSpd<=1000)                
					{
						time=3;
					}
				if (minSpd>1000 && minSpd<=2000) 
					{
						time=8;
					}
				if (minSpd>2000)                
					{
						time=15;
					}
				Move();
					TopFlag=0;
					BotFlag=1;
					MoveFlag=0;
					StopFlag=1;
					Schet=0;
					CycleFlag=1;
					GoodFlag=1;
			}
		}
		}
		if (Mode == '3')
		{
			TopFlag=0;
			BotFlag=1;
			MoveFlag=0;
			StopFlag=1;
			Schet=0;
			CycleFlag=1;
			GoodFlag=1;
			AccelerationFlag=0;
			TimeFlag=0;
			if (spd!=0){
				CheckSpd=spd;
			}
				if (CheckSpd<=1000)                
					{
						time=3;
					}
				if (CheckSpd>1000 && CheckSpd<=2000) 
					{
						time=8;
					}
				if (CheckSpd>2000)                
					{
						time=15;
					}
					spd=0;
				Move();
		}
		if (SpeedFlag<30000)
			{                                    
			if (distance<120000)
				{
					motorSpeed=((float)distance/(TIMER_CLOCK_TICK*CIRCLE_LEN))*60;
					FactSpeed=FactSpeed+motorSpeed;
					SpeedFlag++;

				}
			}
		if (SpeedFlag%3000==0)
			{      
				EncodSpd=FactSpeed/3000;
				FactSpeed=0;
			}
		if (SpeedFlag==30000)
			{	
				SpeedFlag=0;
			}
		if (SpeedFlag==0)
			{	
				Transmite_EncoderSpeed();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35999;
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period =575;
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
static void MX_TIM5_Init(uint32_t CountPer,uint16_t SpdMode)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = CountPer;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = SpdMode;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 409;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 35;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LedG_Pin|LedR_Pin|Direction_Pin|Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Disable_GPIO_Port, Disable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LedG_Pin LedR_Pin PB14 */
  GPIO_InitStruct.Pin = LedG_Pin|LedR_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Disable_Pin */
  GPIO_InitStruct.Pin = Disable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Disable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT1_Pin */
  GPIO_InitStruct.Pin = OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Direction_Pin Step_Pin */
  GPIO_InitStruct.Pin = Direction_Pin|Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/*Configure GPIO pin : OUT2_Pin */
  GPIO_InitStruct.Pin = OUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUT2_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pin : OUT3_Pin */
  GPIO_InitStruct.Pin = OUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUT3_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pin : OUT4_Pin */
  GPIO_InitStruct.Pin = OUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUT4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Для таймера выставлен приоритет 0, для прерывания выставлен приоритет 1

void Move(void)
{
	if(LastSpd1!=spd)
	{
		LastSpd=LastSpd1;
		LastSpd1=spd;
	}
	
	if (spd<LastSpd)
	{
		MovementMode=0;
	}
	
	if (spd>LastSpd)
	{
		MovementMode=1;
	}
	StepTime=(uint32_t)(time/0.1);
	
	if((CurSpd==0)&&(MovementMode==0))
	{
		HAL_TIM_Base_Stop_IT(&htim5);
	}
	else
	{
		if (CurSpd>207)
		{
			TPS=((20*CurSpd)/3);
			SpdMode=1;
		}
		else
		{
			if (CurSpd>=35)
			{
				TPS=((120*CurSpd)/3);
				SpdMode=11;
			}
			else
			{
				TPS=((4200*CurSpd)/3);
				SpdMode=419;
			}
		}
		CountPer=((MICROCONTROLLER_FREQ/TPS)-1);
		
		if (SpdTest!=CurSpd)
		{
			MX_TIM5_Init(CountPer,SpdMode);
			HAL_TIM_Base_Start_IT(&htim5);
			SpdTest=CurSpd;
		}
		HAL_TIM_Base_Start_IT(&htim6);
	}
}

void RecieveData_Usb(void)
{
	HAL_Delay(200);

	var=buffer[1];
	Mode=buffer[2];
	
	if (Mode=='1')
		{
			memcpy((uint8_t *)bufSpd, (uint8_t *)buffer + 4, 4 * sizeof(buffer[4]));
			memcpy((uint8_t *)bufTime, (uint8_t *)buffer + 8, 2 * sizeof(buffer[4]));
			spd=2*atoi(&bufSpd[0]);
			if (spd!=0){
				dir =buffer[3];
			}
			time=atoi(&bufTime[0]);
			
			len = MIN_LEN;
			memset(buffer, 0, strlen(buffer));
		}
	else if (Mode=='2')
		{
			memcpy((uint8_t *)bufCycle, (uint8_t *)buffer + 4, 2 * sizeof(buffer[4]) );
			memcpy((uint8_t *)bufMaxSpd, (uint8_t *)buffer + 6, 4 * sizeof(buffer[4]));
			
			cycle=atoi(&bufCycle[0]);
			maxSpd=2*atoi(&bufMaxSpd[0]);

			memcpy((uint8_t *)bufMinSpd, (uint8_t *)buffer + 10, 4 * sizeof(buffer[4]));	
			memcpy((uint8_t *)bufTime, (uint8_t *)buffer + 14, 2 * sizeof(buffer[4]));		
			
			minSpd=2*atoi(&bufMinSpd[0]);	
			time=atoi(&bufTime[0]);
			
			CurCycle=0;
			AccelerationFlag=0;
			TimeFlag=0;
			
			len = MIN_LEN;
			memset(buffer, 0, strlen(buffer));
		}
		else if (Mode=='3')
		{
			len = MIN_LEN;
			memset(buffer, 0, strlen(buffer));
		}
}

// Transmite USB 

void Transmite_EncoderSpeed(void)
{
	bufferSpd[0]= 'a';
	
	bufferSpd[1]= ((EncodSpd/1000)+0x30);
	bufferSpd[2]=(((EncodSpd/100)-((EncodSpd/1000)*10))+0x30);
	bufferSpd[3]=(((EncodSpd/10)-((EncodSpd/1000)*100)-((EncodSpd%1000)/100)*10)+0x30);
	bufferSpd[4]=((EncodSpd%10)+0x30);
	
	bufferSpd[5]= ((OurSpeed/1000)+0x30);
	bufferSpd[6]=(((OurSpeed/100)-((OurSpeed/1000)*10))+0x30);
	bufferSpd[7]=(((OurSpeed/10)-((OurSpeed/1000)*100)-((OurSpeed%1000)/100)*10)+0x30);
	bufferSpd[8]=((OurSpeed%10)+0x30);
	
	bufferSpd[9]=errorDU+0x30;
	
	bufferSpd[10]=errorPHASE+0x30;
	
	bufferSpd[11]=errorInit+0x30;
	
	CDC_Transmit_FS(bufferSpd,12);
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
