/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
#include "string.h"
#include <math.h>
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;

extern uint32_t spd;
extern uint32_t CurSpd;
extern float StepSpd;
extern uint32_t MovementMode;
extern uint32_t lir_angle;
uint32_t lastLir_angle=0;
uint32_t sumCircle=0;
uint16_t lastSumCircle;
uint32_t distance=0;
extern uint32_t StepTime;
extern uint32_t LastSpd;
extern uint8_t dir;
float motorSpeed;
extern uint8_t MoveFlag;
extern uint8_t Mode;
extern uint32_t Schet;
extern uint8_t StopFlag;
extern uint8_t TopFlag;
extern uint8_t BotFlag;
extern uint8_t TimFlag;
extern uint8_t CycleFlag;
uint8_t GoodFlag=1;
extern uint8_t Tim2Flag;
extern float SensorSpd;

extern uint16_t massive_avrg_values[4];
extern uint16_t avrage_sum;
extern int32_t el_mas[500];
uint16_t counter_test =0;
extern uint16_t Avarage_value;

extern int32_t Counter;
uint16_t values[2];
extern uint32_t Timer_TICK;
extern uint32_t Pulse_Period;
extern uint32_t OurSpeed;
float allSpd;
uint16_t c;

extern uint8_t errorPHASE;
uint8_t errorDU = 0;
extern uint8_t var;
uint8_t  massive_err[2]= {0,0};
uint8_t CurErr;
uint16_t InitDelay2=0;
uint16_t InitDelay70=0;
uint8_t FlagInitDelay70=0;
extern uint8_t errorInit;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
//void EXTI9_5_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

//  /* USER CODE END EXTI9_5_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(OUT1_Pin);
//  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

//  /* USER CODE END EXTI9_5_IRQn 1 */
//}

/**
  * @brief This function handles TIM2 global interrupt.
  */
bool state1 = 0;
bool state2 = 0;
bool state3 = 0;
bool state4 = 0;
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	//ошибка иницализации
	if (Mode == '4')
		{
	if (var=='0')
				{ 
					state1 = HAL_GPIO_ReadPin(GPIOB, OUT1_Pin);
					state4 = HAL_GPIO_ReadPin(GPIOA, OUT4_Pin);
					state2 = HAL_GPIO_ReadPin(GPIOB, OUT2_Pin);
					state3 = HAL_GPIO_ReadPin(GPIOA, OUT3_Pin);
					if(errorInit==1){
					if (state1==1 && state2==1 && state3==1 && state4==1)
					{
						InitDelay2++;
					}else{
						InitDelay2=0;
					}
					if (InitDelay2==3){
						FlagInitDelay70=1;
					}
						if (FlagInitDelay70==1){
							InitDelay70++;
						}
						if (InitDelay70==23){
							FlagInitDelay70=0;
							InitDelay70=0;
							if (state1 != state4)
							{
								errorInit=0;
							}
						}
					}
				}
				if (var=='1')
				{
					state1 = HAL_GPIO_ReadPin(GPIOB, OUT1_Pin);
					state4 = HAL_GPIO_ReadPin(GPIOA, OUT4_Pin);
					state3 = HAL_GPIO_ReadPin(GPIOA, OUT3_Pin);
					if(errorInit==1){
					if (state1==1 && state3==1 && state4==1)
					{
						InitDelay2++;
					}else{
						InitDelay2=0;
					}
					if (InitDelay2==3){
						FlagInitDelay70=1;
					}
						if (FlagInitDelay70==1){
							InitDelay70++;
						}
						if (InitDelay70==23){
							FlagInitDelay70=0;
							InitDelay70=0;
							if (state1 != state4)
							{
								errorInit=0;
							}
						}
					}
				}
			}
	
				if (var=='0')
				{ 
					//ошибка фазы
					if ((!state1 && !state2 && !state3 && state4) || (!state1 && state2 && state3 && state4) || (state1 && state2 && state3 && !state4)|| (state1 && !state2 && !state3 && !state4))
						{
							CurErr = 0;
						}
						else
						{
							CurErr = 1;
						}
						massive_err[0] = massive_err[1];
						massive_err[1] = CurErr;
						if (massive_err[0] == massive_err[1] && massive_err[0] != 0){
							errorPHASE = 1;
						}else{
							errorPHASE = 0;
						}
					//ошибка ДУ
					if(state1 && state2 && state3 && state4)
						{
						 errorDU = 1;
						}
					else
						{
							errorDU = 0;
						}
				}
				
				if (var=='1')
				{
					state1 = HAL_GPIO_ReadPin(GPIOB, OUT1_Pin);
					state3 = HAL_GPIO_ReadPin(GPIOA, OUT3_Pin);
					state4 = HAL_GPIO_ReadPin(GPIOA, OUT4_Pin);
					//ошибка фазы
					if (state1 == !state4)
					{
							//errorPHASE = 0;
							CurErr = 0;
					}
					else
					{
							CurErr = 1;
					}
						massive_err[0] = massive_err[1];
						massive_err[1] = CurErr;
						if (massive_err[0] == massive_err[1] && massive_err[0] != 0)
						{
							errorPHASE = 1;
						}
						else
						{
							errorPHASE = 0;
						}
					//ошибка ДУ
				 if(state1 == 1 && state3 == 1 && state4 == 1)
				 {
						 errorDU = 1;
				 }
				 else
				 {
						 errorDU = 0;
				 }
				}
				
				if(!Pulse_Period)
				{
					SensorSpd = 0;
					allSpd = 0;
					c++;
				}
				else
				{	
					SensorSpd = 60/(Pulse_Period*0.000001*4*200); // 4-2*2,одна двойка хз что,вторая таймертики
					allSpd += SensorSpd;
					c++;
				}
				if (c == 999)
				{
					OurSpeed=allSpd/1000;
					allSpd = 0;
				  c = 0;
				}
			
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
			int cnt = 0;
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	values[Counter] = HAL_GPIO_ReadPin(GPIOA, OUT4_Pin); //Считывание состояния на ножке пина, и запись в массив(после отладки можно заменить на простую переменную)
	avrage_sum += values[Counter];
	
	++Counter;
	
	//разбиваем все на отрезки по 2(учитывая тик таймера работаем с промежутками ТИК_Таймера*2)
	//находим среднее значение
	//обнуляем счетчик записи значений и среднего
	//записываем очередь для проверки massive_avrg_values
	//при попадании на фронт поднимаем флаг
	//плюсуем таймер тик пока не попадем на условие среза(считаем сколько прошло времени)
	//умножаем таймер тик на время работы таймера(по идее должен выйти норм период)
		
	if(Counter == 2)
	{
		if(Tim2Flag == 1)
	{
		Timer_TICK+=1;
	}
		float lol = avrage_sum/2.0;
		Avarage_value =round(lol);
		Counter = 0;
		avrage_sum = 0;

		massive_avrg_values[0] = massive_avrg_values[1];
		massive_avrg_values[1] = massive_avrg_values[2];
		massive_avrg_values[2] = massive_avrg_values[3];
		massive_avrg_values[3] = Avarage_value;
		
		if(massive_avrg_values[3] == 1 && massive_avrg_values[2]==1 && 	massive_avrg_values[1] == 0 && massive_avrg_values[0] == 0) //проверка нужной последовательности для выставления флага наличия фронта
		{
			Tim2Flag = 1; 
			Pulse_Period=Timer_TICK*8;  //умножаем количество произведенных тиков на время работы таймера
			Timer_TICK=0;
		}	
		if ((!massive_avrg_values[3] && !massive_avrg_values[2] && !massive_avrg_values[1] && !massive_avrg_values[0]) || (massive_avrg_values[3] && massive_avrg_values[2] && massive_avrg_values[1] && massive_avrg_values[0]))
		{
			++cnt;
			if (cnt==2375)
			{
				Pulse_Period = 0;
				cnt = 0;
			}		
		}
		else 
		{
			cnt = 0; 
		}	
		if(CurSpd == 0){
			Pulse_Period=0;
		}
		
		if(CurSpd == 300)
		{	
			if (counter_test < 500)
			{		
				el_mas[counter_test] = Avarage_value;							
				counter_test++;							
			}
		}
	}
	/* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
			
			

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	HAL_GPIO_TogglePin(Step_GPIO_Port,Step_Pin);
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */
if (TimFlag==1)
	{
		if ((Mode=='2' && Schet==0) || (Mode=='1') || (Mode=='3'))
			{               //Движение вверх или вниз
				if (MovementMode==0)
					{
						StepSpd=(LastSpd-spd)/StepTime;
					}
				if (MovementMode==1)
					{
						StepSpd=(spd-LastSpd)/StepTime;
					}
				if ((MovementMode==1)&&(CurSpd<spd))                //Разгон
					{				
						if ((Mode=='3' || Mode=='1' || (Mode=='2' && GoodFlag==1)))
							{
								CurSpd+=StepSpd;
								if (CurSpd>=spd)
									{
										CurSpd=spd;
										MoveFlag=1;
										TopFlag=1;
										BotFlag=0;
										GoodFlag=0;
									}
								else
									{
										MoveFlag=0;
									}
							}
					}
				if ((MovementMode==0)&&(CurSpd>spd)&&(CurSpd>0))    //Торможение
					{
						if (Mode=='3' || Mode=='1' || (Mode=='2' && GoodFlag==1))
							{
								CurSpd-=StepSpd;
								if (CurSpd<=spd)
									{
										CurSpd=spd;
										MoveFlag=1;
										BotFlag=1;
										TopFlag=0;
										GoodFlag=0;
									}
								else
									{
										MoveFlag=0;
									}
							}
					}
		}
		TimFlag=0;
}
//////////////////////////////////////
	if (Mode=='2' &&  MoveFlag==1)
		{                       //2-ух секундный перерыв
			StopFlag=0;
			Schet++;
		if (Schet==20)
			{
				Schet=0;
				MoveFlag=0;
				StopFlag=1;
				CycleFlag=1;
				GoodFlag=1;
			}
		}
  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	union 
		{                                        //Получение информации с датчика
     uint8_t RX_Data[3];
    } spi_data;
		HAL_SPI_Receive(&hspi3,spi_data.RX_Data,sizeof(spi_data.RX_Data),100);
		lir_angle =((spi_data.RX_Data[0] << 16)|(spi_data.RX_Data[1] << 8)|(spi_data.RX_Data[2] << 0))>>5;
		lir_angle=lir_angle&0x0001FFFF;		
		sumCircle+=1;
if (lastLir_angle!=lir_angle)
	{
///////////////////////////////////
	if(dir=='0')
		{
			if (lir_angle<lastLir_angle)
				{
					distance=(lastLir_angle-lir_angle);	
				}
			else
				{
					distance=(lastLir_angle+131071-lir_angle);
				}
		}
///////////////////////////////////	
	if (dir=='1')
		{
			if (lir_angle>lastLir_angle)
				{
					distance=(lir_angle-lastLir_angle);	
				}
			else
				{
					distance=(lir_angle+131071-lastLir_angle);
				}
		}
		lastLir_angle=lir_angle;
	}	
///////////////////////////////////
else
	{
	distance=0;
	}
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
