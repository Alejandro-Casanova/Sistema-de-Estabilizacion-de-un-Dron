/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "dwt_stm32_delay.h"
#include "accelerometer.h"
#include "shared_resources.h"
#include "can_bus.h"
#include "config.h"
#include "led_ctrl.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
SPI_HandleTypeDef hspi1;

// Semaphore used for triggering sporadic task from interrupt
SemaphoreHandle_t xSemaphoreInterrupt = NULL; 

// Semaphore used for triggering sporadic task from CAN msg RX routine
SemaphoreHandle_t xSemaphoreCAN = NULL; 

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);

/* Tareas perodicas */
void ReadRotTask(void * argument);
void MotCntrlTask(void * argument);
void AltTask(void * argument);
void VibrationsTask(void * argument);
void SystemActivationTask(void * argument);

/*Prioridades de las Tareas Periodicas (Asignación por DMS)*/
#define PR_TASK1_MOT 4 // Control Motores. Plazo: 150
#define PR_TASK2_ALT 2 // Check Altitud	. Plazo: 300
#define PR_TASK3_HOR 3 // Horizontabilidad. Plazo: 200
#define PR_TASK4_VIB 5 // Detectar Vibraciones. Plazo: 100
#define PR_TASK5_SYS 1 // Capturar Altitud. Plazo: 600

/*Periodos de las tareas*/
#define T_TASK1_MOT 150 // Control Motores
#define T_TASK2_ALT 300 // Check Altitud	
#define T_TASK3_HOR 200 // Horizontabilidad
#define T_TASK4_VIB 350 // Detectar Vibraciones
#define T_TASK5_SYS 600 // Capturar Altitud

#define TRUE 1
#define FALSE 0

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	/* Create Semaphores */
	xSemaphoreCAN = xSemaphoreCreateBinary();
	xSemaphoreInterrupt = xSemaphoreCreateBinary();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
	
	CAN_InitTransmissions();
	

#ifdef CAN_BUS_MODE_TX
	// Initialization for Node 1 (TX)
	ACC_InitAccelerometer();
	SHR_Init(); // Init shared resources

	/* Create tasks */
	xTaskCreate(ReadRotTask, "readRot", configMINIMAL_STACK_SIZE, 0, PR_TASK3_HOR, 0);
	xTaskCreate(AltTask, "altitude", configMINIMAL_STACK_SIZE, 0, PR_TASK2_ALT, 0);
	xTaskCreate(VibrationsTask, "vibrations", configMINIMAL_STACK_SIZE, 0, PR_TASK4_VIB, 0);
	xTaskCreate(SystemActivationTask, "systemAct", configMINIMAL_STACK_SIZE, 0, PR_TASK5_SYS, 0);
#else
	// Initialization for Node 2 (RX)
	xTaskCreate(MotCntrlTask, "motCtrl", configMINIMAL_STACK_SIZE, 0, PR_TASK1_MOT, 0);
#endif
	
  /* Start scheduler */
	vTaskStartScheduler();
  
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
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
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/*Configure GPIO pins : PB0 PB3: Interrupcion botones externos */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|
	                      GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}
	
/* USER CODE BEGIN Header_ReadRotTask */
/**
  * @brief  Function implementing the Tarea1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_ReadRotTask */
void ReadRotTask(void * argument)
{
	const int ANGLE_MARGIN = 2; // Width of dead zone around zero
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t cmdMotRotX, cmdMotRotY; // Will store motor rotation commands for each axis
	double x, y; // Will store drone rotation angles
	
  /* Infinite loop */
  for(;;){
		
		x = ACC_CalculateRotationX();
		y = ACC_CalculateRotationY();
		
		// Check if any tilt correction is required on X axis
		if(x > ANGLE_MARGIN){
			cmdMotRotX = CAN_MOT_CMD_ROT_X_1;
		}else if (x < -ANGLE_MARGIN){
			cmdMotRotX = CAN_MOT_CMD_ROT_X_2;
		}else {
			cmdMotRotX = CAN_MOT_CMD_ROT_X_0;
		}
		
		// Check if any tilt correction is required on Y axis
		if(y > ANGLE_MARGIN){
			cmdMotRotY = CAN_MOT_CMD_ROT_Y_1;
		}else if(y < -ANGLE_MARGIN){
			cmdMotRotY = CAN_MOT_CMD_ROT_Y_2;
		}else{
			cmdMotRotY = CAN_MOT_CMD_ROT_Y_0;
		}
		
		// Signal tilt correction commands via CAN bus
		CAN_sendByte(cmdMotRotX);
		CAN_sendByte(cmdMotRotY);
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T_TASK3_HOR));
  }
}

void MotCntrlTask(void * argument)
{
	// Turn off actuation on all motors
	M1Off(); M2Off(); M3Off(); M4Off();
	
	uint8_t emerFlag = 0; // Emergency activation flag
	uint8_t cmdMotOnOff = 0, cmdMotAlt = 0, cmdMotRotX = 0, cmdMotRotY = 0; // Local variables to store motor commands
	
  /* Infinite loop */
  for(;;)
  {	
		// Emergency Status detected on previous iteration, just skip indefinately 
		// (no other way to exit emergency state, but to reset the device)
		if(emerFlag) continue;
		
		// Read CAN RX msg when semaphore is released, decode it, and store it in the local 
		// motor control command variables
		if(xSemaphoreTake(xSemaphoreCAN, portMAX_DELAY) == pdTRUE){
			switch(CAN_getByteReceived()){
				case CAN_VIBR_EMER_0:
					emerFlag = 0;
					break;
				case CAN_VIBR_EMER_1:
					emerFlag = 1;
					break;
				case CAN_MOT_CMD_ON_OFF_0:
					cmdMotOnOff = 0;
					break;
				case CAN_MOT_CMD_ON_OFF_1:
					cmdMotOnOff = 1;
					break;
				case CAN_MOT_CMD_ALT_0:
					cmdMotAlt = 0;
					break;
				case CAN_MOT_CMD_ALT_1:
					cmdMotAlt = 1;
					break;
				case CAN_MOT_CMD_ROT_X_0:
					cmdMotRotX = 0;
					break;
				case CAN_MOT_CMD_ROT_X_1:
					cmdMotRotX = 1;
					break;
				case CAN_MOT_CMD_ROT_X_2:
					cmdMotRotX = 2;
					break;
				case CAN_MOT_CMD_ROT_Y_0:
					cmdMotRotY = 0;
					break;
				case CAN_MOT_CMD_ROT_Y_1:
					cmdMotRotY = 1;
					break;
				case CAN_MOT_CMD_ROT_Y_2:
					cmdMotRotY = 2;
					break;
				default:
					emerFlag = 1; // Unexpected MSG
			}
		}else{
			continue;
		}
		
		// Emergency status detected for the first time
		if(emerFlag){
			// Stop motor actuation
			M1Off(); M2Off(); M3Off(); M4Off();
			
			// Turns off System State LED
			L1Off();
			
			// Turns on Emergency State LED
			L2On();
			
			continue;
		}
		
		// Operate on motor command values
		if(cmdMotOnOff){ // If system is ON
			
			// Turns on System State LED
			L1On();
			
			if(cmdMotAlt){
				// If height must be corrected (priority over angle)
			  // Activate all motors
				M1On(); M2On(); M3On(); M4On();
				
			}else{
				// If no height correction is needed, check if any angle correction is needed
				switch(cmdMotRotX){ // X axis correction
					case 0: // Turn both motors off
						M1Off(); M2Off();
						break;
					case 1: // Turn forward
						M1On(); M2Off();
						break;
					case 2: // Turn backwards
						M1Off(); M2On();
						break;
					default:
						break;
				}
				
				switch(cmdMotRotY){ // Y axis correction
					case 0: // Turn both motors off
						M3Off(); M4Off();
						break;
					case 1: // Turn forward
						M3Off(); M4On();
						break;
					case 2: // Turn backwards
						M3On(); M4Off();
						break;
					default:
						break;
				}
			}
		}else{ // If system is OFF
			
			// Turn off System Status LED
			L1Off();
			
			// Turn off actuation on all motors
			M1Off(); M2Off(); M3Off(); M4Off();
		}		
  }
}

void AltTask(void * argument)
{ 
	const int HEIGHT_MARGIN = 5; // Prevents flickering when close to target height
	
	TickType_t xLastWakeTime;
	uint8_t motAltCmd = 0;
	int currentHeight = 0, targetHeight = 0;
	
	ADC_ChannelConfTypeDef sConfigN = {0}; // Local Variable
	sConfigN.Channel= ADC_CHANNEL_0; // Channel 0 selected for ADC
	sConfigN.Rank = 1;
	sConfigN.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
		
		HAL_ADC_ConfigChannel(&hadc1, &sConfigN); // Channel 0 is configured for ADC
		HAL_ADC_Start(&hadc1); // Start ADC conversion
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
			// Conversion finished
			// Read from ADC current height
			currentHeight = HAL_ADC_GetValue(&hadc1); // get value from ADC
			
			// Save current height on shared variable
			SHR_SetCurrentHeight(currentHeight);
			
			targetHeight = SHR_GetTargetHeight();
			
			// Check for height loss
			if(currentHeight < targetHeight - HEIGHT_MARGIN)
				motAltCmd = CAN_MOT_CMD_ALT_1; // Height loss detected
			else
				motAltCmd = CAN_MOT_CMD_ALT_0;
			
			// Signal motor command via CAN bus
			CAN_sendByte(motAltCmd);
			
		}
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T_TASK2_ALT));
	}
}

void VibrationsTask(void * argument)
{ 
	const float VIBRATION_MARGIN = 0.2; // Threshold value to consider a vibration has taken place
	const int MAX_VIBRATIONS_ALLOWED = 3; // Max number of consecutive vibrations detected before activating emergency state
	
	uint8_t shakeCounter = 0; // Stores consecutive number of vibrations detected
	double lastX, lastY, lastZ, currentX, currentY, currentZ; // Stores vibration values from the current and previous iteration
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	// Init vibration values for the previous iteration
	ACC_CalculateVibration(&lastX, &lastY, &lastZ);
	
	for(;;){
		
		// Read current vibration values
		ACC_CalculateVibration(&currentX, &currentY, &currentZ);
		
		// Check if vibration threshold has been surpassed
		if(fabs(currentX - lastX) > VIBRATION_MARGIN || fabs(currentY - lastY) > VIBRATION_MARGIN || fabs(currentZ - lastZ) > VIBRATION_MARGIN){
			shakeCounter++;
		}else{
			shakeCounter = 0;
		}
		
		// Check if max number of consecutive vibrations has been surpassed
		if(shakeCounter >= MAX_VIBRATIONS_ALLOWED){
			// Signal Instability Emergency Mode
			CAN_sendByte(CAN_VIBR_EMER_1);
		}
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T_TASK4_VIB));
	}
}

/* Interrupt Handling Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  long yield = pdFALSE;
	
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
	
	xSemaphoreGiveFromISR(xSemaphoreInterrupt, &yield);
  portYIELD_FROM_ISR(yield);
}

void SystemActivationTask(void * argument){
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t systemOn = 0; // System state is stored in a local variable, given that this task is the only one that modifies its value
	
	for(;;){
			
		// Wait for the semaphore to be released by the hardware interrupt callback
		if(xSemaphoreTake(xSemaphoreInterrupt, portMAX_DELAY) == pdTRUE)
		{
			// If system was OFF
			if(!systemOn){
				
				// Assign targetHeight = currentHeight
				SHR_AssignTargetHeight();  
				
				// Makes sure motors are off when system turns on 
				// (this command might be '1' for a brief time if the system was previously deactivated while height correction was taking place)
				CAN_sendByte(CAN_MOT_CMD_ALT_0);
			}
			
			// Toggle system state
			systemOn = !systemOn;
			
			// Signal new system state
			CAN_sendByte(systemOn ? CAN_MOT_CMD_ON_OFF_1 : CAN_MOT_CMD_ON_OFF_0);
		}
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(T_TASK5_SYS));
		
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
