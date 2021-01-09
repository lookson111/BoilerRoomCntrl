/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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
uint8_t b_ER1 = 0;
uint8_t b_ER2 = 0;
uint8_t b_rt = 0;
uint8_t b_lt = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

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
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if (HAL_GPIO_ReadPin(Wtr_flow_met_GPIO_Port, Wtr_flow_met_Pin)) {
		wtr_flow_met++;
	}

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	// Обрабатываем нажате кнопок
	b_ER1 = HAL_GPIO_ReadPin(ER11_LINE1_GPIO_Port, ER11_LINE1_Pin);
	b_ER2 = HAL_GPIO_ReadPin(ER11_LINE2_GPIO_Port, ER11_LINE2_Pin);
	if (!b_ER1 & !b_ER2) {
		b_lt = 0;
		b_rt = 0;
	}
	if (b_ER1 & b_ER2) {
		if (b_lt) {
			count_lt--;
			b_lt = 0;
		}
		if (b_rt) {
			count_lt++;
			b_rt = 0;
		}
	}
	TIM2->CR1 ^= TIM_CR1_CEN;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if (!fl_transmit_485) {
		modBusData.available = 1;
		modBusData.buffer = cout_rcvUART;
		cout_rcvUART = 0;
	} else {
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
		fl_transmit_485 = 0;
	}

	TIM4->CR1 ^= TIM_CR1_CEN;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  //если причина прерывания регистор приема не пуст
  if((USART3->SR & USART_SR_RXNE)!=0)
  {
  	modBusData.frame[cout_rcvUART] = USART3->DR;							//прочитать принятый байт
  	cout_rcvUART++;
  	if (cout_rcvUART >= 32) {
  		cout_rcvUART = 0;
  	}
  	fl_transmit_485 = 0;
  	USART3->SR ^= USART_SR_TC;
  	TIM4->ARR = modBusData.T1_5;
  	TIM4->CNT = 0;
  	TIM4->DIER |= TIM_DIER_UIE;
  	TIM4->CR1 |= TIM_CR1_CEN;
  }
  //если причина прерывания  окончание передачи
  if((USART3->SR & USART_SR_TC)!=0)
  {
//  	if (modBusData.buffer < modBusData.bufferSize) {
//  		USART3->DR = (uint8_t*)modBusData.frame[modBusData.buffer];
//  	}
//  	fl_transmit_485 = 1;
//  	RCC->APB1ENR	|= RCC_APB1ENR_USART3EN;				// USART3 Clock ON
//  	USART3->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |		// USART1 ON, TX ON, RX ON
//  			     USART_CR1_RXNEIE;					// RXNE Int ON
//  	NVIC_EnableIRQ (USART3_IRQn);
//    USART3->SR ^= USART_SR_TC;         //очистить флаг
//  	TIM4->ARR = modBusData.T3_5;
//  	TIM4->CNT = 0;
//  	TIM4->DIER |= TIM_DIER_UIE;
//  	TIM4->CR1 |= TIM_CR1_CEN;
    //.....                               //что-то делаем
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  if (EXTI->PR & (1<<12)) // Прерывание от EXTI12?
  {
  	EXTI->PR |= (1<<12); // Сбросить флаг EXTI12.
        // Обработка события EXTI12
  	if (HAL_GPIO_ReadPin(ER11_BUTTON_GPIO_Port, ER11_BUTTON_Pin)) {
  		enterButton = 1;
  	}
  }
	if (EXTI->PR & (1<<11)) //(b_ER1 & !enterButton) {
	{
  	TIM2->ARR = 2000;
  	TIM2->CNT = 0;
  	TIM2->DIER |= TIM_DIER_UIE;
  	TIM2->CR1 |= TIM_CR1_CEN;
  	if (!b_rt) {
  		b_lt = 1;
  	}
	}
	if (EXTI->PR & (1<<10)) {//(b_ER2 & !enterButton) {
  	TIM2->ARR = 2000;
  	TIM2->CNT = 0;
  	TIM2->DIER |= TIM_DIER_UIE;
  	TIM2->CR1 |= TIM_CR1_CEN;
  	if (!b_lt) {
  		b_rt = 1;
  	}
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
