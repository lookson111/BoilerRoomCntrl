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
#include "stm32f1xx_it.h"
#include "main.h"
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
volatile uint8_t b_ER1 = 0;
volatile uint8_t b_ER2 = 0;
volatile uint8_t b_rt = 0;
volatile uint8_t b_lt = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */
extern int count_lt;
extern int count_rt;
extern int8_t enterButton;
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
    while (1) {
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
    while (1) {
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
    while (1) {
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
    while (1) {
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
    HAL_GPIO_EXTI_IRQHandler(Wtr_flow_met_Pin);
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
    // Check if update interrupt flag is set
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;

        // Handle button presses - debounce sampling
        b_ER1 = HAL_GPIO_ReadPin(ER11_LINE1_GPIO_Port, ER11_LINE1_Pin);
        b_ER2 = HAL_GPIO_ReadPin(ER11_LINE2_GPIO_Port, ER11_LINE2_Pin);

        // Check which button is pressed after debounce delay
        if (b_ER1 == GPIO_PIN_SET) {
            // Left button (LINE1) confirmed pressed
            count_lt++;
        } else if (b_ER2 == GPIO_PIN_SET) {
            // Right button (LINE2) confirmed pressed
            count_lt--;
        }

        // Clear latches
        b_lt = 0;
        b_rt = 0;

        // Stop TIM2 (one-shot mode)
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_UIE;
    }
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
    if ((USART3->SR & USART_SR_RXNE) != 0) {
        modBusData.frame[cout_rcvUART] = USART3->DR;
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
    if ((USART3->SR & USART_SR_TC) != 0) {
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
    // EXTI12: Enter button
    if (EXTI->PR & (1 << 12)) {
        EXTI->PR |= (1 << 12); // Clear EXTI12 flag
        if (HAL_GPIO_ReadPin(ER11_BUTTON_GPIO_Port, ER11_BUTTON_Pin)) {
            enterButton = 1;
        }
    }

    // EXTI11: LINE2 (Right) button - start debounce timer
    if (EXTI->PR & (1 << 11)) {
        EXTI->PR |= (1 << 11); // Clear EXTI11 flag
        // Only start timer if not already running
        if (!(TIM2->CR1 & TIM_CR1_CEN)) {
            TIM2->ARR = 2000; // ~55us debounce at 72MHz
            TIM2->CNT = 0;
            TIM2->DIER |= TIM_DIER_UIE;
            TIM2->CR1 |= TIM_CR1_CEN;
        }
    }

    // EXTI10: LINE1 (Left) button - start debounce timer
    if (EXTI->PR & (1 << 10)) {
        EXTI->PR |= (1 << 10); // Clear EXTI10 flag
        // Only start timer if not already running
        if (!(TIM2->CR1 & TIM_CR1_CEN)) {
            TIM2->ARR = 2000; // ~55us debounce at 72MHz
            TIM2->CNT = 0;
            TIM2->DIER |= TIM_DIER_UIE;
            TIM2->CR1 |= TIM_CR1_CEN;
        }
    }

    HAL_GPIO_EXTI_IRQHandler(ER11_LINE1_Pin);
    HAL_GPIO_EXTI_IRQHandler(ER11_LINE2_Pin);
    HAL_GPIO_EXTI_IRQHandler(ER11_BUTTON_Pin);
    /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
