/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "..\modbus\SimpleModbusSlave.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
uint32_t wtr_flow_met;
int count_lt;
int count_rt;
int8_t ER11_Button;
int8_t fl_on_off;
uint8_t buff[32];
uint8_t cout_rcvUART;
uint8_t fl_transmit_485;
uint32_t tim4cnt;

ModBusTypeDef modBusData;
uint8_t fl_endreceive485;
uint8_t slaveID;
enum
{
    BR_U321_WtrFlowMet
	 ,BR_U322_WtrFlowMet
//  ,DIN_Rain
//  ,ADC_VAL_LightSens
//  ,ADC_FL1_DPressure
//  ,ADC_FL2_DPressure
//  ,ADC_FL1_DHTTemprt
//  ,ADC_FL2_DHTTemprt
//  ,ADC_FL1_DHTHumidt
//  ,ADC_FL2_DHTHumidt
  ,HOLDING_REGS_SIZE_BR
};
uint16_t holdingRegs[HOLDING_REGS_SIZE_BR];

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DHT22_2_Pin GPIO_PIN_13
#define DHT22_2_GPIO_Port GPIOC
#define DHT22_1_Pin GPIO_PIN_0
#define DHT22_1_GPIO_Port GPIOA
#define TR_1_Pin GPIO_PIN_1
#define TR_1_GPIO_Port GPIOA
#define TR_2_Pin GPIO_PIN_2
#define TR_2_GPIO_Port GPIOA
#define TR_4_Pin GPIO_PIN_3
#define TR_4_GPIO_Port GPIOA
#define TR_3_Pin GPIO_PIN_4
#define TR_3_GPIO_Port GPIOA
#define wtr_hm_in_Pin GPIO_PIN_5
#define wtr_hm_in_GPIO_Port GPIOA
#define Emul_PhotoRez_Pin GPIO_PIN_6
#define Emul_PhotoRez_GPIO_Port GPIOA
#define Wtr_flow_met_Pin GPIO_PIN_7
#define Wtr_flow_met_GPIO_Port GPIOA
#define Wtr_flow_met_EXTI_IRQn EXTI9_5_IRQn
#define PM_2_Pin GPIO_PIN_0
#define PM_2_GPIO_Port GPIOB
#define PM_1_Pin GPIO_PIN_1
#define PM_1_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_2
#define RS485_RE_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define DISP_CS_Pin GPIO_PIN_12
#define DISP_CS_GPIO_Port GPIOB
#define DISP_DC_Pin GPIO_PIN_14
#define DISP_DC_GPIO_Port GPIOB
#define DISP_RES_Pin GPIO_PIN_8
#define DISP_RES_GPIO_Port GPIOA
#define DISP_BLK_Pin GPIO_PIN_9
#define DISP_BLK_GPIO_Port GPIOA
#define ER11_LINE1_Pin GPIO_PIN_10
#define ER11_LINE1_GPIO_Port GPIOA
#define ER11_LINE1_EXTI_IRQn EXTI15_10_IRQn
#define ER11_LINE2_Pin GPIO_PIN_11
#define ER11_LINE2_GPIO_Port GPIOA
#define ER11_LINE2_EXTI_IRQn EXTI15_10_IRQn
#define ER11_BUTTON_Pin GPIO_PIN_12
#define ER11_BUTTON_GPIO_Port GPIOA
#define ER11_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define WtoHS_Pin GPIO_PIN_15
#define WtoHS_GPIO_Port GPIOA
#define CPW_HEAT_HOME_Pin GPIO_PIN_4
#define CPW_HEAT_HOME_GPIO_Port GPIOB
#define WATER_VALVE_Pin GPIO_PIN_5
#define WATER_VALVE_GPIO_Port GPIOB
#define CP_HOT_WATER_Pin GPIO_PIN_8
#define CP_HOT_WATER_GPIO_Port GPIOB
#define Water_Heat_Home_Pin GPIO_PIN_9
#define Water_Heat_Home_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ER11_ON 0x1F // состояние байта при котором будем считать что триггер кнопки сработал

#define ER11_ND 0x00
#define ER11_L1 0x01
#define ER11_L2 0x02
#define ER11_BT 0x03
#define ER11_LT 0x04
#define ER11_RT 0x05
#define ER11_ER 0xFF


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
