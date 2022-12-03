/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint32_t micros();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define SONAR_TRIGGER_Pin GPIO_PIN_0
#define SONAR_TRIGGER_GPIO_Port GPIOC
#define SONAR_ECHO_Pin GPIO_PIN_1
#define SONAR_ECHO_GPIO_Port GPIOC
#define SONAR_ECHO_EXTI_IRQn EXTI1_IRQn
#define RSSI_Pin GPIO_PIN_2
#define RSSI_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define VOLTAGE_IN_Pin GPIO_PIN_6
#define VOLTAGE_IN_GPIO_Port GPIOA
#define CURRENT_IN_Pin GPIO_PIN_7
#define CURRENT_IN_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_4
#define LD1_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOC
#define EXTERNAL_PAD_Pin GPIO_PIN_1
#define EXTERNAL_PAD_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_2
#define LD3_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RC_PPM_Pin GPIO_PIN_12
#define RC_PPM_GPIO_Port GPIOC
#define RC_PPM_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
