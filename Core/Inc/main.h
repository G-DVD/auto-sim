/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void create_packet_and_send(uint8_t data_type, const void* data, uint16_t data_length);
void transmit_UART_message(uint8_t *packet, uint16_t packet_length);
uint16_t calculate_crc(const uint8_t *data, uint16_t length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INFRA_2_Pin GPIO_PIN_3
#define INFRA_2_GPIO_Port GPIOC
#define INFRA_1_Pin GPIO_PIN_0
#define INFRA_1_GPIO_Port GPIOA
#define SPI1_LE_Pin GPIO_PIN_4
#define SPI1_LE_GPIO_Port GPIOA
#define BUTTON_1_Pin GPIO_PIN_1
#define BUTTON_1_GPIO_Port GPIOB
#define BUTTON_2_Pin GPIO_PIN_2
#define BUTTON_2_GPIO_Port GPIOB
#define SPI2_LE_Pin GPIO_PIN_12
#define SPI2_LE_GPIO_Port GPIOB
#define SPI1_OE_Pin GPIO_PIN_15
#define SPI1_OE_GPIO_Port GPIOA
#define SPI2_OE_Pin GPIO_PIN_9
#define SPI2_OE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
