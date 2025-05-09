/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdatomic.h> // Include atomic support for volatile values
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t prev_node;
	uint8_t next_node;
	uint8_t progress; // 0 - 100 %
} Position;

typedef struct {
	float velocity; // reference
	float output;
} VelocityController;

typedef struct {
	float angle; // reference
	float output;
} AngleController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MESSAGE_TYPE_POSITION 0x01
#define MESSAGE_TYPE_VELOCITY 0x02
#define MESSAGE_TYPE_ANGLE 0x03
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		// Transmission completed
	}
}
#define NODE_COUNT (sizeof(nodes)/sizeof(nodes[0]))
#define PATH_LENGTH 8

// List of points
const char nodes[] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R', 'S'};
// Node order in nodes[]: A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S
// Index:                0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18

#define NO_NEIGHBOR 255

const uint8_t neighbors[19][5] = {
    /* A */ { 1, 2, 3, NO_NEIGHBOR, NO_NEIGHBOR },          // B, C, D
    /* B */ { 0, 3, 4, NO_NEIGHBOR, NO_NEIGHBOR },          // A, D, E
    /* C */ { 0, 5, 7, NO_NEIGHBOR, NO_NEIGHBOR },			// A, F, H
    /* D */ { 0, 1, 5, 6, 8 },          					// A, B, F, I, G
    /* E */ { 1, 6, 9, NO_NEIGHBOR, NO_NEIGHBOR },          // B, G, J
    /* F */ { 2, 3, 7, 8, 6 },                    			// C, D, H, I, G
    /* G */ { 5, 3, 4, 9, 8 },                   			// F, D, E, J, I
    /* H */ { 2, 12, 5, 10, NO_NEIGHBOR },                  // C, M, F, K
    /* I */ { 3, 5, 6, 10, 11 },                            // D, F, G, K, L
    /* J */ { 4, 11, 14, 6, NO_NEIGHBOR },        			// E, L, O, G
    /* K */ { 7, 8, 12, 13, 11 },                  			// H, I, M, N, L
    /* L */ { 10, 8, 9, 13, 14 },                   		// K, I, J, N, O
    /* M */ { 7, 10, 16, 15, NO_NEIGHBOR },        			// H, K, Q, P
    /* N */ { 10, 11, 16, 17, 8 },                			// K, L, Q, R, I
    /* O */ { 9, 17, 18, 11, NO_NEIGHBOR },        			// J, R, S, L
    /* P */ { 16, 12, NO_NEIGHBOR, NO_NEIGHBOR, NO_NEIGHBOR }, // Q, M
    /* Q */ { 12, 13, 15, 17, NO_NEIGHBOR },       			// M, N, P, R
    /* R */ { 13, 14, 18, 16, NO_NEIGHBOR },       			// N, O, S, Q
    /* S */ { 14, 17, NO_NEIGHBOR, NO_NEIGHBOR, NO_NEIGHBOR }, // O, R
};

// Path (indexes in node array)
uint8_t current_node = 18; // e.g., start at 'S'
uint8_t prev_node = 18;    // initialize to same as current_node
uint8_t next_node;
uint8_t progress = 0;

// Returns a random neighbor of `node`, excluding `prev_node` if possible.
// If there are no other neighbors, returns prev_node (so you "bounce back").
uint8_t get_random_neighbor(uint8_t node, uint8_t prev_node) {
    uint8_t possible[5];
    uint8_t count = 0;
    // Collect all valid neighbors except prev_node
    for (uint8_t i = 0; i < 5; ++i) {
        uint8_t n = neighbors[node][i];
        if (n != NO_NEIGHBOR && n != prev_node) {
            possible[count++] = n;
        }
    }
    if (count == 0) {
        // Only possible move is to go back to prev_node
        return prev_node;
    }
    // Pick a random neighbor from the possible ones
    return possible[rand() % count];
}


float random_float(float min, float max)
{
	return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		// 100 ms passed
		if (progress >= 100)
		{
			progress = 0;
			// Move to next node
			prev_node = current_node;
			current_node = next_node;
			next_node = get_random_neighbor(current_node, prev_node);
		}

		Position pos;
		pos.prev_node = nodes[current_node];
		pos.next_node = nodes[next_node];
		pos.progress = progress++;
		create_packet_and_send(MESSAGE_TYPE_POSITION, &pos, sizeof(Position));

		VelocityController vc;
		vc.velocity = 50.0f;
		vc.output = random_float(vc.velocity - 10.0f, vc.velocity + 10.0f);
		create_packet_and_send(MESSAGE_TYPE_VELOCITY, &vc, sizeof(VelocityController));

		AngleController ac;
		ac.angle = 30.0f;
		ac.output = random_float(ac.angle - 10.0f, ac.angle + 10.0f);
		create_packet_and_send(MESSAGE_TYPE_ANGLE, &ac, sizeof(AngleController));
	}
}

uint16_t calculate_crc(const uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}
	return crc;
}

void transmit_UART_message(uint8_t *packet, uint16_t packet_length)
{
	HAL_UART_Transmit(&huart2, packet, packet_length, HAL_MAX_DELAY);
}

void create_packet_and_send(uint8_t data_type, const void* data, uint16_t data_length)
{
	uint8_t header[2] = {0xA5, 0x5A};
	uint32_t time_stamp = HAL_GetTick();
	// header(2) + packet_length(2) + data_type(1) + data_length(2) + time_stamp(4) + data(data_length) + CRC(2)
	uint16_t packet_length = 2 + 2 + 1 + 2 + 4 + data_length + 2;
	uint8_t packet[256] = {0};
	uint16_t offset = 0;

	// Header
	memcpy(packet + offset, header, sizeof(header));
	offset += sizeof(header);

	// Packet length
	packet[offset++] = (packet_length >> 8) & 0xFF;
	packet[offset++] = packet_length & 0xFF;

	// Data type
	packet[offset++] = data_type;

	// Data length
	packet[offset++] = (data_length >> 8) & 0xFF;
	packet[offset++] = data_length & 0xFF;

	// Time stamp
	packet[offset++] = (time_stamp >> 24) & 0xFF;
	packet[offset++] = (time_stamp >> 16) & 0xFF;
	packet[offset++] = (time_stamp >> 8) & 0xFF;
	packet[offset++] = time_stamp & 0xFF;

	// Data
	memcpy(packet + offset, data, data_length);
	offset += data_length;

	// CRC
	uint16_t crc = calculate_crc(packet, offset);
	packet[offset++] = crc & 0xFF;
	packet[offset++] = (crc >> 8) & 0xFF;

	// Send to UART
	transmit_UART_message(packet, packet_length);
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
  next_node = get_random_neighbor(current_node, prev_node);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
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
  htim2.Init.Prescaler = 999; // 999 - 100 ms
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1599; // 1599 . 100 ms
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  __HAL_RCC_USART2_CLK_ENABLE(); // Enable USART2 clock
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Enable UART receive interrupt
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // Configure PA2 (USART2_TX)
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA3 (USART2_RX)
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_LE_Pin|SPI1_OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_LE_Pin|SPI2_OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : INFRA_2_Pin */
  GPIO_InitStruct.Pin = INFRA_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INFRA_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INFRA_1_Pin */
  GPIO_InitStruct.Pin = INFRA_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INFRA_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_LE_Pin SPI1_OE_Pin */
  GPIO_InitStruct.Pin = SPI1_LE_Pin|SPI1_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_LE_Pin SPI2_OE_Pin */
  GPIO_InitStruct.Pin = SPI2_LE_Pin|SPI2_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
