#ifndef MAIN_H
#define MAIN_H

#include "userTCP.h"
#include "stm32f4xx_hal_uart.h"
#include "tcp_priv.h"

#define LED1_PORT  	GPIOD
#define LED1_PIN 	 	GPIO_PIN_12
#define LED2_PORT  	GPIOD
#define LED2_PIN 	 	GPIO_PIN_13
#define LED3_PORT  	GPIOD
#define LED3_PIN 	 	GPIO_PIN_14
#define LED4_PORT  	GPIOD
#define LED4_PIN 	 	GPIO_PIN_15
#define START_PORT 	GPIOA
#define START_PIN  	GPIO_PIN_0

#define LED1_ON() 	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET)
#define LED1_OFF() 	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED2_ON() 	HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET)
#define LED2_OFF() 	HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED3_ON() 	HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_SET)
#define LED3_OFF() 	HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, GPIO_PIN_RESET)
#define LED4_ON() 	HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_SET)
#define LED4_OFF() 	HAL_GPIO_WritePin(LED4_PORT, LED4_PIN, GPIO_PIN_RESET)
#define LED4_TOGGLE()  HAL_GPIO_TogglePin(LED4_PORT, LED4_PIN)

#define START_UART() __HAL_UART_ENABLE(&huart1)
#define STOP_UART()  __HAL_UART_DISABLE(&huart1)

#define START_KEY() HAL_UART_Receive_DMA(&huart1, key, sizeof(key))
#define STOP_KEY()  HAL_UART_DMAStop(&huart1)

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* MAIN_H */
