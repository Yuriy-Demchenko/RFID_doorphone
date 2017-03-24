/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId keyTaskRxHandle;
osThreadId keyTaskTxHandle;
osMessageQId rxQueueHandle;
osMessageQId txQueueHandle;
osSemaphoreId rxBinarySemHandle;
osSemaphoreId txBinarySemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t key[12] = {0};

extern uint8_t openEnable; //Global door phone enable
uint8_t keyEnable = 0;
uint8_t keyValidity = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void keyRx(void const * argument);
void keyTx(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	STOP_UART();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of rxBinarySem */
  osSemaphoreDef(rxBinarySem);
  rxBinarySemHandle = osSemaphoreCreate(osSemaphore(rxBinarySem), 1);
	xSemaphoreTake(rxBinarySemHandle, 200 / portTICK_PERIOD_MS);

  /* definition and creation of txBinarySem */
  osSemaphoreDef(txBinarySem);
  txBinarySemHandle = osSemaphoreCreate(osSemaphore(txBinarySem), 1);
	xSemaphoreTake(txBinarySemHandle, 200 / portTICK_PERIOD_MS);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of keyTaskRx */
  osThreadDef(keyTaskRx, keyRx, osPriorityAboveNormal, 0, 128);
  keyTaskRxHandle = osThreadCreate(osThread(keyTaskRx), NULL);

  /* definition and creation of keyTaskTx */
  osThreadDef(keyTaskTx, keyTx, osPriorityAboveNormal, 0, 128);
  keyTaskTxHandle = osThreadCreate(osThread(keyTaskTx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of rxQueue */
  osMessageQDef(rxQueue, 20, uint8_t);
  rxQueueHandle = osMessageCreate(osMessageQ(rxQueue), NULL);

  /* definition and creation of txQueue */
  osMessageQDef(txQueue, 12, uint8_t);
  txQueueHandle = osMessageCreate(osMessageQ(txQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken;
	
	xSemaphoreGiveFromISR(txBinarySemHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  //MX_LWIP_Init(); //TEST - uncomment
	uint8_t i = 1;
	u32_t test=0;
  /* USER CODE BEGIN 5 */
	//xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //TEST - uncomment
		
	while(HAL_GPIO_ReadPin(START_PORT, START_PIN) != GPIO_PIN_SET)
	{}
		tcp_recv_null(&i, NULL, NULL, NULL);
		if(test != 0 )
		{
			i = 2;
		}
		//openEnable = 1;
//	START_UART();
//	START_KEY();
//	keyEnable = 1;
//	LED1_ON();
		//LED2_ON();
		osDelay(300);
	while(HAL_GPIO_ReadPin(START_PORT, START_PIN) == GPIO_PIN_SET)
	  {}
			
  /* Infinite loop */
  for(;;)
  {
    //osDelay(1);
		if(i < 4)
		{
		while(HAL_GPIO_ReadPin(START_PORT, START_PIN) != GPIO_PIN_SET)
	  {}
			
			tcp_recv_null(&i, NULL, NULL, NULL);
		//xSemaphoreGive(rxBinarySemHandle);
//		i++;
//		if (i == 4)
//		{
//			openEnable = 0;
//			STOP_UART();
//			STOP_KEY();
//		}
	}
			osDelay(300);
  }
  /* USER CODE END 5 */ 
}

/* keyRx function */
void keyRx(void const * argument)
{
  /* USER CODE BEGIN keyRx */
	uint8_t j = 0;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(rxBinarySemHandle, portMAX_DELAY); //Receive semaphore from TCP input task (after getting response from server)
		
		if(keyValidity == 1) //Key is valid
		{
		//openEnable = 0;
		LED3_ON();
		keyEnable = 1;
		osDelay(3000);
		LED3_OFF();
	}
		else if(keyValidity == 0) //Key is invalid
		{
			keyEnable = 1;
			for(j = 0; j < 5; j++)
			{
			LED4_ON();
			osDelay(200);
			LED4_OFF();
			osDelay(200);
			}
		}
		if(openEnable == 1)
		{
		LED1_ON();
		START_UART();
		START_KEY();
		//LED4_TOGGLE();
		}
		
		
  }
  /* USER CODE END keyRx */
}

/* keyTx function */
void keyTx(void const * argument)
{
  /* USER CODE BEGIN keyTx */
	uint8_t i = 2, k = 0;
  /* Infinite loop */
  for(;;)
  {
		xSemaphoreTake(txBinarySemHandle, portMAX_DELAY); //Receive semaphore from UART interrupt callback
		//Send key to server
		//tcpSend(key, sizeof(key));
		LED1_OFF();
		STOP_UART();
		STOP_KEY();
		keyEnable = 0;
		osDelay(1000);
		k++;
		if(k == 1 || k == 3 || k == 5)
		{
			i = 2;
		}
		else if(k == 2 || k == 4 || k == 6)
		{
			i = 4;
		}
//		if(k == 3)
//		{
//			i = 3;
//		}else if(k == 4)
//		{
//			i = 2;
//		}
		tcp_recv_null(&i, NULL, NULL, NULL);
		osDelay(2000);
		
		
		if(openEnable == 1 && keyEnable == 0)
		{
		//LED4_TOGGLE();
		keyEnable = 1;
		START_UART();
		START_KEY();
		LED1_ON();
		}
    //osDelay(1);
  }
  /* USER CODE END keyTx */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
