/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "CMSIS_RTOS_V2/cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define sysclk 170000000U//Hz
#define RTOS_PER 1 //ms
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
pmbus_device_t pdevice;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void init_TIM6(void);

// Task function prototypes
void UartTask(void *argument);
void PmbusTask(void *argument);
void CommandProcessingTask(void *argument);
void LogTask(void *argument);
void SupervisorTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Definitions for UartTask */
osMessageQueueId_t logQueue;
osMessageQueueId_t cmdQueue;

osMutexId_t pmbusMutex;
const osMutexAttr_t pMutex_attr ={
		.name = "PMBUSMutex"
};

typedef struct {
    uint8_t commandType;
    uint8_t data[254];
    uint8_t dataLength;
} CommandMessage;

typedef struct {
    uint8_t message;
    uint8_t data[254];
    uint8_t dataLength;
} LogMessage;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Initialize CMSIS-RTOS

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();

  /* USER CODE BEGIN 2 */
  osKernelInitialize();

  // Create tasks
  osThreadAttr_t task_attributes = {
	    .priority = osPriorityNormal,
          .stack_size = 2048
      };
  task_attributes.name = "UART";

  osThreadNew(UartTask, NULL, &task_attributes);

  task_attributes.name = "PMBUS";
  task_attributes.priority = osPriorityHigh,
  osThreadNew(PmbusTask, NULL, &task_attributes);

  task_attributes.name = "CMD";
  task_attributes.priority = osPriorityAboveNormal;
  osThreadNew(CommandProcessingTask, NULL, &task_attributes);

  task_attributes.name = "LOG";
  task_attributes.priority = osPriorityLow;
  osThreadNew(LogTask, NULL, &task_attributes);

  task_attributes.name = "SUP";
  task_attributes.priority = osPriorityHigh1;
  task_attributes.stack_size = 1024;
  osThreadNew(SupervisorTask, NULL, &task_attributes);

  // Create message queues
  logQueue = osMessageQueueNew(20, sizeof(LogMessage), NULL);
  cmdQueue = osMessageQueueNew(10, sizeof(CommandMessage), NULL);

  // Create mutex
  pmbusMutex = osMutexNew(&pMutex_attr);

  I2C_Init();
  EnableI2C();

  osKernelStart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  while(!pdevice.address){
		  if(scanPMBUSwire(&pdevice));
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void init_TIM6(void)
{
    // Enable TIM6 Clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    while (!(RCC->APB1ENR1 & RCC_APB1ENR1_TIM6EN));

    uint32_t uWprescaler = (sysclk / 1000000U) - 1U;  // Assuming sysclk is 170MHz

    TIM6->CR1 &= ~TIM_CR1_CEN;  // Disable TIM6 while configuring
    TIM6->PSC = uWprescaler;    // Set the Timer 6 Prescaler
    TIM6->ARR = (RTOS_PER * 1000U) - 1U;  // Set the auto-reload register for the desired period
    TIM6->CR1 |= TIM_CR1_URS;   // Only counter overflow/underflow generates an update interrupt
    TIM6->DIER |= TIM_DIER_UIE; // Enable update interrupt

    // NVIC configuration for TIM6 interrupt
    NVIC_SetPriority(TIM6_DAC_IRQn, 0);  // Highest priority
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Enable TIM6
    TIM6->CR1 |= TIM_CR1_CEN;
}


void UartTask(void *argument)
{
    for (;;) {
        // Handle UART communication
        // Parse incoming commands and send to cmdQueue
        // Send responses back to PC
        osDelay(1);  // Small delay to prevent tight loop
    }
}

void PmbusTask(void *argument)
{
    for (;;) {
        // Take I2C mutex
        if (osMutexAcquire(pmbusMutex, osWaitForever) == osOK) {
            // Perform PMBUS operations
            // Release I2C mutex
            osMutexRelease(pmbusMutex);
        }
        osDelay(1);  // Small delay to prevent tight loop
    }
}

void CommandProcessingTask(void *argument)
{
    for (;;) {
        // Wait for commands from cmdQueue
        // Process commands
        // Send results to appropriate task
        osDelay(1);  // Small delay to prevent tight loop
    }
}

void LogTask(void *argument)
{
    for (;;) {
        // Wait for log messages from logQueue
        // Write log messages to storage or output
        osDelay(1);  // Small delay to prevent tight loop
    }
}

void SupervisorTask(void *argument)
{
    for (;;) {
        // Monitor task execution times
        // Check system health
        // Handle any system-wide issues
        osDelay(1000);  // Check every second
    }
}
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
