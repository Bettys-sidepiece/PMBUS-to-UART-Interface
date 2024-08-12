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
#include "sysutil.h"


/* Private includes ----------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef hlpuart1;
IWDG_HandleTypeDef hiwdg;
pmbus_device_t pdevice;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
void init_TIM6(void);
void HardFault_Handler_C(uint32_t *hardfault_args);


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_LPUART1_UART_Init();
    init_TIM6();
    initWatchdog();
    initPeripherals();

    /* Initialize CMSIS-RTOS2 */
    osKernelInitialize();
    initRTOS();
    /* Start the scheduler */

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for(;;);
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


void UartTask(void *argument) {
    for (;;) {
        if (HAL_UART_Receive(&hlpuart1, &rxbuffer[rx_index], 1, 100) == HAL_OK) {
            if (rxbuffer[rx_index] == '\n') {
                // Ensure there are at least 4 bytes for type and cmd
                if (rx_index >= 4) {
                    Command_t cmd;
                    snprintf(syscheck, sizeof(syscheck), "Raw Message: %c %c %c %c",
                             rxbuffer[0], rxbuffer[1], rxbuffer[2], rxbuffer[3]);
                    logMessage(LOG_DEBUG, syscheck);
                    // Determine the command type
                    cmd.type = rxbuffer[0] - '0'; // Assuming single-digit type

                    snprintf(syscheck, sizeof(syscheck), "Command type: %d", cmd.type);
                    logMessage(LOG_DEBUG, syscheck);

                    // Set command type based on the determined value
                    cmd.type = (cmd.type == 0) ? PMBUS_CMD :
                               (cmd.type == 1) ? SYSTEM_CMD : CONFIG_CMD;

                    int parsed_value = (rxbuffer[1] - '0') * 100 + (rxbuffer[2] - '0') * 10 + (rxbuffer[3] - '0');
                    snprintf(syscheck, sizeof(syscheck), "Parsed value: %d", parsed_value);
                    logMessage(LOG_DEBUG, syscheck);

                    // Parse command
                    cmd.cmd = (rxbuffer[1] - '0') * 100 + (rxbuffer[2] - '0') * 10 + (rxbuffer[3] - '0');
                    snprintf(syscheck, sizeof(syscheck), "Command: %d", cmd.cmd);
                    logMessage(LOG_DEBUG, syscheck);

                    //Parse PMBUS data
                    if(!cmd.type){
				  cmd.pmbus_rw = (rxbuffer[4] - '0');
				  char ch;
				  //parse the pmbus w/r bit
				  if(cmd.pmbus_rw >= 0 && cmd.pmbus_rw <= 1){
					ch = cmd.pmbus_rw && 1? 'W' : 'R';
					snprintf(syscheck, sizeof(syscheck),"PMBUS W/R bit : %c", ch);
					logMessage(LOG_DEBUG,syscheck);
				  }else{
					logMessage(LOG_WARNING,"Invalid PMBUS W/R bit");
				  }

				  // Convert data from ASCII to integers
				  cmd.length = 0;
				  for (int i = 5; i < rx_index; i++) {
				  if (rxbuffer[i] >= '0' && rxbuffer[i] <= '9') {
				  cmd.data[cmd.length++] = rxbuffer[i] - '0';
				  } else {
					 logMessage(LOG_WARNING, "Invalid ASCII data");
					 break;
					 }
				  }
                    }else{
                  	  // Convert data from ASCII to integers
				  cmd.length = 0;
				  for (int i = 4; i < rx_index; i++) {
				  if (rxbuffer[i] >= '0' && rxbuffer[i] <= '9') {
				  cmd.data[cmd.length++] = rxbuffer[i] - '0';
				  } else {
					 logMessage(LOG_WARNING, "Invalid ASCII data");
					 break;
					 }
				  }
                    }

                    snprintf(syscheck, sizeof(syscheck), "Data length: %d", cmd.length);
                    logMessage(LOG_DEBUG, syscheck);

                    // Send command to processing task
                    osMessageQueuePut(cmdQueue, &cmd, 0, 0);

                    // Signal command processing task
                    osEventFlagsSet(cmdEventFlags, 0x01);
                } else {
                    logMessage(LOG_WARNING, "Received incomplete command");
                }

                // Reset buffer
                rx_index = 0;
            } else {
                rx_index++;
                if (rx_index >= UART_BUFFER_SIZE) {
                    // Buffer overflow, reset
                    logMessage(LOG_WARNING, "UART buffer overflow, resetting buffer");
                    rx_index = 0;
                }
            }
        }

        osThreadYield(); // Allow other tasks to run
    }
}



void PmbusTask(void *argument)
{
    for (;;) {
	  /* Acquire PMBUS mutex with timeout */
	  if(osMutexAcquire(pmbusMutex, 1000) == osOK)
	  {
		/* Perform PMBUS operation */
		 // if(device_montior){}
		/* Release mutex */
		osMutexRelease(pmbusMutex);
	  }
	  else
	  {
		/* Mutex acquisition failed, log error */
		logMessage(LOG_ERROR,"PMBUS mutex timeout");
	  }

	  osThreadYield(); // Allow other tasks to run
        }
}

void CommandProcessingTask(void *argument)
{
    Command_t cmd;
    for (;;) {
        // Wait for command flag with a timeout
        osStatus_t status = osEventFlagsWait(cmdEventFlags, 0x01, osFlagsWaitAny, 5000);  // 5 second timeout

        if (status == osErrorTimeout) {
            logMessage(LOG_INFO, "Command processing task timeout");
            continue;
        }

        // Retrieve command from queue
        if (osMessageQueueGet(cmdQueue, &cmd, NULL, 0) == osOK) {
            // Process command
            switch(cmd.type) {
                case PMBUS_CMD:
                    // PMBUS command
                    if (osMutexAcquire(pmbusMutex, 1000) == osOK) {
                        ProcessPmbusCommand(&cmd);
                        osMutexRelease(pmbusMutex);
                    } else {
                        logMessage(LOG_ERROR, "Failed to acquire PMBUS mutex");
                    }
                    break;
                case SYSTEM_CMD:
                    // System control command
                    ProcessSystemCommand(&cmd);
                    break;
                case CONFIG_CMD:
                    // Configuration command
                    ProcessConfigCommand(&cmd);
                    break;
                default:
                    logMessage(LOG_WARNING, "Unknown command type received");
                    break;
            }
        } else {
            logMessage(LOG_ERROR, "Failed to retrieve command from queue");
        }

        // Kick the watchdog
        kickWatchdog();
    }
}

void LogTask(void *argument)
{
    LogEntry log;
    static char buffer[MAX_LOG_MESSAGE + 50];  // Extra space for timestamp and level
    for(;;) {
        if (osMessageQueueGet(logQueue, &log, NULL, osWaitForever) == osOK) {
            // Format log message
            int len = snprintf(buffer, sizeof(buffer), "[%lu:%02lu:%02lu:%02lu] %s: %s\r\n",
                               log.timestamp.days,log.timestamp.hours,
					 log.timestamp.minutes,log.timestamp.seconds,
                               log.level == LOG_INFO ? "INFO" :
                               log.level == LOG_WARNING ? "WARNING" :
                               log.level == LOG_ERROR ? "ERROR" :
					 log.level == LOG_CRITICAL ? "CRITICAL" : "DEBUG",
                               log.message);

            // Transmit log message via UART with timeout
            if (HAL_UART_Transmit(&hlpuart1, (uint8_t*)buffer, len, 100) != HAL_OK) {
                // UART transmission failed
            	logMessage(LOG_CRITICAL,"UART transmission failed");
            }
        }

        // Kick the watchdog
        kickWatchdog();
    }
}

void SupervisorTask(void *argument)
{
    uint32_t last_runtime = osKernelGetTickCount();
    for (;;) {
        // Monitor task execution times
        // Check system health
        // Handle any system-wide issues

        kickWatchdog();
        checkTaskHealth();

        /* Check if other tasks are running */
        if (osThreadGetState(taskHandles[0]) == osThreadBlocked &&  // UART Task
            osThreadGetState(taskHandles[2]) == osThreadBlocked &&  // Command Processing Task
            osThreadGetState(taskHandles[1]) == osThreadBlocked)    // PMBUS Task
        {
            /* All main tasks are blocked, this might indicate a problem */
            logMessage(LOG_CRITICAL, "All main tasks blocked");

            startIncrementalRecovery();
        }

        /* Check for system overrun */
        uint32_t current_time = osKernelGetTickCount();
        if (current_time - last_runtime > 1100) // allowing 10% margin
        {
            logMessage(LOG_WARNING, "System overrun detected");
        }

        last_runtime = current_time;

        if (currentRecoveryStage != RECOVERY_STAGE_COMPLETE) {
            incrementalRecoveryStep();
        }

        logMessage(LOG_INFO, "System heartbeat: Normal operation");
        osDelay(1000); // Check every second
    }
}



void initWatchdog(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload = 4095;  // ~26 seconds at 32 kHz LSI

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
}

void kickWatchdog(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

void HardFault_Handler_C(uint32_t *hardfault_args) {

    logMessage(LOG_CRITICAL,"Fatal System Error (Hard Fault)");
    volatile uint32_t stacked_r0 = hardfault_args[0];
    volatile uint32_t stacked_r1 = hardfault_args[1];
    volatile uint32_t stacked_r2 = hardfault_args[2];
    volatile uint32_t stacked_r3 = hardfault_args[3];
    volatile uint32_t stacked_r12 = hardfault_args[4];
    volatile uint32_t stacked_lr = hardfault_args[5];
    volatile uint32_t stacked_pc = hardfault_args[6];
    volatile uint32_t stacked_psr = hardfault_args[7];

    volatile uint32_t _CFSR = SCB->CFSR;
    volatile uint32_t _HFSR = SCB->HFSR;
    volatile uint32_t _DFSR = SCB->DFSR;
    volatile uint32_t _AFSR = SCB->AFSR;
    volatile uint32_t _MMAR = SCB->MMFAR;
    volatile uint32_t _BFAR = SCB->BFAR;

    __asm("BKPT #0");

    while (1);
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
