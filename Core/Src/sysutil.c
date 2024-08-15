#include "sysutil.h"

osThreadId_t supervisorTaskHandle;
osThreadId_t uartTaskHandle;
osThreadId_t cmdProcessingTaskHandle;
osThreadId_t pmbusTaskHandle;
osThreadId_t logTaskHandle;

osMessageQueueId_t cmdQueue;
osMessageQueueId_t logQueue;

osMutexId_t pmbusMutex;

osEventFlagsId_t cmdEventFlags;

SystemState savedState;
osTimerId_t stateTimer;

osThreadId_t taskHandles[MAX_TASKS];
TaskHealth taskHealthStatus[MAX_TASKS];
RecoveryStage currentRecoveryStage;
osTimerId_t recoveryTimer;

uint8_t rxbuffer[UART_BUFFER_SIZE];
uint16_t rx_index = 0;

char syscheck[MAX_LOG_MESSAGE];
char response[MAX_LOG_MESSAGE];
uint8_t log_verbosity = NORMAL;

void (*taskFunctions[MAX_TASKS])(void *) = {
    UartTask,
    PmbusTask,
    CommandProcessingTask,
    LogTask,
    SupervisorTask
};

osThreadAttr_t taskAttributes[MAX_TASKS] = {
    {.name = "UART", .priority = osPriorityNormal, .stack_size = 256 * 4},
    {.name = "PMBUS", .priority = osPriorityNormal, .stack_size = 256 * 4},
    {.name = "CMD", .priority = osPriorityAboveNormal, .stack_size = 256 * 4},
    {.name = "LOG", .priority = osPriorityAboveNormal, .stack_size = 256 * 4},
    {.name = "SUP", .priority = osPriorityHigh, .stack_size = 256 * 4}
};


void initPeripherals()
{

	if(((LPUART1->CR1 & USART_CR1_RE)) == RESET){
	  LPUART1->CR1 |= USART_CR1_RE; //Enable UART RX
	}

	I2C_Init();
	EnableI2C();
	pdevice.address = 0;

}

void setLogVerbosity(LogVerbosity verbose)
{
	log_verbosity = verbose;
	const char* str = (verbose == MUTE) ? "LOW":(verbose == NORMAL) ? "MEDIUM" : "HIGH";
	snprintf(syscheck, sizeof(syscheck), "Log verbosity set to %s", str);
	logMessage(LOG_INFO, syscheck);
}

void logMessage(LogLevel level, const char* message)
{
    // MUTE: only LOG_CRITICAL and LOG_ERROR
    // NORMAL: LOG_CRITICAL, LOG_ERROR, and LOG_WARNING
    // LOUD: (LOG_CRITICAL, LOG_ERROR, LOG_WARNING, LOG_INFO)
    // VERBOSE: all levels (LOG_CRITICAL, LOG_ERROR, LOG_WARNING, LOG_INFO, and LOG_DEBUG)
    if ((log_verbosity == MUTE && level >= LOG_ERROR) ||
	  (log_verbosity == NORMAL && level >= LOG_WARNING) ||
	  (log_verbosity == LOUD && level >= LOG_INFO )||
	  (log_verbosity == VERBOSE)) {

	    LogEntry entry;
	    entry.level = level;
	    strncpy(entry.message, message, MAX_LOG_MESSAGE - 1);
	    entry.message[MAX_LOG_MESSAGE - 1] = '\0';
	    entry.timestamp = getUpTime();

	    osMessageQueuePut(logQueue, &entry, 0, 0);
    }
}

const char* pmbusErrorToString(pmbus_error_t error) {
    switch (error) {
	    case PMBUS_INVALID_DATA:	   return "INVALID_DATA_LENGTH";
	    case PMBUS_PEC_MISMATCH:     return "PEC_MISMATCH";
	    case PMBUS_OK:               return "PMBUS_OK";
	    case PMBUS_NACK:             return "NACK_ERROR";
	    case PMBUS_TIME_OUT:         return "PMBUS_TIMEOUT";
	    case PMBUS_INVALID_ENTRY:    return "INVALID_ENTRY";
	    case PMBUS_WRITE_OVER_FLOW:  return "WRITE_OVERFLOW";
	    case PMBUS_RX_ERROR:	   return "RXNE_ERROR";
	    default:                     return "UNKNOWN_I2C_ERROR";
    }
}

void logPmbusError(LogLevel level, pmbus_error_t error, const char* context) {
    snprintf(response, MAX_LOG_MESSAGE, "%s: %s", context , pmbusErrorToString(error));
    logMessage(level, response);
}


void stateTimerCallback(void *argument)
{
    saveSystemState();
}

void initStateRecovery(void)
{
    osTimerAttr_t timer_attr = {
        .name = "StateTimer"
    };
    stateTimer = osTimerNew((osTimerFunc_t)stateTimerCallback, osTimerPeriodic, NULL, &timer_attr);
    osTimerStart(stateTimer, 60000);  // Save state every 60 seconds
}

void freeUpResources(void)
{
    // Disable unused peripherals
    __HAL_RCC_USART3_CLK_DISABLE();
    __HAL_RCC_I2C2_CLK_DISABLE();

    // Reset non-critical communication interfaces
    HAL_UART_DeInit(&hlpuart1);
    HAL_I2C_DeInit(&hi2c1);

    // Re-initialize critical interfaces
    HAL_UART_Init(&hlpuart1);
    HAL_I2C_Init(&hi2c1);

    // If using dynamic memory, you might want to free some here
    // However, be cautious with dynamic memory in embedded systems
}

void checkTaskHealth(void)
{
    for (int i = 0; i < 5; i++) {
        if (osThreadGetState(taskHandles[i]) == osThreadBlocked) {
            if (taskHealthStatus[i] == TASK_HEALTHY) {
                taskHealthStatus[i] = TASK_SUSPECTED;

            } else if (taskHealthStatus[i] == TASK_SUSPECTED) {
                taskHealthStatus[i] = TASK_FAULTY;
                isolateTask(i);
            }
        } else {
            taskHealthStatus[i] = TASK_HEALTHY;

            snprintf(syscheck,MAX_LOG_MESSAGE,"%s task is healthy", osThreadGetName(taskHandles[i]));
            logMessage(LOG_INFO,syscheck);

        }
    }
}

void isolateTask(int taskIndex)
{
    osThreadSuspend(taskHandles[taskIndex]);
    snprintf(syscheck, MAX_LOG_MESSAGE,"%s task suspended due to suspected fault",osThreadGetName(taskHandles[taskIndex]));
    logMessage(LOG_WARNING, syscheck);

    // Attempt to restart the task
    osThreadTerminate(taskHandles[taskIndex]);
    taskHandles[taskIndex] = osThreadNew(taskFunctions[taskIndex], NULL, &taskAttributes[taskIndex]);

    if (taskHandles[taskIndex] == NULL) {
	snprintf(syscheck, MAX_LOG_MESSAGE,"Failed to restart faulty %s",osThreadGetName(taskHandles[taskIndex]));
	logMessage(LOG_ERROR, syscheck);
    }
}

void startIncrementalRecovery(void)
{
    logMessage(LOG_DEBUG,"System recovery protocol initiated");
    currentRecoveryStage = RECOVERY_STAGE_CORE;
    incrementalRecoveryStep();
}

void incrementalRecoveryStep(void)
{
    switch (currentRecoveryStage) {
        case RECOVERY_STAGE_CORE:
      	logMessage(LOG_DEBUG,"[1] System core reset");
            freeUpResources();
            currentRecoveryStage = RECOVERY_STAGE_COMMUNICATION;
            break;

        case RECOVERY_STAGE_COMMUNICATION:
      	logMessage(LOG_DEBUG,"[2] System communications reset");
            HAL_UART_Init(&hlpuart1);
            HAL_I2C_Init(&hi2c1);
            currentRecoveryStage = RECOVERY_STAGE_TASKS;
            break;

        case RECOVERY_STAGE_TASKS:
      	logMessage(LOG_DEBUG,"[3] System task recovery");
            for (int i = 0; i < 5; i++) {
                if (taskHealthStatus[i] == TASK_FAULTY) {
                    isolateTask(i);
                }
            }
            currentRecoveryStage = RECOVERY_STAGE_COMPLETE;
            break;

        case RECOVERY_STAGE_COMPLETE:
            logMessage(LOG_DEBUG, "[4]System recovery protocol complete");
            break;
    }

    if (currentRecoveryStage != RECOVERY_STAGE_COMPLETE) {
        // Schedule next recovery step
        osTimerStart(recoveryTimer, 1000);  // 1 second delay between steps
    }
}

void initIncrementalRecovery(void)
{
    osTimerAttr_t timer_attr = {.name = "RecoveryTimer"};
    recoveryTimer = osTimerNew((osTimerFunc_t)incrementalRecoveryStep, osTimerOnce, NULL, &timer_attr);
}

void initRecoveryMechanisms(void)
{
    initLogging();
    initStateRecovery();
    initWatchdog();
    initIncrementalRecovery();
}

void initRTOS()
{
    SendOSVersion();
    snprintf(syscheck,sizeof(syscheck),"Firmware Version:%s",FIRMWARE_VER);
    SendResponse(syscheck);
    SendHardwareInfo();
    /* Create mutex */
    pmbusMutex = osMutexNew(NULL);

    /* Create queues */
    cmdQueue = osMessageQueueNew(CMD_QUEUE_SIZE, sizeof(Command_t), NULL);
    logQueue = osMessageQueueNew(LOG_QUEUE_SIZE, sizeof(LogEntry), NULL);

    /* Create event flags */
    cmdEventFlags = osEventFlagsNew(NULL);
    /* Create tasks */
      for(int i = 0; i < MAX_TASKS; i++){
	    taskHandles[i] = osThreadNew(*taskFunctions[i], NULL, &taskAttributes[i]);
	    if(taskHandles[i] == NULL){
		    char bufx[MAX_LOG_MESSAGE];
		    const char* name = taskAttributes->name;
		    snprintf(bufx, MAX_LOG_MESSAGE,"Error: Failed to create %s task", name);
		    logMessage(LOG_ERROR,bufx);
	    }
    }

      logMessage(LOG_INFO,"All tasks succesfully configured and created");
      logMessage(LOG_INFO,"System initialised and operational");
}


// Command Processiing functions
void ProcessPmbusCommand(Command_t *cmd)
{
    if(pdevice.address >= 0xB0 && pdevice.address <= 0xEE){
	    logMessage(LOG_DEBUG,"PMBUS command is being processed");
	    if (cmd->cmd >= 0x00 && cmd->cmd < 0xFA) {
		  commandHandlers[cmd->cmd](cmd);
	    } else {
		    snprintf(syscheck,sizeof(syscheck),"Unknown or unimplemented PMBus command: 0x%02X",cmd->cmd);
		    logMessage(LOG_WARNING,syscheck);
	    }
    }else{
	    snprintf(syscheck, sizeof(syscheck),"Invalid device address 0x%02X", pdevice.address);
	    logMessage(LOG_ERROR,syscheck);
    }
}

void ProcessSystemCommand(Command_t *cmd)
{
    SystemCmd sysCmd = (SystemCmd)cmd->cmd;
    pmbus_error_t res;
    switch(sysCmd) {
        case SYS_GET_OS_VERSION:
            SendOSVersion();
            break;
            //FIXME -- Scan Address is buggy
        case SYS_SCAN_ACTIVE_ADDR:
      	  logMessage(LOG_INFO,"PMBus Address scan started");
      	  res = scanPMBUSwire(&pdevice);
      	  if(res == PMBUS_OK){

      		  snprintf(response,sizeof(response),"PMBus Address found: 0x%02X",((((pdevice.address) << 1)) & 0xFF) >> 1);

      	  }else{

      		  logPmbusError(LOG_ERROR,res,"PMBUS Address Scan");
			  snprintf(response,sizeof(response),"No PMBUS device found");

		  }

      	  logMessage(LOG_DEBUG,response);
      	  SendResponse(response);
		break;

        case SYS_UPDATE_FIRMWARE:
            InitiateFirmwareUpdate();
            break;

        case SYS_RESET:
            PerformSystemReset();
            break;

        case SYS_GET_UPTIME:
            SendSystemUptime();
            break;

        case SYS_GET_MEMORY_STATS:
            SendMemoryStatistics();
            break;

        case SYS_GET_CPU_USAGE:
            SendCPUUsage(); //FIXME --  Fix SendCPUUsage reponse. Doesnt send any data to reciever
            break;

        case SYS_GET_HARDWARE_INFO:
            SendHardwareInfo();
            break;

        default:
      	  logMessage(LOG_ERROR,"Unknown system command");
            break;
    }
}

void SendResponse(const char* response)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
}

void SendOSVersion(void)
{
	snprintf(response, sizeof(response), "\nOS Version: %s\r\n\n", OS_VERSION);
	SendResponse(response);
}

void InitiateFirmwareUpdate(void)
{

}

void PerformSystemReset(void)
{
	logMessage(LOG_WARNING,"Performing system reset...");
	logMessage(LOG_WARNING,"System will reset in 3 seconds.\n");
	HAL_Delay(3000);  // Wait for 3 seconds
	HAL_NVIC_SystemReset();
}

Uptime_t getUpTime(void) {
    uint32_t total_seconds = osKernelGetTickCount() / configTICK_RATE_HZ;

    Uptime_t uptime;
    uptime.days = total_seconds / (24 * 3600);
    uptime.hours = (total_seconds % (24 * 3600)) / 3600;
    uptime.minutes = (total_seconds % 3600) / 60;
    uptime.seconds = total_seconds % 60;

    return uptime;
}

void SendSystemUptime(void) {
    Uptime_t uptime = getUpTime();

    char uptimeStr[64];
    int written = snprintf(uptimeStr, sizeof(uptimeStr),
                           "System Uptime: [%lu:%02lu:%02lu:%02lu]\r\n",
                           uptime.days, uptime.hours, uptime.minutes, uptime.seconds);

    if (written >= 0 && written < sizeof(uptimeStr)) {
        SendResponse(uptimeStr);
    } else {
        logMessage(LOG_ERROR,"Unable to format system uptime.");
    }
}

void SendMemoryStatistics(void){
	uint32_t total = configTOTAL_HEAP_SIZE;
	uint32_t used = total - xPortGetFreeHeapSize();
	uint32_t free = xPortGetFreeHeapSize();

	snprintf(response, sizeof(response),
		 "\nMemory Statistics:\r\n"
		 "Total: %lu bytes\r\n"
		 "Used: %lu bytes\r\n"
		 "Free: %lu bytes\r\n\n",
		 total, used, free);

	SendResponse(response);
}

void SendCPUUsage(void) {
    TaskStatus_t xTaskDetails;
    UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;

    // Get number of tasks
    uxArraySize = uxTaskGetNumberOfTasks();

    snprintf(syscheck,sizeof(syscheck),"Number of tasks: %lu", (uint32_t)uxArraySize);
    logMessage(LOG_DEBUG, syscheck);

    if (uxArraySize == 0) {
        logMessage(LOG_WARNING, "No tasks reported");
        SendResponse("No tasks available.\r\n");
        return;
    }

    // Sending header
    SendResponse("CPU Usage:\r\n");

    // Get total run time
    uxArraySize = uxTaskGetSystemState(NULL, 0, &ulTotalRunTime);

    // Loop through all tasks
    for (x = 0; x < uxArraySize; x++) {
        vTaskGetInfo(NULL, &xTaskDetails, pdTRUE, eInvalid);

        uint32_t ulStatsAsPercentage;
        if (ulTotalRunTime > 0) {
            ulStatsAsPercentage = (xTaskDetails.ulRunTimeCounter * 100UL) / ulTotalRunTime;
        } else {
            ulStatsAsPercentage = 0;
        }

        snprintf(syscheck, sizeof(syscheck), "%s: %lu%%\r\n",
                 xTaskDetails.pcTaskName, ulStatsAsPercentage);

        SendResponse(syscheck);

        snprintf(syscheck, sizeof(syscheck),"Task %s: Runtime %lu, Percentage %lu%%",xTaskDetails.pcTaskName,
      		  xTaskDetails.ulRunTimeCounter, ulStatsAsPercentage);
        logMessage(LOG_DEBUG,syscheck);
    }

    logMessage(LOG_DEBUG, "CPU usage information sent");
}

void SendHardwareInfo(void){
	snprintf(response, sizeof(response),
	    "\nHardware Information:\r\n"
	    "MCU: STM32G474xx\r\n"
	    "Clock Speed: %lu MHz\r\n"
	    "Flash Size: %u KB\r\n"
	    "RAM Size: %u KB\r\n"
	    "Unique Device ID: %08lX%08lX%08lX\r\n\n",
	    HAL_RCC_GetSysClockFreq() / 1000000,
	    *(__IO uint16_t*)(0x1FFF75E0), // Flash size register
	    *(__IO uint16_t*)(0x1FFF75E2), // RAM size register
	    HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
	SendResponse(response);
}

void ProcessConfigCommand(Command_t *cmd) {
    switch(cmd->cmd) {
        case CONF_SET_ADDRESS:
      	if(cmd->length >= 1){
      		uint16_t temp = (cmd->data[4] << 8)|(cmd->data[5]);
      		if(temp > 0xB0 && temp < 0xF0){
      			pdevice.address = temp;
      			snprintf(syscheck, sizeof(syscheck),"PMBUS address set to 0x%02X",((((pdevice.address << 1) | 0) & 0xFF) >> 1));
      			logMessage(LOG_INFO,syscheck);
      		}else{
      			logMessage(LOG_WARNING,"Address entered is invalid.[Valid address: 0xB0 - 0xF0]");
      		}
      	}else{
      		logMessage(LOG_WARNING,"No data sent.");
      	}
            break;

        case CONF_GET_ADDRESS:
      	  if(pdevice.address > 0){
      		  snprintf(response, sizeof(response),"PMBUS address: 0x%02X\nI2C write address: 0x%02X\nI2C read address: 0x%02X",
      				  ((((pdevice.address << 1) | 0) & 0xFF) >> 1),
      				  (((pdevice.address << 1) | 1) & 0xFF),
					  (((pdevice.address << 1) | 0) & 0xFF));
      		  logMessage(LOG_INFO,response);
      	  }else{
      		  logMessage(LOG_WARNING,"No PMBUS address set");
      	  }
            break;

        case CONF_VERBOSE_LOGGING:
            if (cmd->length > 0) {
                switch(cmd->data[0]) {
                    case 0:
                        log_verbosity = MUTE;
                        snprintf(response, sizeof(response), "Log verbosity set to MUTE\r\n");
                        break;
                    case 1:
                        log_verbosity = NORMAL;
                        snprintf(response, sizeof(response), "Log verbosity set to NORMAL\r\n");
                        break;
                    case 2:
                        log_verbosity = LOUD;
                        snprintf(response, sizeof(response), "Log verbosity set to LOUD\r\n");
                        break;
                    case 3:
                        log_verbosity = VERBOSE;
                        snprintf(response, sizeof(response), "Log verbosity set to VERBOSE\r\n");
                        break;
                    default:
                        snprintf(response, sizeof(response), "Invalid verbosity level\r\n");
                        logMessage(LOG_WARNING, response);
                        break;
                }
            } else {
            	const char* current_verbosity =
				(log_verbosity == MUTE) ? "MUTE" :
				(log_verbosity == NORMAL) ? "NORMAL" :
				(log_verbosity == LOUD) ? "LOUD" : "VERBOSE";

            	if (snprintf(syscheck, sizeof(syscheck), "Current log verbosity: %s\r\n", current_verbosity) >= sizeof(syscheck)) {
				logMessage(LOG_WARNING, "Response truncated");
            	}
            	SendResponse(syscheck);
            }
            logMessage(LOG_INFO, response);
            break;

        case CONF_SET_UART_BAUD:
		if (cmd->data[0] < UART_BAUD_9600 || cmd->data[0] > UART_BAUD_921600) {
			snprintf(response, sizeof(response), "Invalid baud rate selection");
			logMessage(LOG_WARNING, response);
		} else {
			uint32_t baudRateValue;
			switch (cmd->data[0]) {
				case UART_BAUD_9600:   baudRateValue = 9600;   break;
				case UART_BAUD_19200:  baudRateValue = 19200;  break;
				case UART_BAUD_38400:  baudRateValue = 38400;  break;
				case UART_BAUD_57600:  baudRateValue = 57600;  break;
				case UART_BAUD_115200: baudRateValue = 115200; break;
				case UART_BAUD_230400: baudRateValue = 230400; break;
				case UART_BAUD_460800: baudRateValue = 460800; break;
				case UART_BAUD_921600: baudRateValue = 921600; break;
				default: baudRateValue = 115200; break;
			}

			hlpuart1.Init.BaudRate = baudRateValue;
			if (HAL_UART_Init(&hlpuart1) == HAL_OK) {
				snprintf(response, sizeof(response), "UART baud rate set to %lu", baudRateValue);
				logMessage(LOG_INFO, response);
			} else {
				snprintf(response, sizeof(response), "Failed to set UART baud rate");
				logMessage(LOG_ERROR, response);
			}
		}
		break;

        case CONF_GET_UART_BAUD:
		snprintf(response, sizeof(response), "Current UART baud rate: %ld\r\n", hlpuart1.Init.BaudRate);
		logMessage(LOG_INFO, response);
            break;
            //FIXME - Fix PMBUS frequency changing
        case CONF_SET_PMBUS_FREQUENCY:
      	if(cmd->length > 0){

      		I2C1->CR1 = 0;  // Disable I2C
      		logMessage(LOG_DEBUG,"I2C Disabled");

      		switch(cmd->data[0]){
      			case STD_MODE:
      				setI2cFreq(STD_MODE);
      				snprintf(response, sizeof(response),"PMBUS set to 100kHz");
      				break;
      			case FAST_MODE_1 :
      				setI2cFreq(FAST_MODE_1);
      				snprintf(response, sizeof(response),"PMBUS set to 400kHz");
      				break;
      			case FAST_PLUS_MODE_1 :
					setI2cFreq(FAST_PLUS_MODE_1);
					snprintf(response, sizeof(response),"PMBUS set to 1000kHz");
					break;
      			case FAST_MODE_2 :
      				setI2cFreq(FAST_MODE_2);
      				snprintf(response, sizeof(response),"PMBUS set to 400kHz+");
      				break;
				case FAST_PLUS_MODE_2 :
					setI2cFreq(FAST_PLUS_MODE_2);
					snprintf(response, sizeof(response),"PMBUS set to 1000kHz+");
					break;
				default:
					logMessage(LOG_WARNING,
					"Invalid PMBUS frequency\n[0] : 100kHz\n[1] : 400kHz\n[2] : 1MHz\n[3] : 400kHz+\n[4] : 1MHz+");
					break;
      		}

      		logMessage(LOG_INFO,response);
      		SendResponse(response);

      		I2C1->CR1 |= (0 << 1);  // enable I2C
      		logMessage(LOG_DEBUG,"I2C Enabled");
      	}
            break;

           //TODO - Make sure this works
        case CONF_GET_PMBUS_FREQUENCY:
      	  char* freq = ( pmbusSpeed == STD_MODE?"100kHz":
      	  pmbusSpeed == FAST_MODE_1?"400kHz": "1000kHz");
      	  switch(pmbusSpeed){
      		  case STD_MODE: freq = "100kHz"; break;
      		  case FAST_MODE_1: freq = "400kHz"; break;
      		  case FAST_PLUS_MODE_1: freq = "1MHz"; break;
      		  case FAST_MODE_2: freq = "400Khz+"; break;
      		  case FAST_PLUS_MODE_2: freq = "1MHz+"; break;
      		  default: freq = "Undefined"; break;
      	  }
      	  snprintf(response, sizeof(response), "Current PMBUS frequency: %s\r\n", freq);
      	  logMessage(LOG_INFO, response);
      	  break;

        case CONF_RESET_TO_DEFAULT:
      	  // Reset UART baud rate to default (usually 115200)
      	  hlpuart1.Init.BaudRate = 115200;
      	  if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
      		  logMessage(LOG_ERROR, "Failed to reset UART baud rate");
      	  }
      	  // Reset PMBUS address to default (e.g., 0xB0)
      	  pdevice.address = 0x00;
      	  // Reset PMBUS frequency to default (e.g., 100kHz)
      	  setI2cFreq(STD_MODE);
      	  // Reset log verbosity to default (e.g., NORMAL)
      	  log_verbosity = NORMAL;

      	  // Reset any other configurable settings to their default values

      	  logMessage(LOG_INFO, "All settings reset to default values");
      	  snprintf(response, sizeof(response), "Reset to default settings\r\n");
      	  logMessage(LOG_INFO, response);
      	  break;

	  default:
		  snprintf(response, sizeof(response), "Unknown configuration command\r\n");
		  logMessage(LOG_WARNING, response);
		  break;
    	    }
    // Send the response back via UART
    SendResponse(response);
    memset(response, 0, sizeof(response));
}
