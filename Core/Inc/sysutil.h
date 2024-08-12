/*
 * sysutil.h
 *
 *  Created on: Aug 5, 2024
 *      Author: nx024656
 */

#ifndef INC_SYSUTIL_H_
#define INC_SYSUTIL_H_

#include "pmbusdevice.h"
#include "pmbuscmd.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "FreeRTOS/include/FreeRTOS.h"
#include "CMSIS_RTOS_V2/cmsis_os2.h"
#include "FreeRTOS/include/task.h"
#include "smbus.h"

#define sysclk 170000000U//Hz
#define RTOS_PER 1 //ms
/* Definitions */
#define UART_BUFFER_SIZE 256
#define CMD_QUEUE_SIZE 10
#define LOG_QUEUE_SIZE 20
#define MAX_LOG_MESSAGE 256
#define MAX_RESPONSE 1024
#define MAX_TASKS 5

#define MAX_HARDWARE_INFO_LENGTH 256
#define BOOTLOADER_START_ADDRESS 0x08000000  // Adjust this based on memory map

#define OS_VERSION  "1.1.0"
#define FIRMWARE_VER "1.1.0"

//Enums
typedef enum{
	MUTE,
	NORMAL,
	LOUD,
	VERBOSE
}LogVerbosity;

typedef enum{
	PMBUS_CMD,
	SYSTEM_CMD,
	CONFIG_CMD
}CmdType;

typedef enum {
    UART_BAUD_9600 = 0,
    UART_BAUD_19200,
    UART_BAUD_38400,
    UART_BAUD_57600,
    UART_BAUD_115200,
    UART_BAUD_230400,
    UART_BAUD_460800,
    UART_BAUD_921600
} UartBaudRate;

typedef enum {
	LOG_DEBUG=-1,
	LOG_INFO = 0,
	LOG_WARNING,
	LOG_ERROR,
	LOG_CRITICAL
} LogLevel;

typedef enum {
	TASK_HEALTHY,
	TASK_SUSPECTED,
	TASK_FAULTY
} TaskHealth;

typedef enum {
	RECOVERY_STAGE_CORE,
	RECOVERY_STAGE_COMMUNICATION,
	RECOVERY_STAGE_TASKS,
	RECOVERY_STAGE_COMPLETE
} RecoveryStage;

typedef enum {
	SYS_GET_OS_VERSION,
	SYS_SCAN_ACTIVE_ADDR,
	SYS_UPDATE_FIRMWARE,
	SYS_RESET,
	SYS_GET_UPTIME,
	SYS_GET_MEMORY_STATS,
	SYS_GET_CPU_USAGE,
	SYS_GET_HARDWARE_INFO,
} SystemCmd;

typedef enum {
	CONF_SET_ADDRESS,
	CONF_GET_ADDRESS,
	CONF_VERBOSE_LOGGING,
	CONF_SET_UART_BAUD,
	CONF_GET_UART_BAUD,
	CONF_SET_PMBUS_FREQUENCY,
	CONF_GET_PMBUS_FREQUENCY,
	CONF_ENABLE_LOGGING,
	CONF_DISABLE_LOGGING,
	CONF_ENABLE_DEV_MONITOR,
	CONF_DISABLE_DEV_MONITOR,
	CONF_RESET_TO_DEFAULT,
} ConfigCmd;

/* Structures */
typedef struct {
	uint8_t type;
	uint8_t data[UART_BUFFER_SIZE];
	uint16_t cmd;
	uint8_t pmbus_rw;
	uint16_t length;
} Command_t;

typedef struct{
	uint32_t days;
	uint32_t hours;
	uint32_t minutes;
	uint32_t seconds;
}Uptime_t;

typedef struct {
	LogLevel level;
	char message[MAX_LOG_MESSAGE];
	Uptime_t timestamp;
} LogEntry;

typedef struct {
	uint32_t lastValidState;
	float lastValidVoltage;
	float lastValidCurrent;
    // Add other important state variables
} SystemState;

typedef void (*CommandHandler)(Command_t *cmd);

extern uint8_t log_verbosity;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern pmbus_device_t pdevice;

extern osThreadId_t supervisorTaskHandle;
extern osThreadId_t uartTaskHandle;
extern osThreadId_t cmdProcessingTaskHandle;
extern osThreadId_t pmbusTaskHandle;
extern osThreadId_t logTaskHandle;

extern osMessageQueueId_t cmdQueue;
extern osMessageQueueId_t logQueue;

extern osMutexId_t pmbusMutex;

extern osEventFlagsId_t cmdEventFlags;

extern SystemState savedState;
extern osTimerId_t stateTimer;

extern osThreadId_t taskHandles[MAX_TASKS];
extern TaskHealth taskHealthStatus[MAX_TASKS];
extern RecoveryStage currentRecoveryStage;
extern osTimerId_t recoveryTimer;

extern uint8_t rxbuffer[UART_BUFFER_SIZE];
extern uint16_t rx_index;
extern char syscheck[MAX_LOG_MESSAGE];
extern const CommandHandler commandHandlers[];

extern void (*taskFunctions[MAX_TASKS])(void *);

void initPeripherals();

//WDG function prototypes
void initWatchdog(void);
void kickWatchdog(void);

/* Task function prototypes */
void SupervisorTask(void *argument);
void UartTask(void *argument);
void CommandProcessingTask(void *argument);
void PmbusTask(void *argument);
void LogTask(void *argument);

//Error Logging Fuction prototypes
void initLogging(void);
void logMessage(LogLevel level, const char* message);

//State Recovery Function prototyes
void saveSystemState(void);
void restoreSystemState(void);
void stateTimerCallback(void *argument);
void initStateRecovery(void);

//resource allocation
void freeUpResources(void);

//fault Isolation function prototype
void checkTaskHealth(void);
void isolateTask(int taskIndex);

//Incremental Recovery function prototype
void startIncrementalRecovery(void);
void incrementalRecoveryStep(void);
void initIncrementalRecovery(void);

void initRecoveryMechanisms(void);

void initRTOS();

Uptime_t getUpTime(void);

void ProcessPmbusCommand(Command_t *cmd);
void ProcessConfigCommand(Command_t *cmd);
void ProcessSystemCommand(Command_t *cmd);

void SendResponse(const char* response);
void InitiateFirmwareUpdate(void);
void SendOSVersion(void);
void SendSystemStatus(void);
void PerformSystemReset(void);
void SendSystemUptime(void);
void SendMemoryStatistics(void);
void SendCPUUsage(void);
void SendHardwareInfo(void);

//PMBUS Command Handler Fuction prototype
void HandlePage(Command_t *cmd);
void HandleOperation(Command_t *cmd);
void HandleOnOffConfig(Command_t *cmd);
void HandleClearFault(Command_t *cmd);
void HandlePhase(Command_t *cmd);
void HandleWriteProtect(Command_t *cmd);
void HandleStoreDefaultAll(Command_t *cmd);
void HandleRestoreDefaultAll(Command_t *cmd);
void HandleCapability(Command_t *cmd);
void HandleSmbAlertMask(Command_t *cmd);
void HandleVoutMode(Command_t *cmd);
void HandleVoutCommand(Command_t *cmd);
void HandleVoutMax(Command_t *cmd);
void HandleVoutMarginHigh(Command_t *cmd);
void HandleVoutMarginLow(Command_t *cmd);
void HandleVoutTransitionRate(Command_t *cmd);
void HandleVoutDroop(Command_t *cmd);
void HandleVoutScaleLoop(Command_t *cmd);
void HandleVoutScaleMonitor(Command_t *cmd);
void HandleVoutMin(Command_t *cmd);
void HandleFrequencySwitch(Command_t *cmd);
void HandleVinOn(Command_t *cmd);
void HandleIoutCalGain(Command_t *cmd);
void HandleIoutCalOffset(Command_t *cmd);
void HandleVoutOvFaultLimit(Command_t *cmd);
void HandleVoutOvFaultResponse(Command_t *cmd);
void HandleVoutUvFaultLimit(Command_t *cmd);
void HandleVoutUvFaultResponse(Command_t *cmd);
void HandleIoutOcFaultLimit(Command_t *cmd);
void HandleIoutOcFaultResponse(Command_t *cmd);
void HandleIoutOcWarnLimit(Command_t *cmd);
void HandleOtFaultLimit(Command_t *cmd);
void HandleOtFaultResponse(Command_t *cmd);
void HandleOtWarnLimit(Command_t *cmd);
void HandleVinOvFaultLimit(Command_t *cmd);
void HandleVinOvFaultResponse(Command_t *cmd);
void HandleVinUvFaultLimit(Command_t *cmd);
void HandleVinUvFaultResponse(Command_t *cmd);
void HandleIinOcFaultLimit(Command_t *cmd);
void HandleIinOcFaultResponse(Command_t *cmd);
void HandleIinOcWarnLimit(Command_t *cmd);
void HandleTonDelay(Command_t *cmd);
void HandlePinOpWarnLimit(Command_t *cmd);
void HandleStatusByte(Command_t *cmd);
void HandleStatusWord(Command_t *cmd);
void HandleStatusVout(Command_t *cmd);
void HandleStatusIout(Command_t *cmd);
void HandleStatusInput(Command_t *cmd);
void HandleStatusTemperature(Command_t *cmd);
void HandleStatusCml(Command_t *cmd);
void HandleStatusMfrSpecific(Command_t *cmd);
void HandleReadVin(Command_t *cmd);
void HandleReadIin(Command_t *cmd);
void HandleReadVout(Command_t *cmd);
void HandleReadIout(Command_t *cmd);
void HandleReadTemperature1(Command_t *cmd);
void HandleReadPout(Command_t *cmd);
void HandleReadPin(Command_t *cmd);
void HandlePmbusRevision(Command_t *cmd);
void HandleMfrId(Command_t *cmd);
void HandleMfrModel(Command_t *cmd);
void HandleMfrRevision(Command_t *cmd);
void HandleMfrDate(Command_t *cmd);
void HandleMfrSerial(Command_t *cmd);
void HandleIcDeviceId(Command_t *cmd);
void HandleIcDeviceRev(Command_t *cmd);
void HandleUserData00(Command_t *cmd);
void HandleUserData01(Command_t *cmd);
void HandleUserData02(Command_t *cmd);
void HandleUserData03(Command_t *cmd);
void HandleUserData04(Command_t *cmd);
void HandleUserData05(Command_t *cmd);
void HandleUserData06(Command_t *cmd);
void HandleUserData07(Command_t *cmd);
void HandleUserData08(Command_t *cmd);
void HandleUserData09(Command_t *cmd);
void HandleUserData10(Command_t *cmd);
void HandleUserData11(Command_t *cmd);
void HandleUserData12(Command_t *cmd);
void HandleMfrSpecific00(Command_t *cmd);
void HandleMfrSpecific03(Command_t *cmd);
void HandleMfrSpecific04(Command_t *cmd);
void HandleMfrSpecific05(Command_t *cmd);
void HandleMfrSpecific06(Command_t *cmd);
void HandleMfrSpecific07(Command_t *cmd);
void HandleMfrSpecific08(Command_t *cmd);
void HandleMfrSpecific09(Command_t *cmd);
void HandleMfrSpecific10(Command_t *cmd);
void HandleMfrSpecific11(Command_t *cmd);
void HandleMfrSpecific12(Command_t *cmd);
void HandleMfrSpecific13(Command_t *cmd);
void HandleMfrSpecific14(Command_t *cmd);
void HandleMfrSpecific15(Command_t *cmd);
void HandleMfrSpecific20(Command_t *cmd);
void HandleMfrSpecific32(Command_t *cmd);
void HandleMfrSpecific42(Command_t *cmd);

#endif /* INC_SYSUTIL_H_ */
