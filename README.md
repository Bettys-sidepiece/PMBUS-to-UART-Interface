# PMBus Command Interface Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Command Structure](#command-structure)
3. [PMBus Commands](#pmbus-commands)
4. [System Commands](#system-commands)
5. [Configuration Commands](#configuration-commands)
6. [Error Handling](#error-handling)

## Introduction

This document provides detailed information on the PMBus Command Interface, including PMBus commands, system commands, and configuration commands. The interface is designed for STM32G4 microcontrollers using FreeRTOS.

## Command Structure

All commands follow this general structure:

```
[TYPE][CMD][DATA]\n
```

- `[TYPE]`: Single digit representing the command type
  - `0`: PMBus command
  - `1`: System command
  - `2`: Configuration command
- `[CMD]`: Three-digit command code
- `[DATA]`: Optional data (format depends on the command)
- `\n`: Newline character (required to end the command)

## Configuration Commands

- `CONF_SET_ADDRESS` (000): Set PMBus address
- `CONF_GET_ADDRESS` (001): Get current PMBus address
- `CONF_VERBOSE_LOGGING` (002): Set log verbosity
- `CONF_SET_UART_BAUD` (003): Set UART baud rate
- `CONF_GET_UART_BAUD` (004): Get current UART baud rate
- `CONF_SET_PMBUS_FREQUENCY` (005): Set PMBus frequency
- `CONF_GET_PMBUS_FREQUENCY` (006): Get current PMBus frequency
- `CONF_ENABLE_LOGGING` (007): Enable logging
- `CONF_DISABLE_LOGGING` (008): Disable logging
- `CONF_RESET_TO_DEFAULT` (009): Reset to default settings

## Log Verbosity Levels

- MUTE (0): Only critical and error messages
- NORMAL (1): Critical, error, and warning messages
- LOUD (2): All messages except debug
- VERBOSE (3): All messages including debug

## Usage

Send commands via UART in the following format:
`<command_type>` `<command_code>` `<data>` `<\n>`

Where:
- `<command_type>` is 0 for PMBus, 1 for System, 2 for Config
- `<command_code>` is a three-digit number
- `<data>` is optional and command-specific
- `\n` is the newline character

Example: 20021\n

This sets the log verbosity to NORMAL.

## PMBus Commands

PMBus commands interact directly with the PMBus device. They include an additional bit for read/write operations.

Format: `[0][CMD][R/W][DATA][\n]`


- `[R/W]`: `0` for read, `1` for write
- `[DATA]`: Hexadecimal data for write operations (omitted for read operations)

### Available PMBus Commands

| Command             | Code | Description                        |
| ------------------- | ---- | ---------------------------------- |
| PAGE                | 000  | Set/Get the current page           |
| OPERATION           | 001  | Set/Get the operation state        |
| ON_OFF_CONFIG       | 002  | Set/Get the on/off configuration   |
| CLEAR_FAULT         | 003  | Clear all faults                   |
| PHASE               | 004  | Set/Get the current phase          |
| WRITE_PROTECT       | 010  | Set/Get write protection status    |
| STORE_DEFAULT_ALL   | 011  | Store current settings as defaults |
| RESTORE_DEFAULT_ALL | 012  | Restore default settings           |
| CAPABILITY          | 025  | Get device capabilities            |
| VOUT_MODE           | 031  | Get the VOUT mode                  |
| VOUT_COMMAND        | 033  | Set/Get the output voltage         |

(Note: This list is not exhaustive. Refer to the PMBus specification for a complete list of commands.)

Example usage:

- Read VOUT_COMMAND: `00330\n`
- Write VOUT_COMMAND (set to 1.0V): `003311000\n`

## System Commands

System commands control overall system behavior.

Format: `[1][CMD][DATA][\n]`

### Available System Commands

| Command               | Code | Description                     |
| --------------------- | ---- | ------------------------------- |
| SYS_GET_OS_VERSION    | 000  | Get the OS version              |
| SYS_SCAN_ACTIVE_ADDR  | 001  | Scan for active PMBus addresses |
| SYS_UPDATE_FIRMWARE   | 002  | Initiate firmware update        |
| SYS_RESET             | 003  | Perform system reset            |
| SYS_GET_UPTIME        | 004  | Get system uptime               |
| SYS_GET_MEMORY_STATS  | 005  | Get memory usage statistics     |
| SYS_GET_CPU_USAGE     | 006  | Get CPU usage information       |
| SYS_GET_HARDWARE_INFO | 007  | Get hardware information        |

Example usage:

- Get OS Version: `1000\n`
- Scan for active PMBus addresses: `1001\n`

## Configuration Commands

Configuration commands modify the behavior of the PMBus interface itself.

Format: `[2][CMD][DATA][\n]`

### Available Configuration Commands

| Command                  | Code | Description                         | Data Format   |
| ------------------------ | ---- | ----------------------------------- | ------------- |
| CONF_SET_ADDRESS         | 000  | Set PMBus address                   | 2 bytes (hex) |
| CONF_GET_ADDRESS         | 001  | Get current PMBus address           | None          |
| CONF_VERBOSE_LOGGING     | 002  | Set log verbosity                   | 1 byte (0-3)  |
| CONF_SET_UART_BAUD       | 003  | Set UART baud rate                  | 1 byte (enum) |
| CONF_GET_UART_BAUD       | 004  | Get current UART baud rate          | None          |
| CONF_SET_PMBUS_FREQUENCY | 005  | Set PMBus clock frequency           | 1 byte (enum) |
| CONF_GET_PMBUS_FREQUENCY | 006  | Get current PMBus clock frequency   | None          |
| CONF_RESET_TO_DEFAULT    | 007  | Reset all configurations to default | None          |

#### CONF_VERBOSE_LOGGING Data Values

- 0: MUTE (critical errors only)
- 1: NORMAL (errors and warnings)
- 2: LOUD (errors, warnings, and info)
- 3: VERBOSE (all messages including debug)

#### CONF_SET_UART_BAUD Data Values

- 0: 9600 baud
- 1: 19200 baud
- 2: 38400 baud
- 3: 57600 baud
- 4: 115200 baud
- 5: 230400 baud
- 6: 460800 baud
- 7: 921600 baud

#### CONF_SET_PMBUS_FREQUENCY Data Values

- 0: 100 kHz (Standard mode)
- 1: 400 kHz (Fast mode)
- 2: 1 MHz (Fast mode plus)

Example usage:

- Set PMBus address to 0xB2: `2000178\n`
- Set log verbosity to NORMAL: `20021\n`
- Set UART baud rate to 115200: `20034\n`

## Error Handling

If a command is invalid or cannot be executed, an error message will be logged. The system will attempt to continue operation. Check the system logs for detailed error information.

Common errors include:

- Invalid command format
- Unknown command
- Invalid data for a command
- PMBus communication errors
- System resource limitations

For critical errors, the system may initiate a recovery procedure or reset.

# PMBus Commands Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Command Format](#command-format)
3. [PMBus Commands](#pmbus-commands)
   - [Basic Operation Commands](#basic-operation-commands)
   - [Output Voltage Commands](#output-voltage-commands)
   - [Input Commands](#input-commands)
   - [Fault Commands](#fault-commands)
   - [Temperature Commands](#temperature-commands)
   - [Status Commands](#status-commands)
   - [Reading Commands](#reading-commands)
   - [Manufacturer Specific Commands](#manufacturer-specific-commands)
   - [User Data Commands](#user-data-commands)

## Introduction

This document provides a comprehensive list of PMBus commands implemented in this system. Each command is listed with its name and corresponding decimal command code.

## Command Format

PMBus commands are sent in the following format:

```
[0][CMD][R/W][DATA][\n]
```

- `[0]`: Indicates a PMBus command
- `[CMD]`: Three-digit decimal command code
- `[R/W]`: `0` for read, `1` for write
- `[DATA]`: Decimal data for write operations (omitted for read operations)
- `[\n]`: Newline character to end the command

Example:

- To read VOUT_COMMAND (command code 33): `00330\n`
- To write VOUT_COMMAND: `003311000\n`

## PMBus Commands

### Basic Operation Commands

| Command             | Code | Description                        |
| ------------------- | ---- | ---------------------------------- |
| PAGE                | 000  | Set/Get the current page           |
| OPERATION           | 001  | Set/Get the operation state        |
| ON_OFF_CONFIG       | 002  | Set/Get the on/off configuration   |
| CLEAR_FAULT         | 003  | Clear all faults                   |
| PHASE               | 004  | Set/Get the current phase          |
| WRITE_PROTECT       | 010  | Set/Get write protection status    |
| STORE_DEFAULT_ALL   | 011  | Store current settings as defaults |
| RESTORE_DEFAULT_ALL | 012  | Restore default settings           |
| CAPABILITY          | 019  | Get device capabilities            |
| SMBALERT_MASK       | 027  | Set/Get the SMBAlert mask          |

### Output Voltage Commands

| Command              | Code | Description                                          |
| -------------------- | ---- | ---------------------------------------------------- |
| VOUT_MODE            | 032  | Get the supported output voltage format and exponent |
| VOUT_COMMAND         | 033  | Set/Get the output voltage                           |
| VOUT_MAX             | 036  | Set/Get the maximum output voltage                   |
| VOUT_MARGIN_HIGH     | 037  | Set/Get the high margin output voltage               |
| VOUT_MARGIN_LOW      | 038  | Set/Get the low margin output voltage                |
| VOUT_TRANSITION_RATE | 039  | Set/Get the output voltage transition rate           |
| VOUT_DROOP           | 040  | Set/Get the output voltage droop                     |
| VOUT_SCALE_LOOP      | 041  | Set/Get the output voltage scale loop                |
| VOUT_SCALE_MONITOR   | 042  | Set/Get the output voltage scale monitor             |
| VOUT_MIN             | 043  | Set/Get the minimum output voltage                   |

### Input Commands

| Command          | Code | Description                                   |
| ---------------- | ---- | --------------------------------------------- |
| FREQUENCY_SWITCH | 051  | Set/Get the switching frequency               |
| VIN_ON           | 053  | Set/Get the input voltage turn-on threshold   |
| IOUT_CAL_GAIN    | 056  | Set/Get the output current calibration gain   |
| IOUT_CAL_OFFSET  | 057  | Set/Get the output current calibration offset |

### Fault Commands

| Command                | Code | Description                                    |
| ---------------------- | ---- | ---------------------------------------------- |
| VOUT_OV_FAULT_LIMIT    | 064  | Set/Get the output overvoltage fault limit     |
| VOUT_OV_FAULT_RESPONSE | 065  | Set/Get the output overvoltage fault response  |
| VOUT_UV_FAULT_LIMIT    | 068  | Set/Get the output undervoltage fault limit    |
| VOUT_UV_FAULT_RESPONSE | 069  | Set/Get the output undervoltage fault response |
| IOUT_OC_FAULT_LIMIT    | 070  | Set/Get the output overcurrent fault limit     |
| IOUT_OC_FAULT_RESPONSE | 071  | Set/Get the output overcurrent fault response  |
| IOUT_OC_WARN_LIMIT     | 074  | Set/Get the output overcurrent warning limit   |
| VIN_OV_FAULT_LIMIT     | 085  | Set/Get the input overvoltage fault limit      |
| VIN_OV_FAULT_RESPONSE  | 086  | Set/Get the input overvoltage fault response   |
| VIN_UV_FAULT_LIMIT     | 089  | Set/Get the input undervoltage fault limit     |
| VIN_UV_FAULT_RESPONSE  | 090  | Set/Get the input undervoltage fault response  |
| IIN_OC_FAULT_LIMIT     | 091  | Set/Get the input overcurrent fault limit      |
| IIN_OC_FAULT_RESPONSE  | 092  | Set/Get the input overcurrent fault response   |
| IIN_OC_WARN_LIMIT      | 093  | Set/Get the input overcurrent warning limit    |

### Temperature Commands

| Command           | Code | Description                                |
| ----------------- | ---- | ------------------------------------------ |
| OT_FAULT_LIMIT    | 079  | Set/Get the overtemperature fault limit    |
| OT_FAULT_RESPONSE | 080  | Set/Get the overtemperature fault response |
| OT_WARN_LIMIT     | 081  | Set/Get the overtemperature warning limit  |

### Status Commands

| Command             | Code | Description                                     |
| ------------------- | ---- | ----------------------------------------------- |
| STATUS_BYTE         | 120  | Read the status byte                            |
| STATUS_WORD         | 121  | Read the status word                            |
| STATUS_VOUT         | 122  | Read the output voltage status                  |
| STATUS_IOUT         | 123  | Read the output current status                  |
| STATUS_INPUT        | 124  | Read the input status                           |
| STATUS_TEMPERATURE  | 125  | Read the temperature status                     |
| STATUS_CML          | 126  | Read the communication, memory and logic status |
| STATUS_MFR_SPECIFIC | 128  | Read manufacturer specific status information   |

### Reading Commands

| Command            | Code | Description             |
| ------------------ | ---- | ----------------------- |
| READ_VIN           | 136  | Read the input voltage  |
| READ_IIN           | 137  | Read the input current  |
| READ_VOUT          | 139  | Read the output voltage |
| READ_IOUT          | 140  | Read the output current |
| READ_TEMPERATURE_1 | 141  | Read temperature 1      |
| READ_POUT          | 148  | Read the output power   |
| READ_PIN           | 149  | Read the input power    |

### Manufacturer Specific Commands

| Command                            | Code    | Description                             |
| ---------------------------------- | ------- | --------------------------------------- |
| PMBUS_REVISION                     | 152     | Read the supported PMBus version        |
| MFR_ID                             | 153     | Read the manufacturer ID                |
| MFR_MODEL                          | 154     | Read the manufacturer model             |
| MFR_REVISION                       | 155     | Read the manufacturer revision          |
| MFR_DATE                           | 156     | Read the manufacture date               |
| MFR_SERIAL                         | 157     | Read the manufacturer serial number     |
| IC_DEVICE_ID                       | 173     | Read the IC device ID                   |
| IC_DEVICE_REV                      | 174     | Read the IC device revision             |
| MFR_SPECIFIC_00                    | 208     | Manufacturer specific command 00        |
| MFR_SPECIFIC_03 to MFR_SPECIFIC_15 | 211-223 | Manufacturer specific commands 03 to 15 |
| MFR_SPECIFIC_20                    | 228     | Manufacturer specific command 20        |
| MFR_SPECIFIC_32                    | 240     | Manufacturer specific command 32        |
| MFR_SPECIFIC_42                    | 250     | Manufacturer specific command 42        |

### User Data Commands

| Command                      | Code    | Description                   |
| ---------------------------- | ------- | ----------------------------- |
| USER_DATA_00 to USER_DATA_12 | 176-188 | Read/Write user data 00 to 12 |

Note: This list includes all the PMBus commands implemented in this system. Some commands may have specific data formats or restrictions. Always refer to the PMBus specification and your device's datasheet for detailed information on how to use each command.

graph TD
%% System Initialization
A[Start System] --> B[Initialize Hardware]
B --> C[Initialize RTOS]
C --> D[Start Tasks]
D --> E[UART Task]
D --> F[PMBus Task]
D --> G[Command Processing Task]
D --> H[Log Task]
D --> I[Supervisor Task]

    %% Command Processing Flow
    E --> J[Receive Command]
    J --> K{Parse Command}
    K -->|PMBus| L[Queue PMBus Command]
    K -->|System| M[Queue System Command]
    K -->|Config| N[Queue Config Command]
    K -->|Invalid| O[Log Error]

    %% Command Execution
    L --> P[Command Processing Task]
    M --> P
    N --> P

    P --> Q{Command Type}
    Q -->|PMBus| R[Acquire PMBus Mutex]
    Q -->|System| S[Execute System Command]
    Q -->|Config| T[Execute Config Command]

    %% PMBus Communication Subgraph
    subgraph PMBus Communication
        R --> U{Read/Write}
        U -->|Read| V[Read from PMBus Device]
        U -->|Write| W[Write to PMBus Device]
        V --> X[Format PMBus Response]
        W --> X
        X --> Y[Calculate PEC]
        Y --> Z[Release PMBus Mutex]
    end

    %% Logging and Response
    Z --> AA[Log Result]
    S --> AA
    T --> AA
    AA --> AB[Send Response]
    AB --> J[Wait for Next Command]

    %% Supervisor Task
    I --> AC[Monitor System Health]
    AC --> AD[Manage Recovery]

    %% Styles
    style A fill:#98FB98,stroke:#333,stroke-width:2px
    style B fill:#98FB98,stroke:#333,stroke-width:2px
    style C fill:#98FB98,stroke:#333,stroke-width:2px
    style D fill:#98FB98,stroke:#333,stroke-width:2px
    style E fill:#DDA0DD,stroke:#333,stroke-width:2px
    style F fill:#DDA0DD,stroke:#333,stroke-width:2px
    style G fill:#DDA0DD,stroke:#333,stroke-width:2px
    style H fill:#DDA0DD,stroke:#333,stroke-width:2px
    style I fill:#DDA0DD,stroke:#333,stroke-width:2px
    style J fill:#87CEFA,stroke:#333,stroke-width:2px
    style K fill:#FFA07A,stroke:#333,stroke-width:2px
    style L fill:#FFD700,stroke:#333,stroke-width:2px
    style M fill:#87CEFA,stroke:#333,stroke-width:2px
    style N fill:#87CEFA,stroke:#333,stroke-width:2px
    style O fill:#FF6347,stroke:#333,stroke-width:2px
    style P fill:#DDA0DD,stroke:#333,stroke-width:2px
    style Q fill:#FFA07A,stroke:#333,stroke-width:2px
    style R fill:#FFD700,stroke:#333,stroke-width:2px
    style S fill:#87CEFA,stroke:#333,stroke-width:2px
    style T fill:#87CEFA,stroke:#333,stroke-width:2px
    style U fill:#FFA07A,stroke:#333,stroke-width:2px
    style V fill:#FFD700,stroke:#333,stroke-width:2px
    style W fill:#FFD700,stroke:#333,stroke-width:2px
    style X fill:#FFD700,stroke:#333,stroke-width:2px
    style Y fill:#FFD700,stroke:#333,stroke-width:2px
    style Z fill:#FFD700,stroke:#333,stroke-width:2px
    style AA fill:#87CEFA,stroke:#333,stroke-width:2px
    style AB fill:#98FB98,stroke:#333,stroke-width:2px
    style AC fill:#DDA0DD,stroke:#333,stroke-width:2px
    style AD fill:#DDA0DD,stroke:#333,stroke-width:2px

    classDef emphasisBox fill:#f9f,stroke:#333,stroke-width:4px;
    class R,U,V,W,X,Y emphasisBox;
=======
Some functions are not fully implemented in the provided code and may need to be completed based on specific hardware and system requirements.

