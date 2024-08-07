# UART to PMBus Interface

This project implements a UART to PMBus (version 1.3) interface with system management and configuration capabilities.

## Features

- PMBus command processing (PMBus version 1.3)
- System command handling
- Configuration options
- Logging system with adjustable verbosity

## Command Types

1. PMBus Commands (0)
2. System Commands (1)
3. Configuration Commands (2)

## System Commands

- `SYS_GET_OS_VERSION` (000): Get the OS version
- `SYS_UPDATE_FIRMWARE` (001): Initiate firmware update
- `SYS_RESET` (002): Perform system reset
- `SYS_GET_UPTIME` (003): Get system uptime
- `SYS_GET_MEMORY_STATS` (004): Get memory statistics
- `SYS_GET_CPU_USAGE` (005): Get CPU usage
- `SYS_GET_HARDWARE_INFO` (006): Get hardware information

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
`<command_type>``<command_code>``<data>`\n

Where:
- `<command_type>` is 0 for PMBus, 1 for System, 2 for Config
- `<command_code>` is a three-digit number
- `<data>` is optional and command-specific
- `\n` is the newline character

Example: 20021\n

This sets the log verbosity to NORMAL.

## PMBus Commands

PMBus commands (type 0) are implemented according to the PMBus Power System Management Protocol Specification Version 1.3. Refer to the PMBus specification for detailed command descriptions and usage.

## Note

Some functions are not fully implemented in the provided code and may need to be completed based on specific hardware and system requirements.
