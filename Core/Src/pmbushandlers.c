/*
 * pmbusHandlers.c
 *
 *  Created on: Aug 8, 2024
 *      Author: nx024656
 */
#include "sysutil.h"

const CommandHandler commandHandlers[] = {
    [PAGE] = HandlePage,
    [OPERATION] = HandleOperation,
    [ON_OFF_CONFIG] = HandleOnOffConfig,
    [CLEAR_FAULT] = HandleClearFault,
    [PHASE] = HandlePhase,
    [WRITE_PROTECT] = HandleWriteProtect,
    [STORE_DEFAULT_ALL] = HandleStoreDefaultAll,
    [RESTORE_DEFAULT_ALL] = HandleRestoreDefaultAll,
    [CAPABILITY] = HandleCapability,
    [SMBALERT_MASK] = HandleSmbAlertMask,
    [VOUT_MODE] = HandleVoutMode,
    [VOUT_COMMAND] = HandleVoutCommand,
    [VOUT_MAX] = HandleVoutMax,
    [VOUT_MARGIN_HIGH] = HandleVoutMarginHigh,
    [VOUT_MARGIN_LOW] = HandleVoutMarginLow,
    [VOUT_TRANSITION_RATE] = HandleVoutTransitionRate,
    [VOUT_DROOP] = HandleVoutDroop,
    [VOUT_SCALE_LOOP] = HandleVoutScaleLoop,
    [VOUT_SCALE_MONITOR] = HandleVoutScaleMonitor,
    [VOUT_MIN] = HandleVoutMin,
    [FREQUENCY_SWITCH] = HandleFrequencySwitch,
    [VIN_ON] = HandleVinOn,
    [IOUT_CAL_GAIN] = HandleIoutCalGain,
    [IOUT_CAL_OFFSET] = HandleIoutCalOffset,
    [VOUT_OV_FAULT_LIMIT] = HandleVoutOvFaultLimit,
    [VOUT_OV_FAULT_RESPONSE] = HandleVoutOvFaultResponse,
    [VOUT_UV_FAULT_LIMIT] = HandleVoutUvFaultLimit,
    [VOUT_UV_FAULT_RESPONSE] = HandleVoutUvFaultResponse,
    [IOUT_OC_FAULT_LIMIT] = HandleIoutOcFaultLimit,
    [IOUT_OC_FAULT_RESPONSE] = HandleIoutOcFaultResponse,
    [IOUT_OC_WARN_LIMIT] = HandleIoutOcWarnLimit,
    [OT_FAULT_LIMIT] = HandleOtFaultLimit,
    [OT_FAULT_RESPONSE] = HandleOtFaultResponse,
    [OT_WARN_LIMIT] = HandleOtWarnLimit,
    [VIN_OV_FAULT_LIMIT] = HandleVinOvFaultLimit,
    [VIN_OV_FAULT_RESPONSE] = HandleVinOvFaultResponse,
    [VIN_UV_FAULT_LIMIT] = HandleVinUvFaultLimit,
    [VIN_UV_FAULT_RESPONSE] = HandleVinUvFaultResponse,
    [IIN_OC_FAULT_LIMIT] = HandleIinOcFaultLimit,
    [IIN_OC_FAULT_RESPONSE] = HandleIinOcFaultResponse,
    [IIN_OC_WARN_LIMIT] = HandleIinOcWarnLimit,
    [TON_DELAY] = HandleTonDelay,
    [PIN_OP_WARN_LIMIT] = HandlePinOpWarnLimit,
    [STATUS_BYTE] = HandleStatusByte,
    [STATUS_WORD] = HandleStatusWord,
    [STATUS_VOUT] = HandleStatusVout,
    [STATUS_IOUT] = HandleStatusIout,
    [STATUS_INPUT] = HandleStatusInput,
    [STATUS_TEMPERATURE] = HandleStatusTemperature,
    [STATUS_CML] = HandleStatusCml,
    [STATUS_MFR_SPECIFIC] = HandleStatusMfrSpecific,
    [READ_VIN] = HandleReadVin,
    [READ_IIN] = HandleReadIin,
    [READ_VOUT] = HandleReadVout,
    [READ_IOUT] = HandleReadIout,
    [READ_TEMPERATURE_1] = HandleReadTemperature1,
    [READ_POUT] = HandleReadPout,
    [READ_PIN] = HandleReadPin,
    [PMBUS_REVISION] = HandlePmbusRevision,
    [MFR_ID] = HandleMfrId,
    [MFR_MODEL] = HandleMfrModel,
    [MFR_REVISION] = HandleMfrRevision,
    [MFR_DATE] = HandleMfrDate,
    [MFR_SERIAL] = HandleMfrSerial,
    [IC_DEVICE_ID] = HandleIcDeviceId,
    [IC_DEVICE_REV] = HandleIcDeviceRev,
    [USER_DATA_00] = HandleUserData00,
    [USER_DATA_01] = HandleUserData01,
    [USER_DATA_02] = HandleUserData02,
    [USER_DATA_03] = HandleUserData03,
    [USER_DATA_04] = HandleUserData04,
    [USER_DATA_05] = HandleUserData05,
    [USER_DATA_06] = HandleUserData06,
    [USER_DATA_07] = HandleUserData07,
    [USER_DATA_08] = HandleUserData08,
    [USER_DATA_09] = HandleUserData09,
    [USER_DATA_10] = HandleUserData10,
    [USER_DATA_11] = HandleUserData11,
    [USER_DATA_12] = HandleUserData12,
    [MFR_SPECIFIC_00] = HandleMfrSpecific00,
    [MFR_SPECIFIC_03] = HandleMfrSpecific03,
    [MFR_SPECIFIC_04] = HandleMfrSpecific04,
    [MFR_SPECIFIC_05] = HandleMfrSpecific05,
    [MFR_SPECIFIC_06] = HandleMfrSpecific06,
    [MFR_SPECIFIC_07] = HandleMfrSpecific07,
    [MFR_SPECIFIC_08] = HandleMfrSpecific08,
    [MFR_SPECIFIC_09] = HandleMfrSpecific09,
    [MFR_SPECIFIC_10] = HandleMfrSpecific10,
    [MFR_SPECIFIC_11] = HandleMfrSpecific11,
    [MFR_SPECIFIC_12] = HandleMfrSpecific12,
    [MFR_SPECIFIC_13] = HandleMfrSpecific13,
    [MFR_SPECIFIC_14] = HandleMfrSpecific14,
    [MFR_SPECIFIC_15] = HandleMfrSpecific15,
    [MFR_SPECIFIC_20] = HandleMfrSpecific20,
    [MFR_SPECIFIC_32] = HandleMfrSpecific32,
    [MFR_SPECIFIC_42] = HandleMfrSpecific42
};

//*****************************************************PMBUS Command Handlers*****************************************************
void HandlePage(Command_t *cmd) {
    uint8_t pageByte;
    logMessage(LOG_DEBUG, "Device page handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getPage(pdevice.address, &pageByte)) {
            cmd->data[0] = pageByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "PAGE: 0x%02X", pageByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read PAGE");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_WARNING, "Invalid data length for PAGE");
            return;
        }

        // Parse the PAGE byte
        switch (cmd->data[0]) {
            case 0: pageByte = 0x00; break;
            case 1: pageByte = 0x01; break;
            case 2: pageByte = 0xFF; break;
            default:
                logMessage(LOG_ERROR, "Invalid page byte");
                return;
        	  }

        snprintf(syscheck, sizeof(syscheck),"Parsed pageByte:0x%02X",pageByte);
        logMessage(LOG_DEBUG,syscheck);

        if (setPage(pdevice.address, pageByte)) {
            snprintf(syscheck, sizeof(syscheck), "PAGE set to: 0x%02X", pageByte);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to write PAGE");
        }
    }
}


void HandleOperation(Command_t *cmd) {
    uint8_t operationByte;
    logMessage(LOG_DEBUG, "Device operation handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getOpStatus(pdevice.address, &operationByte)) {
            cmd->data[0] = operationByte;
            snprintf(syscheck, sizeof(syscheck), "OPERATION: 0x%02X", operationByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read OPERATION");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for OPERATION");
            return;
        }

        operationByte = cmd->data[0];

        // Validate the OPERATION byte
        if ((operationByte & 0x03) != 0) {
            logMessage(LOG_ERROR, "Invalid OPERATION value: bits 0-1 must be 0");
            return;
        }

        // Process the OPERATION command
        uint8_t onBit = (operationByte >> 7) & 0x01;
        uint8_t marginBits = (operationByte >> 2) & 0x07;

        if (setOpStatus(pdevice.address, operationByte)) {
            snprintf(syscheck, sizeof(syscheck), "OPERATION set to: 0x%02X", operationByte);
            logMessage(LOG_INFO, syscheck);
            snprintf(syscheck, sizeof(syscheck), "ON bit: %d, MARGIN: %d", onBit, marginBits);
            logMessage(LOG_DEBUG, syscheck);

            // Implement the actual power control logic here
            if (onBit) {
                logMessage(LOG_INFO, "Power conversion enabled");
                // ... (implement power-on logic)
            } else {
                logMessage(LOG_INFO, "Power conversion disabled");
                // ... (implement power-off logic)
            }

            // Handle margin if necessary
            switch (marginBits) {
                case 0:
                    logMessage(LOG_INFO, "Margin Off");
                    break;
                case 1:
                    logMessage(LOG_INFO, "Margin Low");
                    break;
                case 2:
                    logMessage(LOG_INFO, "Margin High");
                    break;
                default:
                    snprintf(syscheck, sizeof(syscheck), "Unsupported MARGIN value: %d", marginBits);
                    logMessage(LOG_WARNING, syscheck);
            }
            SendResponse("OPERATION command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write OPERATION");
        }
    }
}


void HandleOnOffConfig(Command_t *cmd) {
    uint8_t configByte;
    logMessage(LOG_DEBUG, "ON_OFF_CONFIG handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getOnOffConfig(pdevice.address, &configByte)) {
            cmd->data[0] = configByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "ON_OFF_CONFIG: 0x%02X", configByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read ON_OFF_CONFIG");
            cmd->length = 0;  // Indicate error by setting length to 0
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for ON_OFF_CONFIG");
            return;
        }

        configByte = cmd->data[0];

        // Validate the ON_OFF_CONFIG byte
        if ((configByte & 0xE1) != 0xE1) {
            logMessage(LOG_ERROR, "Invalid ON_OFF_CONFIG value: bits 7, 6, 5, and 0 must be 1");
            return;
        }

        if (setOnOffConfig(pdevice.address, configByte)) {
            snprintf(syscheck, sizeof(syscheck), "ON_OFF_CONFIG set to: 0x%02X", configByte);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);

            // Parse and log individual bits
            uint8_t pu = (configByte >> 4) & 0x01;
            uint8_t cmd_bit = (configByte >> 3) & 0x01;
            uint8_t cp = (configByte >> 2) & 0x01;

            snprintf(syscheck, sizeof(syscheck), "PU: %d, CMD: %d, CP: %d", pu, cmd_bit, cp);
            logMessage(LOG_DEBUG, syscheck);

            // Implement specific behaviors based on the configuration
            if (cmd_bit) {
                logMessage(LOG_INFO, "Device will respond to the ON bit in OPERATION command");
            } else {
                logMessage(LOG_INFO, "Device ignores the ON bit in OPERATION command");
            }

            if (cp) {
                logMessage(LOG_INFO, "Device responds to AVR_EN/BEN pins");
            } else {
                logMessage(LOG_INFO, "Device ignores AVR_EN/BEN pins, ON/OFF controlled only by OPERATION command");
            }

            SendResponse("ON_OFF_CONFIG command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write ON_OFF_CONFIG");
        }
    }
}


void HandleClearFault(Command_t *cmd) {
    if (cmd->pmbus_rw) {
        logMessage(LOG_ERROR, "CLEAR_FAULT is a write-only command");
        return;
    }

    if (clearfaults(pdevice.address)) {
        logMessage(LOG_INFO, "CLEAR_FAULT executed successfully");
        SendResponse("All faults cleared");
    } else {
        logMessage(LOG_ERROR, "Failed to execute CLEAR_FAULT");
    }
}


void HandlePhase(Command_t *cmd) {
    uint8_t phaseByte;
    logMessage(LOG_DEBUG, "PHASE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getPhase(pdevice.address, &phaseByte)) {
            cmd->data[0] = phaseByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "PHASE: 0x%02X", phaseByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read PHASE");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for PHASE");
            return;
        }

        phaseByte = cmd->data[0];

        // Validate the PHASE byte
        if (phaseByte > 0x0F && phaseByte != 0xFF) {
            logMessage(LOG_ERROR, "Invalid PHASE value: must be 0x00-0x0F or 0xFF");
            return;
        }

        if (setPhase(pdevice.address, phaseByte)) {
            snprintf(syscheck, sizeof(syscheck), "PHASE set to: 0x%02X", phaseByte);
            logMessage(LOG_INFO, syscheck);

            if (phaseByte == 0xFF) {
                logMessage(LOG_INFO, "Commands will access all phases in the selected PAGE");
            } else {
                snprintf(syscheck, sizeof(syscheck), "Commands will address phase %d in the selected PAGE", phaseByte);
                logMessage(LOG_INFO, syscheck);
            }

            SendResponse("PHASE command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write PHASE");
        }
    }
}


void HandleWriteProtect(Command_t *cmd) {
    uint8_t protect_level;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getWriteStatus(pdevice.address, &protect_level)) {
            cmd->data[0] = protect_level;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "WRITE_PROTECT status: 0x%02X", protect_level);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read WRITE_PROTECT status");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for WRITE_PROTECT");
            return;
        }

        protect_level = cmd->data[0];
        if (setWriteStatus(pdevice.address, protect_level, 1)) {
            snprintf(syscheck, sizeof(syscheck), "WRITE_PROTECT set to: 0x%02X", protect_level);
            logMessage(LOG_INFO, syscheck);
            SendResponse("WRITE_PROTECT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set WRITE_PROTECT");
        }
    }
}


void HandleStoreDefaultAll(Command_t *cmd) {
    if (!cmd->pmbus_rw) {
        logMessage(LOG_ERROR, "STORE_DEFAULT_ALL is a write-only command");
        return;
    }

    if (saveToNVM(pdevice.address)) {
        logMessage(LOG_INFO, "STORE_DEFAULT_ALL executed successfully");
        SendResponse("All settings stored as defaults");
    } else {
        logMessage(LOG_ERROR, "Failed to execute STORE_DEFAULT_ALL");
    }
}


void HandleRestoreDefaultAll(Command_t *cmd) {
    if (!cmd->pmbus_rw) {
        logMessage(LOG_ERROR, "RESTORE_DEFAULT_ALL is a write-only command");
        return;
    }

    uint8_t writeProtect;
    if (getWriteStatus(pdevice.address, &writeProtect) && (writeProtect != 0x00)) {
        logMessage(LOG_ERROR, "RESTORE_DEFAULT_ALL blocked by WRITE_PROTECT");
        return;
    }

    if (restoreDevice(pdevice.address, 0)) {
        logMessage(LOG_INFO, "RESTORE_DEFAULT_ALL executed successfully");
        SendResponse("All settings restored to defaults");
    } else {
        logMessage(LOG_ERROR, "Failed to execute RESTORE_DEFAULT_ALL");
    }
}

void HandleCapability(Command_t *cmd) {
    uint8_t capabilityByte;

    logMessage(LOG_DEBUG, "CAPABILITY handle callback");

    // CAPABILITY is read-only
    if (cmd->pmbus_rw) {
        logMessage(LOG_WARNING, "CAPABILITY is a read-only command");
        return;
    }

    if (getCap(pdevice.address, &capabilityByte)) {
        cmd->data[0] = capabilityByte;
        cmd->length = 1;
        snprintf(syscheck, sizeof(syscheck), "CAPABILITY: 0x%02X\n", capabilityByte);
        SendResponse(syscheck);
        logMessage(LOG_INFO, syscheck);

        // Parse and log individual capabilities
        uint8_t pec = (capabilityByte >> 7) & 0x01;
        uint8_t maxBusSpeed = (capabilityByte >> 5) & 0x03;
        uint8_t smbalert = (capabilityByte >> 4) & 0x01;

        snprintf(syscheck, sizeof(syscheck), "\tPEC: %d\n\tMax Bus Speed: %d\n\tSMBALERT: %d",
                 pec, maxBusSpeed, smbalert);
        logMessage(LOG_DEBUG, syscheck);
        SendResponse(syscheck);

        if (pec) {
            logMessage(LOG_INFO, "Packet Error Checking is supported");
        }

        switch (maxBusSpeed) {
            case 0:
                logMessage(LOG_INFO, "Maximum supported bus speed is 100 kHz");
                break;
            case 1:
                logMessage(LOG_INFO, "Maximum supported bus speed is 400 kHz");
                break;
            case 2:
                logMessage(LOG_INFO, "Maximum supported bus speed is 1 MHz");
                break;
            default:
                logMessage(LOG_INFO, "Reserved bus speed value");
        }

        if (smbalert) {
            logMessage(LOG_INFO, "Device supports SMBus Alert Response protocol");
        } else {
            logMessage(LOG_INFO, "Device does not support SMBus Alert Response protocol");
        }
    } else {
        logMessage(LOG_ERROR, "Failed to read CAPABILITY");
        cmd->length = 0;
    }
}


void HandleSmbAlertMask(Command_t *cmd) {
    uint16_t mask_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, 0x1B, &mask_value)) {
            cmd->data[0] = mask_value & 0xFF;
            cmd->data[1] = (mask_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "SMBALERT_MASK: 0x%04X", mask_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read SMBALERT_MASK");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for SMBALERT_MASK");
            return;
        }

        mask_value = (cmd->data[1] << 8) | cmd->data[0];
        if (PMBUS_WriteWord(pdevice.address, 0x1B, mask_value)) {
            snprintf(syscheck, sizeof(syscheck), "SMBALERT_MASK set to: 0x%04X", mask_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("SMBALERT_MASK executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set SMBALERT_MASK");
        }
    }
}


void HandleVoutMode(Command_t *cmd) {
    uint8_t modeByte;
    logMessage(LOG_DEBUG, "VOUT_MODE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getVoutMode(pdevice.address, &modeByte)) {
            cmd->data[0] = modeByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MODE: 0x%02X", modeByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            uint8_t mode = (modeByte >> 5) & 0x07;
            int8_t exponent = (int8_t)(modeByte & 0x1F);
            if (exponent > 15) exponent -= 32; // Convert to signed 5-bit value

            const char* modeStr;
            switch (mode) {
                case 0: modeStr = "Linear"; break;
                case 1: modeStr = "VID"; break;
                case 2: modeStr = "Direct"; break;
                default: modeStr = "Reserved";
            }

            snprintf(syscheck, sizeof(syscheck), "Mode: %s, Exponent: %d", modeStr, exponent);
            logMessage(LOG_DEBUG, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_MODE");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "VOUT_MODE is read-only");
    }
}


void HandleVoutCommand(Command_t *cmd)
{
    uint16_t voutValue;
    logMessage(LOG_DEBUG, "VOUT_COMMAND handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getVoutCommand(pdevice.address, &voutValue)) {
            cmd->data[0] = voutValue & 0xFF;
            cmd->data[1] = (voutValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_COMMAND: 0x%04X", voutValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_COMMAND");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_COMMAND");
            return;
        }

        voutValue = (cmd->data[1] << 8) | cmd->data[0];

        if (setVoutCommand(pdevice.address, voutValue)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_COMMAND set to: 0x%04X", voutValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_COMMAND executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write VOUT_COMMAND");
        }
    }
}


void HandleVoutMax(Command_t *cmd)
{
    uint16_t maxValue;
    logMessage(LOG_DEBUG, "VOUT_MAX handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getVoutMax(pdevice.address, &maxValue)) {
            cmd->data[0] = maxValue & 0xFF;
            cmd->data[1] = (maxValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MAX: 0x%04X", maxValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_MAX");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MAX");
            return;
        }

        maxValue = (cmd->data[1] << 8) | cmd->data[0];

        if (setVoutMax(pdevice.address, maxValue)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_MAX set to: 0x%04X", maxValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_MAX executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write VOUT_MAX");
        }
    }
}


void HandleVoutMarginHigh(Command_t *cmd)
{
    uint16_t marginValue;
    logMessage(LOG_DEBUG, "VOUT_MARGIN_HIGH handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getVoutMarginHigh(pdevice.address, &marginValue)) {
            cmd->data[0] = marginValue & 0xFF;
            cmd->data[1] = (marginValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_HIGH: 0x%04X", marginValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_MARGIN_HIGH");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MARGIN_HIGH");
            return;
        }

        marginValue = (cmd->data[1] << 8) | cmd->data[0];

        if (setVoutMarginHigh(pdevice.address, marginValue)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_HIGH set to: 0x%04X", marginValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_MARGIN_HIGH executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write VOUT_MARGIN_HIGH");
        }
    }
}


void HandleVoutMarginLow(Command_t *cmd)
{
    uint16_t marginValue;
    logMessage(LOG_DEBUG, "VOUT_MARGIN_LOW handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (getVoutMarginLow(pdevice.address, &marginValue)) {
            cmd->data[0] = marginValue & 0xFF;
            cmd->data[1] = (marginValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_LOW: 0x%04X", marginValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_MARGIN_LOW");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MARGIN_LOW");
            return;
        }

        marginValue = (cmd->data[1] << 8) | cmd->data[0];

        if (setVoutMarginLow(pdevice.address, marginValue)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_LOW set to: 0x%04X", marginValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_MARGIN_LOW executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write VOUT_MARGIN_LOW");
        }
    }
}

void HandleVoutMin(Command_t *cmd) {
    uint16_t vout_min;
    float voltage;
    uint8_t vout_mode;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, 0x2B, &vout_min) && getVoutMode(pdevice.address, &vout_mode)) {
            voltage = ulinear16_to_float(vout_min, vout_mode);
            cmd->data[0] = vout_min & 0xFF;
            cmd->data[1] = (vout_min >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MIN: 0x%04X (%.3fV)", vout_min, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_MIN");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MIN");
            return;
        }

        vout_min = (cmd->data[1] << 8) | cmd->data[0];
        if (getVoutMode(pdevice.address, &vout_mode)) {
            voltage = ulinear16_to_float(vout_min, vout_mode);
            if (PMBUS_WriteWord(pdevice.address, 0x2B, vout_min)) {
                snprintf(syscheck, sizeof(syscheck), "VOUT_MIN set to: 0x%04X (%.3fV)", vout_min, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse("VOUT_MIN executed successfully");
            } else {
                logMessage(LOG_ERROR, "Failed to set VOUT_MIN");
            }
        } else {
            logMessage(LOG_ERROR, "Failed to get VOUT_MODE for VOUT_MIN conversion");
        }
    }
}

void HandleVoutTransitionRate(Command_t *cmd)
{
    uint16_t rate_value;
    float rate_mv_us;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VOUT_TRANSITION_RATE, &rate_value)) {
            rate_mv_us = slinear11_to_float(rate_value);
            cmd->data[0] = rate_value & 0xFF;
            cmd->data[1] = (rate_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_TRANSITION_RATE: 0x%04X (%.3f mV/µs)", rate_value, rate_mv_us);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_TRANSITION_RATE");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_TRANSITION_RATE");
            return;
        }

        rate_value = (cmd->data[1] << 8) | cmd->data[0];
        rate_mv_us = slinear11_to_float(rate_value);

        // Check if the value is within the acceptable range
        if (rate_mv_us < 0.3125 || rate_mv_us > 35) {
            logMessage(LOG_ERROR, "VOUT_TRANSITION_RATE value out of range");
            return;
        }

        if (PMBUS_WriteWord(pdevice.address, VOUT_TRANSITION_RATE, rate_value)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_TRANSITION_RATE set to: 0x%04X (%.3f mV/µs)", rate_value, rate_mv_us);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_TRANSITION_RATE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VOUT_TRANSITION_RATE");
        }
    }
}


void HandleVoutDroop(Command_t *cmd)
{
    uint16_t droop_value;
    float droop_mv_a;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VOUT_DROOP, &droop_value)) {
            droop_mv_a = slinear11_to_float(droop_value);
            cmd->data[0] = droop_value & 0xFF;
            cmd->data[1] = (droop_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_DROOP: 0x%04X (%.3f mV/A)", droop_value, droop_mv_a);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_DROOP");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_DROOP");
            return;
        }

        droop_value = (cmd->data[1] << 8) | cmd->data[0];
        droop_mv_a = slinear11_to_float(droop_value);

        // Check if the value is within the acceptable range (adjust as needed)
        if (droop_mv_a < 0 || droop_mv_a > 1) {
            logMessage(LOG_ERROR, "VOUT_DROOP value out of range");
            return;
        }

        if (PMBUS_WriteWord(pdevice.address, VOUT_DROOP, droop_value)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_DROOP set to: 0x%04X (%.3f mV/A)", droop_value, droop_mv_a);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_DROOP executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VOUT_DROOP");
        }
    }
}


void HandleVoutScaleLoop(Command_t *cmd)
{
    uint16_t scale_value;
    float scale_factor;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VOUT_SCALE_LOOP, &scale_value)) {
            scale_factor = slinear11_to_float(scale_value);
            cmd->data[0] = scale_value & 0xFF;
            cmd->data[1] = (scale_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_LOOP: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_SCALE_LOOP");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_SCALE_LOOP");
            return;
        }

        scale_value = (cmd->data[1] << 8) | cmd->data[0];
        scale_factor = slinear11_to_float(scale_value);

        if (PMBUS_WriteWord(pdevice.address, VOUT_SCALE_LOOP, scale_value)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_LOOP set to: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_SCALE_LOOP executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VOUT_SCALE_LOOP");
        }
    }
}


void HandleVoutScaleMonitor(Command_t *cmd)
{
    uint16_t scale_value;
    float scale_factor;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VOUT_SCALE_MONITOR, &scale_value)) {
            scale_factor = slinear11_to_float(scale_value);
            cmd->data[0] = scale_value & 0xFF;
            cmd->data[1] = (scale_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_MONITOR: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_DEBUG, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_SCALE_MONITOR");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_SCALE_MONITOR");
            return;
        }

        scale_value = (cmd->data[1] << 8) | cmd->data[0];
        scale_factor = slinear11_to_float(scale_value);

        if (PMBUS_WriteWord(pdevice.address, VOUT_SCALE_MONITOR, scale_value)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_MONITOR set to: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_DEBUG, syscheck);
            SendResponse("VOUT_SCALE_MONITOR executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VOUT_SCALE_MONITOR");
        }
    }
}

void HandleFrequencySwitch(Command_t *cmd) {
    uint16_t reg_val;
    logMessage(LOG_DEBUG, "FREQUENCY_SWITCH handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, FREQUENCY_SWITCH, &reg_val)) {
            cmd->data[0] = reg_val & 0xFF;
            cmd->data[1] = (reg_val >> 8) & 0xFF;
            cmd->length = 2;

            snprintf(syscheck, sizeof(syscheck), "Raw frequency: 0x%04X", reg_val);
            logMessage(LOG_DEBUG, syscheck);

            float freq = slinear11_to_float(reg_val);
            snprintf(syscheck, sizeof(syscheck), "Switch Frequency: %.2f kHz", freq);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read FREQUENCY_SWITCH");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for FREQUENCY_SWITCH");
            return;
        }

        reg_val = (cmd->data[1] << 8) | cmd->data[0];
        float freq = slinear11_to_float(reg_val);

        // Check if the frequency is within the acceptable range
        if (freq < 300 || freq > 1000) { // change this for various VR controllers
      	//snprintf(syscheck,sizeof(syscheck),"FREQUENCY_SWITCH value out of range (300-1000 kHz)");
            logMessage(LOG_ERROR, "FREQUENCY_SWITCH value out of range (300-1000 kHz)");
            return;
        }

        snprintf(syscheck, sizeof(syscheck), "Data to be written: 0x%04X (%.2f kHz)", reg_val, freq);
        logMessage(LOG_DEBUG, syscheck);

        if (PMBUS_WriteWord(pdevice.address, FREQUENCY_SWITCH, reg_val)) {
            logMessage(LOG_INFO, "Frequency set successfully");
            SendResponse("FREQUENCY_SWITCH command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write FREQUENCY_SWITCH");
        }
    }
}

void HandleVinOn(Command_t *cmd) {
    uint16_t vinOn;
    logMessage(LOG_DEBUG, "VIN_ON handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VIN_ON, &vinOn)) {
            cmd->data[0] = vinOn & 0xFF;
            cmd->data[1] = (vinOn >> 8) & 0xFF;
            cmd->length = 2;
            float voltage = slinear11_to_float(vinOn);
            snprintf(syscheck, sizeof(syscheck), "VIN_ON: 0x%04X (%.2f V)", vinOn, voltage);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VIN_ON");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_ON");
            return;
        }

        vinOn = (cmd->data[1] << 8) | cmd->data[0];
        float voltage = slinear11_to_float(vinOn);

        if (voltage < 4.0 || voltage > 11.25) {
            logMessage(LOG_ERROR, "VIN_ON value out of range");
            return;
        }

        if (PMBUS_WriteWord(pdevice.address, VIN_ON, vinOn)) {
            snprintf(syscheck, sizeof(syscheck), "VIN_ON set to: 0x%04X (%.2f V)", vinOn, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_ON command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write VIN_ON");
        }
    }
}

void HandleIoutCalGain(Command_t *cmd) {
    uint16_t ioutCalGain;
    logMessage(LOG_DEBUG, "IOUT_CAL_GAIN handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, IOUT_CAL_GAIN, &ioutCalGain)) {
            cmd->data[0] = ioutCalGain & 0xFF;
            cmd->data[1] = (ioutCalGain >> 8) & 0xFF;
            cmd->length = 2;
            float gain = slinear11_to_float(ioutCalGain);
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_GAIN: 0x%04X (%.6f mΩ)", ioutCalGain, gain * 1000);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IOUT_CAL_GAIN");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_CAL_GAIN");
            return;
        }

        ioutCalGain = (cmd->data[1] << 8) | cmd->data[0];
        float gain = slinear11_to_float(ioutCalGain);

        if (gain < 4.765625 || gain > 5.25) {
            logMessage(LOG_ERROR, "IOUT_CAL_GAIN value out of range");
            return;
        }

        if (PMBUS_WriteWord(pdevice.address, IOUT_CAL_GAIN, ioutCalGain)) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_GAIN set to: 0x%04X (%.6f mΩ)", ioutCalGain, gain * 1000);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_CAL_GAIN command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write IOUT_CAL_GAIN");
        }
    }
}

void HandleIoutCalOffset(Command_t *cmd) {
    uint16_t ioutCalOffset;
    logMessage(LOG_DEBUG, "IOUT_CAL_OFFSET handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, IOUT_CAL_OFFSET, &ioutCalOffset)) {
            cmd->data[0] = ioutCalOffset & 0xFF;
            cmd->data[1] = (ioutCalOffset >> 8) & 0xFF;
            cmd->length = 2;
            float offset = slinear11_to_float(ioutCalOffset);
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_OFFSET: 0x%04X (%.3f A)", ioutCalOffset, offset);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IOUT_CAL_OFFSET");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_CAL_OFFSET");
            return;
        }

        ioutCalOffset = (cmd->data[1] << 8) | cmd->data[0];
        float offset = slinear11_to_float(ioutCalOffset);
        uint8_t phase = PMBUS_ReadByte(pdevice.address, PHASE, NULL);

        if (phase != 0x80) {
            // Individual phases
            if (offset < -0.875 || offset > 1.0) {
                logMessage(LOG_ERROR, "IOUT_CAL_OFFSET value out of range for individual phases");
                return;
            }
        } else {
            // Total current
            if (offset < -3.75 || offset > 4.0) {
                logMessage(LOG_ERROR, "IOUT_CAL_OFFSET value out of range for total current");
                return;
            }
        }

        if (PMBUS_WriteWord(pdevice.address, IOUT_CAL_OFFSET, ioutCalOffset)) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_OFFSET set to: 0x%04X (%.3f A)", ioutCalOffset, offset);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_CAL_OFFSET command executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to write IOUT_CAL_OFFSET");
        }
    }
}

void HandleVoutOvFaultLimit(Command_t *cmd) {
    uint16_t voutOvFaultLimit;
    logMessage(LOG_DEBUG, "VOUT_OV_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VOUT_OV_FAULT_LIMIT, &voutOvFaultLimit)) {
            cmd->data[0] = voutOvFaultLimit & 0xFF;
            cmd->data[1] = (voutOvFaultLimit >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_OV_FAULT_LIMIT: 0x%04X", voutOvFaultLimit);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_OV_FAULT_LIMIT");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "VOUT_OV_FAULT_LIMIT is read-only");
    }
}

void HandleVoutOvFaultResponse(Command_t *cmd) {
    uint8_t voutOvFaultResponse;
    logMessage(LOG_DEBUG, "VOUT_OV_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, VOUT_OV_FAULT_RESPONSE, &voutOvFaultResponse)) {
            cmd->data[0] = voutOvFaultResponse;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VOUT_OV_FAULT_RESPONSE: 0x%02X", voutOvFaultResponse);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_OV_FAULT_RESPONSE");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "VOUT_OV_FAULT_RESPONSE is read-only");
    }
}

void HandleVoutUvFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float voltage;
    uint8_t vout_mode;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VOUT_UV_FAULT_LIMIT, &limit_value) && getVoutMode(pdevice.address, &vout_mode)) {
            voltage = ulinear16_to_float(limit_value, vout_mode);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_LIMIT: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_UV_FAULT_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_UV_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        if (getVoutMode(pdevice.address, &vout_mode)) {
            voltage = ulinear16_to_float(limit_value, vout_mode);
            if (PMBUS_WriteWord(pdevice.address, VOUT_UV_FAULT_LIMIT, limit_value)) {
                snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_LIMIT set to: 0x%04X (%.3fV)", limit_value, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse("VOUT_UV_FAULT_LIMIT executed successfully");
            } else {
                logMessage(LOG_ERROR, "Failed to set VOUT_UV_FAULT_LIMIT");
            }
        } else {
            logMessage(LOG_ERROR, "Failed to get VOUT_MODE for VOUT_UV_FAULT_LIMIT conversion");
        }
    }
}

void HandleVoutUvFaultResponse(Command_t *cmd) {
    uint8_t response_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, VOUT_UV_FAULT_RESPONSE, &response_value)) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VOUT_UV_FAULT_RESPONSE");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_UV_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        if (PMBUS_WriteByte(pdevice.address, VOUT_UV_FAULT_RESPONSE, response_value)) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_UV_FAULT_RESPONSE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VOUT_UV_FAULT_RESPONSE");
        }
    }
}

void HandleIoutOcFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, IOUT_OC_FAULT_LIMIT, &limit_value)) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IOUT_OC_FAULT_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_OC_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, IOUT_OC_FAULT_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_OC_FAULT_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set IOUT_OC_FAULT_LIMIT");
        }
    }
}


void HandleIoutOcWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, IOUT_OC_WARN_LIMIT, &limit_value)) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_WARN_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IOUT_OC_WARN_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_OC_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, IOUT_OC_WARN_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_WARN_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_OC_WARN_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set IOUT_OC_WARN_LIMIT");
        }
    }
}


void HandleIoutOcFaultResponse(Command_t *cmd) {
    uint8_t response_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, OT_FAULT_RESPONSE, &response_value)) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IOUT_OC_FAULT_RESPONSE");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_OC_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        if (PMBUS_WriteByte(pdevice.address, OT_FAULT_RESPONSE, response_value)) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_OC_FAULT_RESPONSE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set IOUT_OC_FAULT_RESPONSE");
        }
    }
}


void HandleOtFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float temperature;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, OT_FAULT_LIMIT, &limit_value)) {
            temperature = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_LIMIT: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read OT_FAULT_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for OT_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        temperature = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, OT_FAULT_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_LIMIT set to: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse("OT_FAULT_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set OT_FAULT_LIMIT");
        }
    }
}

void HandleOtFaultResponse(Command_t *cmd) {
    uint8_t response_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, OT_FAULT_RESPONSE, &response_value)) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read OT_FAULT_RESPONSE");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for OT_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        if (PMBUS_WriteByte(pdevice.address, OT_FAULT_RESPONSE, response_value)) {
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("OT_FAULT_RESPONSE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set OT_FAULT_RESPONSE");
        }
    }
}

void HandleOtWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float temperature;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, OT_WARN_LIMIT, &limit_value)) {
            temperature = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "OT_WARN_LIMIT: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read OT_WARN_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for OT_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        temperature = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, OT_WARN_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "OT_WARN_LIMIT set to: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse("OT_WARN_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set OT_WARN_LIMIT");
        }
    }
}

void HandleVinOvFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float voltage;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VIN_OV_FAULT_LIMIT, &limit_value)) {
            voltage = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_LIMIT: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VIN_OV_FAULT_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_OV_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        voltage = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, VIN_OV_FAULT_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_LIMIT set to: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_OV_FAULT_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VIN_OV_FAULT_LIMIT");
        }
    }
}

void HandleVinOvFaultResponse(Command_t *cmd) {
    uint8_t response_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, VIN_OV_FAULT_RESPONSE, &response_value)) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VIN_OV_FAULT_RESPONSE");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_OV_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        if (PMBUS_WriteByte(pdevice.address, VIN_OV_FAULT_RESPONSE, response_value)) {
            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_OV_FAULT_RESPONSE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VIN_OV_FAULT_RESPONSE");
        }
    }
}

void HandleVinUvFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float voltage;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, VIN_UV_FAULT_LIMIT, &limit_value)) {
            voltage = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_LIMIT: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VIN_UV_FAULT_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_UV_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        voltage = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, VIN_UV_FAULT_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_LIMIT set to: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_UV_FAULT_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VIN_UV_FAULT_LIMIT");
        }
    }
}

void HandleVinUvFaultResponse(Command_t *cmd) {
    uint8_t response_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, VIN_UV_FAULT_RESPONSE, &response_value)) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read VIN_UV_FAULT_RESPONSE");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_UV_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        if (PMBUS_WriteByte(pdevice.address, VIN_UV_FAULT_RESPONSE, response_value)) {
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_UV_FAULT_RESPONSE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set VIN_UV_FAULT_RESPONSE");
        }
    }
}

void HandleIinOcFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, IIN_OC_FAULT_LIMIT, &limit_value)) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IIN_OC_FAULT_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IIN_OC_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, IIN_OC_FAULT_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IIN_OC_FAULT_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set IIN_OC_FAULT_LIMIT");
        }
    }
}

void HandleIinOcFaultResponse(Command_t *cmd) {
    uint8_t response_value;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadByte(pdevice.address, IIN_OC_FAULT_RESPONSE, &response_value)) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IIN_OC_FAULT_RESPONSE");
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for IIN_OC_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        if (PMBUS_WriteByte(pdevice.address, IIN_OC_FAULT_RESPONSE, response_value)) {
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IIN_OC_FAULT_RESPONSE executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set IIN_OC_FAULT_RESPONSE");
        }
    }
}

void HandleIinOcWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, IIN_OC_WARN_LIMIT, &limit_value)) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_WARN_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IIN_OC_WARN_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IIN_OC_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, IIN_OC_WARN_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_WARN_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IIN_OC_WARN_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set IIN_OC_WARN_LIMIT");
        }
    }
}

void HandleTonDelay(Command_t *cmd) {
    uint16_t delay_value;
    float delay_ms;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, TON_DELAY, &delay_value)) {
            delay_ms = slinear11_to_float(delay_value) * 1000; // Convert to milliseconds
            cmd->data[0] = delay_value & 0xFF;
            cmd->data[1] = (delay_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "TON_DELAY: 0x%04X (%.2f ms)", delay_value, delay_ms);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read TON_DELAY");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for TON_DELAY");
            return;
        }

        delay_value = (cmd->data[1] << 8) | cmd->data[0];
        delay_ms = slinear11_to_float(delay_value) * 1000; // Convert to milliseconds
        if (PMBUS_WriteWord(pdevice.address, TON_DELAY, delay_value)) {
            snprintf(syscheck, sizeof(syscheck), "TON_DELAY set to: 0x%04X (%.2f ms)", delay_value, delay_ms);
            logMessage(LOG_INFO, syscheck);
            SendResponse("TON_DELAY executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set TON_DELAY");
        }
    }
}

void HandlePinOpWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float power;

    if (!cmd->pmbus_rw) {
        // Read operation
        if (PMBUS_ReadWord(pdevice.address, PIN_OP_WARN_LIMIT, &limit_value)) {
            power = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "PIN_OP_WARN_LIMIT: 0x%04X (%.3f W)", limit_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read PIN_OP_WARN_LIMIT");
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for PIN_OP_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        power = slinear11_to_float(limit_value);
        if (PMBUS_WriteWord(pdevice.address, PIN_OP_WARN_LIMIT, limit_value)) {
            snprintf(syscheck, sizeof(syscheck), "PIN_OP_WARN_LIMIT set to: 0x%04X (%.3f W)", limit_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse("PIN_OP_WARN_LIMIT executed successfully");
        } else {
            logMessage(LOG_ERROR, "Failed to set PIN_OP_WARN_LIMIT");
        }
    }
}

void HandleStatusByte(Command_t *cmd) {
    uint8_t statusByte;
    logMessage(LOG_DEBUG, "STATUS_BYTE handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusByte(pdevice.address, &statusByte)) {
            cmd->data[0] = statusByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_BYTE: 0x%02X", statusByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_BYTE
            if (statusByte & 0x40) logMessage(LOG_DEBUG, "OFF: Unit is not providing power to the output");
            if (statusByte & 0x20) logMessage(LOG_DEBUG, "VOUT_OV: Output Over-Voltage Fault Condition");
            if (statusByte & 0x10) logMessage(LOG_DEBUG, "IOUT_OC: Output Over-Current Fault Condition");
            if (statusByte & 0x08) logMessage(LOG_DEBUG, "VIN_UV: Input Under-Voltage Fault Condition");
            if (statusByte & 0x04) logMessage(LOG_DEBUG, "TEMP: Over-Temperature Fault/Warning");
            if (statusByte & 0x02) logMessage(LOG_DEBUG, "CML: Communications, Memory or Logic Fault");
            if (statusByte & 0x01) logMessage(LOG_DEBUG, "OTHER: Other Fault");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_BYTE");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "STATUS_BYTE is read-only");
    }
}

void HandleStatusWord(Command_t *cmd) {
    uint16_t statusWord;
    logMessage(LOG_DEBUG, "STATUS_WORD handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusWord(pdevice.address, &statusWord)) {
            cmd->data[0] = statusWord & 0xFF;
            cmd->data[1] = (statusWord >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "STATUS_WORD: 0x%04X", statusWord);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_WORD (additional bits to STATUS_BYTE)
            if (statusWord & 0x8000) logMessage(LOG_DEBUG, "VOUT fault");
            if (statusWord & 0x4000) logMessage(LOG_DEBUG, "IOUT fault");
            if (statusWord & 0x2000) logMessage(LOG_DEBUG, "INPUT fault");
            if (statusWord & 0x1000) logMessage(LOG_DEBUG, "MFR Specific");
            if (statusWord & 0x0800) logMessage(LOG_DEBUG, "POWER_GOOD negated");
            if (statusWord & 0x0400) logMessage(LOG_DEBUG, "FAN fault");
            if (statusWord & 0x0200) logMessage(LOG_DEBUG, "OTHER");
            if (statusWord & 0x0100) logMessage(LOG_DEBUG, "UNKNOWN");

            // Lower byte (same as STATUS_BYTE)
            if (statusWord & 0x0040) logMessage(LOG_DEBUG, "OFF: Unit is not providing power to the output");
            if (statusWord & 0x0020) logMessage(LOG_DEBUG, "VOUT_OV: Output Over-Voltage Fault Condition");
            if (statusWord & 0x0010) logMessage(LOG_DEBUG, "IOUT_OC: Output Over-Current Fault Condition");
            if (statusWord & 0x0008) logMessage(LOG_DEBUG, "VIN_UV: Input Under-Voltage Fault Condition");
            if (statusWord & 0x0004) logMessage(LOG_DEBUG, "TEMP: Over-Temperature Fault/Warning");
            if (statusWord & 0x0002) logMessage(LOG_DEBUG, "CML: Communications, Memory or Logic Fault");
            if (statusWord & 0x0001) logMessage(LOG_DEBUG, "OTHER: Other Fault");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_WORD");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "STATUS_WORD is read-only");
    }
}

void HandleStatusMfrSpecific(Command_t *cmd) {
    uint8_t statusMfrSpecific;
    logMessage(LOG_DEBUG, "STATUS_MFR_SPECIFIC handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusMfrSpecific(pdevice.address, &statusMfrSpecific)) {
            cmd->data[0] = statusMfrSpecific;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_MFR_SPECIFIC: 0x%02X", statusMfrSpecific);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_MFR_SPECIFIC
            if (statusMfrSpecific & 0x80) logMessage(LOG_DEBUG, "Power Stage Fault");
            if (statusMfrSpecific & 0x40) logMessage(LOG_DEBUG, "VSNS pin open");
            if (statusMfrSpecific & 0x20) logMessage(LOG_DEBUG, "Maximum Phase Warning");
            if (statusMfrSpecific & 0x10) logMessage(LOG_DEBUG, "TSNS_LOW");
            if (statusMfrSpecific & 0x08) logMessage(LOG_DEBUG, "RST_VID (Page 0)");
            if (statusMfrSpecific & 0x01) logMessage(LOG_DEBUG, "Phase current share fault");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_MFR_SPECIFIC");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_MFR_SPECIFIC");
            return;
        }
        if (PMBUS_WriteByte(pdevice.address, 0x80, cmd->data[0])) {
            logMessage(LOG_INFO, "STATUS_MFR_SPECIFIC bits cleared");
        } else {
            logMessage(LOG_ERROR, "Failed to clear STATUS_MFR_SPECIFIC bits");
        }
    }
}

void HandleStatusCml(Command_t *cmd) {
    uint8_t statusCml;
    logMessage(LOG_DEBUG, "STATUS_CML handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusCml(pdevice.address, &statusCml)) {
            cmd->data[0] = statusCml;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_CML: 0x%02X", statusCml);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_CML
            if (statusCml & 0x80) logMessage(LOG_DEBUG, "Invalid or Unsupported Command Received");
            if (statusCml & 0x40) logMessage(LOG_DEBUG, "Invalid or Unsupported Data Received");
            if (statusCml & 0x20) logMessage(LOG_DEBUG, "Packet Error Check Failed");
            if (statusCml & 0x08) logMessage(LOG_DEBUG, "Memory/NVM Error");
            if (statusCml & 0x02) logMessage(LOG_DEBUG, "Other Communication Faults");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_CML");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_CML");
            return;
        }
        if (PMBUS_WriteByte(pdevice.address, 0x7E, cmd->data[0])) {
            logMessage(LOG_INFO, "STATUS_CML bits cleared");
        } else {
            logMessage(LOG_ERROR, "Failed to clear STATUS_CML bits");
        }
    }
}

void HandleStatusTemperature(Command_t *cmd) {
    uint8_t statusTemperature;
    logMessage(LOG_DEBUG, "STATUS_TEMPERATURE handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusTemperature(pdevice.address, &statusTemperature)) {
            cmd->data[0] = statusTemperature;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_TEMPERATURE: 0x%02X", statusTemperature);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_TEMPERATURE
            if (statusTemperature & 0x80) logMessage(LOG_DEBUG, "Over-Temperature Fault");
            if (statusTemperature & 0x40) logMessage(LOG_DEBUG, "Over-Temperature Warning");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_TEMPERATURE");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_TEMPERATURE");
            return;
        }
        if (PMBUS_WriteByte(pdevice.address, 0x7D, cmd->data[0])) {
            logMessage(LOG_INFO, "STATUS_TEMPERATURE bits cleared");
        } else {
            logMessage(LOG_ERROR, "Failed to clear STATUS_TEMPERATURE bits");
        }
    }
}

void HandleStatusInput(Command_t *cmd) {
    uint8_t statusInput;
    logMessage(LOG_DEBUG, "STATUS_INPUT handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusInput(pdevice.address, &statusInput)) {
            cmd->data[0] = statusInput;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_INPUT: 0x%02X", statusInput);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_INPUT
            if (statusInput & 0x80) logMessage(LOG_DEBUG, "Input Over-Voltage Fault");
            if (statusInput & 0x10) logMessage(LOG_DEBUG, "Input Under-Voltage Fault");
            if (statusInput & 0x08) logMessage(LOG_DEBUG, "Unit Off for insufficient input voltage");
            if (statusInput & 0x04) logMessage(LOG_DEBUG, "Input Over-Current Fault");
            if (statusInput & 0x02) logMessage(LOG_DEBUG, "Input Over-Current Warning");
            if (statusInput & 0x01) logMessage(LOG_DEBUG, "Input Over-Power Warning");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_INPUT");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_INPUT");
            return;
        }
        if (PMBUS_WriteByte(pdevice.address, 0x7C, cmd->data[0])) {
            logMessage(LOG_INFO, "STATUS_INPUT bits cleared");
        } else {
            logMessage(LOG_ERROR, "Failed to clear STATUS_INPUT bits");
        }
    }
}

void HandleStatusIout(Command_t *cmd) {
    uint8_t statusIout;
    logMessage(LOG_DEBUG, "STATUS_IOUT handle callback");

    if (!cmd->pmbus_rw) {
        if (getStatusIout(pdevice.address, &statusIout)) {
            cmd->data[0] = statusIout;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_IOUT: 0x%02X", statusIout);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_IOUT
            if (statusIout & 0x80) logMessage(LOG_DEBUG, "Output Over-Current Fault");
            if (statusIout & 0x20) logMessage(LOG_DEBUG, "Output Over-Current Warning");
            if (statusIout & 0x08) logMessage(LOG_DEBUG, "Current Share Fault");
            if (statusIout & 0x04) logMessage(LOG_DEBUG, "Power Limiting Mode");
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_IOUT");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_IOUT");
            return;
        }
        if (PMBUS_WriteByte(pdevice.address, 0x7B, cmd->data[0])) {
            logMessage(LOG_INFO, "STATUS_IOUT bits cleared");
        } else {
            logMessage(LOG_ERROR, "Failed to clear STATUS_IOUT bits");
        }
    }
}

void HandleStatusVout(Command_t *cmd) {
    uint8_t status_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadByte(pdevice.address, STATUS_VOUT, &status_value)) {
            cmd->data[0] = status_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_VOUT: 0x%02X", status_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read STATUS_VOUT");
        }
    } else {
        logMessage(LOG_ERROR, "STATUS_VOUT is a read-only command");
    }
}

void HandleReadVin(Command_t *cmd) {
    uint16_t vin_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_VIN, &vin_value)) {
            float voltage = slinear11_to_float(vin_value);
            cmd->data[0] = vin_value & 0xFF;
            cmd->data[1] = (vin_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_VIN: 0x%04X (%.3f V)", vin_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_VIN");
        }
    } else {
        logMessage(LOG_ERROR, "READ_VIN is a read-only command");
    }
}

void HandleReadIin(Command_t *cmd) {
    uint16_t iin_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_IIN, &iin_value)) {
            float current = slinear11_to_float(iin_value);
            cmd->data[0] = iin_value & 0xFF;
            cmd->data[1] = (iin_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_IIN: 0x%04X (%.3f A)", iin_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_IIN");
        }
    } else {
        logMessage(LOG_ERROR, "READ_IIN is a read-only command");
    }
}

void HandleReadVout(Command_t *cmd) {
    uint16_t vout_value;
    uint8_t vout_mode;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_VOUT, &vout_value) && getVoutMode(pdevice.address, &vout_mode)) {
            float voltage = ulinear16_to_float(vout_value, vout_mode);
            cmd->data[0] = vout_value & 0xFF;
            cmd->data[1] = (vout_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_VOUT: 0x%04X (%.3f V)", vout_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_VOUT");
        }
    } else {
        logMessage(LOG_ERROR, "READ_VOUT is a read-only command");
    }
}

void HandleReadIout(Command_t *cmd) {
    uint16_t iout_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_IOUT, &iout_value)) {
            float current = slinear11_to_float(iout_value);
            cmd->data[0] = iout_value & 0xFF;
            cmd->data[1] = (iout_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_IOUT: 0x%04X (%.3f A)", iout_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_IOUT");
        }
    } else {
        logMessage(LOG_ERROR, "READ_IOUT is a read-only command");
    }
}

void HandleReadTemperature1(Command_t *cmd) {
    uint16_t temp_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_TEMPERATURE_1, &temp_value)) {
            float temperature = slinear11_to_float(temp_value);
            cmd->data[0] = temp_value & 0xFF;
            cmd->data[1] = (temp_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_TEMPERATURE_1: 0x%04X (%.2f °C)", temp_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_TEMPERATURE_1");
        }
    } else {
        logMessage(LOG_ERROR, "READ_TEMPERATURE_1 is a read-only command");
    }
}

void HandleReadPout(Command_t *cmd) {
    uint16_t pout_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_POUT, &pout_value)) {
            float power = slinear11_to_float(pout_value);
            cmd->data[0] = pout_value & 0xFF;
            cmd->data[1] = (pout_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_POUT: 0x%04X (%.3f W)", pout_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_POUT");
        }
    } else {
        logMessage(LOG_ERROR, "READ_POUT is a read-only command");
    }
}

void HandleReadPin(Command_t *cmd) {
    uint16_t pin_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, READ_PIN, &pin_value)) {
            float power = slinear11_to_float(pin_value);
            cmd->data[0] = pin_value & 0xFF;
            cmd->data[1] = (pin_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_PIN: 0x%04X (%.3f W)", pin_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read READ_PIN");
        }
    } else {
        logMessage(LOG_ERROR, "READ_PIN is a read-only command");
    }
}

void HandlePmbusRevision(Command_t *cmd) {
    uint8_t revision_value;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadByte(pdevice.address, PMBUS_REVISION, &revision_value)) {
            cmd->data[0] = revision_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "PMBUS_REVISION: 0x%02X", revision_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read PMBUS_REVISION");
        }
    } else {
        logMessage(LOG_ERROR, "PMBUS_REVISION is a read-only command");
    }
}

void HandleMfrId(Command_t *cmd) {
    uint8_t mfr_id[32];
    uint8_t length;

    if (!cmd->pmbus_rw) {
        if (PMBUS_BlockRead(pdevice.address, MFR_ID, mfr_id, &length)) {
            memcpy(cmd->data, mfr_id, length);
            cmd->length = length;
            mfr_id[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_ID: %s", mfr_id);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read MFR_ID");
        }
    } else {
        logMessage(LOG_ERROR, "MFR_ID is a read-only command");
    }
}

void HandleMfrModel(Command_t *cmd) {
    uint8_t mfr_model[32];
    uint8_t length;

    if (!cmd->pmbus_rw) {
        if (PMBUS_BlockRead(pdevice.address, MFR_MODEL, mfr_model, &length)) {
            memcpy(cmd->data, mfr_model, length);
            cmd->length = length;
            mfr_model[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_MODEL: %s", mfr_model);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read MFR_MODEL");
        }
    } else {
        logMessage(LOG_ERROR, "MFR_MODEL is a read-only command");
    }
}

void HandleMfrRevision(Command_t *cmd) {
    uint8_t mfr_revision[32];
    uint8_t length;

    if (!cmd->pmbus_rw) {
        if (PMBUS_BlockRead(pdevice.address, MFR_REVISION, mfr_revision, &length)) {
            memcpy(cmd->data, mfr_revision, length);
            cmd->length = length;
            mfr_revision[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_REVISION: %s", mfr_revision);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read MFR_REVISION");
        }
    } else {
        logMessage(LOG_ERROR, "MFR_REVISION is a read-only command");
    }
}

void HandleMfrDate(Command_t *cmd) {
    uint8_t mfr_date[32];
    uint8_t length;

    if (!cmd->pmbus_rw) {
        if (PMBUS_BlockRead(pdevice.address, MFR_DATE, mfr_date, &length)) {
            memcpy(cmd->data, mfr_date, length);
            cmd->length = length;
            mfr_date[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_DATE: %s", mfr_date);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read MFR_DATE");
        }
    } else {
        logMessage(LOG_ERROR, "MFR_DATE is a read-only command");
    }
}

void HandleMfrSerial(Command_t *cmd) {
    uint8_t mfr_serial[32];
    uint8_t length;

    if (!cmd->pmbus_rw) {
        if (PMBUS_BlockRead(pdevice.address, MFR_SERIAL, mfr_serial, &length)) {
            memcpy(cmd->data, mfr_serial, length);
            cmd->length = length;
            mfr_serial[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_SERIAL: %s", mfr_serial);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read MFR_SERIAL");
        }
    } else {
        logMessage(LOG_ERROR, "MFR_SERIAL is a read-only command");
    }
}

void HandleIcDeviceId(Command_t *cmd) {
    uint32_t device_id;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadWord(pdevice.address, IC_DEVICE_ID, (uint16_t*)&device_id) &&
            PMBUS_ReadWord(pdevice.address, IC_DEVICE_ID + 1, (uint16_t*)(&device_id + 2))) {
            cmd->data[0] = device_id & 0xFF;
            cmd->data[1] = (device_id >> 8) & 0xFF;
            cmd->data[2] = (device_id >> 16) & 0xFF;
            cmd->data[3] = (device_id >> 24) & 0xFF;
            cmd->length = 4;
            snprintf(syscheck, sizeof(syscheck), "IC_DEVICE_ID: 0x%08X", (unsigned int)device_id);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IC_DEVICE_ID");
        }
    } else {
        logMessage(LOG_ERROR, "IC_DEVICE_ID is a read-only command");
    }
}

void HandleIcDeviceRev(Command_t *cmd) {
    uint8_t device_rev;

    if (!cmd->pmbus_rw) {
        if (PMBUS_ReadByte(pdevice.address, IC_DEVICE_REV, &device_rev)) {
            cmd->data[0] = device_rev;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "IC_DEVICE_REV: 0x%02X", device_rev);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logMessage(LOG_ERROR, "Failed to read IC_DEVICE_REV");
        }
    } else {
        logMessage(LOG_ERROR, "IC_DEVICE_REV is a read-only command");
    }
}

void HandleUserData00(Command_t *cmd) {
    // Process USER_DATA_00 command
    // For example, handle user data 00
}

void HandleUserData01(Command_t *cmd) {
    // Process USER_DATA_01 command
    // For example, handle user data 01
}

void HandleUserData02(Command_t *cmd) {
    // Process USER_DATA_02 command
    // For example, handle user data 02
}

void HandleUserData03(Command_t *cmd) {
    // Process USER_DATA_03 command
    // For example, handle user data 03
}

void HandleUserData04(Command_t *cmd) {
    // Process USER_DATA_04 command
    // For example, handle user data 04
}

void HandleUserData05(Command_t *cmd) {
    // Process USER_DATA_05 command
    // For example, handle user data 05
}

void HandleUserData06(Command_t *cmd) {
    // Process USER_DATA_06 command
    // For example, handle user data 06
}

void HandleUserData07(Command_t *cmd) {
    // Process USER_DATA_07 command
    // For example, handle user data 07
}

void HandleUserData08(Command_t *cmd) {
    // Process USER_DATA_08 command
    // For example, handle user data 08
}

void HandleUserData09(Command_t *cmd) {
    // Process USER_DATA_09 command
    // For example, handle user data 09
}

void HandleUserData10(Command_t *cmd) {
    // Process USER_DATA_10 command
    // For example, handle user data 10
}

void HandleUserData11(Command_t *cmd) {
    // Process USER_DATA_11 command
    // For example, handle user data 11
}

void HandleUserData12(Command_t *cmd) {
    // Process USER_DATA_12 command
    // For example, handle user data 12
}

void HandleMfrSpecific00(Command_t *cmd) {
    // Process MFR_SPECIFIC_00 command
    // For example, handle manufacturer-specific command 00
}

void HandleMfrSpecific03(Command_t *cmd) {
    // Process MFR_SPECIFIC_03 command
    // For example, handle manufacturer-specific command 03
}

void HandleMfrSpecific04(Command_t *cmd) {
    // Process MFR_SPECIFIC_04 command
    // For example, handle manufacturer-specific command 04
}

void HandleMfrSpecific05(Command_t *cmd) {
    // Process MFR_SPECIFIC_05 command
    // For example, handle manufacturer-specific command 05
}

void HandleMfrSpecific06(Command_t *cmd) {
    // Process MFR_SPECIFIC_06 command
    // For example, handle manufacturer-specific command 06
}

void HandleMfrSpecific07(Command_t *cmd) {
    // Process MFR_SPECIFIC_07 command
    // For example, handle manufacturer-specific command 07
}

void HandleMfrSpecific08(Command_t *cmd) {
    // Process MFR_SPECIFIC_08 command
    // For example, handle manufacturer-specific command 08
}

void HandleMfrSpecific09(Command_t *cmd) {
    // Process MFR_SPECIFIC_09 command
    // For example, handle manufacturer-specific command 09
}

void HandleMfrSpecific10(Command_t *cmd) {
    // Process MFR_SPECIFIC_10 command
    // For example, handle manufacturer-specific command 10
}

void HandleMfrSpecific11(Command_t *cmd) {
    // Process MFR_SPECIFIC_11 command
    // For example, handle manufacturer-specific command 11
}

void HandleMfrSpecific12(Command_t *cmd) {
    // Process MFR_SPECIFIC_12 command
    // For example, handle manufacturer-specific command 12
}

void HandleMfrSpecific13(Command_t *cmd) {
    // Process MFR_SPECIFIC_13 command
    // For example, handle manufacturer-specific command 13
}

void HandleMfrSpecific14(Command_t *cmd) {
    // Process MFR_SPECIFIC_14 command
    // For example, handle manufacturer-specific command 14
}

void HandleMfrSpecific15(Command_t *cmd) {
    // Process MFR_SPECIFIC_15 command
    // For example, handle manufacturer-specific command 15
}

void HandleMfrSpecific20(Command_t *cmd) {
    // Process MFR_SPECIFIC_20 command
    // For example, handle manufacturer-specific command 20
}

void HandleMfrSpecific32(Command_t *cmd) {
    // Process MFR_SPECIFIC_32 command
    // For example, handle manufacturer-specific command 32
}

void HandleMfrSpecific42(Command_t *cmd) {
    // Process MFR_SPECIFIC_42 command
    // For example, handle manufacturer-specific command 42
}


