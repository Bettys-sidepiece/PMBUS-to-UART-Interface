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
//TODO -- Update handle functions ensuring that they work with *pmbus_error_t*
void HandlePage(Command_t *cmd) {
    uint8_t pageByte;
    logMessage(LOG_DEBUG, "Device page handle callback");
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        // Read operation
	  result = getPage(pdevice.address, &pageByte);
        if (result == PMBUS_OK) {
            cmd->data[0] = pageByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "PAGE: 0x%02X", pageByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR,result, "PAGE Read");
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

        result = setPage(pdevice.address, pageByte);

        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "PAGE set to: 0x%02X", pageByte);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
      	logPmbusError(LOG_ERROR,result,"PAGE Write");
        }
    }
}


void HandleOperation(Command_t *cmd) {
    uint8_t operationByte;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "Device operation handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getOpStatus(pdevice.address, &operationByte);
        if (result == PMBUS_OK) {
            cmd->data[0] = operationByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "OPERATION: 0x%02X", operationByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "OPERATION Read");
            cmd->length = 0;
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

        uint8_t onBit = (operationByte >> 7) & 0x01;
        uint8_t marginBits = (operationByte >> 2) & 0x07;

        result = setOpStatus(pdevice.address, operationByte);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "OPERATION set to: 0x%02X", operationByte);
            logMessage(LOG_INFO, syscheck);
            snprintf(syscheck, sizeof(syscheck), "ON bit: %d, MARGIN: %d", onBit, marginBits);
            logMessage(LOG_DEBUG, syscheck);

            if (onBit) logMessage(LOG_INFO, "Power conversion enabled");
            else logMessage(LOG_INFO, "Power conversion disabled");

            // Handle margin if necessary
            switch (marginBits) {
                case 0: logMessage(LOG_INFO, "Margin Off"); break;
                case 1: logMessage(LOG_INFO, "Margin Low"); break;
                case 2: logMessage(LOG_INFO, "Margin High"); break;
                default:
                    snprintf(syscheck, sizeof(syscheck), "Unsupported MARGIN value: %d", marginBits);
                    logMessage(LOG_WARNING, syscheck);
            }
            SendResponse("OPERATION command executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "OPERATION Write");
        }
    }
}

void HandleOnOffConfig(Command_t *cmd) {
    uint8_t configByte;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "ON_OFF_CONFIG handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getOnOffConfig(pdevice.address, &configByte);
        if (result == PMBUS_OK) {
            cmd->data[0] = configByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "ON_OFF_CONFIG: 0x%02X", configByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "ON_OFF_CONFIG Read");
            cmd->length = 0;
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

        result = setOnOffConfig(pdevice.address, configByte);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "ON_OFF_CONFIG set to: 0x%02X", configByte);
            logMessage(LOG_INFO, syscheck);

            uint8_t pu = (configByte >> 4) & 0x01;
            uint8_t cmd_bit = (configByte >> 3) & 0x01;
            uint8_t cp = (configByte >> 2) & 0x01;

            snprintf(syscheck, sizeof(syscheck), "PU: %d, CMD: %d, CP: %d", pu, cmd_bit, cp);
            logMessage(LOG_DEBUG, syscheck);

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
            logPmbusError(LOG_ERROR, result, "ON_OFF_CONFIG Write");
        }
    }
}

void HandleClearFault(Command_t *cmd) {
    pmbus_error_t result;

    if (cmd->pmbus_rw) {
        logMessage(LOG_ERROR, "CLEAR_FAULT is a write-only command");
        return;
    }

    result = clearfaults(pdevice.address);
    if (result == PMBUS_OK) {
        logMessage(LOG_INFO, "CLEAR_FAULT executed successfully");
        SendResponse("All faults cleared");
    } else {
        logPmbusError(LOG_ERROR, result, "CLEAR_FAULT");
    }
}

void HandlePhase(Command_t *cmd) {
    uint8_t phaseByte;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "PHASE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getPhase(pdevice.address, &phaseByte);
        if (result == PMBUS_OK) {
            cmd->data[0] = phaseByte;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "PHASE: 0x%02X", phaseByte);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "PHASE Read");
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

        result = setPhase(pdevice.address, phaseByte);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "PHASE Write");
        }
    }
}

void HandleWriteProtect(Command_t *cmd) {
    uint8_t protect_level;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "WRITE_PROTECT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getWriteStatus(pdevice.address, &protect_level);
        if (result == PMBUS_OK) {
            cmd->data[0] = protect_level;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "WRITE_PROTECT status: 0x%02X", protect_level);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "WRITE_PROTECT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for WRITE_PROTECT");
            return;
        }

        protect_level = cmd->data[0];
        result = setWriteStatus(pdevice.address, protect_level, 1);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "WRITE_PROTECT set to: 0x%02X", protect_level);
            logMessage(LOG_INFO, syscheck);
            SendResponse("WRITE_PROTECT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "WRITE_PROTECT Write");
        }
    }
}

void HandleStoreDefaultAll(Command_t *cmd) {
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        logMessage(LOG_ERROR, "STORE_DEFAULT_ALL is a write-only command");
        return;
    }

    result = saveToNVM(pdevice.address);
    if (result == PMBUS_OK) {
        logMessage(LOG_INFO, "STORE_DEFAULT_ALL executed successfully");
        SendResponse("All settings stored as defaults");
    } else {
        logPmbusError(LOG_ERROR, result, "STORE_DEFAULT_ALL");
    }
}


void HandleRestoreDefaultAll(Command_t *cmd) {
    pmbus_error_t result;
    uint8_t writeProtect;

    if (!cmd->pmbus_rw) {
        logMessage(LOG_ERROR, "RESTORE_DEFAULT_ALL is a write-only command");
        return;
    }

    result = getWriteStatus(pdevice.address, &writeProtect);
    if (result != PMBUS_OK) {
        logPmbusError(LOG_ERROR, result, "RESTORE_DEFAULT_ALL (Get Write Status)");
        return;
    }

    if (writeProtect != 0x00) {
        logMessage(LOG_ERROR, "RESTORE_DEFAULT_ALL blocked by WRITE_PROTECT");
        return;
    }

    result = restoreDevice(pdevice.address, 0);
    if (result == PMBUS_OK) {
        logMessage(LOG_INFO, "RESTORE_DEFAULT_ALL executed successfully");
        SendResponse("All settings restored to defaults");
    } else {
        logPmbusError(LOG_ERROR, result, "RESTORE_DEFAULT_ALL");
    }
}

void HandleCapability(Command_t *cmd) {
    uint8_t capabilityByte;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "CAPABILITY handle callback");

    if (cmd->pmbus_rw) {
        logMessage(LOG_WARNING, "CAPABILITY is a read-only command");
        return;
    }

    result = getCap(pdevice.address, &capabilityByte);
    if (result == PMBUS_OK) {
        cmd->data[0] = capabilityByte;
        cmd->length = 1;
        snprintf(syscheck, sizeof(syscheck), "CAPABILITY: 0x%02X\n", capabilityByte);
        SendResponse(syscheck);
        logMessage(LOG_INFO, syscheck);

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
            case 0: logMessage(LOG_INFO, "Maximum supported bus speed is 100 kHz"); break;
            case 1: logMessage(LOG_INFO, "Maximum supported bus speed is 400 kHz"); break;
            case 2: logMessage(LOG_INFO, "Maximum supported bus speed is 1 MHz"); break;
            default: logMessage(LOG_INFO, "Reserved bus speed value");
        }

        if (smbalert) {
            logMessage(LOG_INFO, "Device supports SMBus Alert Response protocol");
        } else {
            logMessage(LOG_INFO, "Device does not support SMBus Alert Response protocol");
        }
    } else {
        logPmbusError(LOG_ERROR, result, "CAPABILITY Read");
        cmd->length = 0;
    }
}

void HandleSmbAlertMask(Command_t *cmd) {
    uint16_t mask_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, 0x1B, &mask_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = mask_value & 0xFF;
            cmd->data[1] = (mask_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "SMBALERT_MASK: 0x%04X", mask_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "SMBALERT_MASK Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for SMBALERT_MASK");
            return;
        }

        mask_value = (cmd->data[1] << 8) | cmd->data[0];
        result = PMBUS_WriteWord(pdevice.address, 0x1B, mask_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "SMBALERT_MASK set to: 0x%04X", mask_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("SMBALERT_MASK executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "SMBALERT_MASK Write");
        }
    }
}

void HandleVoutMode(Command_t *cmd) {
    uint8_t modeByte;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_MODE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getVoutMode(pdevice.address, &modeByte);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "VOUT_MODE Read");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "VOUT_MODE is read-only");
    }
}

void HandleVoutCommand(Command_t *cmd) {
    uint16_t voutValue;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_COMMAND handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getVoutCommand(pdevice.address, &voutValue);
        if (result == PMBUS_OK) {
            cmd->data[0] = voutValue & 0xFF;
            cmd->data[1] = (voutValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_COMMAND: 0x%04X", voutValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_COMMAND Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_COMMAND");
            return;
        }

        voutValue = (cmd->data[1] << 8) | cmd->data[0];
        result = setVoutCommand(pdevice.address, voutValue);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_COMMAND set to: 0x%04X", voutValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_COMMAND executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_COMMAND Write");
        }
    }
}

void HandleVoutMax(Command_t *cmd) {
    uint16_t maxValue;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_MAX handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getVoutMax(pdevice.address, &maxValue);
        if (result == PMBUS_OK) {
            cmd->data[0] = maxValue & 0xFF;
            cmd->data[1] = (maxValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MAX: 0x%04X", maxValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MAX Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MAX");
            return;
        }

        maxValue = (cmd->data[1] << 8) | cmd->data[0];
        result = setVoutMax(pdevice.address, maxValue);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_MAX set to: 0x%04X", maxValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_MAX executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MAX Write");
        }
    }
}

void HandleVoutMarginHigh(Command_t *cmd) {
    uint16_t marginValue;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_MARGIN_HIGH handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getVoutMarginHigh(pdevice.address, &marginValue);
        if (result == PMBUS_OK) {
            cmd->data[0] = marginValue & 0xFF;
            cmd->data[1] = (marginValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_HIGH: 0x%04X", marginValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MARGIN_HIGH Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MARGIN_HIGH");
            return;
        }

        marginValue = (cmd->data[1] << 8) | cmd->data[0];
        result = setVoutMarginHigh(pdevice.address, marginValue);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_HIGH set to: 0x%04X", marginValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_MARGIN_HIGH executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MARGIN_HIGH Write");
        }
    }
}

void HandleVoutMarginLow(Command_t *cmd) {
    uint16_t marginValue;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_MARGIN_LOW handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = getVoutMarginLow(pdevice.address, &marginValue);
        if (result == PMBUS_OK) {
            cmd->data[0] = marginValue & 0xFF;
            cmd->data[1] = (marginValue >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_LOW: 0x%04X", marginValue);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MARGIN_LOW Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MARGIN_LOW");
            return;
        }

        marginValue = (cmd->data[1] << 8) | cmd->data[0];
        result = setVoutMarginLow(pdevice.address, marginValue);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_MARGIN_LOW set to: 0x%04X", marginValue);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_MARGIN_LOW executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MARGIN_LOW Write");
        }
    }
}

void HandleVoutMin(Command_t *cmd) {
    uint16_t vout_min;
    float voltage;
    uint8_t vout_mode;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_MIN handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, 0x2B, &vout_min);
        if (result == PMBUS_OK) {
            result = getVoutMode(pdevice.address, &vout_mode);
            if (result == PMBUS_OK) {
                voltage = ulinear16_to_float(vout_min, vout_mode);
                cmd->data[0] = vout_min & 0xFF;
                cmd->data[1] = (vout_min >> 8) & 0xFF;
                cmd->length = 2;
                snprintf(syscheck, sizeof(syscheck), "VOUT_MIN: 0x%04X (%.3fV)", vout_min, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse(syscheck);
            } else {
                logPmbusError(LOG_ERROR, result, "VOUT_MIN Read (VOUT_MODE)");
                cmd->length = 0;
            }
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MIN Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_MIN");
            return;
        }

        vout_min = (cmd->data[1] << 8) | cmd->data[0];
        result = getVoutMode(pdevice.address, &vout_mode);
        if (result == PMBUS_OK) {
            voltage = ulinear16_to_float(vout_min, vout_mode);
            result = PMBUS_WriteWord(pdevice.address, 0x2B, vout_min);
            if (result == PMBUS_OK) {
                snprintf(syscheck, sizeof(syscheck), "VOUT_MIN set to: 0x%04X (%.3fV)", vout_min, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse("VOUT_MIN executed successfully");
            } else {
                logPmbusError(LOG_ERROR, result, "VOUT_MIN Write");
            }
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_MIN Write (VOUT_MODE)");
        }
    }
}

void HandleVoutTransitionRate(Command_t *cmd) {
    uint16_t rate_value;
    float rate_mv_us;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_TRANSITION_RATE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VOUT_TRANSITION_RATE, &rate_value);
        if (result == PMBUS_OK) {
            rate_mv_us = slinear11_to_float(rate_value);
            cmd->data[0] = rate_value & 0xFF;
            cmd->data[1] = (rate_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_TRANSITION_RATE: 0x%04X (%.3f mV/µs)", rate_value, rate_mv_us);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_TRANSITION_RATE Read");
            cmd->length = 0;
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

        result = PMBUS_WriteWord(pdevice.address, VOUT_TRANSITION_RATE, rate_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_TRANSITION_RATE set to: 0x%04X (%.3f mV/µs)", rate_value, rate_mv_us);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_TRANSITION_RATE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_TRANSITION_RATE Write");
        }
    }
}

void HandleVoutDroop(Command_t *cmd) {
    uint16_t droop_value;
    float droop_mv_a;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_DROOP handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VOUT_DROOP, &droop_value);
        if (result == PMBUS_OK) {
            droop_mv_a = slinear11_to_float(droop_value);
            cmd->data[0] = droop_value & 0xFF;
            cmd->data[1] = (droop_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_DROOP: 0x%04X (%.3f mV/A)", droop_value, droop_mv_a);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_DROOP Read");
            cmd->length = 0;
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

        result = PMBUS_WriteWord(pdevice.address, VOUT_DROOP, droop_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_DROOP set to: 0x%04X (%.3f mV/A)", droop_value, droop_mv_a);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_DROOP executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_DROOP Write");
        }
    }
}

void HandleVoutScaleLoop(Command_t *cmd) {
    uint16_t scale_value;
    float scale_factor;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_SCALE_LOOP handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VOUT_SCALE_LOOP, &scale_value);
        if (result == PMBUS_OK) {
            scale_factor = slinear11_to_float(scale_value);
            cmd->data[0] = scale_value & 0xFF;
            cmd->data[1] = (scale_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_LOOP: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_SCALE_LOOP Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_SCALE_LOOP");
            return;
        }

        scale_value = (cmd->data[1] << 8) | cmd->data[0];
        scale_factor = slinear11_to_float(scale_value);

        result = PMBUS_WriteWord(pdevice.address, VOUT_SCALE_LOOP, scale_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_LOOP set to: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_SCALE_LOOP executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_SCALE_LOOP Write");
        }
    }
}

void HandleVoutScaleMonitor(Command_t *cmd) {
    uint16_t scale_value;
    float scale_factor;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_SCALE_MONITOR handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VOUT_SCALE_MONITOR, &scale_value);
        if (result == PMBUS_OK) {
            scale_factor = slinear11_to_float(scale_value);
            cmd->data[0] = scale_value & 0xFF;
            cmd->data[1] = (scale_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_MONITOR: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_DEBUG, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_SCALE_MONITOR Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_SCALE_MONITOR");
            return;
        }

        scale_value = (cmd->data[1] << 8) | cmd->data[0];
        scale_factor = slinear11_to_float(scale_value);

        result = PMBUS_WriteWord(pdevice.address, VOUT_SCALE_MONITOR, scale_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_SCALE_MONITOR set to: 0x%04X (%.3f)", scale_value, scale_factor);
            logMessage(LOG_DEBUG, syscheck);
            SendResponse("VOUT_SCALE_MONITOR executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_SCALE_MONITOR Write");
        }
    }
}

void HandleFrequencySwitch(Command_t *cmd) {
    uint16_t reg_val;
    float freq;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "FREQUENCY_SWITCH handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, FREQUENCY_SWITCH, &reg_val);
        if (result == PMBUS_OK) {
            cmd->data[0] = reg_val & 0xFF;
            cmd->data[1] = (reg_val >> 8) & 0xFF;
            cmd->length = 2;

            snprintf(syscheck, sizeof(syscheck), "Raw frequency: 0x%04X", reg_val);
            logMessage(LOG_DEBUG, syscheck);

            freq = slinear11_to_float(reg_val);
            snprintf(syscheck, sizeof(syscheck), "Switch Frequency: %.2f kHz", freq);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "FREQUENCY_SWITCH Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for FREQUENCY_SWITCH");
            return;
        }

        reg_val = (cmd->data[1] << 8) | cmd->data[0];
        freq = slinear11_to_float(reg_val);

        // Check if the frequency is within the acceptable range
        if (freq < 300 || freq > 1000) { // change this for various VR controllers
            logMessage(LOG_ERROR, "FREQUENCY_SWITCH value out of range (300-1000 kHz)");
            return;
        }

        snprintf(syscheck, sizeof(syscheck), "Data to be written: 0x%04X (%.2f kHz)", reg_val, freq);
        logMessage(LOG_DEBUG, syscheck);

        result = PMBUS_WriteWord(pdevice.address, FREQUENCY_SWITCH, reg_val);
        if (result == PMBUS_OK) {
            logMessage(LOG_INFO, "Frequency set successfully");
            SendResponse("FREQUENCY_SWITCH command executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "FREQUENCY_SWITCH Write");
        }
    }
}

void HandleVinOn(Command_t *cmd) {
    uint16_t vinOn;
    float voltage;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VIN_ON handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VIN_ON, &vinOn);
        if (result == PMBUS_OK) {
            cmd->data[0] = vinOn & 0xFF;
            cmd->data[1] = (vinOn >> 8) & 0xFF;
            cmd->length = 2;
            voltage = slinear11_to_float(vinOn);
            snprintf(syscheck, sizeof(syscheck), "VIN_ON: 0x%04X (%.2f V)", vinOn, voltage);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_ON Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_ON");
            return;
        }

        vinOn = (cmd->data[1] << 8) | cmd->data[0];
        voltage = slinear11_to_float(vinOn);

        if (voltage < 4.0 || voltage > 11.25) {
            logMessage(LOG_ERROR, "VIN_ON value out of range");
            return;
        }

        result = PMBUS_WriteWord(pdevice.address, VIN_ON, vinOn);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VIN_ON set to: 0x%04X (%.2f V)", vinOn, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_ON command executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_ON Write");
        }
    }
}

void HandleIoutCalGain(Command_t *cmd) {
    uint16_t ioutCalGain;
    float gain;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IOUT_CAL_GAIN handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, IOUT_CAL_GAIN, &ioutCalGain);
        if (result == PMBUS_OK) {
            cmd->data[0] = ioutCalGain & 0xFF;
            cmd->data[1] = (ioutCalGain >> 8) & 0xFF;
            cmd->length = 2;
            gain = slinear11_to_float(ioutCalGain);
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_GAIN: 0x%04X (%.6f mΩ)", ioutCalGain, gain * 1000);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_CAL_GAIN Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_CAL_GAIN");
            return;
        }

        ioutCalGain = (cmd->data[1] << 8) | cmd->data[0];
        gain = slinear11_to_float(ioutCalGain);

        if (gain < 4.765625 || gain > 5.25) {
            logMessage(LOG_ERROR, "IOUT_CAL_GAIN value out of range");
            return;
        }

        result = PMBUS_WriteWord(pdevice.address, IOUT_CAL_GAIN, ioutCalGain);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_GAIN set to: 0x%04X (%.6f mΩ)", ioutCalGain, gain * 1000);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_CAL_GAIN command executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_CAL_GAIN Write");
        }
    }
}

void HandleIoutCalOffset(Command_t *cmd) {
    uint16_t ioutCalOffset;
    float offset;
    uint8_t phase;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IOUT_CAL_OFFSET handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, IOUT_CAL_OFFSET, &ioutCalOffset);
        if (result == PMBUS_OK) {
            cmd->data[0] = ioutCalOffset & 0xFF;
            cmd->data[1] = (ioutCalOffset >> 8) & 0xFF;
            cmd->length = 2;
            offset = slinear11_to_float(ioutCalOffset);
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_OFFSET: 0x%04X (%.3f A)", ioutCalOffset, offset);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_CAL_OFFSET Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_CAL_OFFSET");
            return;
        }

        ioutCalOffset = (cmd->data[1] << 8) | cmd->data[0];
        offset = slinear11_to_float(ioutCalOffset);
        result = PMBUS_ReadByte(pdevice.address, PHASE, &phase);
        if (result != PMBUS_OK) {
            logPmbusError(LOG_ERROR, result, "IOUT_CAL_OFFSET (PHASE Read)");
            return;
        }

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

        result = PMBUS_WriteWord(pdevice.address, IOUT_CAL_OFFSET, ioutCalOffset);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_CAL_OFFSET set to: 0x%04X (%.3f A)", ioutCalOffset, offset);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_CAL_OFFSET command executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_CAL_OFFSET Write");
        }
    }
}

void HandleVoutOvFaultLimit(Command_t *cmd) {
    uint16_t voutOvFaultLimit;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_OV_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VOUT_OV_FAULT_LIMIT, &voutOvFaultLimit);
        if (result == PMBUS_OK) {
            cmd->data[0] = voutOvFaultLimit & 0xFF;
            cmd->data[1] = (voutOvFaultLimit >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "VOUT_OV_FAULT_LIMIT: 0x%04X", voutOvFaultLimit);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_OV_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "VOUT_OV_FAULT_LIMIT is read-only");
    }
}

void HandleVoutOvFaultResponse(Command_t *cmd) {
    uint8_t voutOvFaultResponse;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_OV_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, VOUT_OV_FAULT_RESPONSE, &voutOvFaultResponse);
        if (result == PMBUS_OK) {
            cmd->data[0] = voutOvFaultResponse;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VOUT_OV_FAULT_RESPONSE: 0x%02X", voutOvFaultResponse);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_OV_FAULT_RESPONSE Read");
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
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_UV_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VOUT_UV_FAULT_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            result = getVoutMode(pdevice.address, &vout_mode);
            if (result == PMBUS_OK) {
                voltage = ulinear16_to_float(limit_value, vout_mode);
                cmd->data[0] = limit_value & 0xFF;
                cmd->data[1] = (limit_value >> 8) & 0xFF;
                cmd->length = 2;
                snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_LIMIT: 0x%04X (%.3fV)", limit_value, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse(syscheck);
            } else {
                logPmbusError(LOG_ERROR, result, "VOUT_UV_FAULT_LIMIT (VOUT_MODE Read)");
                cmd->length = 0;
            }
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_UV_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_UV_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        result = getVoutMode(pdevice.address, &vout_mode);
        if (result == PMBUS_OK) {
            voltage = ulinear16_to_float(limit_value, vout_mode);
            result = PMBUS_WriteWord(pdevice.address, VOUT_UV_FAULT_LIMIT, limit_value);
            if (result == PMBUS_OK) {
                snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_LIMIT set to: 0x%04X (%.3fV)", limit_value, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse("VOUT_UV_FAULT_LIMIT executed successfully");
            } else {
                logPmbusError(LOG_ERROR, result, "VOUT_UV_FAULT_LIMIT Write");
            }
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_UV_FAULT_LIMIT (VOUT_MODE Read)");
        }
    }
}

void HandleVoutUvFaultResponse(Command_t *cmd) {
    uint8_t response_value;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VOUT_UV_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, VOUT_UV_FAULT_RESPONSE, &response_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_UV_FAULT_RESPONSE Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for VOUT_UV_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        result = PMBUS_WriteByte(pdevice.address, VOUT_UV_FAULT_RESPONSE, response_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VOUT_UV_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VOUT_UV_FAULT_RESPONSE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VOUT_UV_FAULT_RESPONSE Write");
        }
    }
}

void HandleIoutOcFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IOUT_OC_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, IOUT_OC_FAULT_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_OC_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_OC_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, IOUT_OC_FAULT_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_OC_FAULT_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_OC_FAULT_LIMIT Write");
        }
    }
}

void HandleIoutOcWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IOUT_OC_WARN_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, IOUT_OC_WARN_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_WARN_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_OC_WARN_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_OC_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, IOUT_OC_WARN_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_WARN_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_OC_WARN_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_OC_WARN_LIMIT Write");
        }
    }
}

void HandleIoutOcFaultResponse(Command_t *cmd) {
    uint8_t response_value;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IOUT_OC_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, OT_FAULT_RESPONSE, &response_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_OC_FAULT_RESPONSE Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for IOUT_OC_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        result = PMBUS_WriteByte(pdevice.address, OT_FAULT_RESPONSE, response_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IOUT_OC_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IOUT_OC_FAULT_RESPONSE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IOUT_OC_FAULT_RESPONSE Write");
        }
    }
}

void HandleOtFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float temperature;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "OT_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, OT_FAULT_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            temperature = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_LIMIT: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "OT_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for OT_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        temperature = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, OT_FAULT_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_LIMIT set to: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse("OT_FAULT_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "OT_FAULT_LIMIT Write");
        }
    }
}

void HandleOtFaultResponse(Command_t *cmd) {
    uint8_t response_value;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "OT_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, OT_FAULT_RESPONSE, &response_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "OT_FAULT_RESPONSE Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for OT_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        result = PMBUS_WriteByte(pdevice.address, OT_FAULT_RESPONSE, response_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "OT_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("OT_FAULT_RESPONSE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "OT_FAULT_RESPONSE Write");
        }
    }
}

void HandleOtWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float temperature;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "OT_WARN_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, OT_WARN_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            temperature = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "OT_WARN_LIMIT: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "OT_WARN_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for OT_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        temperature = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, OT_WARN_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "OT_WARN_LIMIT set to: 0x%04X (%.2f°C)", limit_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse("OT_WARN_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "OT_WARN_LIMIT Write");
        }
    }
}

void HandleVinOvFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float voltage;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VIN_OV_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VIN_OV_FAULT_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            voltage = slinear11_to_float(limit_value);

            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;

            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_LIMIT: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_OV_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_OV_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];

        voltage = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, VIN_OV_FAULT_LIMIT, limit_value);

        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_LIMIT set to: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_OV_FAULT_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_OV_FAULT_LIMIT Write");
        }
    }
}


void HandleVinOvFaultResponse(Command_t *cmd) {
    uint8_t response_value;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VIN_OV_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, VIN_OV_FAULT_RESPONSE, &response_value);
        if (result == PMBUS_OK) {

            cmd->data[0] = response_value;
            cmd->length = 1;

            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_OV_FAULT_RESPONSE Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_OV_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        result = PMBUS_WriteByte(pdevice.address, VIN_OV_FAULT_RESPONSE, response_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VIN_OV_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_OV_FAULT_RESPONSE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_OV_FAULT_RESPONSE Write");
        }
    }
}

void HandleVinUvFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float voltage;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VIN_UV_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, VIN_UV_FAULT_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            voltage = slinear11_to_float(limit_value);

            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;

            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_LIMIT: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_UV_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_UV_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        voltage = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, VIN_UV_FAULT_LIMIT, limit_value);

        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_LIMIT set to: 0x%04X (%.3fV)", limit_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_UV_FAULT_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_UV_FAULT_LIMIT Write");
        }
    }
}

void HandleVinUvFaultResponse(Command_t *cmd) {
    uint8_t response_value;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "VIN_UV_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, VIN_UV_FAULT_RESPONSE, &response_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_UV_FAULT_RESPONSE Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for VIN_UV_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        result = PMBUS_WriteByte(pdevice.address, VIN_UV_FAULT_RESPONSE, response_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "VIN_UV_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("VIN_UV_FAULT_RESPONSE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "VIN_UV_FAULT_RESPONSE Write");
        }
    }
}

void HandleIinOcFaultLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IIN_OC_FAULT_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, IIN_OC_FAULT_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IIN_OC_FAULT_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IIN_OC_FAULT_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, IIN_OC_FAULT_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IIN_OC_FAULT_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IIN_OC_FAULT_LIMIT Write");
        }
    }
}

void HandleIinOcFaultResponse(Command_t *cmd) {
    uint8_t response_value;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IIN_OC_FAULT_RESPONSE handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadByte(pdevice.address, IIN_OC_FAULT_RESPONSE, &response_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = response_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_RESPONSE: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IIN_OC_FAULT_RESPONSE Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for IIN_OC_FAULT_RESPONSE");
            return;
        }

        response_value = cmd->data[0];
        result = PMBUS_WriteByte(pdevice.address, IIN_OC_FAULT_RESPONSE, response_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_FAULT_RESPONSE set to: 0x%02X", response_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IIN_OC_FAULT_RESPONSE executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IIN_OC_FAULT_RESPONSE Write");
        }
    }
}

void HandleIinOcWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float current;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "IIN_OC_WARN_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, IIN_OC_WARN_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            current = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_WARN_LIMIT: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IIN_OC_WARN_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for IIN_OC_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        current = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, IIN_OC_WARN_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "IIN_OC_WARN_LIMIT set to: 0x%04X (%.3fA)", limit_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse("IIN_OC_WARN_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "IIN_OC_WARN_LIMIT Write");
        }
    }
}

void HandleTonDelay(Command_t *cmd) {
    uint16_t delay_value;
    float delay_ms;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "TON_DELAY handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, TON_DELAY, &delay_value);
        if (result == PMBUS_OK) {
            delay_ms = slinear11_to_float(delay_value) * 1000; // Convert to milliseconds
            cmd->data[0] = delay_value & 0xFF;
            cmd->data[1] = (delay_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "TON_DELAY: 0x%04X (%.2f ms)", delay_value, delay_ms);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "TON_DELAY Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for TON_DELAY");
            return;
        }

        delay_value = (cmd->data[1] << 8) | cmd->data[0];
        delay_ms = slinear11_to_float(delay_value) * 1000; // Convert to milliseconds
        result = PMBUS_WriteWord(pdevice.address, TON_DELAY, delay_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "TON_DELAY set to: 0x%04X (%.2f ms)", delay_value, delay_ms);
            logMessage(LOG_INFO, syscheck);
            SendResponse("TON_DELAY executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "TON_DELAY Write");
        }
    }
}

void HandlePinOpWarnLimit(Command_t *cmd) {
    uint16_t limit_value;
    float power;
    pmbus_error_t result;

    logMessage(LOG_DEBUG, "PIN_OP_WARN_LIMIT handle callback");

    if (!cmd->pmbus_rw) {
        // Read operation
        result = PMBUS_ReadWord(pdevice.address, PIN_OP_WARN_LIMIT, &limit_value);
        if (result == PMBUS_OK) {
            power = slinear11_to_float(limit_value);
            cmd->data[0] = limit_value & 0xFF;
            cmd->data[1] = (limit_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "PIN_OP_WARN_LIMIT: 0x%04X (%.3f W)", limit_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "PIN_OP_WARN_LIMIT Read");
            cmd->length = 0;
        }
    } else {
        // Write operation
        if (cmd->length != 2) {
            logMessage(LOG_ERROR, "Invalid data length for PIN_OP_WARN_LIMIT");
            return;
        }

        limit_value = (cmd->data[1] << 8) | cmd->data[0];
        power = slinear11_to_float(limit_value);
        result = PMBUS_WriteWord(pdevice.address, PIN_OP_WARN_LIMIT, limit_value);
        if (result == PMBUS_OK) {
            snprintf(syscheck, sizeof(syscheck), "PIN_OP_WARN_LIMIT set to: 0x%04X (%.3f W)", limit_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse("PIN_OP_WARN_LIMIT executed successfully");
        } else {
            logPmbusError(LOG_ERROR, result, "PIN_OP_WARN_LIMIT Write");
        }
    }
}

void HandleStatusByte(Command_t *cmd) {
    uint8_t statusByte;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_BYTE handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusByte(pdevice.address, &statusByte);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "STATUS_BYTE Read");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "STATUS_BYTE is read-only");
    }
}

void HandleStatusWord(Command_t *cmd) {
    uint16_t statusWord;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_WORD handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusWord(pdevice.address, &statusWord);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "STATUS_WORD Read");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "STATUS_WORD is read-only");
    }
}

void HandleStatusMfrSpecific(Command_t *cmd) {
    uint8_t statusMfrSpecific;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_MFR_SPECIFIC handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusMfrSpecific(pdevice.address, &statusMfrSpecific);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "STATUS_MFR_SPECIFIC Read");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_MFR_SPECIFIC");
            return;
        }
        result = PMBUS_WriteByte(pdevice.address, STATUS_MFR_SPECIFIC, cmd->data[0]);
        if (result == PMBUS_OK) {
            logMessage(LOG_INFO, "STATUS_MFR_SPECIFIC bits cleared");
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_MFR_SPECIFIC Write");
        }
    }
}

void HandleStatusCml(Command_t *cmd) {
    uint8_t statusCml;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_CML handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusCml(pdevice.address, &statusCml);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "STATUS_CML Read");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_CML");
            return;
        }
        result = PMBUS_WriteByte(pdevice.address, STATUS_CML, cmd->data[0]);
        if (result == PMBUS_OK) {
            logMessage(LOG_INFO, "STATUS_CML bits cleared");
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_CML Write");
        }
    }
}

void HandleStatusTemperature(Command_t *cmd) {
    uint8_t statusTemperature;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_TEMPERATURE handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusTemperature(pdevice.address, &statusTemperature);
        if (result == PMBUS_OK) {
            cmd->data[0] = statusTemperature;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_TEMPERATURE: 0x%02X", statusTemperature);
            SendResponse(syscheck);
            logMessage(LOG_INFO, syscheck);

            // Parse STATUS_TEMPERATURE
            if (statusTemperature & 0x80) logMessage(LOG_DEBUG, "Over-Temperature Fault");
            if (statusTemperature & 0x40) logMessage(LOG_DEBUG, "Over-Temperature Warning");
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_TEMPERATURE Read");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_TEMPERATURE");
            return;
        }
        result = PMBUS_WriteByte(pdevice.address, STATUS_TEMPERATURE, cmd->data[0]);
        if (result == PMBUS_OK) {
            logMessage(LOG_INFO, "STATUS_TEMPERATURE bits cleared");
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_TEMPERATURE Write");
        }
    }
}

void HandleStatusInput(Command_t *cmd) {
    uint8_t statusInput;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_INPUT handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusInput(pdevice.address, &statusInput);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "STATUS_INPUT Read");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_INPUT");
            return;
        }
        result = PMBUS_WriteByte(pdevice.address, STATUS_INPUT, cmd->data[0]);
        if (result == PMBUS_OK) {
            logMessage(LOG_INFO, "STATUS_INPUT bits cleared");
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_INPUT Write");
        }
    }
}

void HandleStatusIout(Command_t *cmd) {
    uint8_t statusIout;
    pmbus_error_t result;
    logMessage(LOG_DEBUG, "STATUS_IOUT handle callback");

    if (!cmd->pmbus_rw) {
        result = getStatusIout(pdevice.address, &statusIout);
        if (result == PMBUS_OK) {
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
            logPmbusError(LOG_ERROR, result, "STATUS_IOUT Read");
            cmd->length = 0;
        }
    } else {
        // Handle write operation (clearing status bits)
        if (cmd->length != 1) {
            logMessage(LOG_ERROR, "Invalid data length for STATUS_IOUT");
            return;
        }
        result = PMBUS_WriteByte(pdevice.address, STATUS_IOUT, cmd->data[0]);
        if (result == PMBUS_OK) {
            logMessage(LOG_INFO, "STATUS_IOUT bits cleared");
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_IOUT Write");
        }
    }
}

void HandleStatusVout(Command_t *cmd) {
    uint8_t status_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadByte(pdevice.address, STATUS_VOUT, &status_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = status_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "STATUS_VOUT: 0x%02X", status_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "STATUS_VOUT Read");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "STATUS_VOUT is a read-only command");
    }
}

void HandleReadVin(Command_t *cmd) {
    uint16_t vin_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_VIN, &vin_value);
        if (result == PMBUS_OK) {
            float voltage = slinear11_to_float(vin_value);
            cmd->data[0] = vin_value & 0xFF;
            cmd->data[1] = (vin_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_VIN: 0x%04X (%.3f V)", vin_value, voltage);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "READ_VIN");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_VIN is a read-only command");
    }
}

void HandleReadIin(Command_t *cmd) {
    uint16_t iin_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_IIN, &iin_value);
        if (result == PMBUS_OK) {
            float current = slinear11_to_float(iin_value);
            cmd->data[0] = iin_value & 0xFF;
            cmd->data[1] = (iin_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_IIN: 0x%04X (%.3f A)", iin_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "READ_IIN");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_IIN is a read-only command");
    }
}

void HandleReadVout(Command_t *cmd) {
    uint16_t vout_value;
    uint8_t vout_mode;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_VOUT, &vout_value);
        if (result == PMBUS_OK) {
            result = getVoutMode(pdevice.address, &vout_mode);
            if (result == PMBUS_OK) {
                float voltage = ulinear16_to_float(vout_value, vout_mode);
                cmd->data[0] = vout_value & 0xFF;
                cmd->data[1] = (vout_value >> 8) & 0xFF;
                cmd->length = 2;
                snprintf(syscheck, sizeof(syscheck), "READ_VOUT: 0x%04X (%.3f V)", vout_value, voltage);
                logMessage(LOG_INFO, syscheck);
                SendResponse(syscheck);
            } else {
                logPmbusError(LOG_ERROR, result, "VOUT_MODE Read");
                cmd->length = 0;
            }
        } else {
            logPmbusError(LOG_ERROR, result, "READ_VOUT");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_VOUT is a read-only command");
    }
}

void HandleReadIout(Command_t *cmd) {
    uint16_t iout_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_IOUT, &iout_value);
        if (result == PMBUS_OK) {
            float current = slinear11_to_float(iout_value);
            cmd->data[0] = iout_value & 0xFF;
            cmd->data[1] = (iout_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_IOUT: 0x%04X (%.3f A)", iout_value, current);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "READ_IOUT");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_IOUT is a read-only command");
    }
}

void HandleReadTemperature1(Command_t *cmd) {
    uint16_t temp_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_TEMPERATURE_1, &temp_value);
        if (result == PMBUS_OK) {
            float temperature = slinear11_to_float(temp_value);
            cmd->data[0] = temp_value & 0xFF;
            cmd->data[1] = (temp_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_TEMPERATURE_1: 0x%04X (%.2f °C)", temp_value, temperature);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "READ_TEMPERATURE_1");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_TEMPERATURE_1 is a read-only command");
    }
}

void HandleReadPout(Command_t *cmd) {
    uint16_t pout_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_POUT, &pout_value);
        if (result == PMBUS_OK) {
            float power = slinear11_to_float(pout_value);
            cmd->data[0] = pout_value & 0xFF;
            cmd->data[1] = (pout_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_POUT: 0x%04X (%.3f W)", pout_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "READ_POUT");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_POUT is a read-only command");
    }
}

void HandleReadPin(Command_t *cmd) {
    uint16_t pin_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, READ_PIN, &pin_value);
        if (result == PMBUS_OK) {
            float power = slinear11_to_float(pin_value);
            cmd->data[0] = pin_value & 0xFF;
            cmd->data[1] = (pin_value >> 8) & 0xFF;
            cmd->length = 2;
            snprintf(syscheck, sizeof(syscheck), "READ_PIN: 0x%04X (%.3f W)", pin_value, power);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "READ_PIN");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "READ_PIN is a read-only command");
    }
}

void HandlePmbusRevision(Command_t *cmd) {
    uint8_t revision_value;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadByte(pdevice.address, PMBUS_REVISION, &revision_value);
        if (result == PMBUS_OK) {
            cmd->data[0] = revision_value;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "PMBUS_REVISION: 0x%02X", revision_value);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "PMBUS_REVISION");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "PMBUS_REVISION is a read-only command");
    }
}

void HandleMfrId(Command_t *cmd) {
    uint8_t mfr_id[32];
    uint8_t length;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_BlockRead(pdevice.address, MFR_ID, mfr_id, &length);
        if (result == PMBUS_OK) {
            memcpy(cmd->data, mfr_id, length);
            cmd->length = length;
            mfr_id[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_ID: %s", mfr_id);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "MFR_ID");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "MFR_ID is a read-only command");
    }
}

void HandleMfrModel(Command_t *cmd) {
    uint8_t mfr_model[32];
    uint8_t length;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_BlockRead(pdevice.address, MFR_MODEL, mfr_model, &length);
        if (result == PMBUS_OK) {
            memcpy(cmd->data, mfr_model, length);
            cmd->length = length;
            mfr_model[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_MODEL: %s", mfr_model);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "MFR_MODEL");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "MFR_MODEL is a read-only command");
    }
}

void HandleMfrRevision(Command_t *cmd) {
    uint8_t mfr_revision[32];
    uint8_t length;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_BlockRead(pdevice.address, MFR_REVISION, mfr_revision, &length);
        if (result == PMBUS_OK) {
            memcpy(cmd->data, mfr_revision, length);
            cmd->length = length;
            mfr_revision[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_REVISION: %s", mfr_revision);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "MFR_REVISION");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "MFR_REVISION is a read-only command");
    }
}

void HandleMfrDate(Command_t *cmd) {
    uint8_t mfr_date[32];
    uint8_t length;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_BlockRead(pdevice.address, MFR_DATE, mfr_date, &length);
        if (result == PMBUS_OK) {
            memcpy(cmd->data, mfr_date, length);
            cmd->length = length;
            mfr_date[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_DATE: %s", mfr_date);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "MFR_DATE");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "MFR_DATE is a read-only command");
    }
}

void HandleMfrSerial(Command_t *cmd) {
    uint8_t mfr_serial[32];
    uint8_t length;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_BlockRead(pdevice.address, MFR_SERIAL, mfr_serial, &length);
        if (result == PMBUS_OK) {
            memcpy(cmd->data, mfr_serial, length);
            cmd->length = length;
            mfr_serial[length] = '\0';  // Null-terminate the string
            snprintf(syscheck, sizeof(syscheck), "MFR_SERIAL: %s", mfr_serial);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "MFR_SERIAL");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "MFR_SERIAL is a read-only command");
    }
}

void HandleIcDeviceId(Command_t *cmd) {
    uint32_t device_id;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadWord(pdevice.address, IC_DEVICE_ID, (uint16_t*)&device_id);
        if (result == PMBUS_OK) {
            result = PMBUS_ReadWord(pdevice.address, IC_DEVICE_ID + 1, (uint16_t*)(&device_id + 2));
            if (result == PMBUS_OK) {
                cmd->data[0] = device_id & 0xFF;
                cmd->data[1] = (device_id >> 8) & 0xFF;
                cmd->data[2] = (device_id >> 16) & 0xFF;
                cmd->data[3] = (device_id >> 24) & 0xFF;
                cmd->length = 4;
                snprintf(syscheck, sizeof(syscheck), "IC_DEVICE_ID: 0x%08X", (unsigned int)device_id);
                logMessage(LOG_INFO, syscheck);
                SendResponse(syscheck);
            } else {
                logPmbusError(LOG_ERROR, result, "IC_DEVICE_ID (high word)");
                cmd->length = 0;
            }
        } else {
            logPmbusError(LOG_ERROR, result, "IC_DEVICE_ID (low word)");
            cmd->length = 0;
        }
    } else {
        logMessage(LOG_ERROR, "IC_DEVICE_ID is a read-only command");
    }
}

void HandleIcDeviceRev(Command_t *cmd) {
    uint8_t device_rev;
    pmbus_error_t result;

    if (!cmd->pmbus_rw) {
        result = PMBUS_ReadByte(pdevice.address, IC_DEVICE_REV, &device_rev);
        if (result == PMBUS_OK) {
            cmd->data[0] = device_rev;
            cmd->length = 1;
            snprintf(syscheck, sizeof(syscheck), "IC_DEVICE_REV: 0x%02X", device_rev);
            logMessage(LOG_INFO, syscheck);
            SendResponse(syscheck);
        } else {
            logPmbusError(LOG_ERROR, result, "IC_DEVICE_REV");
            cmd->length = 0;
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


