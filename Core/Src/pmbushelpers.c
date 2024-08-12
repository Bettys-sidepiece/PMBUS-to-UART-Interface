/*
 * pmbushelpers.c
 *
 *  Created on: Aug 8, 2024
 *      Author: nx024656
 */

#include "smbus.h"
#include "pmbuscmd.h"


const uint8_t crc_table[256] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
        0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
        0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
        0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
        0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
        0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
        0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
        0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
        0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
        0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
        0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
        0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
        0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
        0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
        0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
        0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
    };
//---------------------------------------------------------------PMBUS SUPPORT Functions------------------------------------------------------------------//

void decimalToWord(uint16_t decimal, uint16_t* buffer) {
    uint8_t highByte = (decimal >> 8) & 0xFF;  // Extract high byte
    uint8_t lowByte = decimal & 0xFF;           // Extract low byte

    buffer[1] = highByte;
    buffer[2] = lowByte;
}

void intToHex(uint32_t number, uint8_t* buffer)
{
    for (int i = 0; i < 24; i++)
    {
        buffer[i] = (number >> (i * 8)) & 0xFF;
    }
}

uint16_t wordToDecimal(const uint16_t* buffer) {
    uint8_t highByte = buffer[1];
    uint8_t lowByte = buffer[2];

    uint16_t decimal = ((uint16_t)highByte << 8) | lowByte;

    return decimal;
}

uint32_t bytesToInt32(const uint8_t* buffer) {
    uint32_t result = 0;

    for (int i = 24; i > 0; i--) {
        result |= ((uint32_t)buffer[i]) << (8 * (23 - i));
    }

    return result;
}

//Maps 5 bit linear exponent to LSB value (2^(twos complement of index))
const float LUT_linear_exponents[32] = {
1.0,2.0,4.0,8.0,16.0,32.0,64.0,128.0,256.0,512.0,1024.0,2048.0,4096.0,8192.0,
16384.0,32768.0,0.0000152587890625,0.000030517578125,0.00006103515625,
0.0001220703125,0.000244140625,0.00048828125,0.0009765625,0.001953125,0.00390625,
0.0078125,0.015625,0.03125,0.0625,0.125,0.25,0.5
};

unsigned int float_to_slinear11(float number, signed int exponent)
{
	signed int mantissa;
	float lsb;
	//Decode the exponent and generate twos complement form
	if(exponent < 0) {
		lsb = LUT_linear_exponents[(exponent+32)];
	} else {
		lsb = LUT_linear_exponents[exponent];
	}
	//Decode mantissa based on exponent and generate twos complement form
	mantissa = (signed int)(number / lsb);
	//If numbers are negative, de-sign-extend to 5/11 bit numbers
	mantissa &= 0x07FF;
	exponent &= 0x1F;

	return (mantissa | (exponent << 11));
}

float slinear11_to_float(unsigned int number)
{
	unsigned int exponent;
	int mantissa;
	float lsb;
	exponent = number >> 11;
	mantissa = number & 0x07FF;
	//Sign extend Mantissa to 32 bits (use your int size here)
	if (mantissa > 0x03FF) {
		mantissa |= 0xFFFFF800;
	}
	lsb = LUT_linear_exponents[exponent];
	return ((float)mantissa)*lsb;
}

unsigned int float_to_ulinear16(float number, unsigned char vout_mode)
{
	float lsb;
	lsb = LUT_linear_exponents[(vout_mode & 0x1F)];
	return (unsigned int)(number/lsb);
}

float ulinear16_to_float(unsigned int number, unsigned char vout_mode)
{
	float lsb;
	lsb = LUT_linear_exponents[(vout_mode & 0x1F)];
	return ((float)number)*lsb;
}
//---------------------------------------------------------------------------PMBUS READ/WRITES-------------------------------------------------------------------------//

// Clear all faults on the device
uint8_t clearfaults(uint8_t devAddress) {
    return (PMBUS_SendByte(devAddress, CLEAR_FAULT));
}

// Get the current write protection status
uint8_t getWriteStatus(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, WRITE_PROTECT, buffer);
}

// Set the write protection status
uint8_t setWriteStatus(uint8_t devAddress, uint8_t data, uint8_t status) {
    // Validate the data
    if(data != 0x80 && data != 0x40 && data != 0x20 && data != 0x00) {
        return 0; // Invalid data
    }

    if(!PMBUS_WriteByte(devAddress, WRITE_PROTECT, data)) {
        return 0;
    }
    return 1; // Success
}

// Save current configuration to Non-Volatile Memory
uint8_t saveToNVM(uint8_t devAddress) {
    if(!(PMBUS_SendByte(devAddress, STORE_DEFAULT_ALL))) {
        return PMBUS_SendByte(devAddress, 0x15); // Try STORE_USER_ALL if STORE_DEFAULT_ALL fails
    }
    return 1;
}

// Restore device configuration
uint8_t restoreDevice(uint8_t devAddress, uint8_t status) {
    uint8_t buf[10];
    uint8_t PSW_Status;
    if(!getOpStatus(devAddress, buf)) return 0;

    PSW_Status = buf[0];

    if(!status && !((PSW_Status & 0x80))) {
        if(!PMBUS_SendByte(devAddress, RESTORE_DEFAULT_ALL)) {
            return PMBUS_SendByte(devAddress, 0x16); // Try RESTORE_USER_ALL if RESTORE_DEFAULT_ALL fails
        }
    }
    return 1;
}

// Read the device ID
uint8_t readDeviceID(uint8_t devAddress, uint8_t* buffer) {
    uint8_t count;

    // Perform a Block Read transaction
    if (!PMBUS_BlockRead(devAddress, IC_DEVICE_ID, buffer, &count)) return 0;  // Read failed

    // Check if the count is valid
    if (count == 0 || count > 32) return 0;  // Invalid count
    return 1;  // Read successful
}

// Get the PMBus revision supported by the device
uint8_t PMBUSRev(uint8_t devAddress, uint8_t* buffer) {
    // Perform a Read Byte transaction
    if (!PMBUS_ReadByte(devAddress, PMBUS_REVISION, buffer)) return 0;  // Read failed

    // Extract Part 1 (bits 7:4) and Part 2 (bits 3:0)
    buffer[1] = (*buffer >> 4) & 0x0F;  // Part 1
    buffer[2] = *buffer & 0x0F;         // Part 2

    return 1;  // Read successful
}

// Get the device capabilities
uint8_t getCap(uint8_t devAddress, uint8_t* buffer) {
    // Perform a Read Byte transaction
    if (!PMBUS_ReadByte(devAddress, CAPABILITY, buffer)) return 0;  // Read failed
    return 1;
}

// Set the manufacturer ID
uint8_t setMfrId(uint8_t devAddress, uint8_t *data, uint8_t len) {
    if(!PMBUS_BlockWrite(devAddress, MFR_ID, data, len)) return 0;
    return 1;
}

// Get the manufacturer ID
uint8_t getMfrId(uint8_t devAddress, uint8_t* buffer) {
    uint8_t len;
    if(!PMBUS_BlockRead(devAddress, MFR_ID, buffer, &len)) return 0;
    // Check if the count is valid
    if (len == 0 || len > 32) return 0;  // Invalid length
    return 1;
}

// Set the board revision
uint8_t setBoardRev(uint8_t devAddress, uint8_t *data, uint8_t len) {
    if(!PMBUS_BlockWrite(devAddress, MFR_REVISION, data, len)) return 0;
    return 1;
}

// Get the board revision
uint8_t getBoardRev(uint8_t devAddress, uint8_t* buffer) {
    uint8_t len;
    if(!PMBUS_BlockRead(devAddress, MFR_REVISION, buffer, &len)) return 0;
    // Check if the length is valid
    if (len == 0 || len > 32) return 0;  // Invalid length
    return 1;
}

// Get the current page
uint8_t getPage(uint8_t devAddress, uint8_t* buffer) {
    if(!PMBUS_ReadByte(devAddress, PAGE, buffer)) return 0;
    return 1;
}

// Set the current page
uint8_t setPage(uint8_t devAddress, uint8_t data) {
    if(!PMBUS_WriteByte(devAddress, PAGE, data)) return 0;
    return 1;
}

// Get the current phase
uint8_t getPhase(uint8_t devAddress, uint8_t* buffer) {
    if(!PMBUS_ReadByte(devAddress, PHASE, buffer)) return 0;
    return 1;
}

// Set the current phase
uint8_t setPhase(uint8_t devAddress, uint8_t data) {
    if(!PMBUS_WriteByte(devAddress, PHASE, data)) return 0;
    return 1;
}

// Get the operation status
uint8_t getOpStatus(uint8_t devAddress, uint8_t* buffer) {
    if(!PMBUS_ReadByte(devAddress, OPERATION, buffer)) return 0;
    return 1;
}

// Set the operation status
uint8_t setOpStatus(uint8_t devAddress, uint8_t data) {
    if(!PMBUS_WriteByte(devAddress, OPERATION, data)) return 0;
    return 1;
}

// Get the on/off configuration
uint8_t getOnOffConfig(uint8_t devAddress, uint8_t* buffer) {
    if(!PMBUS_ReadByte(devAddress, ON_OFF_CONFIG, buffer)) return 0;
    return 1;
}

// Set the on/off configuration
uint8_t setOnOffConfig(uint8_t devAddress, uint8_t data) {
    if(!PMBUS_WriteByte(devAddress, ON_OFF_CONFIG, data)) return 0;
    return 1;
}

// Get the VOUT mode
uint8_t getVoutMode(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, VOUT_MODE, buffer);
}

// Get the VOUT command
uint8_t getVoutCommand(uint8_t devAddress, uint16_t* buffer) {
    return PMBUS_ReadWord(devAddress, VOUT_COMMAND, buffer);
}

// Set the VOUT command
uint8_t setVoutCommand(uint8_t devAddress, uint16_t data) {
    return PMBUS_WriteWord(devAddress, VOUT_COMMAND, data);
}

// Get the maximum VOUT
uint8_t getVoutMax(uint8_t devAddress, uint16_t* buffer) {
    return PMBUS_ReadWord(devAddress, VOUT_MAX, buffer);
}

// Set the maximum VOUT
uint8_t setVoutMax(uint8_t devAddress, uint16_t data) {
    return PMBUS_WriteWord(devAddress, VOUT_MAX, data);
}

// Get the high margin VOUT
uint8_t getVoutMarginHigh(uint8_t devAddress, uint16_t* buffer) {
    return PMBUS_ReadWord(devAddress, VOUT_MARGIN_HIGH, buffer);
}

// Set the high margin VOUT
uint8_t setVoutMarginHigh(uint8_t devAddress, uint16_t data) {
    return PMBUS_WriteWord(devAddress, VOUT_MARGIN_HIGH, data);
}

// Get the low margin VOUT
uint8_t getVoutMarginLow(uint8_t devAddress, uint16_t* buffer) {
    return PMBUS_ReadWord(devAddress, VOUT_MARGIN_LOW, buffer);
}

// Set the low margin VOUT
uint8_t setVoutMarginLow(uint8_t devAddress, uint16_t data) {
    return PMBUS_WriteWord(devAddress, VOUT_MARGIN_LOW, data);
}

// Get the status byte
uint8_t getStatusByte(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_BYTE, buffer);
}

// Get the status word
uint8_t getStatusWord(uint8_t devAddress, uint16_t* buffer) {
    return PMBUS_ReadWord(devAddress, STATUS_WORD, buffer);
}

// Get the VOUT status
uint8_t getStatusVout(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_VOUT, buffer);
}

// Get the IOUT status
uint8_t getStatusIout(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_IOUT, buffer);
}

// Get the input status
uint8_t getStatusInput(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_INPUT, buffer);
}

// Get the temperature status
uint8_t getStatusTemperature(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_TEMPERATURE, buffer);
}

// Get the CML (Communication, Memory and Logic) status
uint8_t getStatusCml(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_CML, buffer);
}

// Get the manufacturer-specific status
uint8_t getStatusMfrSpecific(uint8_t devAddress, uint8_t* buffer) {
    return PMBUS_ReadByte(devAddress, STATUS_MFR_SPECIFIC, buffer);
}
