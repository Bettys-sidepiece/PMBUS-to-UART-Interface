#ifndef INC_SMBUS_H_
#define INC_SMBUS_H_

#include <stdint.h>
#include <stm32g474xx.h>

#define BUFFERSIZE	258

// I2C Speed Definitions
#define I2C_10KHZ    			10000
#define I2C_100KHZ   			100000
#define I2C_400KHZ      		400000
#define I2C_1MHZ    			1000000

// I2C Timing Register Value Macros
#define I2C_TIMINGR_PRESC_POS    	28
#define I2C_TIMINGR_SCLDEL_POS   	20
#define I2C_TIMINGR_SDADEL_POS   	16
#define I2C_TIMINGR_SCLH_POS     	8
#define I2C_TIMINGR_SCLL_POS     	0

#define I2C_TIMINGR_CONFIG(presc, scldel, sdadel, sclh, scll) \
    ((presc << I2C_TIMINGR_PRESC_POS) | \
     (scldel << I2C_TIMINGR_SCLDEL_POS) | \
     (sdadel << I2C_TIMINGR_SDADEL_POS) | \
     (sclh << I2C_TIMINGR_SCLH_POS) | \
     (scll << I2C_TIMINGR_SCLL_POS))

// I2C Timing Configurations for 16 MHz clock
#define I2C_TIMINGR_10KHZ    I2C_TIMINGR_CONFIG(3, 4, 2, 0xC3, 0xC7)
#define I2C_TIMINGR_100KHZ   I2C_TIMINGR_CONFIG(3, 4, 2, 0xF, 0x13)
#define I2C_TIMINGR_400KHZ   I2C_TIMINGR_CONFIG(1, 3, 2, 0x3, 0x9)
#define I2C_TIMINGR_1MHZ     I2C_TIMINGR_CONFIG(0, 2, 0, 0x2, 0x4)

// Basic I2C operations
void I2C_Init(void);
void EnableI2C(void);
void DisableI2C(void);
void I2C_Start(uint8_t devAddress);
void I2C_Stop(uint8_t devAddress);
void I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(void);
uint8_t I2C_WaitAck(void);

// PMBus operations
uint8_t CRC8(const uint8_t* data, uint32_t length);
uint8_t PMBUS_SendByte(uint8_t devAddress, uint8_t data);
uint8_t PMBUS_WriteByte(uint8_t devAddress, uint8_t command, uint8_t data);
uint8_t PMBUS_ReadByte(uint8_t devAddress, uint8_t command, uint8_t *data);
uint8_t PMBUS_WriteWord(uint8_t devAddress, uint8_t command, uint16_t data);
uint8_t PMBUS_ReadWord(uint8_t devAddress, uint8_t command, uint16_t *data);
uint8_t PMBUS_BlockWrite(uint8_t devAddress, uint8_t command, uint8_t *data, uint8_t length);
uint8_t PMBUS_BlockRead(uint8_t devAddress, uint8_t command, uint8_t *data, uint8_t *length);
uint8_t PMBUS_BlockWriteBlockRead(uint8_t devAddress, uint8_t command,
                                  uint8_t *write_data, uint8_t write_count,
                                  uint8_t *read_data, uint8_t *read_count);

// Function prototypes
void decimalToWord(uint16_t decimal, uint16_t* buffer);
void intToHex(uint32_t number, uint8_t* buffer);
uint16_t wordToDecimal(const uint16_t* buffer);
uint32_t bytesToInt32(const uint8_t* buffer);
unsigned int float_to_slinear11(float number, signed int exponent);
float slinear11_to_float(unsigned int number);
unsigned int float_to_ulinear16(float number, unsigned char vout_mode);
float ulinear16_to_float(unsigned int number, unsigned char vout_mode);

// Declare the LUT_linear_exponents array
extern const float LUT_linear_exponents[32];

//PMBUS Commands
uint8_t clearfaults(uint8_t devAddress);
uint8_t readDeviceID(uint8_t devAddress, uint8_t* buffer);
uint8_t PMBUSRev(uint8_t devAddress, uint8_t* buffer);
uint8_t getCap(uint8_t devAddress, uint8_t* buffer);
uint8_t setBoardRev(uint8_t devAddress, uint8_t* data, uint8_t len);
uint8_t setMfrId(uint8_t devAddress, uint8_t *data, uint8_t len);
uint8_t getMfrId(uint8_t devAddress, uint8_t* buffer);
uint8_t setBoardRev(uint8_t devAddress, uint8_t* data, uint8_t len);
uint8_t getBoardRev(uint8_t devAddress, uint8_t* buffer);
uint8_t getPage(uint8_t devAddress, uint8_t* buffer);
uint8_t setPage(uint8_t devAddress, uint8_t data);
uint8_t getPhase(uint8_t devAddress, uint8_t* buffer);
uint8_t setPhase(uint8_t devAddress, uint8_t data);
uint8_t getOpStatus(uint8_t devAddress, uint8_t* buffer);
uint8_t setOpstatus(uint8_t devAddress, uint8_t data);

#endif /* INC_SMBUS_H_ */
