#include "smbus.h"

//---------------------------------------------------------------------I2C Configuration--------------------------------------------------------------------------//
i2c_freq_t pmbusSpeed = STD_MODE;

// Create the I2C timing configuration instance
const i2c_timingr_config_t i2c_timing_config[] = {
    [STD_MODE] = {3, 0x13, 0x0F, 0x02, 0x04},
    [FAST_MODE_1] = {1, 0x09, 0x03, 0x02, 0x03},
    [FAST_PLUS_MODE_1] = {0, 0x04, 0x02, 0x00, 0x02},
    [FAST_MODE_2] = {1, 0x0B, 0x05, 0x02, 0x03},  // Slightly longer SCLL and SCLH
    [FAST_PLUS_MODE_2] = {0, 0x06, 0x04, 0x00, 0x02},  // Slightly longer SCLL and SCLH
};

void I2C_Init(void) {
    // Enable GPIOA and GPIOB clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Enable HSI clock
    RCC->CR |= RCC_CR_HSION;
     while ((RCC->CR & RCC_CR_HSIRDY) == 0);  // Wait for HSI to be ready

    // Set HSI as the clock source for I2C1
    RCC->CCIPR &= ~RCC_CCIPR_I2C1SEL_Msk;
    RCC->CCIPR |= (2 << RCC_CCIPR_I2C1SEL_Pos);  // 10: HSI16 clock selected as I2C1 clock

    // Enable I2C1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    // Configure GPIO pins for I2C1
    // PA15 (SCL) - Alternate function mode, Open-drain, Very Fast speed
    GPIOA->MODER &= ~GPIO_MODER_MODE15_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE15_1;  // Alternate function mode
    GPIOA->OTYPER |= GPIO_OTYPER_OT15;    // Open-drain
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL15_Msk;
    GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFSEL15_Pos);  // AF4 for I2C1
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;  // Very Fast speed

    // PB7 (SDA) - Alternate function mode, Open-drain, Very Fast speed
    GPIOB->MODER &= ~GPIO_MODER_MODE7_Msk;
    GPIOB->MODER |= GPIO_MODER_MODE7_1;    // Alternate function mode
    GPIOB->OTYPER |= GPIO_OTYPER_OT7;      // Open-drain
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
    GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL7_Pos);  // AF4 for I2C1
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;  // Very Fast speed

    // Configure I2C1
    I2C1->CR1 = 0;  // Disable I2C

    // Set I2C timing for 400kHz
    //I2C1->TIMINGR = I2C_TIMINGR_100KHZ;
    I2C1->TIMINGR = get_i2c_timing_config(pmbusSpeed);


    // Enable SMBus master mode and related features
    I2C1->CR1 |= I2C_CR1_SMBHEN;  // Enable SMBus host mode
    I2C1->CR1 |= I2C_CR1_PECEN;   // Enable Packet Error Checking (PEC)
    //I2C1->CR1 |= I2C_CR1_STOPIE;
    //I2C1->CR1 |= I2C_CR1_NACKIE;

    // Set the device address (example: 0xC0)
    I2C1->OAR1 &= ~I2C_OAR1_OA1_Msk;
    I2C1->OAR1 |= (0xC0 << 1) & I2C_OAR1_OA1_Msk; // Set own address
    I2C1->OAR1 |= I2C_OAR1_OA1EN;  // Enable own address 1

    // Clear any pending STOPF flag
    I2C1->ICR |= I2C_ICR_STOPCF;
}

// Function to get the timing configuration for a specific speed mode
uint32_t get_i2c_timing_config(i2c_freq_t freq_mode) {
	const i2c_timingr_config_t* config;

	if (freq_mode <= FAST_PLUS_MODE_1) {
		config = &i2c_timing_config[freq_mode];
	} else {
		// Default to Standard Mode if an invalid mode is provided
		config = &i2c_timing_config[STD_MODE];
	}

	return (config->presc << 28) |
		(config->scldel << 20) |
		(config->sdadel << 16) |
		(config->sclh << 8) |
		(config->scll);
}

uint8_t setI2cFreq(i2c_freq_t freq_mode){
	pmbusSpeed = freq_mode;
	if(pmbusSpeed >= STD_MODE && pmbusSpeed <= FAST_PLUS_MODE_2){
		I2C1->TIMINGR = get_i2c_timing_config(freq_mode);
		return 0;
	}
	return 1;
}

void EnableI2C(void)
{
	I2C1->CR1 |= I2C_CR1_PE;
}

void DisableI2C(void)
{
	I2C1->CR1 = 0;
}

void I2C_Start(uint8_t devAddress)
{
    int timeout = 10000;
    // Clear the STOPF flag by writing to the ICR register
    if((I2C1->ISR & I2C_ISR_STOPF) || (I2C1->ISR & I2C_ISR_NACKF)){
	    I2C1->ICR |= I2C_ICR_STOPCF;
	    I2C1->ICR |= I2C_ICR_NACKCF;
    }
    I2C1->CR2 |= I2C_CR2_START;     // Generate start condition
    while (I2C1->CR2 & I2C_CR2_START){  // Wait until start is sent
    	    if(--timeout == 0){
    		    break;
    	    }
    }
}

void I2C_Stop(uint8_t devAddress)
{
	int timeout = 10000;
    I2C1->CR2 |= I2C_CR2_STOP;  // Generate stop condition
    while (I2C1->CR2 & I2C_CR2_STOP){  // Wait until stop is sent
	    if(--timeout == 0){
		    break;
	    }
    }
}

void I2C_WriteByte(uint8_t data)
{
    uint32_t timeout = 10000;
    while ((I2C1->ISR & I2C_ISR_TXE)){
	  I2C1->TXDR = data;
    }  // Wait until TX buffer is empty

    while (!(I2C1->ISR & I2C_ISR_TC)){   // Wait until transfer is complete
    	    if(--timeout == 0){
    		    break;
    	    }
    }
}

uint8_t I2C_ReadByte(void)
{
	 while (!(I2C1->ISR & I2C_ISR_RXNE));  // Wait until RX buffer is not empty
	 return I2C1->RXDR;
}

uint8_t checkForNack(void)
{

    if (I2C1->ISR & I2C_ISR_NACKF) {
        I2C1->ICR |= I2C_ICR_NACKCF;  // Clear NACK flag
        return 0;  // NACK received
    }
    return 1;  // ACK received
}

//-----------------------------------------------------------------------PMBUS Transaction Protocols-------------------------------------------------------------//
uint8_t pecbyte = 0;

uint8_t CRC8(const uint8_t* data, uint32_t length)
{
    uint8_t crc = 0;

    for (uint32_t i = 0; i < length; i++) {
        crc = crc_table[crc ^ data[i]];
    }

    return crc;
}

uint8_t PMBUS_SendByte(uint8_t devAddress, uint8_t data) {
    uint32_t timeout;
    uint8_t buffer[2];
    uint8_t pec;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Address with write bit
    buffer[1] = data;

    // Calculate PEC
    pec = CRC8(buffer, 2);

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Ensure the I2C bus is not busy
    timeout = 10000;
    while (I2C1->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) return PMBUS_TIME_OUT;
    }

    // Set slave address, number of bytes, and generate START
    I2C1->CR2 = (devAddress << 1) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    // Write data byte
    I2C1->TXDR = data;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write PEC byte
    I2C1->TXDR = pec;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    return PMBUS_OK;
}

/**
 * @brief Writes a single byte to a PMBus device.
 *
 * This function writes a single byte of data to a PMBus device specified by `devAddress`
 * using the provided `command`. It calculates and sends a Packet Error Code (PEC) for
 * the write operation to ensure data integrity.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device.
 * @param data The byte of data to be written to the PMBus device.
 *
 * @return uint8_t Returns 1 on success (data written and PEC matched), otherwise 0 (error during write or PEC mismatch).
 */
uint8_t PMBUS_WriteByte(uint8_t devAddress, uint8_t command, uint8_t data) {
    uint32_t timeout;
    uint8_t buffer[3];
    uint8_t pec;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation (address + write bit)
    buffer[1] = command;
    buffer[2] = data;

    // Calculate PEC
    pec = CRC8(buffer, 3);

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Ensure the I2C bus is not busy
    timeout = 10000;
    while (I2C1->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) return PMBUS_TIME_OUT;
    }

    // Set slave address, number of bytes, and generate START
    I2C1->CR2 = (devAddress << 1) | (3 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    // Write command byte
    I2C1->TXDR = command;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write data byte
    I2C1->TXDR = data;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write PEC byte
    I2C1->TXDR = pec;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    return PMBUS_OK;
}

/**
 * @brief Reads a single byte from a PMBus device.
 *
 * This function reads a single byte of data from a PMBus device specified by `devAddress`
 * using the provided `command`. It calculates and verifies the Packet Error Code (PEC)
 * to ensure data integrity for the read operation.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device for initiating the read operation.
 * @param data Pointer to a variable where the read byte will be stored.
 *
 * @return uint8_t Returns 1 on success (data read and PEC matched), otherwise 0 (error during read or PEC mismatch).
 */
uint8_t PMBUS_ReadByte(uint8_t devAddress, uint8_t command, uint8_t *data) {
    uint32_t timeout;
    uint8_t buffer[4];
    uint8_t pec_received, pec_calculated;

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Write command
    I2C1->CR2 = (devAddress << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    I2C1->TXDR = command;

    if (!checkForNack()) return PMBUS_NACK;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TC)) if (--timeout == 0) return PMBUS_TIME_OUT;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation
    buffer[1] = command;
    buffer[2] = (devAddress << 1) | 1;  // Read operation

    // Read data and PEC
    I2C1->CR2 = (devAddress << 1) | I2C_CR2_RD_WRN | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    *data = I2C1->RXDR;
    buffer[3] = *data;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    pec_received = I2C1->RXDR;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Calculate PEC
    pec_calculated = CRC8(buffer, 4);

    // Validate PEC
    if (pec_received != pec_calculated) {
        return PMBUS_PEC_MISMATCH;
    }

    return PMBUS_OK;
}


/**
 * @brief Writes a 16-bit word (2 bytes) to a PMBus device.
 *
 * This function writes a 16-bit word (2 bytes) to a PMBus device specified by `devAddress`
 * using the provided `command`. It calculates and sends a Packet Error Code (PEC) for
 * the write operation to ensure data integrity.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device.
 * @param data The 16-bit word (2 bytes) to be written to the PMBus device.
 *
 * @return uint8_t Returns PMBUS_OK on success (data written and PEC matched), otherwise (error during write or PEC mismatch).
 */
uint8_t PMBUS_WriteWord(uint8_t devAddress, uint8_t command, uint16_t data) {
    uint32_t timeout;
    uint8_t buffer[4];
    uint8_t pec;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation (address + write bit)
    buffer[1] = command;
    buffer[2] = data & 0xFF;  // Low byte
    buffer[3] = (data >> 8) & 0xFF;  // High byte

    // Calculate PEC
    pec = CRC8(buffer, 4);

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Ensure the I2C bus is not busy
    timeout = 10000;
    while (I2C1->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) return PMBUS_TIME_OUT;
    }

    // Set slave address, number of bytes, and generate START
    I2C1->CR2 = (devAddress << 1) | (4 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    // Write command byte
    I2C1->TXDR = command;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write data low byte
    I2C1->TXDR = data & 0xFF;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write data high byte
    I2C1->TXDR = (data >> 8) & 0xFF;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write PEC byte
    I2C1->TXDR = pec;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    return PMBUS_OK;
}


/**
 * @brief Reads a 16-bit word (2 bytes) from a PMBus device.
 *
 * This function reads a 16-bit word (2 bytes) from a PMBus device specified by `devAddress`
 * using the provided `command`. It calculates and verifies the Packet Error Code (PEC)
 * to ensure data integrity.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device for initiating the read operation.
 * @param data Pointer to a variable where the read 16-bit word will be stored.
 *             The data is stored in little-endian format.
 *
 * @return uint8_t Returns 1 on success (data read and PEC matched), otherwise 0 (error during read or PEC mismatch).
 */
uint8_t PMBUS_ReadWord(uint8_t devAddress, uint8_t command, uint16_t *data) {
    uint32_t timeout;
    uint8_t buffer[5];
    uint8_t pec_received, pec_calculated;
    uint8_t low_byte, high_byte;

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Write command
    I2C1->CR2 = (devAddress << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    I2C1->TXDR = command;

    if (!checkForNack()) return PMBUS_NACK;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TC)) if (--timeout == 0) return PMBUS_TIME_OUT;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation
    buffer[1] = command;
    buffer[2] = (devAddress << 1) | 1;  // Read operation

    // Read data and PEC
    I2C1->CR2 = (devAddress << 1) | I2C_CR2_RD_WRN | (3 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    low_byte = I2C1->RXDR;
    buffer[3] = low_byte;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    high_byte = I2C1->RXDR;
    buffer[4] = high_byte;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    pec_received = I2C1->RXDR;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Calculate PEC
    pec_calculated = CRC8(buffer, 5);

    // Validate PEC
    if (pec_received != pec_calculated) {
        return PMBUS_PEC_MISMATCH;
    }

    *data = (uint16_t)low_byte | ((uint16_t)high_byte << 8);
    return PMBUS_OK;
}

/**
 * @brief Performs a block write operation to a PMBus device.
 *
 * This function writes a block of data to a PMBus device specified by `devAddress`
 * using the provided `command`. It calculates and sends a Packet Error Code (PEC) for
 * the write operation to ensure data integrity.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device.
 * @param data Pointer to the buffer containing data to be written.
 * @param length Number of bytes to be written to the PMBus device.
 *
 * @return uint8_t Returns 1 on success (data written and PEC matched), otherwise 0 (error during write or PEC mismatch).
 */
uint8_t PMBUS_BlockWrite(uint8_t devAddress, uint8_t command, uint8_t *data, uint8_t length) {
    if (length > 255) return PMBUS_INVALID_DATA;  // PMBus block write limited to 255 bytes

    uint32_t timeout;
    uint8_t buffer[258];  // Maximum length including address, command, byte count, data, and PEC
    uint8_t pec;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation (address + write bit)
    buffer[1] = command;
    buffer[2] = length;  // Byte count
    for (uint8_t i = 0; i < length; i++) {
        buffer[3 + i] = data[i];  // Data bytes
    }

    // Calculate PEC
    pec = CRC8(buffer, length + 3);

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Ensure the I2C bus is not busy
    timeout = 10000;
    while (I2C1->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) return PMBUS_TIME_OUT;
    }

    // Set slave address, number of bytes, and generate START
    I2C1->CR2 = (devAddress << 1) | ((length + 3) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    // Write command byte
    I2C1->TXDR = command;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write byte count
    I2C1->TXDR = length;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write data bytes
    for (uint8_t i = 0; i < length; i++) {
        I2C1->TXDR = data[i];
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
        if (!checkForNack()) return PMBUS_NACK;
    }

    // Write PEC byte
    I2C1->TXDR = pec;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    return PMBUS_OK;
}


/**
 * @brief Performs a block read operation from a PMBus device.
 *
 * This function reads a block of data from a PMBus device specified by `devAddress`
 * using the provided `command`. It validates the received data using Packet Error Code (PEC)
 * and updates the `data` buffer with the received bytes and `length` with the actual number of bytes read.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device for initiating the read operation.
 * @param data Pointer to the buffer where read data will be stored.
 * @param length Pointer to a variable holding the maximum number of bytes to read.
 *               Upon successful completion, this variable will be updated with the actual number of bytes read.
 *
 * @return uint8_t Returns 1 on success (data read and PEC matched), otherwise 0 (error during read or PEC mismatch).
 */
//FIXME -- Block read doesnt seem to be working correctly
uint8_t PMBUS_BlockRead(uint8_t devAddress, uint8_t command, uint8_t *data, uint8_t *length) {
    uint32_t timeout;
    uint8_t buffer[258];  // Maximum length including address, command, byte count, data, and PEC
    uint8_t pec_received, pec_calculated;
    uint8_t byte_count;

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Write command
    I2C1->CR2 = (devAddress << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    I2C1->TXDR = command;

    if (!checkForNack()) return PMBUS_NACK;

    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TC)) if (--timeout == 0) return PMBUS_TIME_OUT;

    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation
    buffer[1] = command;
    buffer[2] = (devAddress << 1) | 1;  // Read operation

    // Read byte count
    I2C1->CR2 = (devAddress << 1) | I2C_CR2_RD_WRN | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    byte_count = I2C1->RXDR;
    buffer[3] = byte_count;

    if (byte_count > *length) {
        return PMBUS_WRITE_OVER_FLOW;
    }

    // Read data and PEC
    I2C1->CR2 = (devAddress << 1) | I2C_CR2_RD_WRN | ((byte_count + 1) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    for (uint8_t i = 0; i < byte_count; i++) {
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
        data[i] = I2C1->RXDR;
        buffer[4 + i] = data[i];
    }

    // Read PEC
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    pec_received = I2C1->RXDR;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Calculate PEC
    pec_calculated = CRC8(buffer, byte_count + 4);

    // Validate PEC
    if (pec_received != pec_calculated) {
        return PMBUS_PEC_MISMATCH;
    }

    *length = byte_count;
    return PMBUS_OK;
}

/**
 * @brief Performs a PMBus block write followed by a block read operation.
 *
 * This function writes a command and data bytes to a PMBus device specified by `devAddress`,
 * calculates and verifies the Packet Error Code (PEC) for the write operation,
 * then reads a response from the device, calculates and verifies the PEC for the read operation.
 * If successful, it updates `read_data` with the received data and `read_count` with the number of bytes read.
 *
 * @param devAddress The 7-bit device address of the PMBus device.
 * @param command The command byte to be sent to the PMBus device.
 * @param write_data Pointer to the buffer containing data to be written to the PMBus device.
 * @param write_count Number of bytes to be written to the PMBus device.
 * @param read_data Pointer to the buffer where read data will be stored.
 * @param read_count Pointer to a variable holding the maximum number of bytes to read.
 *                   Upon successful completion, this variable will be updated with the actual number of bytes read.
 *
 * @return uint8_t Returns 1 on success (write and read operations completed successfully and PECs matched), otherwise 0.
 */
uint8_t PMBUS_BlockWriteBlockRead(uint8_t devAddress, uint8_t command,
                                  uint8_t *write_data, uint8_t write_count,
                                  uint8_t *read_data, uint8_t *read_count) {
    if (write_count > 255 || *read_count > 255) return PMBUS_INVALID_DATA;

    uint32_t timeout;
    uint8_t buffer[258];  // Maximum length including address, command, byte count, data, and PEC
    uint8_t pec;
    uint8_t byte_count;

    // Write operation
    // Prepare buffer for PEC calculation
    buffer[0] = devAddress << 1;  // Write operation (address + write bit)
    buffer[1] = command;
    buffer[2] = write_count;
    for (uint8_t i = 0; i < write_count; i++) {
        buffer[3 + i] = write_data[i];
    }

    // Calculate PEC for write operation
    pec = CRC8(buffer, write_count + 3);

    // Ensure STOPF is cleared
    I2C1->ICR |= I2C_ICR_STOPCF;

    // Ensure the I2C bus is not busy
    timeout = 10000;
    while (I2C1->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) return PMBUS_TIME_OUT;
    }

    // Set slave address, number of bytes, and generate START
    I2C1->CR2 = (devAddress << 1) | ((write_count + 3) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;

    // Write command byte
    I2C1->TXDR = command;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write byte count
    I2C1->TXDR = write_count;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Write data bytes
    for (uint8_t i = 0; i < write_count; i++) {
        I2C1->TXDR = write_data[i];
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
        if (!checkForNack()) return PMBUS_NACK;
    }

    // Write PEC byte
    I2C1->TXDR = pec;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    if (!checkForNack()) return PMBUS_NACK;

    // Wait for TC flag
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TC)) if (--timeout == 0) return PMBUS_TIME_OUT;

    // Read operation
    // Prepare buffer for PEC calculation
    buffer[0] = (devAddress << 1) | 1;  // Read operation (address + read bit)

    // Read byte count
    I2C1->CR2 = (devAddress << 1) | I2C_CR2_RD_WRN | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    byte_count = I2C1->RXDR;
    buffer[1] = byte_count;

    if (byte_count > *read_count) {
        return PMBUS_WRITE_OVER_FLOW;
    }

    // Read data and PEC
    I2C1->CR2 = (devAddress << 1) | I2C_CR2_RD_WRN | ((byte_count + 1) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    for (uint8_t i = 0; i < byte_count; i++) {
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
        read_data[i] = I2C1->RXDR;
        buffer[2 + i] = read_data[i];
    }

    // Read PEC
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_RXNE)) if (--timeout == 0) return PMBUS_TIME_OUT;
    uint8_t pec_received = I2C1->RXDR;

    // Wait for the stop condition
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) if (--timeout == 0) return PMBUS_TIME_OUT;
    I2C1->ICR |= I2C_ICR_STOPCF;


    // Calculate PEC for read operation
    pec = CRC8(buffer, byte_count + 2);

    // Validate PEC
    if (pec_received != pec) {
        return PMBUS_PEC_MISMATCH;
    }

    *read_count = byte_count;
    return PMBUS_OK;
}
