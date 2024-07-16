/*
 * pmbus_device.h
 *
 *  Created on: Jul 10, 2024
 *      Author: nx024656
 */

#ifndef INC_PMBUS_DEVICE_H_
#define INC_PMBUS_DEVICE_H_


#include <stdlib.h>
#include <string.h>
#include "smbus.h"
#include "stm32g4xx.h"

typedef struct{
  uint8_t pec; //bit 7
  uint16_t speed;// bit 5-6
  uint8_t alert;// bit 4
  uint8_t format;//bit 3
  uint8_t avsbus;//bit 2
  uint8_t res1; //bit 1
  uint8_t res0;//bit 0
} pdevice_cap_t;

typedef struct {
	uint8_t address;
	uint32_t pmbus_rev[2];
	uint32_t deviceID;
	uint32_t vendor;
	uint8_t num_phases;
	uint8_t num_pages;
	pdevice_cap_t deviceCap;
	uint8_t rxBuf[64];
	uint8_t WP_Status;
} pmbus_device_t;



void resetPMBUSdevice(pmbus_device_t *pdev);
int deviceCapabilities(pmbus_device_t *pdev);
int scanPMBUSwire(pmbus_device_t *pdev);

#endif /* INC_PMBUS_DEVICE_H_ */
