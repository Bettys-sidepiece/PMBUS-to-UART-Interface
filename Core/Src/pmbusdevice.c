/*
 * pmbus_device.c
 */
#include <pmbusdevice.h>

int pdevInit(pmbus_device_t *pdev)
{
	return 0;
}

int deviceCapabilities(pmbus_device_t *pdev) {
    uint8_t bit[8];
    uint8_t extract;

    if (pdev->address > 0xC0) {
        if (getCap(pdev->address, pdev->rxBuf)) {
            extract = *pdev->rxBuf;
            bit[0] = extract & 0x01;           // extracts the first bit
            bit[1] = (extract >> 1) & 0x01;    // extracts the second bit
            bit[2] = (extract >> 2) & 0x01;    // extracts the third bit
            bit[3] = (extract >> 3) & 0x01;    // extracts the fourth bit
            bit[4] = (extract >> 4) & 0x01;    // extracts the fifth bit
            bit[5] = (extract >> 5) & 0x03;    // extracts the speed bits [5:6]
            bit[7] = (extract >> 7) & 0x01;    // extracts the eighth bit

            // Update device capabilities
            pdev->deviceCap.pec = bit[7];
            pdev->deviceCap.speed = bit[5];
            pdev->deviceCap.alert = bit[4];
            pdev->deviceCap.format = bit[3];
            pdev->deviceCap.avsbus = bit[2];
            pdev->deviceCap.res1 = bit[1];
            pdev->deviceCap.res0 = bit[0];

            return 1; // successful
        }
    }
    return 0; // failed
}


int scanPMBUSwire(pmbus_device_t *pdev)
{
    pmbus_error_t result;
    for (uint8_t addr = 0xB0; addr <= 0xEE; addr++) {
	    // Scan addresses from 0xC0 to 0xFE
	  result = readDeviceID(addr, pdev->rxBuf);
        if (result == PMBUS_OK) {
            pdev->address = addr;
            if (pdev->rxBuf[0] != 0) {
                pdev->deviceID = pdev->rxBuf[0];
                return PMBUS_OK;
            }else{
                return result;  // Memory allocation failed
            }
        }
    }

    return 0;  // No device found or readDeviceID failed for all addresses
}

void resetPMBUSdevice(pmbus_device_t *pdev)
{
    if (pdev->deviceID != 0) {
		pdev->deviceID = 0;  // Set to NULL after freeing to avoid dangling pointer
    }

    if (pdev->pmbus_rev[0] != 0) {
            pdev->pmbus_rev[0] = 0;
            pdev->pmbus_rev[1] = 0;
    }

    if (pdev->vendor != 0) {
		pdev->vendor = 0;  // Set to NULL after freeing to avoid dangling pointer
        }

    pdev->address = 0;
    pdev->num_pages = 0;
    pdev->num_phases = 0;

    pdev->deviceCap.alert = 0;
    pdev->deviceCap.avsbus = 0;
    pdev->deviceCap.format = 0;
    pdev->deviceCap.pec = 0;
    pdev->deviceCap.res0 = 0;
    pdev->deviceCap.res1 = 0;
    pdev->deviceCap.speed = 0;

}
