/*
 * app.h
 *
 *  Created on: May 26, 2023
 *      Author: TK
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdio.h>
#include <string.h>

#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"

#include "rv_common.h"
#include "can.h"

/* First-time bootup procedure:
 * 1. set this value to 1 to enable flash option byte programming.
 * 2. compile and flash the code.
 * 3. set this value to 0 to allow normal operation.
 * 4. compile and flash the code again.
 * 5. power cycle the board.
 */
#define FIRST_TIME_BOOTUP     0

// USB buffer size, set this value to the same value as in the ioc config file.
#define USB_BUFFER_SIZE       128

// Python-CAN protocol character definitions.
#define PYTHONCAN_START_OF_FRAME      0xAA
#define PYTHONCAN_END_OF_FRAME        0xBB


/**
 * @brief Procedure to configure flash option bytes following G431 User Manual Section 4.4.2 Option bytes programming
 * 
 * 1. Unlock the FLASH_CR with the LOCK clearing sequence. 
 *    Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
 * 2. Unlock the FLASH Option Byte with the LOCK clearing sequence.
 *    Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
 * 3. Program OPTR to boot from flash.
 * 4. Set the Options Start bit OPTSTRT in the Flash control register (FLASH_CR).
 * 5. Wait for the BSY bit to be cleared.
 * 6. Lock Flash.
 * 7. Reload the new settings. The chip still needs a power-cycle to make new changes effective.
 */
void APP_initFlashOption();

/**
 * @brief Handle incoming USB message and send it out on CAN bus
 */
void APP_handleUSBMessage();

/**
 * @brief Handle incoming CAN message and send it out on USB
 */
void APP_handleCANMessage();

void APP_init();

void APP_main();


#endif /* INC_APP_H_ */
