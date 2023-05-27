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

#define FIRST_TIME_BOOTUP     0


#define USB_BUFFER_SIZE       128

#define PYTHONCAN_START_OF_FRAME      0xAA
#define PYTHONCAN_END_OF_FRAME        0xBB


void APP_init();

void APP_main();

void APP_initFlashOption();

void APP_handleUSBMessage();

void APP_handleCANMessage();


#endif /* INC_APP_H_ */
