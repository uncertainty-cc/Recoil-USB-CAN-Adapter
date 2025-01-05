/*
 * app.c
 *
 *  Created on: May 26, 2023
 *      Author: TK
 */

#include "app.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart2;


uint16_t  usb_rx_size;
uint8_t   usb_rx_buffer[USB_BUFFER_SIZE];
uint16_t  usb_tx_size;
uint8_t   usb_tx_buffer[USB_BUFFER_SIZE];

CAN_Frame can_rx_frame;
CAN_Frame can_tx_frame;

uint8_t usb_evt_happened;
uint8_t can_evt_happened;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  APP_handleCANMessage();
}

void APP_initFlashOption() {
  // 1. Unlock the FLASH_CR with the LOCK clearing sequence
  // Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}
  HAL_FLASH_Unlock();

  // 2. Unlock the FLASH Option Byte with the LOCK clearing sequence
  // Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}
  HAL_FLASH_OB_Unlock();

  // 3. Program OPTR
  FLASH->OPTR = 0xFBEFF8AAU;  // default to boot from flash

  // 4. Set the Options Start bit OPTSTRT in the Flash control register (FLASH_CR).
  SET_BITS(FLASH->CR, FLASH_CR_OPTSTRT);

  // 4.1 clear status register
  SET_BITS(FLASH->SR, FLASH_SR_OPTVERR | FLASH_SR_RDERR);

  // 5. Wait for the BSY bit to be cleared.
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  // 6. Lock Flash
  // If LOCK is set by software, OPTLOCK is automatically set too
  HAL_FLASH_Lock();

  // 7. Reload the new settings
  // seems this line will cause error when put before FLASH_Lock(), which will then corrupt all Flash settings
  // so putting it here
  HAL_FLASH_OB_Launch();

  while (1) {
    char str[64];
    sprintf(str, "Flash option change finished. Please power cycle the device.\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
    HAL_Delay(100);
  }
}

void APP_handleUSBMessage() {
  // check if the first byte is the correct Start of Frame
  uint8_t is_valid_frame = usb_rx_buffer[0] == 0xAAU;
  if (!is_valid_frame) {
    // if not, discard and continue receiving
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    return;
  }

  // decode the header section
  can_tx_frame.id_type = CAN_ID_STANDARD;
  can_tx_frame.frame_type = CAN_FRAME_DATA;
//  uint32_t timestamp = ((uart_rx_buffer[1])     // timestamp is not used
//      | (uart_rx_buffer[2] << 8U)
//      | (uart_rx_buffer[3] << 16U)
//      | (uart_rx_buffer[4] << 24U));
  can_tx_frame.size = usb_rx_buffer[5];
  can_tx_frame.id = (((uint32_t)usb_rx_buffer[6] << 0U)
                   | ((uint32_t)usb_rx_buffer[7] << 8U)
                   | ((uint32_t)usb_rx_buffer[8] << 16U)
                   | ((uint32_t)usb_rx_buffer[9] << 24U));


  for (uint16_t i=0; i<can_tx_frame.size; i+=1) {
    can_tx_frame.data[i] = usb_rx_buffer[10+i];
  }

  // does not really need this piece of code
//  if (!HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)) {
//    uint32_t fifo_idx = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
//    HAL_FDCAN_AbortTxRequest(&hfdcan1, fifo_idx);
//  }
  CAN_putTxFrame(&hfdcan1, &can_tx_frame);
  usb_evt_happened = 1;
}

void APP_handleCANMessage() {
  CAN_getRxFrame(&hfdcan1, &can_rx_frame);

  // prepare the USB frame
  usb_tx_buffer[0] = PYTHONCAN_START_OF_FRAME;

  usb_tx_buffer[1] = 0x00U;  // Timestamp
  usb_tx_buffer[2] = 0x00U;
  usb_tx_buffer[3] = 0x00U;
  usb_tx_buffer[4] = 0x00U;

  usb_tx_buffer[5] = can_rx_frame.size;  // DLC

  usb_tx_buffer[6] = READ_BITS(can_rx_frame.id, 0xFFU);  // ID
  usb_tx_buffer[7] = READ_BITS(can_rx_frame.id >> 8U, 0xFFU);
  usb_tx_buffer[8] = READ_BITS(can_rx_frame.id >> 16U, 0xFFU);
  usb_tx_buffer[9] = READ_BITS(can_rx_frame.id >> 24U, 0xFFU);

  usb_tx_size = 10;

  for (uint16_t i=0; i<can_rx_frame.size; i+=1) {
    usb_tx_buffer[10+i] = can_rx_frame.data[i];
  }
  usb_tx_size += can_rx_frame.size + 1;

  usb_tx_buffer[10+can_rx_frame.size] = PYTHONCAN_END_OF_FRAME;

  CDC_Transmit_FS(usb_tx_buffer, usb_tx_size);
  can_evt_happened = 1;
}

void APP_init() {
  #if FIRST_TIME_BOOTUP
  APP_initFlashOption();
  #endif

  usb_evt_happened = 0;
  can_evt_happened = 0;

  FDCAN_FilterTypeDef filter_config;
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 0;
  filter_config.FilterID2 = 0;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void APP_main() {
  // status light logic is handled in the main loop, because we need some
  // delay to make LED lights brighter
  if (usb_evt_happened) {
    usb_evt_happened = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  }
  if (can_evt_happened) {
    can_evt_happened = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  }

  // this value controls the "brightness" and granularity of the LED for each activity
  HAL_Delay(1);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}
