/*!
* @file     BLE.h
*
* @brief    Primary header file for BLE communication driver functions
*
* @details  Primary header file for BLE specific communication via
*           UART and eventually SPI. String is passed to BLE and transmitted via bluetooth
*
*/

#ifndef BLE_H
#define BLE_H

#include "uart_handler.h"
#include "Ble_adi_specific.h"

//initialize UART device for BLE
void ble_InitUART();

//Close UART device
void ble_CloseUart(ADI_UART_HANDLE hUartDevice);

// Print a string to the terminal 
void ble_PRINT_C(char* string);

#endif // BLE_H