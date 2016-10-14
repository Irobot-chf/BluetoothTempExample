/*!
* @file     BLE_ADI_SPECIFIC.h
*
* @brief    adi ADUCM3029 specific functions
*
* @details  hardware specific functions to allow easy porting
*           
*
*/

#ifndef BLE_ADI_SPECIFIC_H
#define BLE_ADI_SPECIFIC_H

#include "uart_handler.h"

//Callback for tx/rx
void ble_UARTCallback();

//open UART
void open_Uart();

//close UART
void close_Uart();

//set Uart config
void set_Uart_Config();

//enable rx and tx transfer
void enable_Uart_Tx_Rx(bool Enable);

//register callback
void register_Callback(bool Enable);

//send string to BLE COM port
void parse_str_to_BLE(char* string,int16_t size_l);

#endif // BLE_ADI_SPECIFIC_H