/*!
* @file      BLE.c
*
* @brief     BLE hardware abstracted library
*
*/

#include "BLE.h"

//initialize UART device for BLE
void ble_InitUART()
{
  //init uart
  open_Uart();
  
  //set Uart config
  set_Uart_Config();
}

//Close UART device
void ble_CloseUart(ADI_UART_HANDLE hUartDevice) {
  
  close_Uart();
}

// Print a string to the terminal 
void ble_PRINT_C(char* string) {

  int16_t size_l = 0;
  size_l = strlen(string);
  
  //register callback
  register_Callback(true);
  
  //send string to BLE COM port
  parse_str_to_BLE(string,size_l);
  
  //enable rx and tx transfer
  enable_Uart_Tx_Rx(true);
  
  //disable rx and tx transfer
  //enable_Uart_Tx_Rx(false);
  
  //Unregister Callback
  //register_Callback(false);
  
}