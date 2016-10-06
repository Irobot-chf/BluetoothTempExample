/*!
* @file      BLE.c
*
* @brief     Driver for BLE bluetooth communication
*
*/

#include "BLE.h"

uint8_t UartDeviceMem[ADI_UART_MEMORY_SIZE];
unsigned char 	RxBuffer[];

void UARTCallback( void *pAppHandle, uint32_t nEvent, void *pArg)
{
   uint8_t Index;
}

//initialize UART device for BLE
ADI_UART_RESULT InitUART()
{
  // Variable for storing the return code from UART device 
  ADI_UART_RESULT  eUartResult;
  
 eUartResult = adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,UartDeviceMem,ADI_UART_MEMORY_SIZE,&hUartDevice);
  

  // Configure  UART device with NO-PARITY, ONE STOP BIT and 8bit word length. 
  eUartResult = adi_uart_SetConfiguration(hUartDevice,
                                              ADI_UART_NO_PARITY,
                                              ADI_UART_ONE_AND_HALF_TWO_STOPBITS,
                                              ADI_UART_WORDLEN_8BITS);
  
  // Baud rate div values are calcuated for PCLK 26Mhz. Please use the
  //host utility UartDivCalculator.exe provided with the installer"
  
  eUartResult = adi_uart_ConfigBaudRate(hUartDevice,
                                            UART_DIV_C_115200,
                                            UART_DIV_M_115200,
                                            UART_DIV_N_115200,
                                            UART_OSR_115200);

  return eUartResult;
}

//Close UART device
void CloseUart(ADI_UART_HANDLE hUartDevice) {
  
  adi_uart_Close(hUartDevice);
}

// Print a string to the terminal 
ADI_UART_RESULT PRINT_C(char* string) {

  int16_t size_l = 0;
  size_l = strlen(string);
  
  // Variable for storing the return code from UART device 
  ADI_UART_RESULT  eUartResult;
  
     
  // Enable the Data flow for Rx 
  eUartResult = adi_uart_EnableRx(hUartDevice,true);

  // Enable the Data flow for Tx 
  eUartResult = adi_uart_EnableTx(hUartDevice,true); 
  
  eUartResult = adi_uart_RegisterCallback(hUartDevice,UARTCallback,(void*)0);
  eUartResult = adi_uart_SubmitRxBuffer(hUartDevice,RxBuffer,1);
  
  for(int i = 0; i < size_l; i++) { // for BLE Payload
    pADI_UART0->COMTX = string[i];
    while((pADI_UART0->COMLSR &((uint16_t) BITM_UART_COMLSR_TEMT)) == 0 );
    }
  
     
  // Disable the Data flow for Rx 
  eUartResult = adi_uart_EnableRx(hUartDevice,false);

  // Diasable the Data flow for Tx 
  eUartResult = adi_uart_EnableTx(hUartDevice,false); 
  
  //Unregister Callback
  eUartResult = adi_uart_RegisterCallback(hUartDevice, 0, 0);
  
  return eUartResult;
}