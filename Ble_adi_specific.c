/*!
* @file      Ble_adi_specific.c
*
* @brief     Driver for BLE bluetooth communication specific to ADUCM3029. Used by BLE.c
*
*/

#include "Ble_adi_specific.h"


uint8_t         UartDeviceMem[ADI_UART_MEMORY_SIZE];
ADI_UART_HANDLE hUartDevice;
unsigned char 	RxBuffer[];


void ble_UARTCallback( void *pAppHandle, uint32_t nEvent, void *pArg)
{
   /* CASEOF (event type) */
    switch (nEvent)
    {
        /* CASE (buffer processed) */
        case ADI_UART_EVENT_TX_BUFFER_PROCESSED:
                adi_uart_EnableTx(hUartDevice, false);
                break;
                
        case ADI_UART_EVENT_RX_BUFFER_PROCESSED:
                adi_uart_EnableRx(hUartDevice, false);
                break;
    }
}


void open_Uart()
{  
  //open Uart
  adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,
                UartDeviceMem,
                ADI_UART_MEMORY_SIZE,
                &hUartDevice);
}


void close_Uart() 
{
  adi_uart_Close(hUartDevice);
}


void set_Uart_Config()
{
  // Configure  UART device with NO-PARITY, ONE STOP BIT and 8bit word length. 
  adi_uart_SetConfiguration(hUartDevice,
                            ADI_UART_NO_PARITY,
                            ADI_UART_ONE_AND_HALF_TWO_STOPBITS,
                            ADI_UART_WORDLEN_8BITS);
  
  //set baud rate at 115200
  adi_uart_ConfigBaudRate(hUartDevice,
                          UART_DIV_C_115200,
                          UART_DIV_M_115200,
                          UART_DIV_N_115200,
                          UART_OSR_115200);
}


void enable_Uart_Tx_Rx(bool Enable)
{
  // Enable the Data flow for Rx 
  adi_uart_EnableRx(hUartDevice,Enable);

  // Enable the Data flow for Tx 
  adi_uart_EnableTx(hUartDevice,Enable); 
}


void register_Callback(bool Enable)
{
  if(Enable)
  {
    //register callback
    adi_uart_RegisterCallback(hUartDevice,ble_UARTCallback,/*(void*)0*/hUartDevice);
    //submit buffer
    adi_uart_SubmitRxBuffer(hUartDevice,RxBuffer,1);
  }
  
  else
  {
    //clear callback (disable interrupt)
    //adi_uart_RegisterCallback(hUartDevice, 0, 0);
  }
}


void parse_str_to_BLE(char* string,int16_t size_l)
{
  for(int i = 0; i < size_l; i++) { // for BLE Payload
      pADI_UART0->COMTX = string[i];
      while((pADI_UART0->COMLSR &((uint16_t) BITM_UART_COMLSR_TEMT)) == 0 );
      }
}
