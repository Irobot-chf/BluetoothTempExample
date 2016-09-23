#include "uart_command_handler.h"
#include "uart_handler.h"
uint8_t UartDeviceMemo[ADI_UART_MEMORY_SIZE];

TUartCommandHandler uartCommandHandler;

extern ADI_UART_HANDLE hUartDevice;

uint8_t getNumOfReceivedChar(TUartCommandHandler *pUartCommandHandler)
{
    return pUartCommandHandler->rxCounter;
}

bool isRxComplete(TUartCommandHandler *pUartCommandHandler)
{
    return pUartCommandHandler->rxComplete;
}

bool isTxComplete(TUartCommandHandler *pUartCommandHandler)
{
    return pUartCommandHandler->txComplete;
}

void uart_callback( void *pAppHandle, uint32_t nEvent, void *pArg)
{
    TUartCommandHandler *pUartCommandHandler = &uartCommandHandler;
    switch(nEvent)
    {
    case ADI_UART_EVENT_RX_BUFFER_PROCESSED:
      memset(pUartCommandHandler->commandBuffer, 0, 16);
      memcpy(pUartCommandHandler->commandBuffer, pUartCommandHandler->rxBuffer, pUartCommandHandler->rxRequestedBytes);
      pUartCommandHandler->rxComplete = 1;
      pUartCommandHandler->rxInProgress = 0;     
      break;
      
    case ADI_UART_EVENT_TX_BUFFER_PROCESSED:
      pUartCommandHandler->txComplete = 1;
      pUartCommandHandler->txInProgress = 0;
      break;
      
    default:
      asm("bkpt 0");
      break;
    }
  //TODO
}

void initHandler(TUartCommandHandler *pUartCommandHandler)
{
    pUartCommandHandler->txComplete = false;
    pUartCommandHandler->rxComplete = false;
    ADI_UART_RESULT  eUartResult;

    adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,UartDeviceMemo,ADI_UART_MEMORY_SIZE,&hUartDevice);
    eUartResult = adi_uart_SetConfiguration(hUartDevice,
                                                ADI_UART_NO_PARITY,
                                                ADI_UART_ONE_AND_HALF_TWO_STOPBITS,
                                                ADI_UART_WORDLEN_8BITS);
    
    eUartResult = adi_uart_ConfigBaudRate(hUartDevice,
                                              UART_DIV_C_57600,
                                              UART_DIV_M_57600,
                                              UART_DIV_N_57600,
                                              UART_OSR_57600);
    /* Enable the Data flow for Rx */
    eUartResult = adi_uart_EnableRx(hUartDevice,true);
    
    /* Enable the Data flow for Tx */
    eUartResult = adi_uart_EnableTx(hUartDevice,true); 
    
    eUartResult = adi_uart_RegisterCallback(hUartDevice, uart_callback,(void*)pUartCommandHandler);
}




void receiveCommand(TUartCommandHandler *pUartCommandHandler, int expectedLength)
{
    if(pUartCommandHandler->rxInProgress)
        return;   // fix
    pUartCommandHandler->rxRequestedBytes = expectedLength;
    pUartCommandHandler->rxCounter = 0;
    pUartCommandHandler->rxInProgress = 1;
    ADI_UART_RESULT  eUartResult;
    eUartResult = adi_uart_SubmitRxBuffer(hUartDevice, pUartCommandHandler->rxBuffer, expectedLength);
}
void sendCommand(TUartCommandHandler *pUartCommandHandler, uint8_t* pCommand, int commandLength)
{
    if(pUartCommandHandler->txInProgress)
      return; // mal by vratit error ne void
    pUartCommandHandler->txInProgress = 1;
    pUartCommandHandler->txComplete = 0;
    memcpy(pUartCommandHandler->txBuffer, pCommand, commandLength);
    adi_uart_SubmitTxBuffer(hUartDevice, pUartCommandHandler->txBuffer, commandLength);
    //adi_uart_Write(hUartDevice, uartCommandHandler->txBuffer, commandLength);
}
