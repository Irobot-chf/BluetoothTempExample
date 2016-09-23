#ifndef _UART_COMMAND_HANDLER_
#define _UART_COMMAND_HANDLER_

#include <drivers/uart/adi_uart.h>

typedef struct{
    uint8_t rxBuffer[128];
    uint8_t txBuffer[128];
    uint8_t commandBuffer[16];
    uint8_t rxCounter;
    uint8_t rxRequestedBytes;
    uint32_t txComplete;
    uint32_t rxComplete;
    uint32_t txInProgress;
    uint32_t rxInProgress;
}TUartCommandHandler;

uint8_t getNumOfReceivedChar(TUartCommandHandler *pUartCommandHandler);
void initHandler(TUartCommandHandler *pUartCommandHandler);
void receiveCommand(TUartCommandHandler *pUartCommandHandler, int expectedLength);
void sendCommand(TUartCommandHandler *pUartCommandHandler, uint8_t* pCommand, int commandLength);
void uart_callback( void *pAppHandle, uint32_t nEvent, void *pArg);
bool isRxComplete(TUartCommandHandler *pUartCommandHandler);
bool isTxComplete(TUartCommandHandler *pUartCommandHandler);

#endif