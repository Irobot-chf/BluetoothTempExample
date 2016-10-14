

#include "uart_handler.h"
#include "Communications.h"


#define SPI_DEV_NUM     0
#define SPI_CS_NUM      ADI_SPI_CS0
#define SPI_BITRATE     300000

uint8_t         UartDeviceMem[ADI_UART_MEMORY_SIZE];
ADI_UART_HANDLE hUartDevice;
unsigned char 	RxBuffer[];
unsigned char   TxBuffer[];
ADI_UART_RESULT  eUartResult;


ADI_SPI_RESULT = eSpiResult;


unsigned char Uart_Init(void)
{
 
  //open Uart
  eUartResult = adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,
                UartDeviceMem,
                ADI_UART_MEMORY_SIZE,
                &hUartDevice);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //configure UART device with NO-PARITY, ONE STOP BIT and 8bit word length. 
  eUartResult = adi_uart_SetConfiguration(hUartDevice,
                            ADI_UART_NO_PARITY,
                            ADI_UART_ONE_AND_HALF_TWO_STOPBITS,
                            ADI_UART_WORDLEN_8BITS);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //set baud rate at 115200
  eUartResult = adi_uart_ConfigBaudRate(hUartDevice,
                          UART_DIV_C_115200,
                          UART_DIV_M_115200,
                          UART_DIV_N_115200,
                          UART_OSR_115200);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  else
    return 0;
}


unsigned char Uart_Close(void)
{
  //close Uart device
  eUartResult = adi_uart_Close(hUartDevice);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  else
    return 0;
}


unsigned char Uart_ReadWrite(char* string)
{
  int16_t size_l = 0;
  size_l = strlen(string);
  
  for(int i = 0; i < size_l; i++) // parse string to Tx buffer
      TxBuffer[i] = string[i];
      
  RxBuffer[0] = '\0';
  
  eUartResult = adi_uart_SubmitRxBuffer(hUartDevice, RxBuffer, 1);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  eUartResult = adi_uart_SubmitTxBuffer(hUartDevice, TxBuffer, size_l);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  // Enable the Data flow for Rx 
  eUartResult = adi_uart_EnableRx(hUartDevice,Enable);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  // Enable the Data flow for Tx 
  eUartResult = adi_uart_EnableTx(hUartDevice,Enable); 
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  else
    return 0;
}
                          

unsigned char Spi_Init()
{
  eSpiResult = adi_spi_Open(SPI_DEV_NUM,SPIMem,ADI_SPI_MEMORY_SIZE,&hSPIDevice);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //Set the SPI clock rate 
  eSpiResult = adi_spi_SetBitrate(hSPIDevice,SPI_BITRATE);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //Set the chip select 
  eSpiResult = adi_spi_SetChipSelect(hSPIDevice, SPI_CS_NUM);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //Enable continue mode (Chip Select remains low until the end of the transaction
  eSpiResult = adi_spi_SetContinousMode(hSPIDevice, true);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //Disable DMA
  eSpiResult = adi_spi_EnableDmaMode(hSPIDevice, false);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  else
    return 0;  
}


unsigned char Spi_Close()
{
  eSpiResult = adi_spi_Close(hSPIDevice);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;

  else
    return 0;
}
/*
unsigned char Spi_Write

unsigned char Spi_ReadWrite
*/