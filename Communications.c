

#include <drivers/uart/adi_uart.h>
#include "Communications.h"

uint8_t         UartDeviceMem[UART_MEMORY_SIZE];
ADI_UART_HANDLE hUartDevice;
unsigned char 	RxBuffer[];
unsigned char   TxBuffer[];
ADI_UART_RESULT  eUartResult;
bool data_sent = false;
bool data_received = false;

//ADI_SPI_RESULT eSpiResult;


/********************************************************************
* UART Interrupt calback                                            *
*********************************************************************/
void UARTCallback( void *pAppHandle, uint32_t nEvent, void *pArg)
{
   //CASEOF (event type)
    switch (nEvent)
    {
        //CASE (TxBuffer has been cleared, Data sent) 
        case ADI_UART_EVENT_TX_BUFFER_PROCESSED:
                adi_uart_EnableTx(hUartDevice, false);//disable tx buffer
                data_sent = true;
                break;
                
        //CASE (RxBuffer has been cleared, Data recieved) 
        case ADI_UART_EVENT_RX_BUFFER_PROCESSED:
                adi_uart_EnableRx(hUartDevice, false);//disable rx buffer
                data_received = true;
                break;
                
    default: break;
    }
}


/**********************************************************************************************
* Function Name: UART_Init                                                                   
* Description  : This function initializes an instance of the UART driver for UART_DEVICE_NUM 
* Arguments    : void                                                                         
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Uart_Init(void)
{
 
  //open Uart
  eUartResult = adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,
                UartDeviceMem,
                UART_MEMORY_SIZE,
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


/**********************************************************************************************
* Function Name: UART_Close                                                                   
* Description  : This function releases a UART handle as initialised by Uart_Init
* Arguments    : void                                                                         
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Uart_Close(void)
{
  //close Uart device
  eUartResult = adi_uart_Close(hUartDevice);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  else
    return 0;
}


/**********************************************************************************************
* Function Name: UART_ReadWrite                                                                   
* Description  : This function parses a string to the TxBuffer, submits it and submits an 
*                empty RxBuffer before enabling the dataflow for both buffers.
*                data_recieved/sent flag set by callback
* Arguments    : char* string = string to be sent                                                                       
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Uart_ReadWrite(char* string)
{
  //clear flags
  data_sent = false;
  data_received = false;
  
  //ensure data transfer is disabled for submitting buffers
  eUartResult = adi_uart_EnableRx(hUartDevice,false);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  eUartResult = adi_uart_EnableTx(hUartDevice,false); 
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //length of string
  int16_t size_l = 0; 
  size_l = strlen(string);
  
  //parse string to TxBuffer for submitting
  for(int i = 0; i < size_l; i++) // parse string to Tx buffer
    TxBuffer[i] = string[i];  //TODO: Possible parsing error. 4th char transfers all remaining chars. 2nd char not transfered on UART
  
  //'empty' RxBuffer using NULL char
  RxBuffer[0] = '\0';

  //submit RxBuffer to receive data
  eUartResult = adi_uart_SubmitRxBuffer(hUartDevice, RxBuffer, 1);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //submit TxBuffer for sending data
  eUartResult = adi_uart_SubmitTxBuffer(hUartDevice, TxBuffer, size_l);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  // Enable the Data flow for Rx. This is disabled by UARTCallback
  eUartResult = adi_uart_EnableRx(hUartDevice,true);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  // Enable the Data flow for Tx. This is disabled by UARTCallback
  eUartResult = adi_uart_EnableTx(hUartDevice,true); 
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  else
    //wait for data sent
    while(data_sent == false)
    {
      Delay_ms(10);
    }
  
    return 0;
}
     

/**********************************************************************************************
* Function Name: UART_Read                                                                  
* Description  : This function submits an empty RxBuffer before enabling the dataflow for the receive
*                buffer. data_recieved flag set by callback
* Arguments    : void                                                                         
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Uart_Read(void)
{
  //clear flag
  data_received = false;
  
  //ensure data transfer is disabled for submitting buffers
  eUartResult = adi_uart_EnableRx(hUartDevice,false);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;

  //'empty' RxBuffer using NULL char
  RxBuffer[0] = '\0';

  //submit RxBuffer to receive data
  eUartResult = adi_uart_SubmitRxBuffer(hUartDevice, RxBuffer, 1);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  // Enable the Data flow for Rx. This is disabled by UARTCallback
  eUartResult = adi_uart_EnableRx(hUartDevice,true);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  else
    
    //wait for data sent
    while(data_received == false)
    {
      Delay_ms(10);
    }
  
    return 0;
}

/**********************************************************************************************
* Function Name: UART_Write                                                                   
* Description  : This function parses a string to the TxBuffer, submits it before 
*                enabling the dataflow for the transfer buffers. data_sent flag set by callback
* Arguments    : char* string = string to be sent                                                                        
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Uart_Write(char* string)
{
  //clear flag
  data_sent = false;
  
  //ensure data transfer is disabled for submitting buffers
  eUartResult = adi_uart_EnableTx(hUartDevice,false); 
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //length of string
  int16_t size_l = 0; 
  size_l = strlen(string);
  
  //parse string to TxBuffer for submitting
  for(int i = 0; i < size_l; i++) // parse string to Tx buffer
      TxBuffer[i] = string[i];
  
  //submit TxBuffer for sending data
  eUartResult = adi_uart_SubmitTxBuffer(hUartDevice, TxBuffer, size_l);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  // Enable the Data flow for Tx. This is disabled by UARTCallback
  eUartResult = adi_uart_EnableTx(hUartDevice,true); 
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  else
    
    //wait for data sent
    while(data_sent == false)
    {
      Delay_ms(10);
    }
  
    return 0;
}

/*
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