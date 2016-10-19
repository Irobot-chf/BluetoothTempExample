

#include <drivers/uart/adi_uart.h>
#include <drivers/spi/adi_spi.h>

#include "Communications.h"

uint8_t                 UartDeviceMem[UART_MEMORY_SIZE];//UART memory size
ADI_UART_HANDLE         hUartDevice;//UART device handle
unsigned char 	        TxBuffer[];//UART transmit buffer
unsigned char 	        RxBuffer[];//UART receive buffer
ADI_UART_RESULT         eUartResult;//UART error variable
bool data_sent =        false;//UART data_sent flag
bool data_received =    false;//UART data_recieved flag


ADI_SPI_RESULT          eSpiResult; //SPI error variable
static ADI_SPI_HANDLE   hSPIDevice; //SPI handle
static uint8_t          SPIMem[ADI_SPI_MEMORY_SIZE];//SPI memory size
static ADI_SPI_TRANSCEIVER transceive;//transceive struct for SPI Read/Writes


/********************************************************************
* UART Interrupt callback                                            *
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
	
	//register callback
  adi_uart_RegisterCallback(hUartDevice,UARTCallback,hUartDevice);
		
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
unsigned char Uart_ReadWrite(char *string)
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

  //'empty' RxBuffer using NULL char
  RxBuffer[0] = '\0';

  //submit RxBuffer to receive data
  eUartResult = adi_uart_SubmitRxBuffer(hUartDevice, RxBuffer, 1);
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //submit TxBuffer for sending data
  eUartResult = adi_uart_SubmitTxBuffer(hUartDevice, string, size_l);
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
unsigned char Uart_Write(char* TxBuffer)
{
  //clear flag
  data_sent = false;
  
  //ensure data transfer is disabled for submitting buffers
  eUartResult = adi_uart_EnableTx(hUartDevice,false); 
  if(eUartResult != ADI_UART_SUCCESS)
    return 1;
  
  //length of string
  int16_t size_l = 0; 
  size_l = strlen(TxBuffer);
  
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


/**********************************************************************************************
* Function Name: Spi_Init                                                                   
* Description  : This function initializes SPI and creates a handle which is configured accordingly
* Arguments    : void                                                                       
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Spi_Init(void)
{
  eSpiResult = adi_spi_Open(SPI_DEV_NUM,SPIMem,ADI_SPI_MEMORY_SIZE,&hSPIDevice);
  if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
  
  //Set the SPI clock rate 
  eSpiResult = adi_spi_SetBitrate(hSPIDevice,SPI_BITRATE);
  if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
  
  //Set the chip select 
  eSpiResult = adi_spi_SetChipSelect(hSPIDevice, SPI_CS_NUM);
  if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
  
  //Enable continue mode (Chip Select remains low until the end of the transaction
  eSpiResult = adi_spi_SetContinousMode(hSPIDevice, true);
  if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
  
  //Disable DMA
  eSpiResult = adi_spi_EnableDmaMode(hSPIDevice, false);
  if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
  
  else
    return 0;  
}


/**********************************************************************************************
* Function Name: Spi_Close                                                                   
* Description  : This function closes the existing SPI handle
* Arguments    : void                                                                       
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Spi_Close(void)
{
  eSpiResult = adi_spi_Close(hSPIDevice);
  if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;

  else
    return 0;
}


/**********************************************************************************************
* Function Name: Spi_ReadWrite                                                                   
* Description  : This function configures a transceive struct to write and read from the SPI object
* Arguments    : uint8_t const* TxArray = Transmit Array
*                uint16_t TxLength = Transmit length (bytes)
*                uint8_t* RxArray = Receive Array
*                uint16_t RxLength = Recieve length (bytes)                                                                   
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Spi_ReadWrite(uint8_t const* TxArray, uint16_t TxLength, uint8_t* RxArray, uint16_t RxLength)
{  
  //enable DMA mode to manage transfers in background
   eSpiResult = adi_spi_EnableDmaMode(hSPIDevice, true);
   if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;

   //setup transceive struct with Tx and Rx buffers
   transceive.TransmitterBytes = TxLength;
   transceive.ReceiverBytes = RxLength;
   transceive.nTxIncrement = true;
   transceive.nRxIncrement = true;
   transceive.pReceiver = RxArray;
   transceive.pTransmitter = (uint8_t *)TxArray;
   
   //commit transeive struct to write read operation
   eSpiResult = adi_spi_ReadWrite(hSPIDevice,&transceive);
   if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
   
   //disable DMA mode
   eSpiResult = adi_spi_EnableDmaMode(hSPIDevice, false);
   if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
   
    else
    return 0;
}


/**********************************************************************************************
* Function Name: Spi_Write                                                                   
* Description  : This function configures a transceive struct to write to the SPI object
* Arguments    : void                                                                       
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
unsigned char Spi_Write(uint8_t const * TxArray, uint8_t TxLength)
{
  //enable DMA mode to manage transfers in background
   eSpiResult = adi_spi_EnableDmaMode(hSPIDevice, true);
   if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
   
   //setup transceive struct with Tx buffer
   transceive.TransmitterBytes = TxLength;                               
   transceive.ReceiverBytes = 0;
   transceive.nTxIncrement = true;
   transceive.nRxIncrement = false;
   transceive.pReceiver = NULL;
   transceive.pTransmitter = (uint8_t *)TxArray;
   
   //commit transeive struct to write read operation
   eSpiResult = adi_spi_ReadWrite(hSPIDevice,&transceive);
   if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
   
   //disable DMA mode
   eSpiResult = adi_spi_EnableDmaMode(hSPIDevice, false);      
   if(eSpiResult != ADI_SPI_SUCCESS)
    return 1;
   
   else
    return 0;
}
