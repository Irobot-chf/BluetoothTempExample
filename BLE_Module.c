
#include "BLE_Module.h"
#include "system.h"
#include "Communications.h"

#define SPI_CS_NUM      ADI_SPI_CS0


static uint8_t dummy_rx[SPI_MAX_LENGTH];    //Provisional, try to use drivers to transmit without receiving

extern void Delay_ms(unsigned int mSec);//delay function


/******************** Local functions ********************/
/**********************************************************************************************
* Function Name: calc_crc                                                                  
* Description  : Calculates a check value
* Arguments    : uint8_t const* bin = binary file to be wrote to BLE module
*                uint32_t length = file length in bytes                                                                   
* Return Value : crc = check value
**********************************************************************************************/
uint8_t calc_crc(uint8_t const* bin, uint32_t length)
{
  uint32_t i;
  uint32_t temp;
  uint8_t crc;
  crc=0xFF;
  
  for(i=0;i<length;i++)
  {
    temp=* ( volatile uint32_t*)(bin+4*i);
    crc^=(0xFF&(temp>>24));
    crc^=(0xFF&(temp>>16));
    crc^=(0xFF&(temp>>8 ));
    crc^=(0xFF&(temp    ));
  }
  return crc;
}


/**********************************************************************************************
* Function Name: send_header                                                                
* Description  : sends header in form of a handshake and crc value. BLE responds 
*                to acknowledge that it's ready for binary file
* Arguments    : uint32_t length = length in bytes
*                uint8_t crc = check value                                      
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
uint32_t send_header(uint32_t length, uint8_t crc)
{
	uint8_t spi_tx[4];//Tx buffer
  uint8_t spi_rx[4];//Rx buffer
  
  //handshake to determine if BLE is ready to recieve check value
  spi_tx[0] = 0x70;//preamble
  spi_tx[1] = 0x50;//preamble
  spi_tx[2] = 0x00;//empty
  spi_tx[3] = length&0xFF;//length LSB
  
  spi_rx[0] = 0x00;
  spi_rx[1] = 0x00;
  spi_rx[2] = 0x00;
  spi_rx[3] = 0x00;
  
  
  Spi_ReadWrite(spi_tx, 4 , spi_rx,4 );
  
  //check if first step was acknowledged
  if( spi_rx[3]!=SPI_ACK)
  {
    return 1;
  }

  //send length and crc
  spi_tx[0] = (length>>8)&0xFF;//length MSB
  spi_tx[1] = crc;//crc
  spi_tx[2] = 0x00;//mode byte 8-bit mode
  spi_rx[0] = 0x00;
  spi_rx[1] = 0x00;
  spi_rx[2] = 0x00;
  spi_rx[3] = 0x00;
    
  Spi_ReadWrite(spi_tx, 3 , spi_rx,3 );
  
  //check that length and crc was acknowledged
  if(spi_rx[2]!=SPI_ACK)
  {
    return 1;
  }
  
  spi_tx[0] = 0x00;//empty before data bytes
  Spi_ReadWrite(spi_tx,1,spi_rx,1);
  
  return 0;
}


/**********************************************************************************************
* Function Name: send_payload                                                               
* Description  : sends data bytes to program BLE module
* Arguments    : uint8_t const* bin = file to be sent
*                uint32_t length = length in bytes                         
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
uint8_t send_payload(uint8_t const* bin, uint32_t length)
{
  uint8_t spi_tx[2];//Tx buffer
  uint8_t spi_rx[2];//Rx buffer
  uint32_t iterations = length/SPI_MAX_LENGTH;//how many loops needed to send payload with limited SPI memory
  uint32_t module = length%SPI_MAX_LENGTH; //check if length matches spi memory length exactly
  
  
  //send payload
  for(uint32_t i = 0 ; i < iterations ; i++)
  {
    Spi_ReadWrite(bin+i*SPI_MAX_LENGTH,SPI_MAX_LENGTH,dummy_rx,SPI_MAX_LENGTH);
  }
  
  //send remainder
  if(module > 0)
  {
    Spi_ReadWrite(bin+iterations*SPI_MAX_LENGTH,module,dummy_rx,module);
  }
  
  //end with empty bytes
  spi_tx[0] = 0x00;
  spi_tx[1] = 0x00;
  
  Spi_ReadWrite(spi_tx,2,spi_rx,2);
  
  //check acknowledgement
  if(spi_rx[1]!=SPI_ACK || spi_rx[0]!=0xAA)
    return 1;
  
  return 0;
}


/**********************************************************************************************
* Function Name: Ble_Spi_Boot                                                               
* Description  : Main boot function
* Arguments    : uint8_t const * bin = file to be sent
*                uint32_t length = length in bytes                        
* Return Value : 0 = Success                                                                    
*                1 = Failure (See eUartResult in debug mode for adi micro specific info)     
**********************************************************************************************/
uint32_t Ble_Spi_Boot(uint8_t const * bin, uint32_t length)
{
  uint8_t crc;//check value
  uint8_t header_ack;//header acknowledgement
  uint8_t payload_ack;//payload acknowledgement
  uint32_t attempt = 0;//attempt count
  
  //Init GPIOs for RST and Indicator LED
  adi_gpio_SetHigh(BLE_LED_PORT, BLE_LED_PIN);
  adi_gpio_OutputEnable(BLE_LED_PORT, BLE_LED_PIN, true);//BLE Ready LED
  adi_gpio_SetHigh(BLE_LED_PORT, BLE_LED_PIN);
  adi_gpio_OutputEnable(BLE_RST_PORT, BLE_RST_PIN, true);//BLE Reset Pin
  
  //Initialize SPI
  if(Spi_Init() != 0)
      return 1;
  
  //Reset dialog (Active High Reset)
  adi_gpio_SetHigh(BLE_RST_PORT,BLE_RST_PIN);
  
  Delay_ms(RESET_LENGTH);//ensure reset is recognised due to internal RC filter
  
  //complete reset
  adi_gpio_SetLow(BLE_RST_PORT,BLE_RST_PIN);
  
  //calculate Checksum
  crc = calc_crc(bin,length/4);
  
  //Add wait 110ms here to improve efficiency.
  Delay_ms(110);
  
  //Boot Dialog
  do{
    //send header
      header_ack = send_header(length/4,crc);
      
      //if header is successful
      if(header_ack == 0)
      {
        //send payload
        payload_ack = send_payload(bin,length);
      }
      attempt++;
    } while(((header_ack | payload_ack) == 1) && (attempt<MAX_ATTEMPTS));
  
  //Uninitialize SPI
  if(Spi_Close() != 0)
      return 1;
  
  if(attempt == MAX_ATTEMPTS)
    return 1;
  
  //On successful boot, D51 goes out
  adi_gpio_SetLow(BLE_LED_PORT, BLE_LED_PIN);
  return 0;
  
}
