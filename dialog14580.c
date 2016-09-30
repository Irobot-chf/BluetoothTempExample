
#include "dialog14580.h"
#include "system.h"
#include "DIALOG_SPI_M3025.h"


#define SPI_ACK 0x02
#define SPI_NACK 0x20

#define RESET_LENGTH     10 //ms
#define MAX_ATTEMPTS     5000

static uint8_t dummy_rx[SPI_MAX_LENGTH];        //Provisional, try to use drivers to transmit without receiving

extern void Delay_ms(unsigned int mSec);

// Local functions
uint8_t calc_crc(uint8_t const * bin, uint32_t length){
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
    crc^=(0xFF&(temp		));
  }
  return crc;
}


uint32_t send_header(uint32_t length, uint8_t crc)
{
  uint8_t spi_tx[4];
  uint8_t spi_rx[4];	
    
  adi_gpio_SetLow(SPI_CS_PORT,SPI_CS_PIN);
  spi_tx[0] = 0xE0;
  spi_tx[1] = 0xC0;
  spi_tx[2] = 0x70;//00;
  spi_tx[3] = 0x50;//05;//length&0xFF;
  spi_rx[0] = 0x00;
  spi_rx[1] = 0x00;
  spi_rx[2] = 0x00;
  spi_rx[3] = 0x00;
  
  
  writeReadSPI(spi_tx, 4 , spi_rx,4 );
  if( spi_rx[3]!=SPI_ACK)
  {
    adi_gpio_SetHigh(SPI_CS_PORT,SPI_CS_PIN);
    return 0;
  }
  
  spi_tx[0] = (length>>8)&0xFF;
  spi_tx[1] = crc;
  spi_tx[2] = 0x00;
  spi_rx[0] = 0x00;
  spi_rx[1] = 0x00;
  spi_rx[2] = 0x00;
  spi_rx[3] = 0x00;
    
  writeReadSPI(spi_tx, 3 , spi_rx,3 );
  if(spi_rx[2]!=SPI_ACK)
  {
    adi_gpio_SetHigh(SPI_CS_PORT,SPI_CS_PIN);
    return 0;
  }
  spi_tx[0] = 0x00;
  writeReadSPI(spi_tx,1,spi_rx,1);
  adi_gpio_SetHigh(SPI_CS_PORT,SPI_CS_PIN);
  return 1;
}


uint8_t send_payload(uint8_t const * bin, uint32_t length)
{
  uint8_t spi_tx[2];
  uint8_t spi_rx[2];
  uint32_t iterations = length/SPI_MAX_LENGTH;
  uint32_t module = length%SPI_MAX_LENGTH;
  
  adi_gpio_SetLow(SPI_CS_PORT,SPI_CS_PIN);
  
  //uint32_t counter = 0;
  for(uint32_t i = 0 ; i< iterations ; i++)
  {
    writeReadSPI(bin+i*SPI_MAX_LENGTH,SPI_MAX_LENGTH,dummy_rx,SPI_MAX_LENGTH);
  }
  
  if(module > 0)
  {
    writeReadSPI(bin+iterations*SPI_MAX_LENGTH,module,dummy_rx,module);
  }
  
  spi_tx[0] = 0x00;
  spi_tx[1] = 0x00;
  
  writeReadSPI(spi_tx,2,spi_rx,2);
  
  
  adi_gpio_SetHigh(SPI_CS_PORT,SPI_CS_PIN);
  
  if(spi_rx[1]!=SPI_ACK || spi_rx[0]!=0xAA)
    return 0;
  
  return 1;
}



//Boot function
uint32_t adi_Dialog14580_SPI_Boot(uint8_t const * bin, uint32_t length)
{
  uint8_t crc;
  uint8_t header_ack;
  uint8_t payload_ack;
  uint32_t attempt = 0;
    
  //Initialize SPI
  if(initDialogSPI() != 0)
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
      header_ack = send_header(length/4,crc);
      if(header_ack)
      {
        payload_ack = send_payload(bin,length);
      }
      attempt++;
    } while(((header_ack&payload_ack)!=1) && (attempt<MAX_ATTEMPTS));
  
  //Uninitialize SPI
  if(unInitDialogSPI() != 0)
      return 2;
  
  if(attempt == MAX_ATTEMPTS)
    return 3;
  
  //On successful boot, D51 goes out
  adi_gpio_SetHigh(BLE_LED_PORT, BLE_LED_PIN);
  return 0;
  
}
