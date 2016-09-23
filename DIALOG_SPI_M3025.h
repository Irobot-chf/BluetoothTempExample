
#ifndef __DIALOG_SPI_M3025_H
#define __DIALOG_SPI_M3025_H


#include <drivers/spi/adi_spi.h>

#define SPI_MAX_LENGTH 252      //MUST BE MULTIPLE OF 4

#define SPI_CS_PIN     ADI_GPIO_PIN_9
#define SPI_CS_PORT    ADI_GPIO_PORT1
                                              
uint8_t initDialogSPI();
uint8_t unInitDialogSPI();
ADI_SPI_RESULT writeSPI(uint8_t const * _array, uint8_t _length);
ADI_SPI_RESULT writeReadSPI(uint8_t const * _arrayW, uint16_t _lengthW, uint8_t* _arrayR, uint16_t _lengthR);

#endif