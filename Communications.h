
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/

#include "adi_types.h"


/******************************************************************************/
/* spi driver parameters                                                      */
/******************************************************************************/

#define SPI_DEV_NUM             0
#define SPI_CS_NUM              ADI_SPI_CS0
#define SPI_BITRATE             300000
#define SPI_MAX_LENGTH          252      //MUST BE MULTIPLE OF 4

/******************************************************************************/
/* UART driver parameters                                                     */
/******************************************************************************/
   
#define UART_DEVICE_NUM         0

#define UART_MEMORY_SIZE    (ADI_UART_BIDIR_MEMORY_SIZE)

/*
                    Boudrate divider for PCLK-26000000

+------------------------------------------------------------------------+
| CALCULATING UART DIV REGISTER VALUE FOR THE  INPUT CLOCK: 26000000     |
|------------------------------------------------------------------------|
|       BAUDRATE       DIV-C     DIV-M     DIV-N         OSR    DIFF     |
|------------------------------------------------------------------------|
|       00009600        0022      0003      1734        0003    0000     |
|------------------------------------------------------------------------|
|       00019200        0011      0003      1735        0003    0000     |
|------------------------------------------------------------------------|
|       00038400        0017      0001      0501        0003    0000     |
|------------------------------------------------------------------------|
|       00057600        0007      0002      0031        0003    0000     |
|------------------------------------------------------------------------|
|       00115200        0007      0002      0031        0002    0000     |
|------------------------------------------------------------------------|
|       00230400        0007      0002      0031        0001    0000     |
|------------------------------------------------------------------------|
|       00460800        0007      0002      0031        0000    0001     |
|------------------------------------------------------------------------|


*/
/* Select the boudrate divider for 57600 */
#define UART_DIV_C_9600         22
#define UART_DIV_C_19200        11
#define UART_DIV_C_38400        17
#define UART_DIV_C_57600        7
#define UART_DIV_C_115200       7
#define UART_DIV_C_230400       7
#define UART_DIV_C_460800       7

#define UART_DIV_M_9600         3
#define UART_DIV_M_19200        3
#define UART_DIV_M_38400        1
#define UART_DIV_M_57600        2
#define UART_DIV_M_115200       2
#define UART_DIV_M_230400       2
#define UART_DIV_M_460800       2

#define UART_DIV_N_9600         1734
#define UART_DIV_N_19200        1735
#define UART_DIV_N_38400        501
#define UART_DIV_N_57600        31
#define UART_DIV_N_115200       31
#define UART_DIV_N_230400       31
#define UART_DIV_N_460800       31

#define UART_OSR_9600           3
#define UART_OSR_19200          3
#define UART_OSR_38400          3
#define UART_OSR_57600          3
#define UART_OSR_115200         2
#define UART_OSR_230400         1
#define UART_OSR_460800         0


/******************************************************************************/
/* Function Prototypes                                                       */
/******************************************************************************/

//delay
extern void Delay_ms(unsigned int mSec);

//init UART
unsigned char Uart_Init(void);

//close UART
unsigned char Uart_Close(void);

//setup buffers to read and write to UART device initialized by init UART
unsigned char Uart_ReadWrite(char* string);

//setup buffers to read UART
unsigned char Uart_Read(void);

//setup buffers to write to UART
unsigned char Uart_Write(char *string);

//initialise SPI
unsigned char Spi_Init(void);

//close SPI
unsigned char Spi_Close(void);

//Write and read using SPI
unsigned char Spi_ReadWrite(uint8_t const * _arrayW, uint16_t _lengthW, uint8_t* _arrayR, uint16_t _lengthR);

//write to SPI
unsigned char Spi_Write(uint8_t const * _array, uint8_t _length);

#endif /* _COMMUNICATION_H_ */
