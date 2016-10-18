/*!
 *****************************************************************************
 * @file:    SpiTest.c
 * @brief:   SPI Device Test for ADuCxxx
 * @version: $Revision: 23315 $
 * @date:    $Date: 2013-09-27 17:08:57 -0400 (Fri, 27 Sep 2013) $
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2011-2013 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*! \addtogroup SPI_Test SPI Test
 *  Example code demonstrating use of the SPI device driver.
 *  @{
 */


#include <stdio.h>
#include <stddef.h>		/* for 'NULL' */
#include <string.h>		/* for strlen */

#include "Communications.h"
#include "DIALOG_SPI_M3025.h"
#include <services/gpio/adi_gpio.h>


#define SPI_DEV_NUM                         0
#define SPI_CS_NUM                          ADI_SPI_CS0


   

/* testing is done using SPI device 1 (ADI_SPI_DEVID_1)        */
/* define the following macro for testing external (off-chip)  */
/* loopback which requires a hardware jumper between the       */
/* MISO & MOSI pins for SPI1, as follows:                      */
/* (across pins J14:2 & J14:4 on EVAL-ADuCRF101MKxZ board)     */
/* (across pins P0.0 & P0.2 on EVAL-ADuCRF101MKxZ/Rev A board) */
/* (across pins P0.0 & P0.2 on EVAL-ADuM360EBZ/Rev A board)    */
/* (across pins P3.5 & P3.6 on EVAL-ADuM350-MOCZ/Rev0 board)   */
/* (across pins P1.5 & P1.6 on EVAL-ADuM320EBZ /Rev0 A1)       */
/* otherwise undefine and internal loopback will be used       */


#if (ADI_CFG_ENABLE_DEPRECATED_DYNAMIC_PIN_MUX_SUPPORT == 0 )
extern int32_t adi_initpinmux(void);
#endif

uint8_t initDialogSPI()
{

  if(adi_spi_Open(SPI_DEV_NUM,SPIMem,ADI_SPI_MEMORY_SIZE,&hSPIDevice) != ADI_SPI_SUCCESS)
  return 1;

  /* Set the SPI clock rate */
  if(adi_spi_SetBitrate(hSPIDevice,300000) != ADI_SPI_SUCCESS)
  return 3;

  /* Set the chip select */
  if(adi_spi_SetChipSelect(hSPIDevice, SPI_CS_NUM) != ADI_SPI_SUCCESS)
  return 4;
  
  /* Enable continue mode (Chip Select remains low until the end of the transaction*/
  if(adi_spi_SetContinousMode(hSPIDevice, true) != ADI_SPI_SUCCESS)
  return 4;
  /* Disable DMA */
  if(adi_spi_EnableDmaMode(hSPIDevice, false) != ADI_SPI_SUCCESS)
  return 6;
        
        
    
    return 0;
}


uint8_t unInitDialogSPI()
{

  if(adi_spi_Close(hSPIDevice) != ADI_SPI_SUCCESS)
  return 1;
      

  return 0;
}


ADI_SPI_RESULT writeSPI(uint8_t const * _array, uint8_t _length)
{  

   ADI_SPI_RESULT err_code = ADI_SPI_SUCCESS;
   
   
   err_code = adi_spi_EnableDmaMode(hSPIDevice, true);
   if(err_code)
     return err_code;
   
   
   
   transceive.TransmitterBytes = _length;                                // Write here the number of bytes to send or receive.
   transceive.ReceiverBytes = 0;
   transceive.nTxIncrement = true;
   transceive.nRxIncrement = false;
   transceive.pReceiver = NULL;
   transceive.pTransmitter = (uint8_t *)_array;
   
   
   err_code = adi_spi_ReadWrite(hSPIDevice,&transceive);
   if(err_code)
     return err_code;
   
   
   err_code = adi_spi_EnableDmaMode(hSPIDevice, false);                       // Disable DMA mode
   if(err_code)
     return err_code;
   
   return ADI_SPI_SUCCESS;
}

ADI_SPI_RESULT writeReadSPI(uint8_t const * _arrayW, uint16_t _lengthW, uint8_t* _arrayR, uint16_t _lengthR)
{
 
  ADI_SPI_RESULT err_code = ADI_SPI_SUCCESS;
   
  
   err_code = adi_spi_EnableDmaMode(hSPIDevice, true);
   if(err_code)
     return err_code;

   transceive.TransmitterBytes = _lengthW;                                // Write here the number of bytes to send or receive.
   transceive.ReceiverBytes = _lengthR;
   transceive.nTxIncrement = true;
   transceive.nRxIncrement = true;
   transceive.pReceiver = _arrayR;
   transceive.pTransmitter = (uint8_t *)_arrayW;
   
   err_code = adi_spi_ReadWrite(hSPIDevice,&transceive);
   
   if(err_code)
     return err_code;
   
   
   err_code = adi_spi_EnableDmaMode(hSPIDevice, false);                       // Disable DMA mode
   if(err_code)
     return err_code;
   
   return ADI_SPI_SUCCESS;

}


/*
** EOF
*/

/*@}*/
