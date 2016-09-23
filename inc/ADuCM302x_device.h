/* ================================================================================

     Project      :   ADuCM302x
     File         :   ADuCM302x_device.h
     Description  :   C Register Definitions

     Date         :   Oct 21, 2014

     Copyright (c) Analog Devices, Inc.  All Rights Reserved.
     This software is proprietary and confidential to Analog Devices, Inc. and
     its licensors.

     This file was auto-generated. Do not make local changes to this file.

   ================================================================================ */

#ifndef _ADuCM302x_DEVICE_H
#define _ADuCM302x_DEVICE_H

/* pickup integer types */
#include <stdint.h>

/* pickup register bitfield and bit masks */
#include "ADuCM302x_typedefs.h"

#ifndef __IO
  #ifdef __cplusplus
    #define     __I     volatile      /* read-only */
    #define     __C
  #else
    #define     __I     volatile      /* read-only */
    #define     __C     const
  #endif
  #define       __O     volatile      /* write-only */
  #define       __IO    volatile      /* read-write */
#endif

#if defined (_MISRA_RULES)
/*
  anonymous unions violate ISO 9899:1990 and therefore MISRA Rule 1.1.
  Use of unions violates MISRA Rule 18.4.
  Anonymous unions are required for this implementation.
  Re-use of identifiers violates MISRA Rule 5.7.
  Field names are repeated for the ADuCM302x register map.
*/
#pragma diag(push)
#pragma diag(suppress:misra_rule_1_1:"Allow anonymous unions")
#pragma diag(suppress:misra_rule_5_7:"Header will re-use identifiers")
#pragma diag(suppress:misra_rule_18_4:"Allow the use of a union")
#endif /* _MISRA_RULES */

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/** @defgroup TMR General Purpose Timer (TMR) Module
 *  General Purpose Timer
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_TMR_TypeDef
 *  \brief  General Purpose Timer
 *  ========================================================================== */
typedef struct _ADI_TMR_TypeDef
{
    __IO     uint16_t LOAD;                          /*!< 16-bit load value */
    __I __C  uint8_t  RESERVED0[2];
    __I __C  uint16_t CURCNT;                        /*!< 16-bit timer value */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t CTL;                           /*!< Control */
    __I __C  uint8_t  RESERVED2[2];
    __O      uint16_t CLRINT;                        /*!< Clear Interrupt */
    __I __C  uint8_t  RESERVED3[2];
    __I __C  uint16_t CAPTURE;                       /*!< Capture */
    __I __C  uint8_t  RESERVED4[2];
    __IO     uint16_t ALOAD;                         /*!< 16-bit load value, asynchronous */
    __I __C  uint8_t  RESERVED5[2];
    __I __C  uint16_t ACURCNT;                       /*!< 16-bit timer value, asynchronous */
    __I __C  uint8_t  RESERVED6[2];
    __I __C  uint16_t STAT;                          /*!< Status */
    __I __C  uint8_t  RESERVED7[2];
    __IO     uint16_t PWMCTL;                        /*!< PWM Control Register */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t PWMMATCH;                      /*!< PWM Match Value */
} ADI_TMR_TypeDef;

/*!@}*/

/** @defgroup RTC Real-Time Clock (RTC) Module
 *  Real-Time Clock
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_RTC_TypeDef
 *  \brief  Real-Time Clock
 *  ========================================================================== */
typedef struct _ADI_RTC_TypeDef
{
    __IO     uint32_t CR0;                           /*!< RTC Control 0 */
    __IO     uint32_t SR0;                           /*!< RTC Status 0 */
    __I __C  uint32_t SR1;                           /*!< RTC Status 1 */
    __IO     uint32_t CNT0;                          /*!< RTC Count 0 */
    __IO     uint16_t CNT1;                          /*!< RTC Count 1 */
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint32_t ALM0;                          /*!< RTC Alarm 0 */
    __IO     uint16_t ALM1;                          /*!< RTC Alarm 1 */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t TRM;                           /*!< RTC Trim */
    __I __C  uint8_t  RESERVED2[2];
    __O      uint16_t GWY;                           /*!< RTC Gateway */
    __I __C  uint8_t  RESERVED3[6];
    __IO     uint16_t CR1;                           /*!< RTC Control 1 */
    __I __C  uint8_t  RESERVED4[2];
    __IO     uint16_t SR2;                           /*!< RTC Status 2 */
    __I __C  uint8_t  RESERVED5[2];
    __I __C  uint32_t SNAP0;                         /*!< RTC Snapshot 0 */
    __I __C  uint16_t SNAP1;                         /*!< RTC Snapshot 1 */
    __I __C  uint8_t  RESERVED6[2];
    __I __C  uint32_t SNAP2;                         /*!< RTC Snapshot 2 */
    __I __C  uint32_t MOD;                           /*!< RTC Modulo */
    __I __C  uint32_t CNT2;                          /*!< RTC Count 2 */
    __IO     uint32_t ALM2;                          /*!< RTC Alarm 2 */
} ADI_RTC_TypeDef;

/*!@}*/

/** @defgroup SYS System Identification and Debug Enable (SYS) Module
 *  System Identification and Debug Enable
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_SYS_TypeDef
 *  \brief  System Identification and Debug Enable
 *  ========================================================================== */
typedef struct _ADI_SYS_TypeDef
{
    __I __C  uint8_t  RESERVED0[32];
    __I __C  uint16_t ADIID;                         /*!< ADI Identification */
    __I __C  uint8_t  RESERVED1[2];
    __I __C  uint16_t CHIPID;                        /*!< Chip Identifier */
    __I __C  uint8_t  RESERVED2[26];
    __O      uint16_t SWDEN;                         /*!< Serial Wire Debug Enable */
} ADI_SYS_TypeDef;

/*!@}*/

/** @defgroup WDT Watchdog Timer (WDT) Module
 *  Watchdog Timer
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_WDT_TypeDef
 *  \brief  Watchdog Timer
 *  ========================================================================== */
typedef struct _ADI_WDT_TypeDef
{
    __IO     uint16_t LOAD;                          /*!< Load value */
    __I __C  uint8_t  RESERVED0[2];
    __I __C  uint16_t CCNT;                          /*!< Current count value */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t CTL;                           /*!< Control */
    __I __C  uint8_t  RESERVED2[2];
    __O      uint16_t RESTART;                       /*!< Clear interrupt */
    __I __C  uint8_t  RESERVED3[10];
    __I __C  uint16_t STAT;                          /*!< Status */
} ADI_WDT_TypeDef;

/*!@}*/

/** @defgroup I2C I2C Master/Slave (I2C) Module
 *  I2C Master/Slave
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_I2C_TypeDef
 *  \brief  I2C Master/Slave
 *  ========================================================================== */
typedef struct _ADI_I2C_TypeDef
{
    __IO     uint16_t MCTL;                          /*!< Master control */
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint16_t MSTAT;                         /*!< Master status */
    __I __C  uint8_t  RESERVED1[2];
    __I __C  uint16_t MRX;                           /*!< Master receive data */
    __I __C  uint8_t  RESERVED2[2];
    __IO     uint16_t MTX;                           /*!< Master transmit data */
    __I __C  uint8_t  RESERVED3[2];
    __IO     uint16_t MRXCNT;                        /*!< Master receive data count */
    __I __C  uint8_t  RESERVED4[2];
    __I __C  uint16_t MCRXCNT;                       /*!< Master current receive data count */
    __I __C  uint8_t  RESERVED5[2];
    __IO     uint16_t ADDR1;                         /*!< 1st master address byte */
    __I __C  uint8_t  RESERVED6[2];
    __IO     uint16_t ADDR2;                         /*!< 2nd master address byte */
    __I __C  uint8_t  RESERVED7[2];
    __IO     uint16_t BYT;                           /*!< Start byte */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t DIV;                           /*!< Serial clock period divisor */
    __I __C  uint8_t  RESERVED9[2];
    __IO     uint16_t SCTL;                          /*!< Slave control */
    __I __C  uint8_t  RESERVED10[2];
    __IO     uint16_t SSTAT;                         /*!< Slave I2C Status/Error/IRQ */
    __I __C  uint8_t  RESERVED11[2];
    __I __C  uint16_t SRX;                           /*!< Slave receive */
    __I __C  uint8_t  RESERVED12[2];
    __IO     uint16_t STX;                           /*!< Slave transmit */
    __I __C  uint8_t  RESERVED13[2];
    __IO     uint16_t ALT;                           /*!< Hardware general call ID */
    __I __C  uint8_t  RESERVED14[2];
    __IO     uint16_t ID0;                           /*!< 1st slave address device ID */
    __I __C  uint8_t  RESERVED15[2];
    __IO     uint16_t ID1;                           /*!< 2nd slave address device ID */
    __I __C  uint8_t  RESERVED16[2];
    __IO     uint16_t ID2;                           /*!< 3rd slave address device ID */
    __I __C  uint8_t  RESERVED17[2];
    __IO     uint16_t ID3;                           /*!< 4th slave address device ID */
    __I __C  uint8_t  RESERVED18[2];
    __IO     uint16_t STAT;                          /*!< Master and slave FIFO status */
    __I __C  uint8_t  RESERVED19[2];
    __O      uint16_t SHCTL;                         /*!< Shared control */
    __I __C  uint8_t  RESERVED20[2];
    __IO     uint16_t TCTL;                          /*!< Timing Control Register */
    __I __C  uint8_t  RESERVED21[2];
    __IO     uint16_t ASTRETCH_SCL;                  /*!< Automatic stretch SCL register */
} ADI_I2C_TypeDef;

/*!@}*/

/** @defgroup SPI Serial Peripheral Interface (SPI) Module
 *  Serial Peripheral Interface
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_SPI_TypeDef
 *  \brief  Serial Peripheral Interface
 *  ========================================================================== */
typedef struct _ADI_SPI_TypeDef
{
    __IO     uint16_t STAT;                          /*!< Status */
    __I __C  uint8_t  RESERVED0[2];
    __I __C  uint16_t RX;                            /*!< Receive */
    __I __C  uint8_t  RESERVED1[2];
    __O      uint16_t TX;                            /*!< Transmit */
    __I __C  uint8_t  RESERVED2[2];
    __IO     uint16_t DIV;                           /*!< SPI baud rate selection */
    __I __C  uint8_t  RESERVED3[2];
    __IO     uint16_t CTL;                           /*!< SPI configuration 1 */
    __I __C  uint8_t  RESERVED4[2];
    __IO     uint16_t IEN;                           /*!< SPI configuration 2 */
    __I __C  uint8_t  RESERVED5[2];
    __IO     uint16_t CNT;                           /*!< Transfer byte count */
    __I __C  uint8_t  RESERVED6[2];
    __IO     uint16_t DMA;                           /*!< SPI DMA enable */
    __I __C  uint8_t  RESERVED7[2];
    __I __C  uint16_t FIFO_STAT;                     /*!< FIFO Status */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t RD_CTL;                        /*!< Read Control */
    __I __C  uint8_t  RESERVED9[2];
    __IO     uint16_t FLOW_CTL;                      /*!< Flow Control */
    __I __C  uint8_t  RESERVED10[2];
    __IO     uint16_t WAIT_TMR;                      /*!< Wait timer for flow control */
    __I __C  uint8_t  RESERVED11[2];
    __IO     uint16_t CS_CTL;                        /*!< Chip-Select control for multi-slave connections */
    __I __C  uint8_t  RESERVED12[2];
    __IO     uint16_t CS_OVERRIDE;                   /*!< Chip-Select Override */
    __I __C  uint8_t  RESERVED13[2];
    __I __C  uint16_t XFR_CNT;                       /*!< Transferred Count */
} ADI_SPI_TypeDef;

/*!@}*/

/** @defgroup UART UART (UART) Module
 *  UART
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_UART_TypeDef
 *  \brief  UART
 *  ========================================================================== */
typedef struct _ADI_UART_TypeDef
{
    union {
    __O      uint16_t TX;                            /*!< Transmit Holding Register */
    __I __C  uint16_t RX;                            /*!< Receive Buffer Register */
    };
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint16_t IEN;                           /*!< Interrupt Enable */
    __I __C  uint8_t  RESERVED1[2];
    __I __C  uint16_t IIR;                           /*!< Interrupt ID */
    __I __C  uint8_t  RESERVED2[2];
    __IO     uint16_t LCR;                           /*!< Line Control */
    __I __C  uint8_t  RESERVED3[2];
    __IO     uint16_t MCR;                           /*!< Modem Control */
    __I __C  uint8_t  RESERVED4[2];
    __I __C  uint16_t LSR;                           /*!< Line Status */
    __I __C  uint8_t  RESERVED5[2];
    __I __C  uint16_t MSR;                           /*!< Modem Status */
    __I __C  uint8_t  RESERVED6[2];
    __IO     uint16_t SCR;                           /*!< Scratch buffer */
    __I __C  uint8_t  RESERVED7[6];
    __IO     uint16_t FBR;                           /*!< Fractional Baud Rate */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t DIV;                           /*!< Baudrate divider */
    __I __C  uint8_t  RESERVED9[6];
    __IO     uint16_t CTL;                           /*!< UART control register */
} ADI_UART_TypeDef;

/*!@}*/

/** @defgroup BEEP Beeper Driver (BEEP) Module
 *  Beeper Driver
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_BEEP_TypeDef
 *  \brief  Beeper Driver
 *  ========================================================================== */
typedef struct _ADI_BEEP_TypeDef
{
    __IO     uint16_t CFG;                           /*!< Beeper configuration */
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint16_t STAT;                          /*!< Beeper status */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t TONEA;                         /*!< Tone A Data */
    __I __C  uint8_t  RESERVED2[2];
    __IO     uint16_t TONEB;                         /*!< Tone B Data */
} ADI_BEEP_TypeDef;

/*!@}*/

/** @defgroup DMA DMA (DMA) Module
 *  DMA
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_DMA_TypeDef
 *  \brief  DMA
 *  ========================================================================== */
typedef struct _ADI_DMA_TypeDef
{
    __I __C  uint32_t STAT;                          /*!< DMA Status */
    __O      uint32_t CFG;                           /*!< DMA Configuration */
    __IO     uint32_t PDBPTR;                        /*!< DMA channel primary control data base pointer */
    __I __C  uint32_t ADBPTR;                        /*!< DMA channel alternate control data base pointer */
    __I __C  uint8_t  RESERVED0[4];
    __O      uint32_t SWREQ;                         /*!< DMA channel software request */
    __I __C  uint8_t  RESERVED1[8];
    __IO     uint32_t RMSK_SET;                      /*!< DMA channel request mask set */
    __O      uint32_t RMSK_CLR;                      /*!< DMA channel request mask clear */
    __IO     uint32_t EN_SET;                        /*!< DMA channel enable set */
    __O      uint32_t EN_CLR;                        /*!< DMA channel enable clear */
    __IO     uint32_t ALT_SET;                       /*!< DMA channel primary-alternate set */
    __O      uint32_t ALT_CLR;                       /*!< DMA channel primary-alternate clear */
    __O      uint32_t PRI_SET;                       /*!< DMA channel priority set */
    __O      uint32_t PRI_CLR;                       /*!< DMA channel priority clear */
    __I __C  uint8_t  RESERVED2[8];
    __IO     uint32_t ERRCHNL_CLR;                   /*!< DMA Per Channel Error Clear */
    __IO     uint32_t ERR_CLR;                       /*!< DMA bus error clear */
    __IO     uint32_t INVALIDDESC_CLR;               /*!< DMA Per Channel Invalid Descriptor Clear */
    __I __C  uint8_t  RESERVED3[1964];
    __IO     uint32_t BS_SET;                        /*!< DMA channel bytes swap enable set */
    __O      uint32_t BS_CLR;                        /*!< DMA channel bytes swap enable clear */
    __I __C  uint8_t  RESERVED4[8];
    __IO     uint32_t SRCADDR_SET;                   /*!< DMA channel source address decrement enable set */
    __O      uint32_t SRCADDR_CLR;                   /*!< DMA channel source address decrement enable clear */
    __IO     uint32_t DSTADDR_SET;                   /*!< DMA channel destination address decrement enable set */
    __O      uint32_t DSTADDR_CLR;                   /*!< DMA channel destination address decrement enable clear */
    __I __C  uint8_t  RESERVED5[1984];
    __I __C  uint32_t REVID;                         /*!< DMA Controller Revision ID */
} ADI_DMA_TypeDef;

/*!@}*/

/** @defgroup FLCC Flash Controller (FLCC) Module
 *  Flash Controller
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_FLCC_TypeDef
 *  \brief  Flash Controller
 *  ========================================================================== */
typedef struct _ADI_FLCC_TypeDef
{
    __IO     uint32_t STAT;                          /*!< Status */
    __IO     uint32_t IEN;                           /*!< Interrupt Enable */
    __IO     uint32_t CMD;                           /*!< Command */
    __IO     uint32_t KH_ADDR;                       /*!< WRITE address */
    __IO     uint32_t KH_DATA0;                      /*!< WRITE lower data */
    __IO     uint32_t KH_DATA1;                      /*!< WRITE upper data */
    __IO     uint32_t PAGE_ADDR0;                    /*!< Lower page address */
    __IO     uint32_t PAGE_ADDR1;                    /*!< Upper page address */
    __O      uint32_t KEY;                           /*!< Key */
    __I __C  uint32_t WR_ABORT_ADDR;                 /*!< Write abort address */
    __IO     uint32_t WRPROT;                        /*!< Write protection */
    __I __C  uint32_t SIGNATURE;                     /*!< Signature */
    __IO     uint32_t UCFG;                          /*!< User configuration */
    __IO     uint32_t TIME_PARAM0;                   /*!< Time parameter 0 */
    __IO     uint32_t TIME_PARAM1;                   /*!< Time parameter 1 */
    __IO     uint32_t ABORT_EN_LO;                   /*!< IRQ Abort Enable (lower bits) */
    __IO     uint32_t ABORT_EN_HI;                   /*!< IRQ Abort Enable (upper bits) */
    __IO     uint32_t ECC_CFG;                       /*!< ECC Config */
    __I __C  uint32_t ECC_ADDR;                      /*!< ECC Status (address) */
} ADI_FLCC_TypeDef;

/*!@}*/

/** @defgroup FLCC_CACHE Cache Controller (FLCC_CACHE) Module
 *  Cache Controller
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_FLCC_CACHE_TypeDef
 *  \brief  Cache Controller
 *  ========================================================================== */
typedef struct _ADI_FLCC_CACHE_TypeDef
{
    __I __C  uint32_t STAT;                          /*!< Cache Status register */
    __IO     uint32_t SETUP;                         /*!< Cache Setup register */
    __O      uint32_t KEY;                           /*!< Cache Key register */
} ADI_FLCC_CACHE_TypeDef;

/*!@}*/

/** @defgroup FLCC_DFT DFT Controller (FLCC_DFT) Module
 *  DFT Controller
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_FLCC_DFT_TypeDef
 *  \brief  DFT Controller
 *  ========================================================================== */
typedef struct _ADI_FLCC_DFT_TypeDef
{
    __I __C  uint8_t  RESERVED0[8];
    __IO     uint32_t CMD;                           /*!< Command register */
} ADI_FLCC_DFT_TypeDef;

/*!@}*/

/** @defgroup GPIO  (GPIO) Module
 *  
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_GPIO_TypeDef
 *  \brief  
 *  ========================================================================== */
typedef struct _ADI_GPIO_TypeDef
{
    __IO     uint32_t CFG;                           /*!< Port  Configuration */
    __IO     uint16_t OEN;                           /*!< Port output enable */
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint16_t PE;                            /*!< Port output pullup/pulldown enable */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t IEN;                           /*!< Port  Input Path Enable */
    __I __C  uint8_t  RESERVED2[2];
    __I __C  uint16_t IN;                            /*!< Port  registered data input */
    __I __C  uint8_t  RESERVED3[2];
    __IO     uint16_t OUT;                           /*!< Port data output */
    __I __C  uint8_t  RESERVED4[2];
    __O      uint16_t SET;                           /*!< Port data out set */
    __I __C  uint8_t  RESERVED5[2];
    __O      uint16_t CLR;                           /*!< Port data out clear */
    __I __C  uint8_t  RESERVED6[2];
    __O      uint16_t TGL;                           /*!< Port  pin toggle */
    __I __C  uint8_t  RESERVED7[2];
    __IO     uint16_t POL;                           /*!< Port interrupt polarity */
    __I __C  uint8_t  RESERVED8[2];
    __IO     uint16_t IENA;                          /*!< Port  interrupt A enable */
    __I __C  uint8_t  RESERVED9[2];
    __IO     uint16_t IENB;                          /*!< Port interrupt B enable */
    __I __C  uint8_t  RESERVED10[2];
    __IO     uint16_t INT;                           /*!< Port interrupt Status */
    __I __C  uint8_t  RESERVED11[2];
    __IO     uint16_t DS;                            /*!< Port  drive strength select */
} ADI_GPIO_TypeDef;

/*!@}*/

/** @defgroup SPORT Serial Port (SPORT) Module
 *  Serial Port
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_SPORT_TypeDef
 *  \brief  Serial Port
 *  ========================================================================== */
typedef struct _ADI_SPORT_TypeDef
{
    __IO     uint32_t CTL_A;                         /*!< Half SPORT 'A' Control Register */
    __IO     uint32_t DIV_A;                         /*!< Half SPORT 'A' Divisor Register */
    __IO     uint32_t IEN_A;                         /*!< Half SPORT A's Interrupt Enable register */
    __IO     uint32_t STAT_A;                        /*!< Half SPORT A's Status register */
    __IO     uint32_t NUMTRAN_A;                     /*!< Half SPORT A Number of transfers register */
    __IO     uint32_t CNVT_A;                        /*!< Half SPORT A's register to be programmed with the desired CONVT pulse width */
    __I __C  uint8_t  RESERVED0[8];
    __O      uint32_t TX_A;                          /*!< Half SPORT 'A' Tx Buffer Register */
    __I __C  uint8_t  RESERVED1[4];
    __I __C  uint32_t RX_A;                          /*!< Half SPORT 'A' Rx Buffer Register */
    __I __C  uint8_t  RESERVED2[20];
    __IO     uint32_t CTL_B;                         /*!< Half SPORT 'B' Control Register */
    __IO     uint32_t DIV_B;                         /*!< Half SPORT 'B' Divisor Register */
    __IO     uint32_t IEN_B;                         /*!< Half SPORT B's Interrupt Enable register */
    __IO     uint32_t STAT_B;                        /*!< Half SPORT B's Status register */
    __IO     uint32_t NUMTRAN_B;                     /*!< Half SPORT B Number of transfers register */
    __IO     uint32_t CNVT_B;                        /*!< Half SPORT B's register to be programmed with the desired CONVT pulse width */
    __I __C  uint8_t  RESERVED3[8];
    __O      uint32_t TX_B;                          /*!< Half SPORT 'B' Tx Buffer Register */
    __I __C  uint8_t  RESERVED4[4];
    __I __C  uint32_t RX_B;                          /*!< Half SPORT 'B' Rx Buffer Register */
} ADI_SPORT_TypeDef;

/*!@}*/

/** @defgroup CRC CRC (CRC) Module
 *  CRC
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_CRC_TypeDef
 *  \brief  CRC
 *  ========================================================================== */
typedef struct _ADI_CRC_TypeDef
{
    __IO     uint32_t CTL;                           /*!< CRC Control Register */
    __IO     uint32_t IPDATA;                        /*!< Input Data Register */
    __IO     uint32_t RESULT;                        /*!< CRC Result Register */
    __IO     uint32_t POLY;                          /*!< Programmable CRC Polynomial */
} ADI_CRC_TypeDef;

/*!@}*/

/** @defgroup RNG Random Number Generator (RNG) Module
 *  Random Number Generator
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_RNG_TypeDef
 *  \brief  Random Number Generator
 *  ========================================================================== */
typedef struct _ADI_RNG_TypeDef
{
    __IO     uint16_t CTL;                           /*!< RNG Control Register */
    __I __C  uint8_t  RESERVED0[2];
    __IO     uint16_t LEN;                           /*!< RNG Sample Length Register */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint16_t STAT;                          /*!< RNG Status Register */
    __I __C  uint8_t  RESERVED2[2];
    __I __C  uint32_t DATA;                          /*!< RNG Data Register */
    __I __C  uint32_t OSCCNT;                        /*!< Oscillator Count */
    __I __C   uint8_t OSCDIFF[4];                    /*!< Oscillator Difference */
} ADI_RNG_TypeDef;

/*!@}*/

/** @defgroup CRYPT crypto (CRYPT) Module
 *  crypto
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_CRYPT_TypeDef
 *  \brief  crypto
 *  ========================================================================== */
typedef struct _ADI_CRYPT_TypeDef
{
    __IO     uint32_t CFG;                           /*!< Configuration Register */
    __IO     uint32_t DATALEN;                       /*!< Payload Data Length */
    __IO     uint32_t PREFIXLEN;                     /*!< Authentication Data Length */
    __IO     uint32_t INTEN;                         /*!< Interrupt Enable Register */
    __IO     uint32_t STAT;                          /*!< Status Register */
    __O      uint32_t INBUF;                         /*!< Input Buffer */
    __I __C  uint32_t OUTBUF;                        /*!< Output Buffer */
    __IO     uint32_t NONCE0;                        /*!< Nonce Bits [31:0] */
    __IO     uint32_t NONCE1;                        /*!< Nonce Bits [63:32] */
    __IO     uint32_t NONCE2;                        /*!< Nonce Bits [95:64] */
    __IO     uint32_t NONCE3;                        /*!< Nonce Bits [127:96] */
    __IO     uint32_t AESKEY0;                       /*!< Key Bits[ 31:0 ] */
    __IO     uint32_t AESKEY1;                       /*!< Key Bits [ 63:32 ] */
    __IO     uint32_t AESKEY2;                       /*!< Key Bits [ 95:64 ] */
    __IO     uint32_t AESKEY3;                       /*!< Key Bits [ 127:96 ] */
    __IO     uint32_t AESKEY4;                       /*!< Key Bits [ 159:128 ] */
    __IO     uint32_t AESKEY5;                       /*!< Key Bits [ 191:160 ] */
    __IO     uint32_t AESKEY6;                       /*!< Key Bits [ 223:192 ] */
    __IO     uint32_t AESKEY7;                       /*!< Key Bits [ 255:224 ] */
    __IO     uint32_t CNTRINIT;                      /*!< Counter Initialization Vector */
    __IO     uint32_t SHAH0;                         /*!< SHA Bits [ 31:0 ] */
    __IO     uint32_t SHAH1;                         /*!< SHA Bits [ 63:32 ] */
    __IO     uint32_t SHAH2;                         /*!< SHA Bits [ 95:64 ] */
    __IO     uint32_t SHAH3;                         /*!< SHA Bits [ 127:96 ] */
    __IO     uint32_t SHAH4;                         /*!< SHA Bits [ 159:128 ] */
    __IO     uint32_t SHAH5;                         /*!< SHA Bits [ 191:160 ] */
    __IO     uint32_t SHAH6;                         /*!< SHA Bits [ 223:192] */
    __IO     uint32_t SHAH7;                         /*!< SHA Bits [ 255:224 ] */
} ADI_CRYPT_TypeDef;

/*!@}*/

/** @defgroup PMG Power Management  (PMG) Module
 *  Power Management 
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_PMG_TypeDef
 *  \brief  Power Management 
 *  ========================================================================== */
typedef struct _ADI_PMG_TypeDef
{
    __IO     uint32_t IEN;                           /*!< Power Supply Monitor Interrupt Enable */
    __IO     uint32_t PSM_STAT;                      /*!< Power supply monitor status */
    __IO     uint32_t PWRMOD;                        /*!< Power Mode Register */
    __O      uint32_t PWRKEY;                        /*!< Key protection for PWRMOD */
    __I __C  uint32_t SHDN_STAT;                     /*!< SHUTDOWN Status Register */
    __IO     uint32_t SRAMRET;                       /*!< Control for Retention SRAM during HIBERNATE Mode */
    __I __C  uint8_t  RESERVED0[40];
    __IO     uint32_t RST_STAT;                      /*!< Reset status */
    __IO     uint32_t CTL1;                          /*!< HPBUCK control */
} ADI_PMG_TypeDef;

/*!@}*/

/** @defgroup XINT External interrupt configuration (XINT) Module
 *  External interrupt configuration
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_XINT_TypeDef
 *  \brief  External interrupt configuration
 *  ========================================================================== */
typedef struct _ADI_XINT_TypeDef
{
    __IO     uint32_t CFG0;                          /*!< External Interrupt configuration 0 */
    __I __C  uint8_t  RESERVED0[12];
    __IO     uint32_t CLR;                           /*!< External Interrupt clear */
    __IO     uint32_t NMICLR;                        /*!< Non-maskable interrupt clear */
} ADI_XINT_TypeDef;

/*!@}*/

/** @defgroup CLKG_OSC Clocking  (CLKG_OSC) Module
 *  Clocking 
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_CLKG_OSC_TypeDef
 *  \brief  Clocking 
 *  ========================================================================== */
typedef struct _ADI_CLKG_OSC_TypeDef
{
    __I __C  uint8_t  RESERVED0[12];
    __O      uint32_t KEY;                           /*!< Key Protection for OSCCTRL */
    __IO     uint32_t CTL;                           /*!< Oscillator Control */
} ADI_CLKG_OSC_TypeDef;

/*!@}*/

/** @defgroup PMG_TST Power Management  (PMG_TST) Module
 *  Power Management 
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_PMG_TST_TypeDef
 *  \brief  Power Management 
 *  ========================================================================== */
typedef struct _ADI_PMG_TST_TypeDef
{
    __I __C  uint8_t  RESERVED0[96];
    __IO     uint32_t SRAM_CTL;                      /*!< Control for SRAM parity and instruction SRAM */
    __IO     uint32_t SRAM_INITSTAT;                 /*!< Initialization Status Register */
    __O      uint16_t CLR_LATCH_GPIOS;               /*!< CLEAR GPIO AFTER SHUTDOWN MODE */
    __I __C  uint8_t  RESERVED1[2];
    __IO     uint32_t SCRPAD_IMG;                    /*!< SCRATCH PAD IMAGE */
    __I __C  uint32_t SCRPAD_3V_RD;                  /*!< SCRATCH PAD SAVED IN BATTERY DOMAIN */
} ADI_PMG_TST_TypeDef;

/*!@}*/

/** @defgroup CLKG_CLK Clocking  (CLKG_CLK) Module
 *  Clocking 
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_CLKG_CLK_TypeDef
 *  \brief  Clocking 
 *  ========================================================================== */
typedef struct _ADI_CLKG_CLK_TypeDef
{
    __IO     uint32_t CTL0;                          /*!< Misc clock settings */
    __IO     uint32_t CTL1;                          /*!< Clock dividers */
    __I __C  uint8_t  RESERVED0[4];
    __IO     uint32_t CTL3;                          /*!< System PLL */
    __I __C  uint8_t  RESERVED1[4];
    __IO     uint32_t CTL5;                          /*!< User clock gating control */
    __IO     uint32_t STAT0;                         /*!< Clocking status */
} ADI_CLKG_CLK_TypeDef;

/*!@}*/

/** @defgroup BUSM Bus matrix (BUSM) Module
 *  Bus matrix
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_BUSM_TypeDef
 *  \brief  Bus matrix
 *  ========================================================================== */
typedef struct _ADI_BUSM_TypeDef
{
    __IO     uint32_t ARBIT0;                        /*!< Arbitration Priority Configuration for FLASH and SRAM0 */
    __IO     uint32_t ARBIT1;                        /*!< Arbitration Priority Configuration for SRAM1 and SIP */
    __IO     uint32_t ARBIT2;                        /*!< Arbitration Priority Configuration for APB32 and APB16 */
    __IO     uint32_t ARBIT3;                        /*!< Arbitration Priority Configuration for APB16 priority for core and for DMA1 */
} ADI_BUSM_TypeDef;

/*!@}*/

/** @defgroup PTI Parallel Test Interface (PTI) Module
 *  Parallel Test Interface
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_PTI_TypeDef
 *  \brief  Parallel Test Interface
 *  ========================================================================== */
typedef struct _ADI_PTI_TypeDef
{
    __IO     uint32_t RST_ISR_STARTADDR;             /*!< Reset ISR Start Address */
    __IO     uint32_t RST_STACK_PTR;                 /*!< Reset Stack Pointer */
    __IO     uint32_t CTL;                           /*!< Parallel Test Interface Control Register */
} ADI_PTI_TypeDef;

/*!@}*/

/** @defgroup NVIC Cortex-M3 Interrupt Controller (NVIC) Module
 *  Cortex-M3 Interrupt Controller
 *  @{
 */

/*! ==========================================================================
 *  \struct ADI_NVIC_TypeDef
 *  \brief  Cortex-M3 Interrupt Controller
 *  ========================================================================== */
typedef struct _ADI_NVIC_TypeDef
{
    __I __C  uint8_t  RESERVED0[4];
    __IO     uint32_t INTNUM;                        /*!< Interrupt Control Type */
    __I __C  uint8_t  RESERVED1[8];
    __IO     uint32_t STKSTA;                        /*!< Systick Control and Status */
    __IO     uint32_t STKLD;                         /*!< Systick Reload Value */
    __IO     uint32_t STKVAL;                        /*!< Systick Current Value */
    __IO     uint32_t STKCAL;                        /*!< Systick Calibration Value */
    __I __C  uint8_t  RESERVED2[224];
    __IO     uint32_t INTSETE0;                      /*!< IRQ0..31 Set_Enable */
    __IO     uint32_t INTSETE1;                      /*!< IRQ32..63 Set_Enable */
    __I __C  uint8_t  RESERVED3[120];
    __IO     uint32_t INTCLRE0;                      /*!< IRQ0..31 Clear_Enable */
    __IO     uint32_t INTCLRE1;                      /*!< IRQ32..63 Clear_Enable */
    __I __C  uint8_t  RESERVED4[120];
    __IO     uint32_t INTSETP0;                      /*!< IRQ0..31 Set_Pending */
    __IO     uint32_t INTSETP1;                      /*!< IRQ32..63 Set_Pending */
    __I __C  uint8_t  RESERVED5[120];
    __IO     uint32_t INTCLRP0;                      /*!< IRQ0..31 Clear_Pending */
    __IO     uint32_t INTCLRP1;                      /*!< IRQ32..63 Clear_Pending */
    __I __C  uint8_t  RESERVED6[120];
    __IO     uint32_t INTACT0;                       /*!< IRQ0..31 Active Bit */
    __IO     uint32_t INTACT1;                       /*!< IRQ32..63 Active Bit */
    __I __C  uint8_t  RESERVED7[248];
    __IO     uint32_t INTPRI0;                       /*!< IRQ0..3 Priority */
    __IO     uint32_t INTPRI1;                       /*!< IRQ4..7 Priority */
    __IO     uint32_t INTPRI2;                       /*!< IRQ8..11 Priority */
    __IO     uint32_t INTPRI3;                       /*!< IRQ12..15 Priority */
    __IO     uint32_t INTPRI4;                       /*!< IRQ16..19 Priority */
    __IO     uint32_t INTPRI5;                       /*!< IRQ20..23 Priority */
    __IO     uint32_t INTPRI6;                       /*!< IRQ24..27 Priority */
    __IO     uint32_t INTPRI7;                       /*!< IRQ28..31 Priority */
    __IO     uint32_t INTPRI8;                       /*!< IRQ32..35 Priority */
    __IO     uint32_t INTPRI9;                       /*!< IRQ36..39 Priority */
    __IO     uint32_t INTPRI10;                      /*!< IRQ40..43 Priority */
    __I __C  uint8_t  RESERVED8[2260];
    __IO     uint32_t INTCPID;                       /*!< CPUID Base */
    __IO     uint32_t INTSTA;                        /*!< Interrupt Control State */
    __IO     uint32_t INTVEC;                        /*!< Vector Table Offset */
    __IO     uint32_t INTAIRC;                       /*!< Application Interrupt/Reset Control */
    __IO     uint16_t INTCON0;                       /*!< System Control */
    __I __C  uint8_t  RESERVED9[2];
    __IO     uint32_t INTCON1;                       /*!< Configuration Control */
    __IO     uint32_t INTSHPRIO0;                    /*!< System Handlers 4-7 Priority */
    __IO     uint32_t INTSHPRIO1;                    /*!< System Handlers 8-11 Priority */
    __IO     uint32_t INTSHPRIO3;                    /*!< System Handlers 12-15 Priority */
    __IO     uint32_t INTSHCSR;                      /*!< System Handler Control and State */
    __IO     uint32_t INTCFSR;                       /*!< Configurable Fault Status */
    __IO     uint32_t INTHFSR;                       /*!< Hard Fault Status */
    __IO     uint32_t INTDFSR;                       /*!< Debug Fault Status */
    __IO     uint32_t INTMMAR;                       /*!< Mem Manage Address */
    __IO     uint32_t INTBFAR;                       /*!< Bus Fault Address */
    __IO     uint32_t INTAFSR;                       /*!< Auxiliary Fault Status */
    __IO     uint32_t INTPFR0;                       /*!< Processor Feature Register 0 */
    __IO     uint32_t INTPFR1;                       /*!< Processor Feature Register 1 */
    __IO     uint32_t INTDFR0;                       /*!< Debug Feature Register 0 */
    __IO     uint32_t INTAFR0;                       /*!< Auxiliary Feature Register 0 */
    __IO     uint32_t INTMMFR0;                      /*!< Memory Model Feature Register 0 */
    __IO     uint32_t INTMMFR1;                      /*!< Memory Model Feature Register 1 */
    __IO     uint32_t INTMMFR2;                      /*!< Memory Model Feature Register 2 */
    __IO     uint32_t INTMMFR3;                      /*!< Memory Model Feature Register 3 */
    __IO     uint32_t INTISAR0;                      /*!< ISA Feature Register 0 */
    __IO     uint32_t INTISAR1;                      /*!< ISA Feature Register 1 */
    __IO     uint32_t INTISAR2;                      /*!< ISA Feature Register 2 */
    __IO     uint32_t INTISAR3;                      /*!< ISA Feature Register 3 */
    __IO     uint32_t INTISAR4;                      /*!< ISA Feature Register 4 */
    __I __C  uint8_t  RESERVED10[396];
    __IO     uint32_t INTTRGI;                       /*!< Software Trigger Interrupt Register */
    __I __C  uint8_t  RESERVED11[204];
    __IO     uint32_t INTPID4;                       /*!< Peripheral Identification Register 4 */
    __IO     uint32_t INTPID5;                       /*!< Peripheral Identification Register 5 */
    __IO     uint32_t INTPID6;                       /*!< Peripheral Identification Register 6 */
    __IO     uint32_t INTPID7;                       /*!< Peripheral Identification Register 7 */
    __IO     uint32_t INTPID0;                       /*!< Peripheral Identification Bits7:0 */
    __IO     uint32_t INTPID1;                       /*!< Peripheral Identification Bits15:8 */
    __IO     uint32_t INTPID2;                       /*!< Peripheral Identification Bits16:23 */
    __IO     uint32_t INTPID3;                       /*!< Peripheral Identification Bits24:31 */
    __IO     uint32_t INTCID0;                       /*!< Component Identification Bits7:0 */
    __IO     uint32_t INTCID1;                       /*!< Component Identification Bits15:8 */
    __IO     uint32_t INTCID2;                       /*!< Component Identification Bits16:23 */
    __IO     uint32_t INTCID3;                       /*!< Component Identification Bits24:31 */
} ADI_NVIC_TypeDef;

/*!@}*/

/* ******************************************************************************
 *    Peripheral Memory Map Declarations
 * *****************************************************************************/
/*!    @defgroup PMEMMAPDEC Peripheral Memory Map Declarations
 *     \addtogroup PMEMMAPDEC
 *     @{ */
#define ADI_TMR0_BASE                  0x40000000    /*!<  Base address of TMR0 */
#define ADI_TMR1_BASE                  0x40000400    /*!<  Base address of TMR1 */
#define ADI_TMR2_BASE                  0x40000800    /*!<  Base address of TMR2 */
#define ADI_RTC0_BASE                  0x40001000    /*!<  Base address of RTC0 */
#define ADI_RTC1_BASE                  0x40001400    /*!<  Base address of RTC1 */
#define ADI_SYS_BASE                   0x40002000    /*!<  Base address of SYS */
#define ADI_WDT0_BASE                  0x40002c00    /*!<  Base address of WDT0 */
#define ADI_I2C0_BASE                  0x40003000    /*!<  Base address of I2C0 */
#define ADI_SPI0_BASE                  0x40004000    /*!<  Base address of SPI0 */
#define ADI_SPI1_BASE                  0x40004400    /*!<  Base address of SPI1 */
#define ADI_SPI2_BASE                  0x40024000    /*!<  Base address of SPI2 */
#define ADI_UART0_BASE                 0x40005000    /*!<  Base address of UART0 */
#define ADI_BEEP0_BASE                 0x40005c00    /*!<  Base address of BEEP0 */
#define ADI_DMA0_BASE                  0x40010000    /*!<  Base address of DMA0 */
#define ADI_FLCC0_BASE                 0x40018000    /*!<  Base address of FLCC0 */
#define ADI_FLCC0_CACHE_BASE           0x40018058    /*!<  Base address of FLCC0_CACHE */
#define ADI_FLCC0_DFT_BASE             0x4001808c    /*!<  Base address of FLCC0_DFT */
#define ADI_FLCC0_TEST_BASE            0x400180c4    /*!<  Base address of FLCC0_TEST */
#define ADI_GPIO0_BASE                 0x40020000    /*!<  Base address of GPIO0 */
#define ADI_GPIO1_BASE                 0x40020040    /*!<  Base address of GPIO1 */
#define ADI_GPIO2_BASE                 0x40020080    /*!<  Base address of GPIO2 */
#define ADI_SPORT0_BASE                0x40038000    /*!<  Base address of SPORT0 */
#define ADI_CRC0_BASE                  0x40040000    /*!<  Base address of CRC0 */
#define ADI_RNG0_BASE                  0x40040400    /*!<  Base address of RNG0 */
#define ADI_CRYPT0_BASE                0x40044000    /*!<  Base address of CRYPT0 */
#define ADI_PMG0_BASE                  0x4004c000    /*!<  Base address of PMG0 */
#define ADI_XINT0_BASE                 0x4004c080    /*!<  Base address of XINT0 */
#define ADI_CLKG0_OSC_BASE             0x4004c100    /*!<  Base address of CLKG0_OSC */
#define ADI_PMG0_TST_BASE              0x4004c200    /*!<  Base address of PMG0_TST */
#define ADI_CLKG0_CLK_BASE             0x4004c300    /*!<  Base address of CLKG0_CLK */
#define ADI_BUSM0_BASE                 0x4004c800    /*!<  Base address of BUSM0 */
#define ADI_PTI0_BASE                  0x4004cd00    /*!<  Base address of PTI0 */
#define ADI_NVIC0_BASE                 0xe000e000    /*!<  Base address of NVIC0 */

/*!    @} */

/* ******************************************************************************
 *    Peripheral Pointer Declarations
 * *****************************************************************************/
/*!    @Defgroup Pptrdec Peripheral Pointer Declarations
 *     \Addtogroup Pptrdec
 *     @{ */
#define pADI_TMR0                      ((ADI_TMR_TypeDef      *) ADI_TMR0_BASE       ) /*!<  Pointer to General Purpose Timer (TMR0) */
#define pADI_TMR1                      ((ADI_TMR_TypeDef      *) ADI_TMR1_BASE       ) /*!<  Pointer to General Purpose Timer (TMR1) */
#define pADI_TMR2                      ((ADI_TMR_TypeDef      *) ADI_TMR2_BASE       ) /*!<  Pointer to General Purpose Timer (TMR2) */
#define pADI_RTC0                      ((ADI_RTC_TypeDef      *) ADI_RTC0_BASE       ) /*!<  Pointer to Real-Time Clock (RTC0) */
#define pADI_RTC1                      ((ADI_RTC_TypeDef      *) ADI_RTC1_BASE       ) /*!<  Pointer to Real-Time Clock (RTC1) */
#define pADI_SYS                       ((ADI_SYS_TypeDef      *) ADI_SYS_BASE        ) /*!<  Pointer to System Identification and Debug Enable (SYS) */
#define pADI_WDT0                      ((ADI_WDT_TypeDef      *) ADI_WDT0_BASE       ) /*!<  Pointer to Watchdog Timer (WDT0) */
#define pADI_I2C0                      ((ADI_I2C_TypeDef      *) ADI_I2C0_BASE       ) /*!<  Pointer to I2C Master/Slave (I2C0) */
#define pADI_SPI0                      ((ADI_SPI_TypeDef      *) ADI_SPI0_BASE       ) /*!<  Pointer to Serial Peripheral Interface (SPI0) */
#define pADI_SPI1                      ((ADI_SPI_TypeDef      *) ADI_SPI1_BASE       ) /*!<  Pointer to Serial Peripheral Interface (SPI1) */
#define pADI_SPI2                      ((ADI_SPI_TypeDef      *) ADI_SPI2_BASE       ) /*!<  Pointer to Serial Peripheral Interface (SPI2) */
#define pADI_UART0                     ((ADI_UART_TypeDef     *) ADI_UART0_BASE      ) /*!<  Pointer to UART (UART0) */
#define pADI_BEEP0                     ((ADI_BEEP_TypeDef     *) ADI_BEEP0_BASE      ) /*!<  Pointer to Beeper Driver (BEEP0) */
#define pADI_DMA0                      ((ADI_DMA_TypeDef      *) ADI_DMA0_BASE       ) /*!<  Pointer to DMA (DMA0) */
#define pADI_FLCC0                     ((ADI_FLCC_TypeDef     *) ADI_FLCC0_BASE      ) /*!<  Pointer to Flash Controller (FLCC0) */
#define pADI_FLCC0_CACHE               ((ADI_FLCC_CACHE_TypeDef *) ADI_FLCC0_CACHE_BASE) /*!<  Pointer to Cache Controller (FLCC0_CACHE) */
#define pADI_FLCC0_DFT                 ((ADI_FLCC_DFT_TypeDef *) ADI_FLCC0_DFT_BASE  ) /*!<  Pointer to DFT Controller (FLCC0_DFT) */
#define pADI_FLCC0_TEST                ((ADI_FLCC_TEST_TypeDef *) ADI_FLCC0_TEST_BASE ) /*!<  Pointer to Flash Controller (FLCC0_TEST) */
#define pADI_GPIO0                     ((ADI_GPIO_TypeDef     *) ADI_GPIO0_BASE      ) /*!<  Pointer to  (GPIO0) */
#define pADI_GPIO1                     ((ADI_GPIO_TypeDef     *) ADI_GPIO1_BASE      ) /*!<  Pointer to  (GPIO1) */
#define pADI_GPIO2                     ((ADI_GPIO_TypeDef     *) ADI_GPIO2_BASE      ) /*!<  Pointer to  (GPIO2) */
#define pADI_SPORT0                    ((ADI_SPORT_TypeDef    *) ADI_SPORT0_BASE     ) /*!<  Pointer to Serial Port (SPORT0) */
#define pADI_CRC0                      ((ADI_CRC_TypeDef      *) ADI_CRC0_BASE       ) /*!<  Pointer to CRC (CRC0) */
#define pADI_RNG0                      ((ADI_RNG_TypeDef      *) ADI_RNG0_BASE       ) /*!<  Pointer to Random Number Generator (RNG0) */
#define pADI_CRYPT0                    ((ADI_CRYPT_TypeDef    *) ADI_CRYPT0_BASE     ) /*!<  Pointer to crypto (CRYPT0) */
#define pADI_PMG0                      ((ADI_PMG_TypeDef      *) ADI_PMG0_BASE       ) /*!<  Pointer to Power Management  (PMG0) */
#define pADI_XINT0                     ((ADI_XINT_TypeDef     *) ADI_XINT0_BASE      ) /*!<  Pointer to External interrupt configuration (XINT0) */
#define pADI_CLKG0_OSC                 ((ADI_CLKG_OSC_TypeDef *) ADI_CLKG0_OSC_BASE  ) /*!<  Pointer to Clocking  (CLKG0_OSC) */
#define pADI_PMG0_TST                  ((ADI_PMG_TST_TypeDef  *) ADI_PMG0_TST_BASE   ) /*!<  Pointer to Power Management  (PMG0_TST) */
#define pADI_CLKG0_CLK                 ((ADI_CLKG_CLK_TypeDef *) ADI_CLKG0_CLK_BASE  ) /*!<  Pointer to Clocking  (CLKG0_CLK) */
#define pADI_BUSM0                     ((ADI_BUSM_TypeDef     *) ADI_BUSM0_BASE      ) /*!<  Pointer to Bus matrix (BUSM0) */
#define pADI_PTI0                      ((ADI_PTI_TypeDef      *) ADI_PTI0_BASE       ) /*!<  Pointer to Parallel Test Interface (PTI0) */
#define pADI_NVIC0                     ((ADI_NVIC_TypeDef     *) ADI_NVIC0_BASE      ) /*!<  Pointer to Cortex-M3 Interrupt Controller (NVIC0) */

/*!    @} */


#define ADI_INT_NUM_INT  64u

/*  =========================================================================
 *! \enum   IRQn_Type
 *! \brief  Interrupt Number Assignments
 *  ========================================================================= */
typedef enum
{
  SYS_RESET_IRQn                 =   -15,    /*!< Cortex-M3 Reset */
  SYS_NonMaskableint_IRQn        =   -14,    /*!< Cortex-M3 Non-maskable Interrupt */
  SYS_HardFault_IRQn             =   -13,    /*!< Cortex-M3 Hardware Fault */
  SYS_MemoryManagement_IRQn      =   -12,    /*!< Cortex-M3 Memory Management Interrupt */
  SYS_BusFault_IRQn              =   -11,    /*!< Cortex-M3 Bus Fault */
  SYS_UsageFault_IRQn            =   -10,    /*!< Cortex-M3 Usage Fault */
  SYS_SVCall_IRQn                =    -5,    /*!< Cortex-M3 SVCall Interrupt */
  SYS_DebugMonitor_IRQn          =    -4,    /*!< Cortex-M3 Debug Monitor */
  SYS_PendSV_IRQn                =    -2,    /*!< Cortex-M3 PendSV Interrupt */
  SysTick_IRQn                   =    -1,    /*!< Cortex-M3 SysTick Interrupt */
  RTC1_EVT_IRQn                  =     0,    /*!< Event */
  XINT0_EVT0_IRQn                =     1,    /*!< External Wakeup Interrupt n */
  XINT0_EVT1_IRQn                =     2,    /*!< External Wakeup Interrupt n */
  XINT0_EVT2_IRQn                =     3,    /*!< External Wakeup Interrupt n */
  XINT0_EVT3_IRQn                =     4,    /*!< External Wakeup Interrupt n */
  WDT0_EXP_IRQn                  =     5,    /*!< Expiration */
  PMG0_VREG_OVR_IRQn             =     6,    /*!< Voltage Regulator (VREG) Overvoltage */
  PMG0_BATT_RANGE_IRQn           =     7,    /*!< Battery Voltage (VBAT) Out of Range */
  RTC0_EVT_IRQn                  =     8,    /*!< Event */
  SYS_GPIO_INTA_IRQn             =     9,    /*!< GPIO Interrupt A */
  SYS_GPIO_INTB_IRQn             =    10,    /*!< GPIO Interrupt B */
  TMR0_EVT_IRQn                  =    11,    /*!< Event */
  TMR1_EVT_IRQn                  =    12,    /*!< Event */
  FLCC0_EVT_IRQn                 =    13,    /*!< Event */
  UART0_EVT_IRQn                 =    14,    /*!< Event */
  SPI0_EVT_IRQn                  =    15,    /*!< Event */
  SPI2_EVT_IRQn                  =    16,    /*!< Event */
  I2C0_SLV_EVT_IRQn              =    17,    /*!< Slave Event */
  I2C0_MST_EVT_IRQn              =    18,    /*!< Master Event */
  DMA0_CHAN_ERR_IRQn             =    19,    /*!< Channel Error */
  DMA0_CH0_DONE_IRQn             =    20,    /*!< Channel 0 Done */
  DMA0_CH1_DONE_IRQn             =    21,    /*!< Channel 1 Done */
  DMA0_CH2_DONE_IRQn             =    22,    /*!< Channel 2 Done */
  DMA0_CH3_DONE_IRQn             =    23,    /*!< Channel 3 Done */
  DMA0_CH4_DONE_IRQn             =    24,    /*!< Channel 4 Done */
  DMA0_CH5_DONE_IRQn             =    25,    /*!< Channel 5 Done */
  DMA0_CH6_DONE_IRQn             =    26,    /*!< Channel 6 Done */
  DMA0_CH7_DONE_IRQn             =    27,    /*!< Channel 7 Done */
  DMA0_CH8_DONE_IRQn             =    28,    /*!< Channel 8 Done */
  DMA0_CH9_DONE_IRQn             =    29,    /*!< Channel 9 Done */
  DMA0_CH10_DONE_IRQn            =    30,    /*!< Channel 10 Done */
  DMA0_CH11_DONE_IRQn            =    31,    /*!< Channel 11 Done */
  DMA0_CH12_DONE_IRQn            =    32,    /*!< Channel 12 Done */
  DMA0_CH13_DONE_IRQn            =    33,    /*!< Channel 13 Done */
  DMA0_CH14_DONE_IRQn            =    34,    /*!< Channel 14 Done */
  DMA0_CH15_DONE_IRQn            =    35,    /*!< Channel 15 Done */
  SPORT0_A_EVT_IRQn              =    36,    /*!< Channel A Event */
  SPORT0_B_EVT_IRQn              =    37,    /*!< Channel B Event */
  CRYPT0_EVT_IRQn                =    38,    /*!< Event */
  TMR2_EVT_IRQn                  =    40,    /*!< Event */
  CLKG0_XTAL_OSC_EVT_IRQn        =    41,    /*!< Crystal Oscillator Event */
  SPI1_EVT_IRQn                  =    42,    /*!< Event */
  CLKG0_PLL_EVT_IRQn             =    43,    /*!< PLL Event */
  RNG0_EVT_IRQn                  =    44,    /*!<  */
  BEEP0_EVT_IRQn                 =    45,    /*!< Event */
  DMA0_CH16_DONE_IRQn            =    56,    /*!< Channel 16 Done */
  DMA0_CH17_DONE_IRQn            =    57,    /*!< Channel 17 Done */
  DMA0_CH18_DONE_IRQn            =    58,    /*!< Channel 18 Done */
  DMA0_CH19_DONE_IRQn            =    59,    /*!< Channel 19 Done */
  DMA0_CH20_DONE_IRQn            =    60,    /*!< Channel 20 Done */
  DMA0_CH21_DONE_IRQn            =    61,    /*!< Channel 21 Done */
  DMA0_CH22_DONE_IRQn            =    62,    /*!< Channel 22 Done */
  DMA0_CH23_DONE_IRQn            =    63,    /*!< Channel 23 Done */
} IRQn_Type;  /* typedef name for fixed interrupt numbers */

#endif
