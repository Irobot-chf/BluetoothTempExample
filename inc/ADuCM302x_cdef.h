/* ================================================================================

     Project      :   ADuCM302x
     File         :   ADuCM302x_cdef.h
     Description  :   C MMR Pointer Definitions

     Date         :   Oct 21, 2014

     Copyright (c) Analog Devices, Inc.  All Rights Reserved.
     This software is proprietary and confidential to Analog Devices, Inc. and
     its licensors.

     This file was auto-generated. Do not make local changes to this file.

   ================================================================================ */

#ifndef _ADuCM302x_CDEF_H
#define _ADuCM302x_CDEF_H

#include <stdint.h>

/* pickup register bitfield and bit masks */
#include "ADuCM302x.h"


/*  =================================================================================
 *      General Purpose Timer (TMR0)
 *  ================================================================================= */
#define pREG_TMR0_LOAD                   ( (volatile  int16_t *) REG_TMR0_LOAD)                   /*  16-bit load value */
#define pREG_TMR0_CURCNT                 ( (volatile  int16_t *) REG_TMR0_CURCNT)                 /*  16-bit timer value */
#define pREG_TMR0_CTL                    ((volatile  uint16_t *) REG_TMR0_CTL)                    /*  Control */
#define pREG_TMR0_CLRINT                 ((volatile  uint16_t *) REG_TMR0_CLRINT)                 /*  Clear Interrupt */
#define pREG_TMR0_CAPTURE                ( (volatile  int16_t *) REG_TMR0_CAPTURE)                /*  Capture */
#define pREG_TMR0_ALOAD                  ( (volatile  int16_t *) REG_TMR0_ALOAD)                  /*  16-bit load value, asynchronous */
#define pREG_TMR0_ACURCNT                ( (volatile  int16_t *) REG_TMR0_ACURCNT)                /*  16-bit timer value, asynchronous */
#define pREG_TMR0_STAT                   ((volatile  uint16_t *) REG_TMR0_STAT)                   /*  Status */
#define pREG_TMR0_PWMCTL                 ((volatile  uint16_t *) REG_TMR0_PWMCTL)                 /*  PWM Control Register */
#define pREG_TMR0_PWMMATCH               ( (volatile  int16_t *) REG_TMR0_PWMMATCH)               /*  PWM Match Value */

/*  =================================================================================
 *      General Purpose Timer (TMR1)
 *  ================================================================================= */
#define pREG_TMR1_LOAD                   ( (volatile  int16_t *) REG_TMR1_LOAD)                   /*  16-bit load value */
#define pREG_TMR1_CURCNT                 ( (volatile  int16_t *) REG_TMR1_CURCNT)                 /*  16-bit timer value */
#define pREG_TMR1_CTL                    ((volatile  uint16_t *) REG_TMR1_CTL)                    /*  Control */
#define pREG_TMR1_CLRINT                 ((volatile  uint16_t *) REG_TMR1_CLRINT)                 /*  Clear Interrupt */
#define pREG_TMR1_CAPTURE                ( (volatile  int16_t *) REG_TMR1_CAPTURE)                /*  Capture */
#define pREG_TMR1_ALOAD                  ( (volatile  int16_t *) REG_TMR1_ALOAD)                  /*  16-bit load value, asynchronous */
#define pREG_TMR1_ACURCNT                ( (volatile  int16_t *) REG_TMR1_ACURCNT)                /*  16-bit timer value, asynchronous */
#define pREG_TMR1_STAT                   ((volatile  uint16_t *) REG_TMR1_STAT)                   /*  Status */
#define pREG_TMR1_PWMCTL                 ((volatile  uint16_t *) REG_TMR1_PWMCTL)                 /*  PWM Control Register */
#define pREG_TMR1_PWMMATCH               ( (volatile  int16_t *) REG_TMR1_PWMMATCH)               /*  PWM Match Value */

/*  =================================================================================
 *      General Purpose Timer (TMR2)
 *  ================================================================================= */
#define pREG_TMR2_LOAD                   ( (volatile  int16_t *) REG_TMR2_LOAD)                   /*  16-bit load value */
#define pREG_TMR2_CURCNT                 ( (volatile  int16_t *) REG_TMR2_CURCNT)                 /*  16-bit timer value */
#define pREG_TMR2_CTL                    ((volatile  uint16_t *) REG_TMR2_CTL)                    /*  Control */
#define pREG_TMR2_CLRINT                 ((volatile  uint16_t *) REG_TMR2_CLRINT)                 /*  Clear Interrupt */
#define pREG_TMR2_CAPTURE                ( (volatile  int16_t *) REG_TMR2_CAPTURE)                /*  Capture */
#define pREG_TMR2_ALOAD                  ( (volatile  int16_t *) REG_TMR2_ALOAD)                  /*  16-bit load value, asynchronous */
#define pREG_TMR2_ACURCNT                ( (volatile  int16_t *) REG_TMR2_ACURCNT)                /*  16-bit timer value, asynchronous */
#define pREG_TMR2_STAT                   ((volatile  uint16_t *) REG_TMR2_STAT)                   /*  Status */
#define pREG_TMR2_PWMCTL                 ((volatile  uint16_t *) REG_TMR2_PWMCTL)                 /*  PWM Control Register */
#define pREG_TMR2_PWMMATCH               ( (volatile  int16_t *) REG_TMR2_PWMMATCH)               /*  PWM Match Value */

/*  =================================================================================
 *      Real-Time Clock (RTC0)
 *  ================================================================================= */
#define pREG_RTC0_CR0                    ((volatile  uint32_t *) REG_RTC0_CR0)                    /*  RTC Control 0 */
#define pREG_RTC0_SR0                    ((volatile  uint32_t *) REG_RTC0_SR0)                    /*  RTC Status 0 */
#define pREG_RTC0_SR1                    ((volatile  uint32_t *) REG_RTC0_SR1)                    /*  RTC Status 1 */
#define pREG_RTC0_CNT0                   ((volatile  uint32_t *) REG_RTC0_CNT0)                   /*  RTC Count 0 */
#define pREG_RTC0_CNT1                   ( (volatile  int16_t *) REG_RTC0_CNT1)                   /*  RTC Count 1 */
#define pREG_RTC0_ALM0                   ((volatile  uint32_t *) REG_RTC0_ALM0)                   /*  RTC Alarm 0 */
#define pREG_RTC0_ALM1                   ( (volatile  int16_t *) REG_RTC0_ALM1)                   /*  RTC Alarm 1 */
#define pREG_RTC0_TRM                    ((volatile  uint16_t *) REG_RTC0_TRM)                    /*  RTC Trim */
#define pREG_RTC0_GWY                    ( (volatile  int16_t *) REG_RTC0_GWY)                    /*  RTC Gateway */
#define pREG_RTC0_CR1                    ((volatile  uint16_t *) REG_RTC0_CR1)                    /*  RTC Control 1 */
#define pREG_RTC0_SR2                    ((volatile  uint16_t *) REG_RTC0_SR2)                    /*  RTC Status 2 */
#define pREG_RTC0_SNAP0                  ((volatile  uint32_t *) REG_RTC0_SNAP0)                  /*  RTC Snapshot 0 */
#define pREG_RTC0_SNAP1                  ( (volatile  int16_t *) REG_RTC0_SNAP1)                  /*  RTC Snapshot 1 */
#define pREG_RTC0_SNAP2                  ((volatile  uint32_t *) REG_RTC0_SNAP2)                  /*  RTC Snapshot 2 */
#define pREG_RTC0_MOD                    ((volatile  uint32_t *) REG_RTC0_MOD)                    /*  RTC Modulo */
#define pREG_RTC0_CNT2                   ((volatile  uint32_t *) REG_RTC0_CNT2)                   /*  RTC Count 2 */
#define pREG_RTC0_ALM2                   ((volatile  uint32_t *) REG_RTC0_ALM2)                   /*  RTC Alarm 2 */

/*  =================================================================================
 *      Real-Time Clock (RTC1)
 *  ================================================================================= */
#define pREG_RTC1_CR0                    ((volatile  uint32_t *) REG_RTC1_CR0)                    /*  RTC Control 0 */
#define pREG_RTC1_SR0                    ((volatile  uint32_t *) REG_RTC1_SR0)                    /*  RTC Status 0 */
#define pREG_RTC1_SR1                    ((volatile  uint32_t *) REG_RTC1_SR1)                    /*  RTC Status 1 */
#define pREG_RTC1_CNT0                   ((volatile  uint32_t *) REG_RTC1_CNT0)                   /*  RTC Count 0 */
#define pREG_RTC1_CNT1                   ( (volatile  int16_t *) REG_RTC1_CNT1)                   /*  RTC Count 1 */
#define pREG_RTC1_ALM0                   ((volatile  uint32_t *) REG_RTC1_ALM0)                   /*  RTC Alarm 0 */
#define pREG_RTC1_ALM1                   ( (volatile  int16_t *) REG_RTC1_ALM1)                   /*  RTC Alarm 1 */
#define pREG_RTC1_TRM                    ((volatile  uint16_t *) REG_RTC1_TRM)                    /*  RTC Trim */
#define pREG_RTC1_GWY                    ( (volatile  int16_t *) REG_RTC1_GWY)                    /*  RTC Gateway */
#define pREG_RTC1_CR1                    ((volatile  uint16_t *) REG_RTC1_CR1)                    /*  RTC Control 1 */
#define pREG_RTC1_SR2                    ((volatile  uint16_t *) REG_RTC1_SR2)                    /*  RTC Status 2 */
#define pREG_RTC1_SNAP0                  ((volatile  uint32_t *) REG_RTC1_SNAP0)                  /*  RTC Snapshot 0 */
#define pREG_RTC1_SNAP1                  ( (volatile  int16_t *) REG_RTC1_SNAP1)                  /*  RTC Snapshot 1 */
#define pREG_RTC1_SNAP2                  ((volatile  uint32_t *) REG_RTC1_SNAP2)                  /*  RTC Snapshot 2 */
#define pREG_RTC1_MOD                    ((volatile  uint32_t *) REG_RTC1_MOD)                    /*  RTC Modulo */
#define pREG_RTC1_CNT2                   ((volatile  uint32_t *) REG_RTC1_CNT2)                   /*  RTC Count 2 */
#define pREG_RTC1_ALM2                   ((volatile  uint32_t *) REG_RTC1_ALM2)                   /*  RTC Alarm 2 */

/*  =================================================================================
 *      System Identification and Debug Enable (SYS)
 *  ================================================================================= */
#define pREG_SYS_ADIID                   ((volatile  uint16_t *) REG_SYS_ADIID)                   /*  ADI Identification */
#define pREG_SYS_CHIPID                  ((volatile  uint16_t *) REG_SYS_CHIPID)                  /*  Chip Identifier */
#define pREG_SYS_SWDEN                   ((volatile  uint16_t *) REG_SYS_SWDEN)                   /*  Serial Wire Debug Enable */

/*  =================================================================================
 *      Watchdog Timer (WDT0)
 *  ================================================================================= */
#define pREG_WDT0_LOAD                   ( (volatile  int16_t *) REG_WDT0_LOAD)                   /*  Load value */
#define pREG_WDT0_CCNT                   ( (volatile  int16_t *) REG_WDT0_CCNT)                   /*  Current count value */
#define pREG_WDT0_CTL                    ((volatile  uint16_t *) REG_WDT0_CTL)                    /*  Control */
#define pREG_WDT0_RESTART                ( (volatile  int16_t *) REG_WDT0_RESTART)                /*  Clear interrupt */
#define pREG_WDT0_STAT                   ((volatile  uint16_t *) REG_WDT0_STAT)                   /*  Status */

/*  =================================================================================
 *      I2C Master/Slave (I2C0)
 *  ================================================================================= */
#define pREG_I2C0_MCTL                   ((volatile  uint16_t *) REG_I2C0_MCTL)                   /*  Master control */
#define pREG_I2C0_MSTAT                  ((volatile  uint16_t *) REG_I2C0_MSTAT)                  /*  Master status */
#define pREG_I2C0_MRX                    ((volatile  uint16_t *) REG_I2C0_MRX)                    /*  Master receive data */
#define pREG_I2C0_MTX                    ((volatile  uint16_t *) REG_I2C0_MTX)                    /*  Master transmit data */
#define pREG_I2C0_MRXCNT                 ((volatile  uint16_t *) REG_I2C0_MRXCNT)                 /*  Master receive data count */
#define pREG_I2C0_MCRXCNT                ((volatile  uint16_t *) REG_I2C0_MCRXCNT)                /*  Master current receive data count */
#define pREG_I2C0_ADDR1                  ((volatile  uint16_t *) REG_I2C0_ADDR1)                  /*  1st master address byte */
#define pREG_I2C0_ADDR2                  ((volatile  uint16_t *) REG_I2C0_ADDR2)                  /*  2nd master address byte */
#define pREG_I2C0_BYT                    ((volatile  uint16_t *) REG_I2C0_BYT)                    /*  Start byte */
#define pREG_I2C0_DIV                    ((volatile  uint16_t *) REG_I2C0_DIV)                    /*  Serial clock period divisor */
#define pREG_I2C0_SCTL                   ((volatile  uint16_t *) REG_I2C0_SCTL)                   /*  Slave control */
#define pREG_I2C0_SSTAT                  ((volatile  uint16_t *) REG_I2C0_SSTAT)                  /*  Slave I2C Status/Error/IRQ */
#define pREG_I2C0_SRX                    ((volatile  uint16_t *) REG_I2C0_SRX)                    /*  Slave receive */
#define pREG_I2C0_STX                    ((volatile  uint16_t *) REG_I2C0_STX)                    /*  Slave transmit */
#define pREG_I2C0_ALT                    ((volatile  uint16_t *) REG_I2C0_ALT)                    /*  Hardware general call ID */
#define pREG_I2C0_ID0                    ((volatile  uint16_t *) REG_I2C0_ID0)                    /*  1st slave address device ID */
#define pREG_I2C0_ID1                    ((volatile  uint16_t *) REG_I2C0_ID1)                    /*  2nd slave address device ID */
#define pREG_I2C0_ID2                    ((volatile  uint16_t *) REG_I2C0_ID2)                    /*  3rd slave address device ID */
#define pREG_I2C0_ID3                    ((volatile  uint16_t *) REG_I2C0_ID3)                    /*  4th slave address device ID */
#define pREG_I2C0_STAT                   ((volatile  uint16_t *) REG_I2C0_STAT)                   /*  Master and slave FIFO status */
#define pREG_I2C0_SHCTL                  ((volatile  uint16_t *) REG_I2C0_SHCTL)                  /*  Shared control */
#define pREG_I2C0_TCTL                   ((volatile  uint16_t *) REG_I2C0_TCTL)                   /*  Timing Control Register */
#define pREG_I2C0_ASTRETCH_SCL           ((volatile  uint16_t *) REG_I2C0_ASTRETCH_SCL)           /*  Automatic stretch SCL register */

/*  =================================================================================
 *      Serial Peripheral Interface (SPI0)
 *  ================================================================================= */
#define pREG_SPI0_STAT                   ((volatile  uint16_t *) REG_SPI0_STAT)                   /*  Status */
#define pREG_SPI0_RX                     ((volatile  uint16_t *) REG_SPI0_RX)                     /*  Receive */
#define pREG_SPI0_TX                     ((volatile  uint16_t *) REG_SPI0_TX)                     /*  Transmit */
#define pREG_SPI0_DIV                    ((volatile  uint16_t *) REG_SPI0_DIV)                    /*  SPI baud rate selection */
#define pREG_SPI0_CTL                    ((volatile  uint16_t *) REG_SPI0_CTL)                    /*  SPI configuration 1 */
#define pREG_SPI0_IEN                    ((volatile  uint16_t *) REG_SPI0_IEN)                    /*  SPI configuration 2 */
#define pREG_SPI0_CNT                    ((volatile  uint16_t *) REG_SPI0_CNT)                    /*  Transfer byte count */
#define pREG_SPI0_DMA                    ((volatile  uint16_t *) REG_SPI0_DMA)                    /*  SPI DMA enable */
#define pREG_SPI0_FIFO_STAT              ((volatile  uint16_t *) REG_SPI0_FIFO_STAT)              /*  FIFO Status */
#define pREG_SPI0_RD_CTL                 ((volatile  uint16_t *) REG_SPI0_RD_CTL)                 /*  Read Control */
#define pREG_SPI0_FLOW_CTL               ((volatile  uint16_t *) REG_SPI0_FLOW_CTL)               /*  Flow Control */
#define pREG_SPI0_WAIT_TMR               ((volatile  uint16_t *) REG_SPI0_WAIT_TMR)               /*  Wait timer for flow control */
#define pREG_SPI0_CS_CTL                 ((volatile  uint16_t *) REG_SPI0_CS_CTL)                 /*  Chip-Select control for multi-slave connections */
#define pREG_SPI0_CS_OVERRIDE            ((volatile  uint16_t *) REG_SPI0_CS_OVERRIDE)            /*  Chip-Select Override */
#define pREG_SPI0_XFR_CNT                ((volatile  uint16_t *) REG_SPI0_XFR_CNT)                /*  Transferred Count */

/*  =================================================================================
 *      Serial Peripheral Interface (SPI1)
 *  ================================================================================= */
#define pREG_SPI1_STAT                   ((volatile  uint16_t *) REG_SPI1_STAT)                   /*  Status */
#define pREG_SPI1_RX                     ((volatile  uint16_t *) REG_SPI1_RX)                     /*  Receive */
#define pREG_SPI1_TX                     ((volatile  uint16_t *) REG_SPI1_TX)                     /*  Transmit */
#define pREG_SPI1_DIV                    ((volatile  uint16_t *) REG_SPI1_DIV)                    /*  SPI baud rate selection */
#define pREG_SPI1_CTL                    ((volatile  uint16_t *) REG_SPI1_CTL)                    /*  SPI configuration 1 */
#define pREG_SPI1_IEN                    ((volatile  uint16_t *) REG_SPI1_IEN)                    /*  SPI configuration 2 */
#define pREG_SPI1_CNT                    ((volatile  uint16_t *) REG_SPI1_CNT)                    /*  Transfer byte count */
#define pREG_SPI1_DMA                    ((volatile  uint16_t *) REG_SPI1_DMA)                    /*  SPI DMA enable */
#define pREG_SPI1_FIFO_STAT              ((volatile  uint16_t *) REG_SPI1_FIFO_STAT)              /*  FIFO Status */
#define pREG_SPI1_RD_CTL                 ((volatile  uint16_t *) REG_SPI1_RD_CTL)                 /*  Read Control */
#define pREG_SPI1_FLOW_CTL               ((volatile  uint16_t *) REG_SPI1_FLOW_CTL)               /*  Flow Control */
#define pREG_SPI1_WAIT_TMR               ((volatile  uint16_t *) REG_SPI1_WAIT_TMR)               /*  Wait timer for flow control */
#define pREG_SPI1_CS_CTL                 ((volatile  uint16_t *) REG_SPI1_CS_CTL)                 /*  Chip-Select control for multi-slave connections */
#define pREG_SPI1_CS_OVERRIDE            ((volatile  uint16_t *) REG_SPI1_CS_OVERRIDE)            /*  Chip-Select Override */
#define pREG_SPI1_XFR_CNT                ((volatile  uint16_t *) REG_SPI1_XFR_CNT)                /*  Transferred Count */

/*  =================================================================================
 *      Serial Peripheral Interface (SPI2)
 *  ================================================================================= */
#define pREG_SPI2_STAT                   ((volatile  uint16_t *) REG_SPI2_STAT)                   /*  Status */
#define pREG_SPI2_RX                     ((volatile  uint16_t *) REG_SPI2_RX)                     /*  Receive */
#define pREG_SPI2_TX                     ((volatile  uint16_t *) REG_SPI2_TX)                     /*  Transmit */
#define pREG_SPI2_DIV                    ((volatile  uint16_t *) REG_SPI2_DIV)                    /*  SPI baud rate selection */
#define pREG_SPI2_CTL                    ((volatile  uint16_t *) REG_SPI2_CTL)                    /*  SPI configuration 1 */
#define pREG_SPI2_IEN                    ((volatile  uint16_t *) REG_SPI2_IEN)                    /*  SPI configuration 2 */
#define pREG_SPI2_CNT                    ((volatile  uint16_t *) REG_SPI2_CNT)                    /*  Transfer byte count */
#define pREG_SPI2_DMA                    ((volatile  uint16_t *) REG_SPI2_DMA)                    /*  SPI DMA enable */
#define pREG_SPI2_FIFO_STAT              ((volatile  uint16_t *) REG_SPI2_FIFO_STAT)              /*  FIFO Status */
#define pREG_SPI2_RD_CTL                 ((volatile  uint16_t *) REG_SPI2_RD_CTL)                 /*  Read Control */
#define pREG_SPI2_FLOW_CTL               ((volatile  uint16_t *) REG_SPI2_FLOW_CTL)               /*  Flow Control */
#define pREG_SPI2_WAIT_TMR               ((volatile  uint16_t *) REG_SPI2_WAIT_TMR)               /*  Wait timer for flow control */
#define pREG_SPI2_CS_CTL                 ((volatile  uint16_t *) REG_SPI2_CS_CTL)                 /*  Chip-Select control for multi-slave connections */
#define pREG_SPI2_CS_OVERRIDE            ((volatile  uint16_t *) REG_SPI2_CS_OVERRIDE)            /*  Chip-Select Override */
#define pREG_SPI2_XFR_CNT                ((volatile  uint16_t *) REG_SPI2_XFR_CNT)                /*  Transferred Count */

/*  =================================================================================
 *      UART (UART0)
 *  ================================================================================= */
#define pREG_UART0_RX                    ((volatile  uint16_t *) REG_UART0_RX)                    /*  Receive Buffer Register */
#define pREG_UART0_TX                    ((volatile  uint16_t *) REG_UART0_TX)                    /*  Transmit Holding Register */
#define pREG_UART0_IEN                   ((volatile  uint16_t *) REG_UART0_IEN)                   /*  Interrupt Enable */
#define pREG_UART0_IIR                   ((volatile  uint16_t *) REG_UART0_IIR)                   /*  Interrupt ID */
#define pREG_UART0_LCR                   ((volatile  uint16_t *) REG_UART0_LCR)                   /*  Line Control */
#define pREG_UART0_MCR                   ((volatile  uint16_t *) REG_UART0_MCR)                   /*  Modem Control */
#define pREG_UART0_LSR                   ((volatile  uint16_t *) REG_UART0_LSR)                   /*  Line Status */
#define pREG_UART0_MSR                   ((volatile  uint16_t *) REG_UART0_MSR)                   /*  Modem Status */
#define pREG_UART0_SCR                   ((volatile  uint16_t *) REG_UART0_SCR)                   /*  Scratch buffer */
#define pREG_UART0_FBR                   ((volatile  uint16_t *) REG_UART0_FBR)                   /*  Fractional Baud Rate */
#define pREG_UART0_DIV                   ( (volatile  int16_t *) REG_UART0_DIV)                   /*  Baudrate divider */
#define pREG_UART0_CTL                   ((volatile  uint16_t *) REG_UART0_CTL)                   /*  UART control register */

/*  =================================================================================
 *      Beeper Driver (BEEP0)
 *  ================================================================================= */
#define pREG_BEEP0_CFG                   ((volatile  uint16_t *) REG_BEEP0_CFG)                   /*  Beeper configuration */
#define pREG_BEEP0_STAT                  ((volatile  uint16_t *) REG_BEEP0_STAT)                  /*  Beeper status */
#define pREG_BEEP0_TONEA                 ((volatile  uint16_t *) REG_BEEP0_TONEA)                 /*  Tone A Data */
#define pREG_BEEP0_TONEB                 ((volatile  uint16_t *) REG_BEEP0_TONEB)                 /*  Tone B Data */

/*  =================================================================================
 *      DMA (DMA0)
 *  ================================================================================= */
#define pREG_DMA0_STAT                   ((volatile  uint32_t *) REG_DMA0_STAT)                   /*  DMA Status */
#define pREG_DMA0_CFG                    ((volatile  uint32_t *) REG_DMA0_CFG)                    /*  DMA Configuration */
#define pREG_DMA0_PDBPTR                 ( (volatile  int32_t *) REG_DMA0_PDBPTR)                 /*  DMA channel primary control data base pointer */
#define pREG_DMA0_ADBPTR                 ( (volatile  int32_t *) REG_DMA0_ADBPTR)                 /*  DMA channel alternate control data base pointer */
#define pREG_DMA0_SWREQ                  ((volatile  uint32_t *) REG_DMA0_SWREQ)                  /*  DMA channel software request */
#define pREG_DMA0_RMSK_SET               ((volatile  uint32_t *) REG_DMA0_RMSK_SET)               /*  DMA channel request mask set */
#define pREG_DMA0_RMSK_CLR               ((volatile  uint32_t *) REG_DMA0_RMSK_CLR)               /*  DMA channel request mask clear */
#define pREG_DMA0_EN_SET                 ((volatile  uint32_t *) REG_DMA0_EN_SET)                 /*  DMA channel enable set */
#define pREG_DMA0_EN_CLR                 ((volatile  uint32_t *) REG_DMA0_EN_CLR)                 /*  DMA channel enable clear */
#define pREG_DMA0_ALT_SET                ((volatile  uint32_t *) REG_DMA0_ALT_SET)                /*  DMA channel primary-alternate set */
#define pREG_DMA0_ALT_CLR                ((volatile  uint32_t *) REG_DMA0_ALT_CLR)                /*  DMA channel primary-alternate clear */
#define pREG_DMA0_PRI_SET                ((volatile  uint32_t *) REG_DMA0_PRI_SET)                /*  DMA channel priority set */
#define pREG_DMA0_PRI_CLR                ((volatile  uint32_t *) REG_DMA0_PRI_CLR)                /*  DMA channel priority clear */
#define pREG_DMA0_ERRCHNL_CLR            ((volatile  uint32_t *) REG_DMA0_ERRCHNL_CLR)            /*  DMA Per Channel Error Clear */
#define pREG_DMA0_ERR_CLR                ((volatile  uint32_t *) REG_DMA0_ERR_CLR)                /*  DMA bus error clear */
#define pREG_DMA0_INVALIDDESC_CLR        ((volatile  uint32_t *) REG_DMA0_INVALIDDESC_CLR)        /*  DMA Per Channel Invalid Descriptor Clear */
#define pREG_DMA0_BS_SET                 ((volatile  uint32_t *) REG_DMA0_BS_SET)                 /*  DMA channel bytes swap enable set */
#define pREG_DMA0_BS_CLR                 ((volatile  uint32_t *) REG_DMA0_BS_CLR)                 /*  DMA channel bytes swap enable clear */
#define pREG_DMA0_SRCADDR_SET            ((volatile  uint32_t *) REG_DMA0_SRCADDR_SET)            /*  DMA channel source address decrement enable set */
#define pREG_DMA0_SRCADDR_CLR            ((volatile  uint32_t *) REG_DMA0_SRCADDR_CLR)            /*  DMA channel source address decrement enable clear */
#define pREG_DMA0_DSTADDR_SET            ((volatile  uint32_t *) REG_DMA0_DSTADDR_SET)            /*  DMA channel destination address decrement enable set */
#define pREG_DMA0_DSTADDR_CLR            ((volatile  uint32_t *) REG_DMA0_DSTADDR_CLR)            /*  DMA channel destination address decrement enable clear */
#define pREG_DMA0_REVID                  ((volatile  uint32_t *) REG_DMA0_REVID)                  /*  DMA Controller Revision ID */

/*  =================================================================================
 *      Flash Controller (FLCC0)
 *  ================================================================================= */
#define pREG_FLCC0_STAT                  ((volatile  uint32_t *) REG_FLCC0_STAT)                  /*  Status */
#define pREG_FLCC0_IEN                   ((volatile  uint32_t *) REG_FLCC0_IEN)                   /*  Interrupt Enable */
#define pREG_FLCC0_CMD                   ((volatile  uint32_t *) REG_FLCC0_CMD)                   /*  Command */
#define pREG_FLCC0_KH_ADDR               ((volatile  uint32_t *) REG_FLCC0_KH_ADDR)               /*  WRITE address */
#define pREG_FLCC0_KH_DATA0              ( (volatile  int32_t *) REG_FLCC0_KH_DATA0)              /*  WRITE lower data */
#define pREG_FLCC0_KH_DATA1              ( (volatile  int32_t *) REG_FLCC0_KH_DATA1)              /*  WRITE upper data */
#define pREG_FLCC0_PAGE_ADDR0            ((volatile  uint32_t *) REG_FLCC0_PAGE_ADDR0)            /*  Lower page address */
#define pREG_FLCC0_PAGE_ADDR1            ((volatile  uint32_t *) REG_FLCC0_PAGE_ADDR1)            /*  Upper page address */
#define pREG_FLCC0_KEY                   ( (volatile  int32_t *) REG_FLCC0_KEY)                   /*  Key */
#define pREG_FLCC0_WR_ABORT_ADDR         ( (volatile  int32_t *) REG_FLCC0_WR_ABORT_ADDR)         /*  Write abort address */
#define pREG_FLCC0_WRPROT                ( (volatile  int32_t *) REG_FLCC0_WRPROT)                /*  Write protection */
#define pREG_FLCC0_SIGNATURE             ( (volatile  int32_t *) REG_FLCC0_SIGNATURE)             /*  Signature */
#define pREG_FLCC0_UCFG                  ((volatile  uint32_t *) REG_FLCC0_UCFG)                  /*  User configuration */
#define pREG_FLCC0_TIME_PARAM0           ((volatile  uint32_t *) REG_FLCC0_TIME_PARAM0)           /*  Time parameter 0 */
#define pREG_FLCC0_TIME_PARAM1           ((volatile  uint32_t *) REG_FLCC0_TIME_PARAM1)           /*  Time parameter 1 */
#define pREG_FLCC0_ABORT_EN_LO           ((volatile  uint32_t *) REG_FLCC0_ABORT_EN_LO)           /*  IRQ Abort Enable (lower bits) */
#define pREG_FLCC0_ABORT_EN_HI           ((volatile  uint32_t *) REG_FLCC0_ABORT_EN_HI)           /*  IRQ Abort Enable (upper bits) */
#define pREG_FLCC0_ECC_CFG               ((volatile  uint32_t *) REG_FLCC0_ECC_CFG)               /*  ECC Config */
#define pREG_FLCC0_ECC_ADDR              ((volatile  uint32_t *) REG_FLCC0_ECC_ADDR)              /*  ECC Status (address) */

/*  =================================================================================
 *      Cache Controller (FLCC0_CACHE)
 *  ================================================================================= */
#define pREG_FLCC0_CACHE_STAT            ((volatile  uint32_t *) REG_FLCC0_CACHE_STAT)            /*  Cache Status register */
#define pREG_FLCC0_CACHE_SETUP           ((volatile  uint32_t *) REG_FLCC0_CACHE_SETUP)           /*  Cache Setup register */
#define pREG_FLCC0_CACHE_KEY             ( (volatile  int32_t *) REG_FLCC0_CACHE_KEY)             /*  Cache Key register */

/*  =================================================================================
 *      DFT Controller (FLCC0_DFT)
 *  ================================================================================= */
#define pREG_FLCC0_DFT_CMD               ((volatile  uint32_t *) REG_FLCC0_DFT_CMD)               /*  Command register */

/*  =================================================================================
 *      Flash Controller (FLCC0_TEST)
 *  ================================================================================= */

/*  =================================================================================
 *       (GPIO0)
 *  ================================================================================= */
#define pREG_GPIO0_CFG                   ((volatile  uint32_t *) REG_GPIO0_CFG)                   /*  Port  Configuration */
#define pREG_GPIO0_OEN                   ( (volatile  int16_t *) REG_GPIO0_OEN)                   /*  Port output enable */
#define pREG_GPIO0_PE                    ( (volatile  int16_t *) REG_GPIO0_PE)                    /*  Port output pullup/pulldown enable */
#define pREG_GPIO0_IEN                   ( (volatile  int16_t *) REG_GPIO0_IEN)                   /*  Port  Input Path Enable */
#define pREG_GPIO0_IN                    ( (volatile  int16_t *) REG_GPIO0_IN)                    /*  Port  registered data input */
#define pREG_GPIO0_OUT                   ( (volatile  int16_t *) REG_GPIO0_OUT)                   /*  Port data output */
#define pREG_GPIO0_SET                   ( (volatile  int16_t *) REG_GPIO0_SET)                   /*  Port data out set */
#define pREG_GPIO0_CLR                   ( (volatile  int16_t *) REG_GPIO0_CLR)                   /*  Port data out clear */
#define pREG_GPIO0_TGL                   ( (volatile  int16_t *) REG_GPIO0_TGL)                   /*  Port  pin toggle */
#define pREG_GPIO0_POL                   ( (volatile  int16_t *) REG_GPIO0_POL)                   /*  Port interrupt polarity */
#define pREG_GPIO0_IENA                  ( (volatile  int16_t *) REG_GPIO0_IENA)                  /*  Port  interrupt A enable */
#define pREG_GPIO0_IENB                  ( (volatile  int16_t *) REG_GPIO0_IENB)                  /*  Port interrupt B enable */
#define pREG_GPIO0_INT                   ( (volatile  int16_t *) REG_GPIO0_INT)                   /*  Port interrupt Status */
#define pREG_GPIO0_DS                    ( (volatile  int16_t *) REG_GPIO0_DS)                    /*  Port  drive strength select */

/*  =================================================================================
 *       (GPIO1)
 *  ================================================================================= */
#define pREG_GPIO1_CFG                   ((volatile  uint32_t *) REG_GPIO1_CFG)                   /*  Port  Configuration */
#define pREG_GPIO1_OEN                   ( (volatile  int16_t *) REG_GPIO1_OEN)                   /*  Port output enable */
#define pREG_GPIO1_PE                    ( (volatile  int16_t *) REG_GPIO1_PE)                    /*  Port output pullup/pulldown enable */
#define pREG_GPIO1_IEN                   ( (volatile  int16_t *) REG_GPIO1_IEN)                   /*  Port  Input Path Enable */
#define pREG_GPIO1_IN                    ( (volatile  int16_t *) REG_GPIO1_IN)                    /*  Port  registered data input */
#define pREG_GPIO1_OUT                   ( (volatile  int16_t *) REG_GPIO1_OUT)                   /*  Port data output */
#define pREG_GPIO1_SET                   ( (volatile  int16_t *) REG_GPIO1_SET)                   /*  Port data out set */
#define pREG_GPIO1_CLR                   ( (volatile  int16_t *) REG_GPIO1_CLR)                   /*  Port data out clear */
#define pREG_GPIO1_TGL                   ( (volatile  int16_t *) REG_GPIO1_TGL)                   /*  Port  pin toggle */
#define pREG_GPIO1_POL                   ( (volatile  int16_t *) REG_GPIO1_POL)                   /*  Port interrupt polarity */
#define pREG_GPIO1_IENA                  ( (volatile  int16_t *) REG_GPIO1_IENA)                  /*  Port  interrupt A enable */
#define pREG_GPIO1_IENB                  ( (volatile  int16_t *) REG_GPIO1_IENB)                  /*  Port interrupt B enable */
#define pREG_GPIO1_INT                   ( (volatile  int16_t *) REG_GPIO1_INT)                   /*  Port interrupt Status */
#define pREG_GPIO1_DS                    ( (volatile  int16_t *) REG_GPIO1_DS)                    /*  Port  drive strength select */

/*  =================================================================================
 *       (GPIO2)
 *  ================================================================================= */
#define pREG_GPIO2_CFG                   ((volatile  uint32_t *) REG_GPIO2_CFG)                   /*  Port  Configuration */
#define pREG_GPIO2_OEN                   ( (volatile  int16_t *) REG_GPIO2_OEN)                   /*  Port output enable */
#define pREG_GPIO2_PE                    ( (volatile  int16_t *) REG_GPIO2_PE)                    /*  Port output pullup/pulldown enable */
#define pREG_GPIO2_IEN                   ( (volatile  int16_t *) REG_GPIO2_IEN)                   /*  Port  Input Path Enable */
#define pREG_GPIO2_IN                    ( (volatile  int16_t *) REG_GPIO2_IN)                    /*  Port  registered data input */
#define pREG_GPIO2_OUT                   ( (volatile  int16_t *) REG_GPIO2_OUT)                   /*  Port data output */
#define pREG_GPIO2_SET                   ( (volatile  int16_t *) REG_GPIO2_SET)                   /*  Port data out set */
#define pREG_GPIO2_CLR                   ( (volatile  int16_t *) REG_GPIO2_CLR)                   /*  Port data out clear */
#define pREG_GPIO2_TGL                   ( (volatile  int16_t *) REG_GPIO2_TGL)                   /*  Port  pin toggle */
#define pREG_GPIO2_POL                   ( (volatile  int16_t *) REG_GPIO2_POL)                   /*  Port interrupt polarity */
#define pREG_GPIO2_IENA                  ( (volatile  int16_t *) REG_GPIO2_IENA)                  /*  Port  interrupt A enable */
#define pREG_GPIO2_IENB                  ( (volatile  int16_t *) REG_GPIO2_IENB)                  /*  Port interrupt B enable */
#define pREG_GPIO2_INT                   ( (volatile  int16_t *) REG_GPIO2_INT)                   /*  Port interrupt Status */
#define pREG_GPIO2_DS                    ( (volatile  int16_t *) REG_GPIO2_DS)                    /*  Port  drive strength select */

/*  =================================================================================
 *      Serial Port (SPORT0)
 *  ================================================================================= */
#define pREG_SPORT0_CTL_A                ((volatile  uint32_t *) REG_SPORT0_CTL_A)                /*  Half SPORT 'A' Control Register */
#define pREG_SPORT0_DIV_A                ((volatile  uint32_t *) REG_SPORT0_DIV_A)                /*  Half SPORT 'A' Divisor Register */
#define pREG_SPORT0_IEN_A                ((volatile  uint32_t *) REG_SPORT0_IEN_A)                /*  Half SPORT A's Interrupt Enable register */
#define pREG_SPORT0_STAT_A               ((volatile  uint32_t *) REG_SPORT0_STAT_A)               /*  Half SPORT A's Status register */
#define pREG_SPORT0_NUMTRAN_A            ((volatile  uint32_t *) REG_SPORT0_NUMTRAN_A)            /*  Half SPORT A Number of transfers register */
#define pREG_SPORT0_CNVT_A               ((volatile  uint32_t *) REG_SPORT0_CNVT_A)               /*  Half SPORT A's register to be programmed with the desired CONVT pulse width */
#define pREG_SPORT0_TX_A                 ((volatile  uint32_t *) REG_SPORT0_TX_A)                 /*  Half SPORT 'A' Tx Buffer Register */
#define pREG_SPORT0_RX_A                 ((volatile  uint32_t *) REG_SPORT0_RX_A)                 /*  Half SPORT 'A' Rx Buffer Register */
#define pREG_SPORT0_CTL_B                ((volatile  uint32_t *) REG_SPORT0_CTL_B)                /*  Half SPORT 'B' Control Register */
#define pREG_SPORT0_DIV_B                ((volatile  uint32_t *) REG_SPORT0_DIV_B)                /*  Half SPORT 'B' Divisor Register */
#define pREG_SPORT0_IEN_B                ((volatile  uint32_t *) REG_SPORT0_IEN_B)                /*  Half SPORT B's Interrupt Enable register */
#define pREG_SPORT0_STAT_B               ((volatile  uint32_t *) REG_SPORT0_STAT_B)               /*  Half SPORT B's Status register */
#define pREG_SPORT0_NUMTRAN_B            ((volatile  uint32_t *) REG_SPORT0_NUMTRAN_B)            /*  Half SPORT B Number of transfers register */
#define pREG_SPORT0_CNVT_B               ((volatile  uint32_t *) REG_SPORT0_CNVT_B)               /*  Half SPORT B's register to be programmed with the desired CONVT pulse width */
#define pREG_SPORT0_TX_B                 ((volatile  uint32_t *) REG_SPORT0_TX_B)                 /*  Half SPORT 'B' Tx Buffer Register */
#define pREG_SPORT0_RX_B                 ((volatile  uint32_t *) REG_SPORT0_RX_B)                 /*  Half SPORT 'B' Rx Buffer Register */

/*  =================================================================================
 *      CRC (CRC0)
 *  ================================================================================= */
#define pREG_CRC0_CTL                    ((volatile  uint32_t *) REG_CRC0_CTL)                    /*  CRC Control Register */
#define pREG_CRC0_IPDATA                 ( (volatile  int32_t *) REG_CRC0_IPDATA)                 /*  Input Data Register */
#define pREG_CRC0_RESULT                 ( (volatile  int32_t *) REG_CRC0_RESULT)                 /*  CRC Result Register */
#define pREG_CRC0_POLY                   ((volatile  uint32_t *) REG_CRC0_POLY)                   /*  Programmable CRC Polynomial */

/*  =================================================================================
 *      Random Number Generator (RNG0)
 *  ================================================================================= */
#define pREG_RNG0_CTL                    ((volatile  uint16_t *) REG_RNG0_CTL)                    /*  RNG Control Register */
#define pREG_RNG0_LEN                    ((volatile  uint16_t *) REG_RNG0_LEN)                    /*  RNG Sample Length Register */
#define pREG_RNG0_STAT                   ((volatile  uint16_t *) REG_RNG0_STAT)                   /*  RNG Status Register */
#define pREG_RNG0_DATA                   ((volatile  uint32_t *) REG_RNG0_DATA)                   /*  RNG Data Register */
#define pREG_RNG0_OSCCNT                 ((volatile  uint32_t *) REG_RNG0_OSCCNT)                 /*  Oscillator Count */
#define pREG_RNG0_OSCDIFF0               ( (volatile   int8_t *) REG_RNG0_OSCDIFF0)               /*  Oscillator Difference */
#define pREG_RNG0_OSCDIFF1               ( (volatile   int8_t *) REG_RNG0_OSCDIFF1)               /*  Oscillator Difference */
#define pREG_RNG0_OSCDIFF2               ( (volatile   int8_t *) REG_RNG0_OSCDIFF2)               /*  Oscillator Difference */
#define pREG_RNG0_OSCDIFF3               ( (volatile   int8_t *) REG_RNG0_OSCDIFF3)               /*  Oscillator Difference */

/*  =================================================================================
 *      crypto (CRYPT0)
 *  ================================================================================= */
#define pREG_CRYPT0_CFG                  ((volatile  uint32_t *) REG_CRYPT0_CFG)                  /*  Configuration Register */
#define pREG_CRYPT0_DATALEN              ((volatile  uint32_t *) REG_CRYPT0_DATALEN)              /*  Payload Data Length */
#define pREG_CRYPT0_PREFIXLEN            ((volatile  uint32_t *) REG_CRYPT0_PREFIXLEN)            /*  Authentication Data Length */
#define pREG_CRYPT0_INTEN                ((volatile  uint32_t *) REG_CRYPT0_INTEN)                /*  Interrupt Enable Register */
#define pREG_CRYPT0_STAT                 ((volatile  uint32_t *) REG_CRYPT0_STAT)                 /*  Status Register */
#define pREG_CRYPT0_INBUF                ( (volatile  int32_t *) REG_CRYPT0_INBUF)                /*  Input Buffer */
#define pREG_CRYPT0_OUTBUF               ( (volatile  int32_t *) REG_CRYPT0_OUTBUF)               /*  Output Buffer */
#define pREG_CRYPT0_NONCE0               ((volatile  uint32_t *) REG_CRYPT0_NONCE0)               /*  Nonce Bits [31:0] */
#define pREG_CRYPT0_NONCE1               ((volatile  uint32_t *) REG_CRYPT0_NONCE1)               /*  Nonce Bits [63:32] */
#define pREG_CRYPT0_NONCE2               ((volatile  uint32_t *) REG_CRYPT0_NONCE2)               /*  Nonce Bits [95:64] */
#define pREG_CRYPT0_NONCE3               ((volatile  uint32_t *) REG_CRYPT0_NONCE3)               /*  Nonce Bits [127:96] */
#define pREG_CRYPT0_AESKEY0              ((volatile  uint32_t *) REG_CRYPT0_AESKEY0)              /*  Key Bits[ 31:0 ] */
#define pREG_CRYPT0_AESKEY1              ((volatile  uint32_t *) REG_CRYPT0_AESKEY1)              /*  Key Bits [ 63:32 ] */
#define pREG_CRYPT0_AESKEY2              ((volatile  uint32_t *) REG_CRYPT0_AESKEY2)              /*  Key Bits [ 95:64 ] */
#define pREG_CRYPT0_AESKEY3              ((volatile  uint32_t *) REG_CRYPT0_AESKEY3)              /*  Key Bits [ 127:96 ] */
#define pREG_CRYPT0_AESKEY4              ((volatile  uint32_t *) REG_CRYPT0_AESKEY4)              /*  Key Bits [ 159:128 ] */
#define pREG_CRYPT0_AESKEY5              ((volatile  uint32_t *) REG_CRYPT0_AESKEY5)              /*  Key Bits [ 191:160 ] */
#define pREG_CRYPT0_AESKEY6              ((volatile  uint32_t *) REG_CRYPT0_AESKEY6)              /*  Key Bits [ 223:192 ] */
#define pREG_CRYPT0_AESKEY7              ((volatile  uint32_t *) REG_CRYPT0_AESKEY7)              /*  Key Bits [ 255:224 ] */
#define pREG_CRYPT0_CNTRINIT             ((volatile  uint32_t *) REG_CRYPT0_CNTRINIT)             /*  Counter Initialization Vector */
#define pREG_CRYPT0_SHAH0                ((volatile  uint32_t *) REG_CRYPT0_SHAH0)                /*  SHA Bits [ 31:0 ] */
#define pREG_CRYPT0_SHAH1                ((volatile  uint32_t *) REG_CRYPT0_SHAH1)                /*  SHA Bits [ 63:32 ] */
#define pREG_CRYPT0_SHAH2                ((volatile  uint32_t *) REG_CRYPT0_SHAH2)                /*  SHA Bits [ 95:64 ] */
#define pREG_CRYPT0_SHAH3                ((volatile  uint32_t *) REG_CRYPT0_SHAH3)                /*  SHA Bits [ 127:96 ] */
#define pREG_CRYPT0_SHAH4                ((volatile  uint32_t *) REG_CRYPT0_SHAH4)                /*  SHA Bits [ 159:128 ] */
#define pREG_CRYPT0_SHAH5                ((volatile  uint32_t *) REG_CRYPT0_SHAH5)                /*  SHA Bits [ 191:160 ] */
#define pREG_CRYPT0_SHAH6                ((volatile  uint32_t *) REG_CRYPT0_SHAH6)                /*  SHA Bits [ 223:192] */
#define pREG_CRYPT0_SHAH7                ((volatile  uint32_t *) REG_CRYPT0_SHAH7)                /*  SHA Bits [ 255:224 ] */

/*  =================================================================================
 *      Power Management  (PMG0)
 *  ================================================================================= */
#define pREG_PMG0_IEN                    ((volatile  uint32_t *) REG_PMG0_IEN)                    /*  Power Supply Monitor Interrupt Enable */
#define pREG_PMG0_PSM_STAT               ((volatile  uint32_t *) REG_PMG0_PSM_STAT)               /*  Power supply monitor status */
#define pREG_PMG0_PWRMOD                 ((volatile  uint32_t *) REG_PMG0_PWRMOD)                 /*  Power Mode Register */
#define pREG_PMG0_PWRKEY                 ((volatile  uint32_t *) REG_PMG0_PWRKEY)                 /*  Key protection for PWRMOD */
#define pREG_PMG0_SHDN_STAT              ((volatile  uint32_t *) REG_PMG0_SHDN_STAT)              /*  SHUTDOWN Status Register */
#define pREG_PMG0_SRAMRET                ((volatile  uint32_t *) REG_PMG0_SRAMRET)                /*  Control for Retention SRAM during HIBERNATE Mode */
#define pREG_PMG0_RST_STAT               ((volatile  uint32_t *) REG_PMG0_RST_STAT)               /*  Reset status */
#define pREG_PMG0_CTL1                   ((volatile  uint32_t *) REG_PMG0_CTL1)                   /*  HPBUCK control */

/*  =================================================================================
 *      External interrupt configuration (XINT0)
 *  ================================================================================= */
#define pREG_XINT0_CFG0                  ((volatile  uint32_t *) REG_XINT0_CFG0)                  /*  External Interrupt configuration 0 */
#define pREG_XINT0_CLR                   ((volatile  uint32_t *) REG_XINT0_CLR)                   /*  External Interrupt clear */
#define pREG_XINT0_NMICLR                ((volatile  uint32_t *) REG_XINT0_NMICLR)                /*  Non-maskable interrupt clear */

/*  =================================================================================
 *      Clocking  (CLKG0_OSC)
 *  ================================================================================= */
#define pREG_CLKG0_OSC_KEY               ((volatile  uint32_t *) REG_CLKG0_OSC_KEY)               /*  Key Protection for OSCCTRL */
#define pREG_CLKG0_OSC_CTL               ((volatile  uint32_t *) REG_CLKG0_OSC_CTL)               /*  Oscillator Control */

/*  =================================================================================
 *      Power Management  (PMG0_TST)
 *  ================================================================================= */
#define pREG_PMG0_TST_SRAM_CTL           ((volatile  uint32_t *) REG_PMG0_TST_SRAM_CTL)           /*  Control for SRAM parity and instruction SRAM */
#define pREG_PMG0_TST_SRAM_INITSTAT      ((volatile  uint32_t *) REG_PMG0_TST_SRAM_INITSTAT)      /*  Initialization Status Register */
#define pREG_PMG0_TST_CLR_LATCH_GPIOS    ((volatile  uint16_t *) REG_PMG0_TST_CLR_LATCH_GPIOS)    /*  CLEAR GPIO AFTER SHUTDOWN MODE */
#define pREG_PMG0_TST_SCRPAD_IMG         ((volatile  uint32_t *) REG_PMG0_TST_SCRPAD_IMG)         /*  SCRATCH PAD IMAGE */
#define pREG_PMG0_TST_SCRPAD_3V_RD       ((volatile  uint32_t *) REG_PMG0_TST_SCRPAD_3V_RD)       /*  SCRATCH PAD SAVED IN BATTERY DOMAIN */

/*  =================================================================================
 *      Clocking  (CLKG0_CLK)
 *  ================================================================================= */
#define pREG_CLKG0_CLK_CTL0              ((volatile  uint32_t *) REG_CLKG0_CLK_CTL0)              /*  Misc clock settings */
#define pREG_CLKG0_CLK_CTL1              ((volatile  uint32_t *) REG_CLKG0_CLK_CTL1)              /*  Clock dividers */
#define pREG_CLKG0_CLK_CTL3              ((volatile  uint32_t *) REG_CLKG0_CLK_CTL3)              /*  System PLL */
#define pREG_CLKG0_CLK_CTL5              ((volatile  uint32_t *) REG_CLKG0_CLK_CTL5)              /*  User clock gating control */
#define pREG_CLKG0_CLK_STAT0             ((volatile  uint32_t *) REG_CLKG0_CLK_STAT0)             /*  Clocking status */

/*  =================================================================================
 *      Bus matrix (BUSM0)
 *  ================================================================================= */
#define pREG_BUSM0_ARBIT0                ((volatile  uint32_t *) REG_BUSM0_ARBIT0)                /*  Arbitration Priority Configuration for FLASH and SRAM0 */
#define pREG_BUSM0_ARBIT1                ((volatile  uint32_t *) REG_BUSM0_ARBIT1)                /*  Arbitration Priority Configuration for SRAM1 and SIP */
#define pREG_BUSM0_ARBIT2                ((volatile  uint32_t *) REG_BUSM0_ARBIT2)                /*  Arbitration Priority Configuration for APB32 and APB16 */
#define pREG_BUSM0_ARBIT3                ((volatile  uint32_t *) REG_BUSM0_ARBIT3)                /*  Arbitration Priority Configuration for APB16 priority for core and for DMA1 */

/*  =================================================================================
 *      Parallel Test Interface (PTI0)
 *  ================================================================================= */
#define pREG_PTI0_RST_ISR_STARTADDR      ( (volatile  int32_t *) REG_PTI0_RST_ISR_STARTADDR)      /*  Reset ISR Start Address */
#define pREG_PTI0_RST_STACK_PTR          ( (volatile  int32_t *) REG_PTI0_RST_STACK_PTR)          /*  Reset Stack Pointer */
#define pREG_PTI0_CTL                    ((volatile  uint32_t *) REG_PTI0_CTL)                    /*  Parallel Test Interface Control Register */

/*  =================================================================================
 *      Cortex-M3 Interrupt Controller (NVIC0)
 *  ================================================================================= */
#define pREG_NVIC0_INTNUM                ((volatile  uint32_t *) REG_NVIC0_INTNUM)                /*  Interrupt Control Type */
#define pREG_NVIC0_STKSTA                ((volatile  uint32_t *) REG_NVIC0_STKSTA)                /*  Systick Control and Status */
#define pREG_NVIC0_STKLD                 ((volatile  uint32_t *) REG_NVIC0_STKLD)                 /*  Systick Reload Value */
#define pREG_NVIC0_STKVAL                ((volatile  uint32_t *) REG_NVIC0_STKVAL)                /*  Systick Current Value */
#define pREG_NVIC0_STKCAL                ((volatile  uint32_t *) REG_NVIC0_STKCAL)                /*  Systick Calibration Value */
#define pREG_NVIC0_INTSETE0              ((volatile  uint32_t *) REG_NVIC0_INTSETE0)              /*  IRQ0..31 Set_Enable */
#define pREG_NVIC0_INTSETE1              ((volatile  uint32_t *) REG_NVIC0_INTSETE1)              /*  IRQ32..63 Set_Enable */
#define pREG_NVIC0_INTCLRE0              ((volatile  uint32_t *) REG_NVIC0_INTCLRE0)              /*  IRQ0..31 Clear_Enable */
#define pREG_NVIC0_INTCLRE1              ((volatile  uint32_t *) REG_NVIC0_INTCLRE1)              /*  IRQ32..63 Clear_Enable */
#define pREG_NVIC0_INTSETP0              ((volatile  uint32_t *) REG_NVIC0_INTSETP0)              /*  IRQ0..31 Set_Pending */
#define pREG_NVIC0_INTSETP1              ((volatile  uint32_t *) REG_NVIC0_INTSETP1)              /*  IRQ32..63 Set_Pending */
#define pREG_NVIC0_INTCLRP0              ((volatile  uint32_t *) REG_NVIC0_INTCLRP0)              /*  IRQ0..31 Clear_Pending */
#define pREG_NVIC0_INTCLRP1              ((volatile  uint32_t *) REG_NVIC0_INTCLRP1)              /*  IRQ32..63 Clear_Pending */
#define pREG_NVIC0_INTACT0               ((volatile  uint32_t *) REG_NVIC0_INTACT0)               /*  IRQ0..31 Active Bit */
#define pREG_NVIC0_INTACT1               ((volatile  uint32_t *) REG_NVIC0_INTACT1)               /*  IRQ32..63 Active Bit */
#define pREG_NVIC0_INTPRI0               ((volatile  uint32_t *) REG_NVIC0_INTPRI0)               /*  IRQ0..3 Priority */
#define pREG_NVIC0_INTPRI1               ((volatile  uint32_t *) REG_NVIC0_INTPRI1)               /*  IRQ4..7 Priority */
#define pREG_NVIC0_INTPRI2               ((volatile  uint32_t *) REG_NVIC0_INTPRI2)               /*  IRQ8..11 Priority */
#define pREG_NVIC0_INTPRI3               ((volatile  uint32_t *) REG_NVIC0_INTPRI3)               /*  IRQ12..15 Priority */
#define pREG_NVIC0_INTPRI4               ((volatile  uint32_t *) REG_NVIC0_INTPRI4)               /*  IRQ16..19 Priority */
#define pREG_NVIC0_INTPRI5               ((volatile  uint32_t *) REG_NVIC0_INTPRI5)               /*  IRQ20..23 Priority */
#define pREG_NVIC0_INTPRI6               ((volatile  uint32_t *) REG_NVIC0_INTPRI6)               /*  IRQ24..27 Priority */
#define pREG_NVIC0_INTPRI7               ((volatile  uint32_t *) REG_NVIC0_INTPRI7)               /*  IRQ28..31 Priority */
#define pREG_NVIC0_INTPRI8               ((volatile  uint32_t *) REG_NVIC0_INTPRI8)               /*  IRQ32..35 Priority */
#define pREG_NVIC0_INTPRI9               ((volatile  uint32_t *) REG_NVIC0_INTPRI9)               /*  IRQ36..39 Priority */
#define pREG_NVIC0_INTPRI10              ((volatile  uint32_t *) REG_NVIC0_INTPRI10)              /*  IRQ40..43 Priority */
#define pREG_NVIC0_INTCPID               ((volatile  uint32_t *) REG_NVIC0_INTCPID)               /*  CPUID Base */
#define pREG_NVIC0_INTSTA                ((volatile  uint32_t *) REG_NVIC0_INTSTA)                /*  Interrupt Control State */
#define pREG_NVIC0_INTVEC                ((volatile  uint32_t *) REG_NVIC0_INTVEC)                /*  Vector Table Offset */
#define pREG_NVIC0_INTAIRC               ((volatile  uint32_t *) REG_NVIC0_INTAIRC)               /*  Application Interrupt/Reset Control */
#define pREG_NVIC0_INTCON0               ((volatile  uint16_t *) REG_NVIC0_INTCON0)               /*  System Control */
#define pREG_NVIC0_INTCON1               ((volatile  uint32_t *) REG_NVIC0_INTCON1)               /*  Configuration Control */
#define pREG_NVIC0_INTSHPRIO0            ((volatile  uint32_t *) REG_NVIC0_INTSHPRIO0)            /*  System Handlers 4-7 Priority */
#define pREG_NVIC0_INTSHPRIO1            ((volatile  uint32_t *) REG_NVIC0_INTSHPRIO1)            /*  System Handlers 8-11 Priority */
#define pREG_NVIC0_INTSHPRIO3            ((volatile  uint32_t *) REG_NVIC0_INTSHPRIO3)            /*  System Handlers 12-15 Priority */
#define pREG_NVIC0_INTSHCSR              ((volatile  uint32_t *) REG_NVIC0_INTSHCSR)              /*  System Handler Control and State */
#define pREG_NVIC0_INTCFSR               ((volatile  uint32_t *) REG_NVIC0_INTCFSR)               /*  Configurable Fault Status */
#define pREG_NVIC0_INTHFSR               ((volatile  uint32_t *) REG_NVIC0_INTHFSR)               /*  Hard Fault Status */
#define pREG_NVIC0_INTDFSR               ((volatile  uint32_t *) REG_NVIC0_INTDFSR)               /*  Debug Fault Status */
#define pREG_NVIC0_INTMMAR               ((volatile  uint32_t *) REG_NVIC0_INTMMAR)               /*  Mem Manage Address */
#define pREG_NVIC0_INTBFAR               ((volatile  uint32_t *) REG_NVIC0_INTBFAR)               /*  Bus Fault Address */
#define pREG_NVIC0_INTAFSR               ((volatile  uint32_t *) REG_NVIC0_INTAFSR)               /*  Auxiliary Fault Status */
#define pREG_NVIC0_INTPFR0               ((volatile  uint32_t *) REG_NVIC0_INTPFR0)               /*  Processor Feature Register 0 */
#define pREG_NVIC0_INTPFR1               ((volatile  uint32_t *) REG_NVIC0_INTPFR1)               /*  Processor Feature Register 1 */
#define pREG_NVIC0_INTDFR0               ((volatile  uint32_t *) REG_NVIC0_INTDFR0)               /*  Debug Feature Register 0 */
#define pREG_NVIC0_INTAFR0               ((volatile  uint32_t *) REG_NVIC0_INTAFR0)               /*  Auxiliary Feature Register 0 */
#define pREG_NVIC0_INTMMFR0              ((volatile  uint32_t *) REG_NVIC0_INTMMFR0)              /*  Memory Model Feature Register 0 */
#define pREG_NVIC0_INTMMFR1              ((volatile  uint32_t *) REG_NVIC0_INTMMFR1)              /*  Memory Model Feature Register 1 */
#define pREG_NVIC0_INTMMFR2              ((volatile  uint32_t *) REG_NVIC0_INTMMFR2)              /*  Memory Model Feature Register 2 */
#define pREG_NVIC0_INTMMFR3              ((volatile  uint32_t *) REG_NVIC0_INTMMFR3)              /*  Memory Model Feature Register 3 */
#define pREG_NVIC0_INTISAR0              ((volatile  uint32_t *) REG_NVIC0_INTISAR0)              /*  ISA Feature Register 0 */
#define pREG_NVIC0_INTISAR1              ((volatile  uint32_t *) REG_NVIC0_INTISAR1)              /*  ISA Feature Register 1 */
#define pREG_NVIC0_INTISAR2              ((volatile  uint32_t *) REG_NVIC0_INTISAR2)              /*  ISA Feature Register 2 */
#define pREG_NVIC0_INTISAR3              ((volatile  uint32_t *) REG_NVIC0_INTISAR3)              /*  ISA Feature Register 3 */
#define pREG_NVIC0_INTISAR4              ((volatile  uint32_t *) REG_NVIC0_INTISAR4)              /*  ISA Feature Register 4 */
#define pREG_NVIC0_INTTRGI               ((volatile  uint32_t *) REG_NVIC0_INTTRGI)               /*  Software Trigger Interrupt Register */
#define pREG_NVIC0_INTPID4               ((volatile  uint32_t *) REG_NVIC0_INTPID4)               /*  Peripheral Identification Register 4 */
#define pREG_NVIC0_INTPID5               ((volatile  uint32_t *) REG_NVIC0_INTPID5)               /*  Peripheral Identification Register 5 */
#define pREG_NVIC0_INTPID6               ((volatile  uint32_t *) REG_NVIC0_INTPID6)               /*  Peripheral Identification Register 6 */
#define pREG_NVIC0_INTPID7               ((volatile  uint32_t *) REG_NVIC0_INTPID7)               /*  Peripheral Identification Register 7 */
#define pREG_NVIC0_INTPID0               ((volatile  uint32_t *) REG_NVIC0_INTPID0)               /*  Peripheral Identification Bits7:0 */
#define pREG_NVIC0_INTPID1               ((volatile  uint32_t *) REG_NVIC0_INTPID1)               /*  Peripheral Identification Bits15:8 */
#define pREG_NVIC0_INTPID2               ((volatile  uint32_t *) REG_NVIC0_INTPID2)               /*  Peripheral Identification Bits16:23 */
#define pREG_NVIC0_INTPID3               ((volatile  uint32_t *) REG_NVIC0_INTPID3)               /*  Peripheral Identification Bits24:31 */
#define pREG_NVIC0_INTCID0               ((volatile  uint32_t *) REG_NVIC0_INTCID0)               /*  Component Identification Bits7:0 */
#define pREG_NVIC0_INTCID1               ((volatile  uint32_t *) REG_NVIC0_INTCID1)               /*  Component Identification Bits15:8 */
#define pREG_NVIC0_INTCID2               ((volatile  uint32_t *) REG_NVIC0_INTCID2)               /*  Component Identification Bits16:23 */
#define pREG_NVIC0_INTCID3               ((volatile  uint32_t *) REG_NVIC0_INTCID3)               /*  Component Identification Bits24:31 */

#endif

