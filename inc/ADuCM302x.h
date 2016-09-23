/* ================================================================================

     Project      :   ADuCM302x
     File         :   ADuCM302x.h
     Description  :   Register Definitions

     Date         :   Oct 21, 2014

     Copyright (c) Analog Devices, Inc.  All Rights Reserved.
     This software is proprietary and confidential to Analog Devices, Inc. and
     its licensors.

     This file was auto-generated. Do not make local changes to this file.

   ================================================================================ */

#ifndef _DEF_ADUCM302X_H
#define _DEF_ADUCM302X_H

#if defined (_MISRA_RULES)
#pragma diag(push)
#pragma diag(suppress:misra_rule_19_7:"ADI header allows function-like macros")
#pragma diag(suppress:misra_rule_19_13:"ADI headers can use the # and ## preprocessor operators")
#endif /* _MISRA_RULES */

/* do not add casts to literal constants in assembly code */
#if defined(_LANGUAGE_ASM) || defined(__ASSEMBLER__)
#define _ADI_MSK( mask, type ) (mask) /* Make a bitmask */
#else
#define _ADI_MSK( mask, type ) ((type)(mask)) /* Make a bitmask */
#endif

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

#ifndef __ADI_GENERATED_DEF_HEADERS__
#define __ADI_GENERATED_DEF_HEADERS__    1
#endif

/* MMR modules defined for the ADuCM302x */

#define __ADI_HAS_BEEP__           1
#define __ADI_HAS_BUSM__           1
#define __ADI_HAS_CLKG_OSC__       1
#define __ADI_HAS_CLKG__           1
#define __ADI_HAS_CLKG_CLK__       1
#define __ADI_HAS_FLCC__           1
#define __ADI_HAS_XINT__           1
#define __ADI_HAS_FLCC_CACHE__     1
#define __ADI_HAS_FLCC_DFT__       1
#define __ADI_HAS_FLCC_TEST__      1
#define __ADI_HAS_GPIO__           1
#define __ADI_HAS_TMR__            1
#define __ADI_HAS_NVIC__           1
#define __ADI_HAS_PMG__            1
#define __ADI_HAS_PMG_TST__        1
#define __ADI_HAS_PTI__            1
#define __ADI_HAS_RNG__            1
#define __ADI_HAS_RTC__            1
#define __ADI_HAS_SPI__            1
#define __ADI_HAS_SPORT__          1
#define __ADI_HAS_SYS__            1
#define __ADI_HAS_WDT__            1
#define __ADI_HAS_CRC__            1
#define __ADI_HAS_CRYPT__          1
#define __ADI_HAS_DMA__            1
#define __ADI_HAS_I2C__            1
#define __ADI_HAS_UART__           1

/* ============================================================================================================================
        General Purpose Timer
   ============================================================================================================================ */

/* ============================================================================================================================
        TMR0
   ============================================================================================================================ */
#define REG_TMR0_LOAD                        0x40000000            /*  TMR0 16-bit load value */
#define REG_TMR0_CURCNT                      0x40000004            /*  TMR0 16-bit timer value */
#define REG_TMR0_CTL                         0x40000008            /*  TMR0 Control */
#define REG_TMR0_CLRINT                      0x4000000C            /*  TMR0 Clear Interrupt */
#define REG_TMR0_CAPTURE                     0x40000010            /*  TMR0 Capture */
#define REG_TMR0_ALOAD                       0x40000014            /*  TMR0 16-bit load value, asynchronous */
#define REG_TMR0_ACURCNT                     0x40000018            /*  TMR0 16-bit timer value, asynchronous */
#define REG_TMR0_STAT                        0x4000001C            /*  TMR0 Status */
#define REG_TMR0_PWMCTL                      0x40000020            /*  TMR0 PWM Control Register */
#define REG_TMR0_PWMMATCH                    0x40000024            /*  TMR0 PWM Match Value */

/* ============================================================================================================================
        TMR1
   ============================================================================================================================ */
#define REG_TMR1_LOAD                        0x40000400            /*  TMR1 16-bit load value */
#define REG_TMR1_CURCNT                      0x40000404            /*  TMR1 16-bit timer value */
#define REG_TMR1_CTL                         0x40000408            /*  TMR1 Control */
#define REG_TMR1_CLRINT                      0x4000040C            /*  TMR1 Clear Interrupt */
#define REG_TMR1_CAPTURE                     0x40000410            /*  TMR1 Capture */
#define REG_TMR1_ALOAD                       0x40000414            /*  TMR1 16-bit load value, asynchronous */
#define REG_TMR1_ACURCNT                     0x40000418            /*  TMR1 16-bit timer value, asynchronous */
#define REG_TMR1_STAT                        0x4000041C            /*  TMR1 Status */
#define REG_TMR1_PWMCTL                      0x40000420            /*  TMR1 PWM Control Register */
#define REG_TMR1_PWMMATCH                    0x40000424            /*  TMR1 PWM Match Value */

/* ============================================================================================================================
        TMR2
   ============================================================================================================================ */
#define REG_TMR2_LOAD                        0x40000800            /*  TMR2 16-bit load value */
#define REG_TMR2_CURCNT                      0x40000804            /*  TMR2 16-bit timer value */
#define REG_TMR2_CTL                         0x40000808            /*  TMR2 Control */
#define REG_TMR2_CLRINT                      0x4000080C            /*  TMR2 Clear Interrupt */
#define REG_TMR2_CAPTURE                     0x40000810            /*  TMR2 Capture */
#define REG_TMR2_ALOAD                       0x40000814            /*  TMR2 16-bit load value, asynchronous */
#define REG_TMR2_ACURCNT                     0x40000818            /*  TMR2 16-bit timer value, asynchronous */
#define REG_TMR2_STAT                        0x4000081C            /*  TMR2 Status */
#define REG_TMR2_PWMCTL                      0x40000820            /*  TMR2 PWM Control Register */
#define REG_TMR2_PWMMATCH                    0x40000824            /*  TMR2 PWM Match Value */

/* ============================================================================================================================
        TMR Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          TMR_CTL                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_TMR_CTL_SYNCBYP                 15            /*  Synchronization bypass */
#define BITP_TMR_CTL_RSTEN                   14            /*  Counter and prescale reset enable */
#define BITP_TMR_CTL_EVTEN                   13            /*  Event select */
#define BITP_TMR_CTL_EVTRANGE                 8            /*  Event select range */
#define BITP_TMR_CTL_RLD                      7            /*  Reload control */
#define BITP_TMR_CTL_CLK                      5            /*  Clock select */
#define BITP_TMR_CTL_EN                       4            /*  Timer enable */
#define BITP_TMR_CTL_MODE                     3            /*  Timer mode */
#define BITP_TMR_CTL_UP                       2            /*  Count up */
#define BITP_TMR_CTL_PRE                      0            /*  Prescaler */
#define BITM_TMR_CTL_SYNCBYP                 (_ADI_MSK(0x00008000,uint16_t))    /*  Synchronization bypass */
#define BITM_TMR_CTL_RSTEN                   (_ADI_MSK(0x00004000,uint16_t))    /*  Counter and prescale reset enable */

#define BITM_TMR_CTL_EVTEN                   (_ADI_MSK(0x00002000,uint16_t))    /*  Event select */
#define BITM_TMR_CTL_EVTRANGE                (_ADI_MSK(0x00001F00,uint16_t))    /*  Event select range */

#define BITM_TMR_CTL_RLD                     (_ADI_MSK(0x00000080,uint16_t))    /*  Reload control */

#define BITM_TMR_CTL_CLK                     (_ADI_MSK(0x00000060,uint16_t))    /*  Clock select */

#define BITM_TMR_CTL_EN                      (_ADI_MSK(0x00000010,uint16_t))    /*  Timer enable */

#define BITM_TMR_CTL_MODE                    (_ADI_MSK(0x00000008,uint16_t))    /*  Timer mode */

#define BITM_TMR_CTL_UP                      (_ADI_MSK(0x00000004,uint16_t))    /*  Count up */

#define BITM_TMR_CTL_PRE                     (_ADI_MSK(0x00000003,uint16_t))    /*  Prescaler */

/* -------------------------------------------------------------------------------------------------------------------------
          TMR_CLRINT                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_TMR_CLRINT_EVTCAPT               1            /*  Clear captured event interrupt */
#define BITP_TMR_CLRINT_TIMEOUT               0            /*  Clear timeout interrupt */

#define BITM_TMR_CLRINT_EVTCAPT              (_ADI_MSK(0x00000002,uint16_t))    /*  Clear captured event interrupt */

#define BITM_TMR_CLRINT_TIMEOUT              (_ADI_MSK(0x00000001,uint16_t))    /*  Clear timeout interrupt */

/* -------------------------------------------------------------------------------------------------------------------------
          TMR_STAT                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_TMR_STAT_CNTRST                  8            /*  Counter reset occurring */
#define BITP_TMR_STAT_PDOK                    7            /*  Clear Interrupt Register synchronization */
#define BITP_TMR_STAT_BUSY                    6            /*  Timer Busy */
#define BITP_TMR_STAT_CAPTURE                 1            /*  Capture event pending */
#define BITP_TMR_STAT_TIMEOUT                 0            /*  Timeout event occurred */
#define BITM_TMR_STAT_CNTRST                 (_ADI_MSK(0x00000100,uint16_t))    /*  Counter reset occurring */

#define BITM_TMR_STAT_PDOK                   (_ADI_MSK(0x00000080,uint16_t))    /*  Clear Interrupt Register synchronization */

#define BITM_TMR_STAT_BUSY                   (_ADI_MSK(0x00000040,uint16_t))    /*  Timer Busy */

#define BITM_TMR_STAT_CAPTURE                (_ADI_MSK(0x00000002,uint16_t))    /*  Capture event pending */

#define BITM_TMR_STAT_TIMEOUT                (_ADI_MSK(0x00000001,uint16_t))    /*  Timeout event occurred */

/* -------------------------------------------------------------------------------------------------------------------------
          TMR_PWMCTL                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_TMR_PWMCTL_IDLESTATE             1            /*  PWM Idle State */
#define BITP_TMR_PWMCTL_MATCH                 0            /*  PWM Match enabled */
#define BITM_TMR_PWMCTL_IDLESTATE            (_ADI_MSK(0x00000002,uint16_t))    /*  PWM Idle State */
#define BITM_TMR_PWMCTL_MATCH                (_ADI_MSK(0x00000001,uint16_t))    /*  PWM Match enabled */


/* ============================================================================================================================
        Real-Time Clock
   ============================================================================================================================ */

/* ============================================================================================================================
        RTC0
   ============================================================================================================================ */
#define REG_RTC0_CR0                         0x40001000            /*  RTC0 RTC Control 0 */
#define REG_RTC0_SR0                         0x40001004            /*  RTC0 RTC Status 0 */
#define REG_RTC0_SR1                         0x40001008            /*  RTC0 RTC Status 1 */
#define REG_RTC0_CNT0                        0x4000100C            /*  RTC0 RTC Count 0 */
#define REG_RTC0_CNT1                        0x40001010            /*  RTC0 RTC Count 1 */
#define REG_RTC0_ALM0                        0x40001014            /*  RTC0 RTC Alarm 0 */
#define REG_RTC0_ALM1                        0x40001018            /*  RTC0 RTC Alarm 1 */
#define REG_RTC0_TRM                         0x4000101C            /*  RTC0 RTC Trim */
#define REG_RTC0_GWY                         0x40001020            /*  RTC0 RTC Gateway */
#define REG_RTC0_CR1                         0x40001028            /*  RTC0 RTC Control 1 */
#define REG_RTC0_SR2                         0x4000102C            /*  RTC0 RTC Status 2 */
#define REG_RTC0_SNAP0                       0x40001030            /*  RTC0 RTC Snapshot 0 */
#define REG_RTC0_SNAP1                       0x40001034            /*  RTC0 RTC Snapshot 1 */
#define REG_RTC0_SNAP2                       0x40001038            /*  RTC0 RTC Snapshot 2 */
#define REG_RTC0_MOD                         0x4000103C            /*  RTC0 RTC Modulo */
#define REG_RTC0_CNT2                        0x40001040            /*  RTC0 RTC Count 2 */
#define REG_RTC0_ALM2                        0x40001044            /*  RTC0 RTC Alarm 2 */

/* ============================================================================================================================
        RTC1
   ============================================================================================================================ */
#define REG_RTC1_CR0                         0x40001400            /*  RTC1 RTC Control 0 */
#define REG_RTC1_SR0                         0x40001404            /*  RTC1 RTC Status 0 */
#define REG_RTC1_SR1                         0x40001408            /*  RTC1 RTC Status 1 */
#define REG_RTC1_CNT0                        0x4000140C            /*  RTC1 RTC Count 0 */
#define REG_RTC1_CNT1                        0x40001410            /*  RTC1 RTC Count 1 */
#define REG_RTC1_ALM0                        0x40001414            /*  RTC1 RTC Alarm 0 */
#define REG_RTC1_ALM1                        0x40001418            /*  RTC1 RTC Alarm 1 */
#define REG_RTC1_TRM                         0x4000141C            /*  RTC1 RTC Trim */
#define REG_RTC1_GWY                         0x40001420            /*  RTC1 RTC Gateway */
#define REG_RTC1_CR1                         0x40001428            /*  RTC1 RTC Control 1 */
#define REG_RTC1_SR2                         0x4000142C            /*  RTC1 RTC Status 2 */
#define REG_RTC1_SNAP0                       0x40001430            /*  RTC1 RTC Snapshot 0 */
#define REG_RTC1_SNAP1                       0x40001434            /*  RTC1 RTC Snapshot 1 */
#define REG_RTC1_SNAP2                       0x40001438            /*  RTC1 RTC Snapshot 2 */
#define REG_RTC1_MOD                         0x4000143C            /*  RTC1 RTC Modulo */
#define REG_RTC1_CNT2                        0x40001440            /*  RTC1 RTC Count 2 */
#define REG_RTC1_ALM2                        0x40001444            /*  RTC1 RTC Alarm 2 */

/* ============================================================================================================================
        RTC Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          RTC_CR0                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_CR0_WPNDINTEN               15            /*  Enable WPENDINT-sourced interrupts to the CPU */
#define BITP_RTC_CR0_WSYNCINTEN              14            /*  Enable WSYNCINT-sourced interrupts to the CPU */
#define BITP_RTC_CR0_WPNDERRINTEN            13            /*  Enable WPENDERRINT-sourced interrupts to the CPU when an RTC register-write pending error occurs */
#define BITP_RTC_CR0_ISOINTEN                12            /*  Enable ISOINT-sourced interrupts to the CPU when isolation of the RTC power domain is activated and subsequently de-activated */
#define BITP_RTC_CR0_MOD60ALMINTEN           11            /*  Enable periodic MOD60ALMINT-sourced interrupts to the CPU */
#define BITP_RTC_CR0_MOD60ALM                 5            /*  Periodic, modulo-60 alarm time in prescaled RTC time units beyond a modulo-60 boundary */
#define BITP_RTC_CR0_MOD60ALMEN               4            /*  Enable RTC modulo-60 counting of time past a modulo-60 boundary */
#define BITP_RTC_CR0_TRMEN                    3            /*  Enable RTC digital trimming */
#define BITP_RTC_CR0_ALMINTEN                 2            /*  Enable ALMINT-sourced alarm interrupts to the CPU */
#define BITP_RTC_CR0_ALMEN                    1            /*  Enable the RTC alarm (absolute) operation */
#define BITP_RTC_CR0_CNTEN                    0            /*  Global enable for the RTC */

#define BITM_RTC_CR0_WPNDINTEN               (_ADI_MSK(0x00008000,uint32_t))    /*  Enable WPENDINT-sourced interrupts to the CPU */

#define BITM_RTC_CR0_WSYNCINTEN              (_ADI_MSK(0x00004000,uint32_t))    /*  Enable WSYNCINT-sourced interrupts to the CPU */

#define BITM_RTC_CR0_WPNDERRINTEN            (_ADI_MSK(0x00002000,uint32_t))    /*  Enable WPENDERRINT-sourced interrupts to the CPU when an RTC register-write pending error occurs */

#define BITM_RTC_CR0_ISOINTEN                (_ADI_MSK(0x00001000,uint32_t))    /*  Enable ISOINT-sourced interrupts to the CPU when isolation of the RTC power domain is activated and subsequently de-activated */

#define BITM_RTC_CR0_MOD60ALMINTEN           (_ADI_MSK(0x00000800,uint32_t))    /*  Enable periodic MOD60ALMINT-sourced interrupts to the CPU */

#define BITM_RTC_CR0_MOD60ALM                (_ADI_MSK(0x000007E0,uint32_t))    /*  Periodic, modulo-60 alarm time in prescaled RTC time units beyond a modulo-60 boundary */

#define BITM_RTC_CR0_MOD60ALMEN              (_ADI_MSK(0x00000010,uint32_t))    /*  Enable RTC modulo-60 counting of time past a modulo-60 boundary */

#define BITM_RTC_CR0_TRMEN                   (_ADI_MSK(0x00000008,uint32_t))    /*  Enable RTC digital trimming */

#define BITM_RTC_CR0_ALMINTEN                (_ADI_MSK(0x00000004,uint32_t))    /*  Enable ALMINT-sourced alarm interrupts to the CPU */

#define BITM_RTC_CR0_ALMEN                   (_ADI_MSK(0x00000002,uint32_t))    /*  Enable the RTC alarm (absolute) operation */

#define BITM_RTC_CR0_CNTEN                   (_ADI_MSK(0x00000001,uint32_t))    /*  Global enable for the RTC */
#define ENUM_RTC_CR0_EXAMPLE_1_THIRTY_TIME_UNITS_DECIMAL (_ADI_MSK(0x000003C0,uint32_t))    /*  MOD60ALM: Example of setting a modulo-60 periodic interrupt from the RTC to be issued to the CPU at 30 time units past a modulo-60 boundary. */
#define ENUM_RTC_CR0_EXAMPLE_2_FIFTYFIVE_TIME_UNITS_DECIMAL (_ADI_MSK(0x000006E0,uint32_t))    /*  MOD60ALM: Example of setting a modulo-60 periodic interrupt from the RTC to be issued to the CPU at 55 time units past a modulo-60 boundary. */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_SR0                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_SR0_WSYNCALM2               17            /*  Synchronisation status of posted writes to RTCALM2 */
#define BITP_RTC_SR0_WSYNCCR1                16            /*  Synchronisation status of posted writes to RTCCR1 */
#define BITP_RTC_SR0_LOCK32KHZ               15            /*  Lock status of the 32kHz crystal oscillator output clock */
#define BITP_RTC_SR0_ISOENB                  14            /*  Master enable for isolation of the always-on RTC power domain */
#define BITP_RTC_SR0_WSYNCTRM                13            /*  Synchronisation status of posted writes to RTCTRM */
#define BITP_RTC_SR0_WSYNCALM1               12            /*  Synchronisation status of posted writes to RTCALM1 */
#define BITP_RTC_SR0_WSYNCALM0               11            /*  Synchronisation status of posted writes to RTCALM0 */
#define BITP_RTC_SR0_WSYNCCNT1               10            /*  Synchronisation status of posted writes to RTCCNT1 */
#define BITP_RTC_SR0_WSYNCCNT0                9            /*  Synchronisation status of posted writes to RTCCNT0 */
#define BITP_RTC_SR0_WSYNCSR0                 8            /*  Synchronisation status of posted clearances to interrupt sources in RTCSR0 */
#define BITP_RTC_SR0_WSYNCCR0                 7            /*  Synchronisation status of posted writes to RTCCR0 */
#define BITP_RTC_SR0_WPNDINT                  6            /*  Write pending interrupt */
#define BITP_RTC_SR0_WSYNCINT                 5            /*  Write synchronisation interrupt */
#define BITP_RTC_SR0_WPNDERRINT               4            /*  Write pending error interrupt source */
#define BITP_RTC_SR0_ISOINT                   3            /*  RTC power-domain isolation interrupt source */
#define BITP_RTC_SR0_MOD60ALMINT              2            /*  Modulo-60 RTC alarm interrupt source */
#define BITP_RTC_SR0_ALMINT                   1            /*  Alarm interrupt source */
#define BITP_RTC_SR0_FAIL                     0            /*  RTC failure interrupt source */

#define BITM_RTC_SR0_WSYNCALM2               (_ADI_MSK(0x00020000,uint32_t))    /*  Synchronisation status of posted writes to RTCALM2 */

#define BITM_RTC_SR0_WSYNCCR1                (_ADI_MSK(0x00010000,uint32_t))    /*  Synchronisation status of posted writes to RTCCR1 */

#define BITM_RTC_SR0_LOCK32KHZ               (_ADI_MSK(0x00008000,uint32_t))    /*  Lock status of the 32kHz crystal oscillator output clock */

#define BITM_RTC_SR0_ISOENB                  (_ADI_MSK(0x00004000,uint32_t))    /*  Master enable for isolation of the always-on RTC power domain */

#define BITM_RTC_SR0_WSYNCTRM                (_ADI_MSK(0x00002000,uint32_t))    /*  Synchronisation status of posted writes to RTCTRM */

#define BITM_RTC_SR0_WSYNCALM1               (_ADI_MSK(0x00001000,uint32_t))    /*  Synchronisation status of posted writes to RTCALM1 */

#define BITM_RTC_SR0_WSYNCALM0               (_ADI_MSK(0x00000800,uint32_t))    /*  Synchronisation status of posted writes to RTCALM0 */

#define BITM_RTC_SR0_WSYNCCNT1               (_ADI_MSK(0x00000400,uint32_t))    /*  Synchronisation status of posted writes to RTCCNT1 */

#define BITM_RTC_SR0_WSYNCCNT0               (_ADI_MSK(0x00000200,uint32_t))    /*  Synchronisation status of posted writes to RTCCNT0 */

#define BITM_RTC_SR0_WSYNCSR0                (_ADI_MSK(0x00000100,uint32_t))    /*  Synchronisation status of posted clearances to interrupt sources in RTCSR0 */

#define BITM_RTC_SR0_WSYNCCR0                (_ADI_MSK(0x00000080,uint32_t))    /*  Synchronisation status of posted writes to RTCCR0 */

#define BITM_RTC_SR0_WPNDINT                 (_ADI_MSK(0x00000040,uint32_t))    /*  Write pending interrupt */

#define BITM_RTC_SR0_WSYNCINT                (_ADI_MSK(0x00000020,uint32_t))    /*  Write synchronisation interrupt */

#define BITM_RTC_SR0_WPNDERRINT              (_ADI_MSK(0x00000010,uint32_t))    /*  Write pending error interrupt source */

#define BITM_RTC_SR0_ISOINT                  (_ADI_MSK(0x00000008,uint32_t))    /*  RTC power-domain isolation interrupt source */

#define BITM_RTC_SR0_MOD60ALMINT             (_ADI_MSK(0x00000004,uint32_t))    /*  Modulo-60 RTC alarm interrupt source */

#define BITM_RTC_SR0_ALMINT                  (_ADI_MSK(0x00000002,uint32_t))    /*  Alarm interrupt source */

#define BITM_RTC_SR0_FAIL                    (_ADI_MSK(0x00000001,uint32_t))    /*  RTC failure interrupt source */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_SR1                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_SR1_WPNDALM2                17            /*  Pending status of posted writes to RTCALM2 */
#define BITP_RTC_SR1_WPNDCR1                 16            /*  Pending status of posted writes to RTCCR1 */
#define BITP_RTC_SR1_DBGWDOG                 15            /*  Indicator that the transaction lock watchdog is currently active */
#define BITP_RTC_SR1_DISPRSCLMIR             14            /*  Mirror of the DISPRESCALE field of RTCCR0 */
#define BITP_RTC_SR1_WPNDTRM                 13            /*  Pending status of posted writes to RTCTRM */
#define BITP_RTC_SR1_WPNDALM1                12            /*  Pending status of posted writes to RTCALM1 */
#define BITP_RTC_SR1_WPNDALM0                11            /*  Pending status of posted writes to RTCALM0 */
#define BITP_RTC_SR1_WPNDCNT1                10            /*  Pending status of posted writes to RTCCNT1 */
#define BITP_RTC_SR1_WPNDCNT0                 9            /*  Pending status of posted writes to RTCCNT0 */
#define BITP_RTC_SR1_WPNDSR0                  8            /*  Pending status of posted clearances of interrupt sources in RTCSR0 */
#define BITP_RTC_SR1_WPNDCR0                  7            /*  Pending status of posted writes to RTCCR0 */
#define BITP_RTC_SR1_WERRCODE                 3            /*  Identifier for the source of a write synchronisation error */
#define BITP_RTC_SR1_DBGPTR                   0            /*  Debug address pointer */

#define BITM_RTC_SR1_WPNDALM2                (_ADI_MSK(0x00020000,uint32_t))    /*  Pending status of posted writes to RTCALM2 */

#define BITM_RTC_SR1_WPNDCR1                 (_ADI_MSK(0x00010000,uint32_t))    /*  Pending status of posted writes to RTCCR1 */

#define BITM_RTC_SR1_DBGWDOG                 (_ADI_MSK(0x00008000,uint32_t))    /*  Indicator that the transaction lock watchdog is currently active */
#define BITM_RTC_SR1_DISPRSCLMIR             (_ADI_MSK(0x00004000,uint32_t))    /*  Mirror of the DISPRESCALE field of RTCCR0 */

#define BITM_RTC_SR1_WPNDTRM                 (_ADI_MSK(0x00002000,uint32_t))    /*  Pending status of posted writes to RTCTRM */

#define BITM_RTC_SR1_WPNDALM1                (_ADI_MSK(0x00001000,uint32_t))    /*  Pending status of posted writes to RTCALM1 */

#define BITM_RTC_SR1_WPNDALM0                (_ADI_MSK(0x00000800,uint32_t))    /*  Pending status of posted writes to RTCALM0 */

#define BITM_RTC_SR1_WPNDCNT1                (_ADI_MSK(0x00000400,uint32_t))    /*  Pending status of posted writes to RTCCNT1 */

#define BITM_RTC_SR1_WPNDCNT0                (_ADI_MSK(0x00000200,uint32_t))    /*  Pending status of posted writes to RTCCNT0 */

#define BITM_RTC_SR1_WPNDSR0                 (_ADI_MSK(0x00000100,uint32_t))    /*  Pending status of posted clearances of interrupt sources in RTCSR0 */

#define BITM_RTC_SR1_WPNDCR0                 (_ADI_MSK(0x00000080,uint32_t))    /*  Pending status of posted writes to RTCCR0 */

#define BITM_RTC_SR1_WERRCODE                (_ADI_MSK(0x00000078,uint32_t))    /*  Identifier for the source of a write synchronisation error */
#define BITM_RTC_SR1_DBGPTR                  (_ADI_MSK(0x00000007,uint32_t))    /*  Debug address pointer */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_CNT0                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_CNT0_CNT1MIR                16            /*  Mirror of RTCCNT1. */
#define BITP_RTC_CNT0_VALUE                   0            /*  Lower 16 prescaled (non-fractional) bits of the RTC real-time count. */
#define BITM_RTC_CNT0_CNT1MIR                (_ADI_MSK(0xFFFF0000,uint32_t))    /*  Mirror of RTCCNT1. */
#define BITM_RTC_CNT0_VALUE                  (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Lower 16 prescaled (non-fractional) bits of the RTC real-time count. */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_ALM0                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_ALM0_ALM1MIR                16            /*  Mirror of RTCALM1. */
#define BITP_RTC_ALM0_VALUE                   0            /*  Lower 16 prescaled (i.e. non-fractional) bits of the RTC alarm target time. */
#define BITM_RTC_ALM0_ALM1MIR                (_ADI_MSK(0xFFFF0000,uint32_t))    /*  Mirror of RTCALM1. */
#define BITM_RTC_ALM0_VALUE                  (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Lower 16 prescaled (i.e. non-fractional) bits of the RTC alarm target time. */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_TRM                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_TRM_IVL2EXPMIN               6            /*  Minimum power-of-two interval of prescaled RTC time units which TRMIVL can select */
#define BITP_RTC_TRM_IVL                      4            /*  Trim interval in prescaled RTC time units */
#define BITP_RTC_TRM_ADD                      3            /*  Trim polarity */
#define BITP_RTC_TRM_VALUE                    0            /*  Trim value in prescaled RTC time units to be added or subtracted from the RTC count at the end of a periodic interval selected by TRMIVL */
#define BITM_RTC_TRM_IVL2EXPMIN              (_ADI_MSK(0x000003C0,uint16_t))    /*  Minimum power-of-two interval of prescaled RTC time units which TRMIVL can select */

#define BITM_RTC_TRM_IVL                     (_ADI_MSK(0x00000030,uint16_t))    /*  Trim interval in prescaled RTC time units */

#define BITM_RTC_TRM_ADD                     (_ADI_MSK(0x00000008,uint16_t))    /*  Trim polarity */

#define BITM_RTC_TRM_VALUE                   (_ADI_MSK(0x00000007,uint16_t))    /*  Trim value in prescaled RTC time units to be added or subtracted from the RTC count at the end of a periodic interval selected by TRMIVL */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_CR1                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_CR1_PRESCALE2EXP             5            /*  Prescale power of 2 division factor for the RTC base clock */
#define BITP_RTC_CR1_CNTMOD60ROLLINTEN        4            /*  Enable for the RTC modulo-60 count roll-over interrupt source, RTCMOD60ROLLINT in RTCSR2 */
#define BITP_RTC_CR1_CNTROLLINTEN             3            /*  Enable for the RTC count roll-over interrupt source, RTCCNTROLLINT in RTCSR2 */
#define BITP_RTC_CR1_TRMINTEN                 2            /*  Enable for the RTC trim interrupt source, RTCTRMINT in RTCSR2 */
#define BITP_RTC_CR1_PSINTEN                  1            /*  Enable for the prescaled, modulo-1 interrupt source, RTCPSINT in RTCSR2 */
#define BITP_RTC_CR1_CNTINTEN                 0            /*  Enable for the RTC count interrupt source, RTCCNTINT in RTCSR2 */

#define BITM_RTC_CR1_PRESCALE2EXP            (_ADI_MSK(0x000001E0,uint16_t))    /*  Prescale power of 2 division factor for the RTC base clock */

#define BITM_RTC_CR1_CNTMOD60ROLLINTEN       (_ADI_MSK(0x00000010,uint16_t))    /*  Enable for the RTC modulo-60 count roll-over interrupt source, RTCMOD60ROLLINT in RTCSR2 */

#define BITM_RTC_CR1_CNTROLLINTEN            (_ADI_MSK(0x00000008,uint16_t))    /*  Enable for the RTC count roll-over interrupt source, RTCCNTROLLINT in RTCSR2 */

#define BITM_RTC_CR1_TRMINTEN                (_ADI_MSK(0x00000004,uint16_t))    /*  Enable for the RTC trim interrupt source, RTCTRMINT in RTCSR2 */

#define BITM_RTC_CR1_PSINTEN                 (_ADI_MSK(0x00000002,uint16_t))    /*  Enable for the prescaled, modulo-1 interrupt source, RTCPSINT in RTCSR2 */

#define BITM_RTC_CR1_CNTINTEN                (_ADI_MSK(0x00000001,uint16_t))    /*  Enable for the RTC count interrupt source, RTCCNTINT in RTCSR2 */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_SR2                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_SR2_WSYNCALM2MIR            15            /*  Mirror of the WSYNCALM2 field of RTCSR0 */
#define BITP_RTC_SR2_WSYNCCR1MIR             14            /*  Mirror of the WSYNCCR1 field of RTCSR0 */
#define BITP_RTC_SR2_WPNDALM2MIR             13            /*  Mirror of the WPENDALM2 field of RTCSR1 */
#define BITP_RTC_SR2_WPNDCR1MIR              12            /*  Mirror of the WPENDCR1 field of RTCSR1 */
#define BITP_RTC_SR2_TRMBDYMIR                7            /*  Mirror of the RTCTRMBDY field of RTCMOD */
#define BITP_RTC_SR2_CNTMOD60ROLL             6            /*  RTC count modulo-60 roll-over */
#define BITP_RTC_SR2_CNTROLL                  5            /*  RTC count roll-over */
#define BITP_RTC_SR2_CNTMOD60ROLLINT          4            /*  RTC modulo-60 count roll-over interrupt source */
#define BITP_RTC_SR2_CNTROLLINT               3            /*  RTC count roll-over interrupt source */
#define BITP_RTC_SR2_TRMINT                   2            /*  RTC trim interrupt source */
#define BITP_RTC_SR2_PSINT                    1            /*  RTC prescaled, modulo-1 boundary interrupt source */
#define BITP_RTC_SR2_CNTINT                   0            /*  RTC count interrupt source */

#define BITM_RTC_SR2_WSYNCALM2MIR            (_ADI_MSK(0x00008000,uint16_t))    /*  Mirror of the WSYNCALM2 field of RTCSR0 */

#define BITM_RTC_SR2_WSYNCCR1MIR             (_ADI_MSK(0x00004000,uint16_t))    /*  Mirror of the WSYNCCR1 field of RTCSR0 */

#define BITM_RTC_SR2_WPNDALM2MIR             (_ADI_MSK(0x00002000,uint16_t))    /*  Mirror of the WPENDALM2 field of RTCSR1 */

#define BITM_RTC_SR2_WPNDCR1MIR              (_ADI_MSK(0x00001000,uint16_t))    /*  Mirror of the WPENDCR1 field of RTCSR1 */
#define BITM_RTC_SR2_TRMBDYMIR               (_ADI_MSK(0x00000080,uint16_t))    /*  Mirror of the RTCTRMBDY field of RTCMOD */

#define BITM_RTC_SR2_CNTMOD60ROLL            (_ADI_MSK(0x00000040,uint16_t))    /*  RTC count modulo-60 roll-over */

#define BITM_RTC_SR2_CNTROLL                 (_ADI_MSK(0x00000020,uint16_t))    /*  RTC count roll-over */

#define BITM_RTC_SR2_CNTMOD60ROLLINT         (_ADI_MSK(0x00000010,uint16_t))    /*  RTC modulo-60 count roll-over interrupt source */

#define BITM_RTC_SR2_CNTROLLINT              (_ADI_MSK(0x00000008,uint16_t))    /*  RTC count roll-over interrupt source */

#define BITM_RTC_SR2_TRMINT                  (_ADI_MSK(0x00000004,uint16_t))    /*  RTC trim interrupt source */

#define BITM_RTC_SR2_PSINT                   (_ADI_MSK(0x00000002,uint16_t))    /*  RTC prescaled, modulo-1 boundary interrupt source */

#define BITM_RTC_SR2_CNTINT                  (_ADI_MSK(0x00000001,uint16_t))    /*  RTC count interrupt source */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_SNAP0                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_SNAP0_VALUE                  0            /*  Sticky snapshot of RTCCNT0 */
#define BITM_RTC_SNAP0_VALUE                 (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Sticky snapshot of RTCCNT0 */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_SNAP2                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_SNAP2_VALUE                  0            /*  Sticky snapshot of RTCCNT2 */
#define BITM_RTC_SNAP2_VALUE                 (_ADI_MSK(0x00007FFF,uint32_t))    /*  Sticky snapshot of RTCCNT2 */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_MOD                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_MOD_CNT0_4TOZERO            11            /*  Mirror of RTCCNT0[5:0]. */
#define BITP_RTC_MOD_TRMBDY                  10            /*  Trim boundary indicator that the most recent RTC count increment has coincided with trimming of the count value */
#define BITP_RTC_MOD_INCR                     6            /*  Most recent increment value which has been added to the RTC count in {RTCCNT1, RTCCNT0} */
#define BITP_RTC_MOD_CNTMOD60                 0            /*  Modulo-60 value of the RTC count, {RTCCNT1, RTCCNT0} */
#define BITM_RTC_MOD_CNT0_4TOZERO            (_ADI_MSK(0x0000F800,uint32_t))    /*  Mirror of RTCCNT0[5:0]. */

#define BITM_RTC_MOD_TRMBDY                  (_ADI_MSK(0x00000400,uint32_t))    /*  Trim boundary indicator that the most recent RTC count increment has coincided with trimming of the count value */
#define BITM_RTC_MOD_INCR                    (_ADI_MSK(0x000003C0,uint32_t))    /*  Most recent increment value which has been added to the RTC count in {RTCCNT1, RTCCNT0} */
#define BITM_RTC_MOD_CNTMOD60                (_ADI_MSK(0x0000003F,uint32_t))    /*  Modulo-60 value of the RTC count, {RTCCNT1, RTCCNT0} */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_CNT2                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_CNT2_VALUE                   0            /*  Fractional bits of the RTC real-time count */
#define BITM_RTC_CNT2_VALUE                  (_ADI_MSK(0x00007FFF,uint32_t))    /*  Fractional bits of the RTC real-time count */

/* -------------------------------------------------------------------------------------------------------------------------
          RTC_ALM2                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RTC_ALM2_VALUE                   0            /*  Fractional (non-prescaled) bits of the RTC alarm target time */
#define BITM_RTC_ALM2_VALUE                  (_ADI_MSK(0x00007FFF,uint32_t))    /*  Fractional (non-prescaled) bits of the RTC alarm target time */


/* ============================================================================================================================
        System Identification and Debug Enable
   ============================================================================================================================ */

/* ============================================================================================================================
        SYS
   ============================================================================================================================ */
#define REG_SYS_ADIID                        0x40002020            /*  SYS ADI Identification */
#define REG_SYS_CHIPID                       0x40002024            /*  SYS Chip Identifier */
#define REG_SYS_SWDEN                        0x40002040            /*  SYS Serial Wire Debug Enable */

/* ============================================================================================================================
        SYS Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          SYS_CHIPID                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SYS_CHIPID_PARTID                4
#define BITP_SYS_CHIPID_REV                   0
#define BITM_SYS_CHIPID_PARTID               (_ADI_MSK(0x0000FFF0,uint16_t))
#define BITM_SYS_CHIPID_REV                  (_ADI_MSK(0x0000000F,uint16_t))


/* ============================================================================================================================
        Watchdog Timer
   ============================================================================================================================ */

/* ============================================================================================================================
        WDT0
   ============================================================================================================================ */
#define REG_WDT0_LOAD                        0x40002C00            /*  WDT0 Load value */
#define REG_WDT0_CCNT                        0x40002C04            /*  WDT0 Current count value */
#define REG_WDT0_CTL                         0x40002C08            /*  WDT0 Control */
#define REG_WDT0_RESTART                     0x40002C0C            /*  WDT0 Clear interrupt */
#define REG_WDT0_STAT                        0x40002C18            /*  WDT0 Status */

/* ============================================================================================================================
        WDT Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          WDT_CTL                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_WDT_CTL_SPARE                    7            /*  Unused spare bit */
#define BITP_WDT_CTL_MODE                     6            /*  Timer mode */
#define BITP_WDT_CTL_EN                       5            /*  Timer enable */
#define BITP_WDT_CTL_PRE                      2            /*  Prescaler */
#define BITP_WDT_CTL_IRQ                      1            /*  Timer interrupt */
#define BITP_WDT_CTL_PMD                      0            /*  Power Mode Disable */
#define BITM_WDT_CTL_SPARE                   (_ADI_MSK(0x00000080,uint16_t))    /*  Unused spare bit */
#define BITM_WDT_CTL_MODE                    (_ADI_MSK(0x00000040,uint16_t))    /*  Timer mode */
#define BITM_WDT_CTL_EN                      (_ADI_MSK(0x00000020,uint16_t))    /*  Timer enable */

#define BITM_WDT_CTL_PRE                     (_ADI_MSK(0x0000000C,uint16_t))    /*  Prescaler */
#define BITM_WDT_CTL_IRQ                     (_ADI_MSK(0x00000002,uint16_t))    /*  Timer interrupt */

#define BITM_WDT_CTL_PMD                     (_ADI_MSK(0x00000001,uint16_t))    /*  Power Mode Disable */
#define ENUM_WDT_CTL_DIV1                    (_ADI_MSK(0x00000000,uint16_t))    /*  PRE: source clock/1. */
#define ENUM_WDT_CTL_DIV16                   (_ADI_MSK(0x00000004,uint16_t))    /*  PRE: source clock/16. */
#define ENUM_WDT_CTL_DIV256                  (_ADI_MSK(0x00000008,uint16_t))    /*  PRE: source clock/256 (default). */
#define ENUM_WDT_CTL_RESERVED                (_ADI_MSK(0x0000000C,uint16_t))    /*  PRE: Reserved */
#define ENUM_WDT_CTL_RUN                     (_ADI_MSK(0x00000000,uint16_t))    /*  PMD: The watchdog timer will continue its count down when Cortex is in halted state. */
#define ENUM_WDT_CTL_SUSPEND                 (_ADI_MSK(0x00000001,uint16_t))    /*  PMD: When hibernate or halted mode is entered, the watchdog counter will suspend its count down. As hibernate mode is exited, the count down will resume from its current count value (the count will not be reset). */

/* -------------------------------------------------------------------------------------------------------------------------
          WDT_STAT                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_WDT_STAT_RSTCTL                  5            /*  Reset Control Register written and locked */
#define BITP_WDT_STAT_LOCKED                  4            /*  Lock status bit */
#define BITP_WDT_STAT_COUNTING                3            /*  Control Register write sync in progress */
#define BITP_WDT_STAT_LOADING                 2            /*  Load Register write sync in progress */
#define BITP_WDT_STAT_CLRIRQ                  1            /*  Clear Interrupt Register write sync in progress */
#define BITP_WDT_STAT_IRQ                     0            /*  WDT Interrupt */

#define BITM_WDT_STAT_RSTCTL                 (_ADI_MSK(0x00000020,uint16_t))    /*  Reset Control Register written and locked */

#define BITM_WDT_STAT_LOCKED                 (_ADI_MSK(0x00000010,uint16_t))    /*  Lock status bit */

#define BITM_WDT_STAT_COUNTING               (_ADI_MSK(0x00000008,uint16_t))    /*  Control Register write sync in progress */

#define BITM_WDT_STAT_LOADING                (_ADI_MSK(0x00000004,uint16_t))    /*  Load Register write sync in progress */

#define BITM_WDT_STAT_CLRIRQ                 (_ADI_MSK(0x00000002,uint16_t))    /*  Clear Interrupt Register write sync in progress */

#define BITM_WDT_STAT_IRQ                    (_ADI_MSK(0x00000001,uint16_t))    /*  WDT Interrupt */


/* ============================================================================================================================
        I2C Master/Slave
   ============================================================================================================================ */

/* ============================================================================================================================
        I2C0
   ============================================================================================================================ */
#define REG_I2C0_MCTL                        0x40003000            /*  I2C0 Master control */
#define REG_I2C0_MSTAT                       0x40003004            /*  I2C0 Master status */
#define REG_I2C0_MRX                         0x40003008            /*  I2C0 Master receive data */
#define REG_I2C0_MTX                         0x4000300C            /*  I2C0 Master transmit data */
#define REG_I2C0_MRXCNT                      0x40003010            /*  I2C0 Master receive data count */
#define REG_I2C0_MCRXCNT                     0x40003014            /*  I2C0 Master current receive data count */
#define REG_I2C0_ADDR1                       0x40003018            /*  I2C0 1st master address byte */
#define REG_I2C0_ADDR2                       0x4000301C            /*  I2C0 2nd master address byte */
#define REG_I2C0_BYT                         0x40003020            /*  I2C0 Start byte */
#define REG_I2C0_DIV                         0x40003024            /*  I2C0 Serial clock period divisor */
#define REG_I2C0_SCTL                        0x40003028            /*  I2C0 Slave control */
#define REG_I2C0_SSTAT                       0x4000302C            /*  I2C0 Slave I2C Status/Error/IRQ */
#define REG_I2C0_SRX                         0x40003030            /*  I2C0 Slave receive */
#define REG_I2C0_STX                         0x40003034            /*  I2C0 Slave transmit */
#define REG_I2C0_ALT                         0x40003038            /*  I2C0 Hardware general call ID */
#define REG_I2C0_ID0                         0x4000303C            /*  I2C0 1st slave address device ID */
#define REG_I2C0_ID1                         0x40003040            /*  I2C0 2nd slave address device ID */
#define REG_I2C0_ID2                         0x40003044            /*  I2C0 3rd slave address device ID */
#define REG_I2C0_ID3                         0x40003048            /*  I2C0 4th slave address device ID */
#define REG_I2C0_STAT                        0x4000304C            /*  I2C0 Master and slave FIFO status */
#define REG_I2C0_SHCTL                       0x40003050            /*  I2C0 Shared control */
#define REG_I2C0_TCTL                        0x40003054            /*  I2C0 Timing Control Register */
#define REG_I2C0_ASTRETCH_SCL                0x40003058            /*  I2C0 Automatic stretch SCL register */

/* ============================================================================================================================
        I2C Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          I2C_MCTL                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_MCTL_STOPBUSCLR             13            /*  Prestop Bus-Clear */
#define BITP_I2C_MCTL_BUSCLR                 12            /*  Bus-Clear Enable */
#define BITP_I2C_MCTL_MTXDMA                 11            /*  Enable master Tx DMA request */
#define BITP_I2C_MCTL_MRXDMA                 10            /*  Enable master Rx DMA request */
#define BITP_I2C_MCTL_MXMITDEC                9            /*  Decrement master TX FIFO status when a byte has been transmitted */
#define BITP_I2C_MCTL_IENCMP                  8            /*  Transaction completed (or stop detected) interrupt enable */
#define BITP_I2C_MCTL_IENACK                  7            /*  ACK not received interrupt enable */
#define BITP_I2C_MCTL_IENALOST                6            /*  Arbitration lost interrupt enable */
#define BITP_I2C_MCTL_IENMTX                  5            /*  Transmit request interrupt enable. */
#define BITP_I2C_MCTL_IENMRX                  4            /*  Receive request interrupt enable */
#define BITP_I2C_MCTL_STRETCHSCL              3            /*  Stretch SCL enable */
#define BITP_I2C_MCTL_LOOPBACK                2            /*  Internal loopback enable */
#define BITP_I2C_MCTL_COMPLETE                1            /*  Start back-off disable */
#define BITP_I2C_MCTL_MASEN                   0            /*  Master enable */
#define BITM_I2C_MCTL_STOPBUSCLR             (_ADI_MSK(0x00002000,uint16_t))    /*  Prestop Bus-Clear */
#define BITM_I2C_MCTL_BUSCLR                 (_ADI_MSK(0x00001000,uint16_t))    /*  Bus-Clear Enable */
#define BITM_I2C_MCTL_MTXDMA                 (_ADI_MSK(0x00000800,uint16_t))    /*  Enable master Tx DMA request */
#define BITM_I2C_MCTL_MRXDMA                 (_ADI_MSK(0x00000400,uint16_t))    /*  Enable master Rx DMA request */
#define BITM_I2C_MCTL_MXMITDEC               (_ADI_MSK(0x00000200,uint16_t))    /*  Decrement master TX FIFO status when a byte has been transmitted */
#define BITM_I2C_MCTL_IENCMP                 (_ADI_MSK(0x00000100,uint16_t))    /*  Transaction completed (or stop detected) interrupt enable */
#define BITM_I2C_MCTL_IENACK                 (_ADI_MSK(0x00000080,uint16_t))    /*  ACK not received interrupt enable */
#define BITM_I2C_MCTL_IENALOST               (_ADI_MSK(0x00000040,uint16_t))    /*  Arbitration lost interrupt enable */
#define BITM_I2C_MCTL_IENMTX                 (_ADI_MSK(0x00000020,uint16_t))    /*  Transmit request interrupt enable. */
#define BITM_I2C_MCTL_IENMRX                 (_ADI_MSK(0x00000010,uint16_t))    /*  Receive request interrupt enable */
#define BITM_I2C_MCTL_STRETCHSCL             (_ADI_MSK(0x00000008,uint16_t))    /*  Stretch SCL enable */
#define BITM_I2C_MCTL_LOOPBACK               (_ADI_MSK(0x00000004,uint16_t))    /*  Internal loopback enable */
#define BITM_I2C_MCTL_COMPLETE               (_ADI_MSK(0x00000002,uint16_t))    /*  Start back-off disable */
#define BITM_I2C_MCTL_MASEN                  (_ADI_MSK(0x00000001,uint16_t))    /*  Master enable */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_MSTAT                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_MSTAT_SCLFILT               14            /*  State of SCL Line */
#define BITP_I2C_MSTAT_SDAFILT               13            /*  State of SDA Line */
#define BITP_I2C_MSTAT_MTXUNDR               12            /*  Master Transmit Underflow */
#define BITP_I2C_MSTAT_MSTOP                 11            /*  STOP driven by this I2C Master */
#define BITP_I2C_MSTAT_LINEBUSY              10            /*  Line is busy */
#define BITP_I2C_MSTAT_MRXOVR                 9            /*  Master Receive FIFO overflow */
#define BITP_I2C_MSTAT_TCOMP                  8            /*  Transaction complete or stop detected */
#define BITP_I2C_MSTAT_NACKDATA               7            /*  ACK not received in response to data write */
#define BITP_I2C_MSTAT_MBUSY                  6            /*  Master busy */
#define BITP_I2C_MSTAT_ALOST                  5            /*  Arbitration lost */
#define BITP_I2C_MSTAT_NACKADDR               4            /*  ACK not received in response to an address */
#define BITP_I2C_MSTAT_MRXREQ                 3            /*  Master Receive request */
#define BITP_I2C_MSTAT_MTXREQ                 2            /*  When read is master Transmit request; when write is Clear master transmit interrupt bit */
#define BITP_I2C_MSTAT_MTXF                   0            /*  Master Transmit FIFO status */
#define BITM_I2C_MSTAT_SCLFILT               (_ADI_MSK(0x00004000,uint16_t))    /*  State of SCL Line */
#define BITM_I2C_MSTAT_SDAFILT               (_ADI_MSK(0x00002000,uint16_t))    /*  State of SDA Line */
#define BITM_I2C_MSTAT_MTXUNDR               (_ADI_MSK(0x00001000,uint16_t))    /*  Master Transmit Underflow */
#define BITM_I2C_MSTAT_MSTOP                 (_ADI_MSK(0x00000800,uint16_t))    /*  STOP driven by this I2C Master */
#define BITM_I2C_MSTAT_LINEBUSY              (_ADI_MSK(0x00000400,uint16_t))    /*  Line is busy */
#define BITM_I2C_MSTAT_MRXOVR                (_ADI_MSK(0x00000200,uint16_t))    /*  Master Receive FIFO overflow */
#define BITM_I2C_MSTAT_TCOMP                 (_ADI_MSK(0x00000100,uint16_t))    /*  Transaction complete or stop detected */
#define BITM_I2C_MSTAT_NACKDATA              (_ADI_MSK(0x00000080,uint16_t))    /*  ACK not received in response to data write */
#define BITM_I2C_MSTAT_MBUSY                 (_ADI_MSK(0x00000040,uint16_t))    /*  Master busy */
#define BITM_I2C_MSTAT_ALOST                 (_ADI_MSK(0x00000020,uint16_t))    /*  Arbitration lost */
#define BITM_I2C_MSTAT_NACKADDR              (_ADI_MSK(0x00000010,uint16_t))    /*  ACK not received in response to an address */
#define BITM_I2C_MSTAT_MRXREQ                (_ADI_MSK(0x00000008,uint16_t))    /*  Master Receive request */
#define BITM_I2C_MSTAT_MTXREQ                (_ADI_MSK(0x00000004,uint16_t))    /*  When read is master Transmit request; when write is Clear master transmit interrupt bit */
#define BITM_I2C_MSTAT_MTXF                  (_ADI_MSK(0x00000003,uint16_t))    /*  Master Transmit FIFO status */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_MRX                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_MRX_VALUE                    0            /*  Master receive register */
#define BITM_I2C_MRX_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Master receive register */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_MTX                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_MTX_VALUE                    0            /*  Master transmit register */
#define BITM_I2C_MTX_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Master transmit register */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_MRXCNT                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_MRXCNT_EXTEND                8            /*  Extended read */
#define BITP_I2C_MRXCNT_VALUE                 0            /*  Receive count */
#define BITM_I2C_MRXCNT_EXTEND               (_ADI_MSK(0x00000100,uint16_t))    /*  Extended read */
#define BITM_I2C_MRXCNT_VALUE                (_ADI_MSK(0x000000FF,uint16_t))    /*  Receive count */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_MCRXCNT                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_MCRXCNT_VALUE                0            /*  Current receive count */
#define BITM_I2C_MCRXCNT_VALUE               (_ADI_MSK(0x000000FF,uint16_t))    /*  Current receive count */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ADDR1                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ADDR1_VALUE                  0            /*  Address byte 1 */
#define BITM_I2C_ADDR1_VALUE                 (_ADI_MSK(0x000000FF,uint16_t))    /*  Address byte 1 */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ADDR2                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ADDR2_VALUE                  0            /*  Address byte 2 */
#define BITM_I2C_ADDR2_VALUE                 (_ADI_MSK(0x000000FF,uint16_t))    /*  Address byte 2 */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_BYT                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_BYT_SBYTE                    0            /*  Start byte */
#define BITM_I2C_BYT_SBYTE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Start byte */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_DIV                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_DIV_HIGH                     8            /*  Serial clock high time */
#define BITP_I2C_DIV_LOW                      0            /*  Serial clock low time */
#define BITM_I2C_DIV_HIGH                    (_ADI_MSK(0x0000FF00,uint16_t))    /*  Serial clock high time */
#define BITM_I2C_DIV_LOW                     (_ADI_MSK(0x000000FF,uint16_t))    /*  Serial clock low time */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_SCTL                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_SCTL_STXDMA                 14            /*  Enable slave Tx DMA request */
#define BITP_I2C_SCTL_SRXDMA                 13            /*  Enable slave Rx DMA request */
#define BITP_I2C_SCTL_IENREPST               12            /*  Repeated start interrupt enable */
#define BITP_I2C_SCTL_STXDEC                 11            /*  Decrement Slave Tx FIFO status when a byte has been transmitted */
#define BITP_I2C_SCTL_IENSTX                 10            /*  Slave Transmit request interrupt enable */
#define BITP_I2C_SCTL_IENSRX                  9            /*  Slave Receive request interrupt enable */
#define BITP_I2C_SCTL_IENSTOP                 8            /*  Stop condition detected interrupt enable */
#define BITP_I2C_SCTL_NACK                    7            /*  NACK next communication */
#define BITP_I2C_SCTL_STRETCHSCL              6            /*  Stretch SCL enable */
#define BITP_I2C_SCTL_EARLYTXR                5            /*  Early transmit request mode */
#define BITP_I2C_SCTL_GCSBCLR                 4            /*  General call status bit clear */
#define BITP_I2C_SCTL_HGCEN                   3            /*  Hardware general call enable */
#define BITP_I2C_SCTL_GCEN                    2            /*  General call enable */
#define BITP_I2C_SCTL_ADR10EN                 1            /*  Enabled 10-bit addressing */
#define BITP_I2C_SCTL_SLVEN                   0            /*  Slave enable */
#define BITM_I2C_SCTL_STXDMA                 (_ADI_MSK(0x00004000,uint16_t))    /*  Enable slave Tx DMA request */
#define BITM_I2C_SCTL_SRXDMA                 (_ADI_MSK(0x00002000,uint16_t))    /*  Enable slave Rx DMA request */
#define BITM_I2C_SCTL_IENREPST               (_ADI_MSK(0x00001000,uint16_t))    /*  Repeated start interrupt enable */
#define BITM_I2C_SCTL_STXDEC                 (_ADI_MSK(0x00000800,uint16_t))    /*  Decrement Slave Tx FIFO status when a byte has been transmitted */
#define BITM_I2C_SCTL_IENSTX                 (_ADI_MSK(0x00000400,uint16_t))    /*  Slave Transmit request interrupt enable */
#define BITM_I2C_SCTL_IENSRX                 (_ADI_MSK(0x00000200,uint16_t))    /*  Slave Receive request interrupt enable */
#define BITM_I2C_SCTL_IENSTOP                (_ADI_MSK(0x00000100,uint16_t))    /*  Stop condition detected interrupt enable */
#define BITM_I2C_SCTL_NACK                   (_ADI_MSK(0x00000080,uint16_t))    /*  NACK next communication */
#define BITM_I2C_SCTL_STRETCHSCL             (_ADI_MSK(0x00000040,uint16_t))    /*  Stretch SCL enable */
#define BITM_I2C_SCTL_EARLYTXR               (_ADI_MSK(0x00000020,uint16_t))    /*  Early transmit request mode */
#define BITM_I2C_SCTL_GCSBCLR                (_ADI_MSK(0x00000010,uint16_t))    /*  General call status bit clear */
#define BITM_I2C_SCTL_HGCEN                  (_ADI_MSK(0x00000008,uint16_t))    /*  Hardware general call enable */
#define BITM_I2C_SCTL_GCEN                   (_ADI_MSK(0x00000004,uint16_t))    /*  General call enable */
#define BITM_I2C_SCTL_ADR10EN                (_ADI_MSK(0x00000002,uint16_t))    /*  Enabled 10-bit addressing */
#define BITM_I2C_SCTL_SLVEN                  (_ADI_MSK(0x00000001,uint16_t))    /*  Slave enable */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_SSTAT                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_SSTAT_START                 14            /*  Start and matching address */
#define BITP_I2C_SSTAT_REPSTART              13            /*  Repeated start and matching address */
#define BITP_I2C_SSTAT_IDMAT                 11            /*  Device ID matched */
#define BITP_I2C_SSTAT_STOP                  10            /*  Stop after start and matching address */
#define BITP_I2C_SSTAT_GCID                   8            /*  General ID */
#define BITP_I2C_SSTAT_GCINT                  7            /*  General call interrupt */
#define BITP_I2C_SSTAT_SBUSY                  6            /*  Slave busy */
#define BITP_I2C_SSTAT_NOACK                  5            /*  Ack not generated by the slave */
#define BITP_I2C_SSTAT_SRXOVR                 4            /*  Slave Receive FIFO overflow */
#define BITP_I2C_SSTAT_SRXREQ                 3            /*  Slave Receive request */
#define BITP_I2C_SSTAT_STXREQ                 2            /*  When read is slave transmit request; when write is clear slave transmit interrupt bit */
#define BITP_I2C_SSTAT_STXUNDR                1            /*  Slave Transmit FIFO underflow */
#define BITP_I2C_SSTAT_STXFSEREQ              0            /*  Slave Tx FIFO Status or early request */
#define BITM_I2C_SSTAT_START                 (_ADI_MSK(0x00004000,uint16_t))    /*  Start and matching address */
#define BITM_I2C_SSTAT_REPSTART              (_ADI_MSK(0x00002000,uint16_t))    /*  Repeated start and matching address */

#define BITM_I2C_SSTAT_IDMAT                 (_ADI_MSK(0x00001800,uint16_t))    /*  Device ID matched */
#define BITM_I2C_SSTAT_STOP                  (_ADI_MSK(0x00000400,uint16_t))    /*  Stop after start and matching address */

#define BITM_I2C_SSTAT_GCID                  (_ADI_MSK(0x00000300,uint16_t))    /*  General ID */
#define BITM_I2C_SSTAT_GCINT                 (_ADI_MSK(0x00000080,uint16_t))    /*  General call interrupt */
#define BITM_I2C_SSTAT_SBUSY                 (_ADI_MSK(0x00000040,uint16_t))    /*  Slave busy */
#define BITM_I2C_SSTAT_NOACK                 (_ADI_MSK(0x00000020,uint16_t))    /*  Ack not generated by the slave */
#define BITM_I2C_SSTAT_SRXOVR                (_ADI_MSK(0x00000010,uint16_t))    /*  Slave Receive FIFO overflow */
#define BITM_I2C_SSTAT_SRXREQ                (_ADI_MSK(0x00000008,uint16_t))    /*  Slave Receive request */
#define BITM_I2C_SSTAT_STXREQ                (_ADI_MSK(0x00000004,uint16_t))    /*  When read is slave transmit request; when write is clear slave transmit interrupt bit */
#define BITM_I2C_SSTAT_STXUNDR               (_ADI_MSK(0x00000002,uint16_t))    /*  Slave Transmit FIFO underflow */
#define BITM_I2C_SSTAT_STXFSEREQ             (_ADI_MSK(0x00000001,uint16_t))    /*  Slave Tx FIFO Status or early request */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_SRX                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_SRX_VALUE                    0            /*  Slave receive register */
#define BITM_I2C_SRX_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave receive register */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_STX                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_STX_VALUE                    0            /*  Slave transmit register */
#define BITM_I2C_STX_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave transmit register */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ALT                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ALT_ID                       0            /*  Slave Alt */
#define BITM_I2C_ALT_ID                      (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave Alt */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ID0                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ID0_VALUE                    0            /*  Slave device ID 0 */
#define BITM_I2C_ID0_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave device ID 0 */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ID1                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ID1_VALUE                    0            /*  Slave device ID 1 */
#define BITM_I2C_ID1_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave device ID 1 */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ID2                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ID2_VALUE                    0            /*  Slave device ID 2 */
#define BITM_I2C_ID2_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave device ID 2 */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ID3                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ID3_VALUE                    0            /*  Slave device ID 3 */
#define BITM_I2C_ID3_VALUE                   (_ADI_MSK(0x000000FF,uint16_t))    /*  Slave device ID 3 */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_STAT                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_STAT_MFLUSH                  9            /*  Flush the master transmit FIFO */
#define BITP_I2C_STAT_SFLUSH                  8            /*  Flush the slave transmit FIFO */
#define BITP_I2C_STAT_MRXF                    6            /*  Master receive FIFO status */
#define BITP_I2C_STAT_MTXF                    4            /*  Master transmit FIFO status */
#define BITP_I2C_STAT_SRXF                    2            /*  Slave receive FIFO status */
#define BITP_I2C_STAT_STXF                    0            /*  Slave transmit FIFO status */
#define BITM_I2C_STAT_MFLUSH                 (_ADI_MSK(0x00000200,uint16_t))    /*  Flush the master transmit FIFO */
#define BITM_I2C_STAT_SFLUSH                 (_ADI_MSK(0x00000100,uint16_t))    /*  Flush the slave transmit FIFO */

#define BITM_I2C_STAT_MRXF                   (_ADI_MSK(0x000000C0,uint16_t))    /*  Master receive FIFO status */

#define BITM_I2C_STAT_MTXF                   (_ADI_MSK(0x00000030,uint16_t))    /*  Master transmit FIFO status */

#define BITM_I2C_STAT_SRXF                   (_ADI_MSK(0x0000000C,uint16_t))    /*  Slave receive FIFO status */

#define BITM_I2C_STAT_STXF                   (_ADI_MSK(0x00000003,uint16_t))    /*  Slave transmit FIFO status */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_SHCTL                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_SHCTL_RST                    0            /*  Reset START STOP detect circuit */
#define BITM_I2C_SHCTL_RST                   (_ADI_MSK(0x00000001,uint16_t))    /*  Reset START STOP detect circuit */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_TCTL                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_TCTL_FILTEROFF               8            /*  Input Filter Control */
#define BITP_I2C_TCTL_THDATIN                 0            /*  Data In Hold Start */

#define BITM_I2C_TCTL_FILTEROFF              (_ADI_MSK(0x00000100,uint16_t))    /*  Input Filter Control */
#define BITM_I2C_TCTL_THDATIN                (_ADI_MSK(0x0000001F,uint16_t))    /*  Data In Hold Start */

/* -------------------------------------------------------------------------------------------------------------------------
          I2C_ASTRETCH_SCL                     Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_I2C_ASTRETCH_SCL_SLVTMO          9            /*  slave automatic stretch timeout */
#define BITP_I2C_ASTRETCH_SCL_MSTTMO          8            /*  master automatic stretch timeout */
#define BITP_I2C_ASTRETCH_SCL_SLV             4            /*  slave automatic stretch mode */
#define BITP_I2C_ASTRETCH_SCL_MST             0            /*  master automatic stretch mode */
#define BITM_I2C_ASTRETCH_SCL_SLVTMO         (_ADI_MSK(0x00000200,uint16_t))    /*  slave automatic stretch timeout */
#define BITM_I2C_ASTRETCH_SCL_MSTTMO         (_ADI_MSK(0x00000100,uint16_t))    /*  master automatic stretch timeout */
#define BITM_I2C_ASTRETCH_SCL_SLV            (_ADI_MSK(0x000000F0,uint16_t))    /*  slave automatic stretch mode */
#define BITM_I2C_ASTRETCH_SCL_MST            (_ADI_MSK(0x0000000F,uint16_t))    /*  master automatic stretch mode */


/* ============================================================================================================================
        Serial Peripheral Interface
   ============================================================================================================================ */

/* ============================================================================================================================
        SPI0
   ============================================================================================================================ */
#define REG_SPI0_STAT                        0x40004000            /*  SPI0 Status */
#define REG_SPI0_RX                          0x40004004            /*  SPI0 Receive */
#define REG_SPI0_TX                          0x40004008            /*  SPI0 Transmit */
#define REG_SPI0_DIV                         0x4000400C            /*  SPI0 SPI baud rate selection */
#define REG_SPI0_CTL                         0x40004010            /*  SPI0 SPI configuration 1 */
#define REG_SPI0_IEN                         0x40004014            /*  SPI0 SPI configuration 2 */
#define REG_SPI0_CNT                         0x40004018            /*  SPI0 Transfer byte count */
#define REG_SPI0_DMA                         0x4000401C            /*  SPI0 SPI DMA enable */
#define REG_SPI0_FIFO_STAT                   0x40004020            /*  SPI0 FIFO Status */
#define REG_SPI0_RD_CTL                      0x40004024            /*  SPI0 Read Control */
#define REG_SPI0_FLOW_CTL                    0x40004028            /*  SPI0 Flow Control */
#define REG_SPI0_WAIT_TMR                    0x4000402C            /*  SPI0 Wait timer for flow control */
#define REG_SPI0_CS_CTL                      0x40004030            /*  SPI0 Chip-Select control for multi-slave connections */
#define REG_SPI0_CS_OVERRIDE                 0x40004034            /*  SPI0 Chip-Select Override */
#define REG_SPI0_XFR_CNT                     0x40004038            /*  SPI0 Transferred Count */

/* ============================================================================================================================
        SPI1
   ============================================================================================================================ */
#define REG_SPI1_STAT                        0x40004400            /*  SPI1 Status */
#define REG_SPI1_RX                          0x40004404            /*  SPI1 Receive */
#define REG_SPI1_TX                          0x40004408            /*  SPI1 Transmit */
#define REG_SPI1_DIV                         0x4000440C            /*  SPI1 SPI baud rate selection */
#define REG_SPI1_CTL                         0x40004410            /*  SPI1 SPI configuration 1 */
#define REG_SPI1_IEN                         0x40004414            /*  SPI1 SPI configuration 2 */
#define REG_SPI1_CNT                         0x40004418            /*  SPI1 Transfer byte count */
#define REG_SPI1_DMA                         0x4000441C            /*  SPI1 SPI DMA enable */
#define REG_SPI1_FIFO_STAT                   0x40004420            /*  SPI1 FIFO Status */
#define REG_SPI1_RD_CTL                      0x40004424            /*  SPI1 Read Control */
#define REG_SPI1_FLOW_CTL                    0x40004428            /*  SPI1 Flow Control */
#define REG_SPI1_WAIT_TMR                    0x4000442C            /*  SPI1 Wait timer for flow control */
#define REG_SPI1_CS_CTL                      0x40004430            /*  SPI1 Chip-Select control for multi-slave connections */
#define REG_SPI1_CS_OVERRIDE                 0x40004434            /*  SPI1 Chip-Select Override */
#define REG_SPI1_XFR_CNT                     0x40004438            /*  SPI1 Transferred Count */

/* ============================================================================================================================
        SPI2
   ============================================================================================================================ */
#define REG_SPI2_STAT                        0x40024000            /*  SPI2 Status */
#define REG_SPI2_RX                          0x40024004            /*  SPI2 Receive */
#define REG_SPI2_TX                          0x40024008            /*  SPI2 Transmit */
#define REG_SPI2_DIV                         0x4002400C            /*  SPI2 SPI baud rate selection */
#define REG_SPI2_CTL                         0x40024010            /*  SPI2 SPI configuration 1 */
#define REG_SPI2_IEN                         0x40024014            /*  SPI2 SPI configuration 2 */
#define REG_SPI2_CNT                         0x40024018            /*  SPI2 Transfer byte count */
#define REG_SPI2_DMA                         0x4002401C            /*  SPI2 SPI DMA enable */
#define REG_SPI2_FIFO_STAT                   0x40024020            /*  SPI2 FIFO Status */
#define REG_SPI2_RD_CTL                      0x40024024            /*  SPI2 Read Control */
#define REG_SPI2_FLOW_CTL                    0x40024028            /*  SPI2 Flow Control */
#define REG_SPI2_WAIT_TMR                    0x4002402C            /*  SPI2 Wait timer for flow control */
#define REG_SPI2_CS_CTL                      0x40024030            /*  SPI2 Chip-Select control for multi-slave connections */
#define REG_SPI2_CS_OVERRIDE                 0x40024034            /*  SPI2 Chip-Select Override */
#define REG_SPI2_XFR_CNT                     0x40024038            /*  SPI2 Transferred Count */

/* ============================================================================================================================
        SPI Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          SPI_STAT                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_STAT_RDY                    15            /*  Detected an edge on Ready indicator for flow-control */
#define BITP_SPI_STAT_CSRISE                 14            /*  Detected a rising edge on CS, in slave CON mode */
#define BITP_SPI_STAT_CSFALL                 13            /*  Detected a falling edge on CS, in slave CON mode */
#define BITP_SPI_STAT_CSERR                  12            /*  Detected a CS error condition in slave mode */
#define BITP_SPI_STAT_CS                     11            /*  CS Status */
#define BITP_SPI_STAT_RXOVR                   7            /*  SPI Rx FIFO overflow */
#define BITP_SPI_STAT_RXIRQ                   6            /*  SPI Rx IRQ */
#define BITP_SPI_STAT_TXIRQ                   5            /*  SPI Tx IRQ */
#define BITP_SPI_STAT_TXUNDR                  4            /*  SPI Tx FIFO underflow */
#define BITP_SPI_STAT_TXDONE                  3            /*  SPI Tx Done in read command mode */
#define BITP_SPI_STAT_TXEMPTY                 2            /*  SPI Tx FIFO empty interrupt */
#define BITP_SPI_STAT_XFRDONE                 1            /*  SPI transfer completion */
#define BITP_SPI_STAT_IRQ                     0            /*  SPI Interrupt status */
#define BITM_SPI_STAT_RDY                    (_ADI_MSK(0x00008000,uint16_t))    /*  Detected an edge on Ready indicator for flow-control */
#define BITM_SPI_STAT_CSRISE                 (_ADI_MSK(0x00004000,uint16_t))    /*  Detected a rising edge on CS, in slave CON mode */
#define BITM_SPI_STAT_CSFALL                 (_ADI_MSK(0x00002000,uint16_t))    /*  Detected a falling edge on CS, in slave CON mode */
#define BITM_SPI_STAT_CSERR                  (_ADI_MSK(0x00001000,uint16_t))    /*  Detected a CS error condition in slave mode */

#define BITM_SPI_STAT_CS                     (_ADI_MSK(0x00000800,uint16_t))    /*  CS Status */
#define BITM_SPI_STAT_RXOVR                  (_ADI_MSK(0x00000080,uint16_t))    /*  SPI Rx FIFO overflow */
#define BITM_SPI_STAT_RXIRQ                  (_ADI_MSK(0x00000040,uint16_t))    /*  SPI Rx IRQ */
#define BITM_SPI_STAT_TXIRQ                  (_ADI_MSK(0x00000020,uint16_t))    /*  SPI Tx IRQ */
#define BITM_SPI_STAT_TXUNDR                 (_ADI_MSK(0x00000010,uint16_t))    /*  SPI Tx FIFO underflow */
#define BITM_SPI_STAT_TXDONE                 (_ADI_MSK(0x00000008,uint16_t))    /*  SPI Tx Done in read command mode */
#define BITM_SPI_STAT_TXEMPTY                (_ADI_MSK(0x00000004,uint16_t))    /*  SPI Tx FIFO empty interrupt */
#define BITM_SPI_STAT_XFRDONE                (_ADI_MSK(0x00000002,uint16_t))    /*  SPI transfer completion */
#define BITM_SPI_STAT_IRQ                    (_ADI_MSK(0x00000001,uint16_t))    /*  SPI Interrupt status */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_RX                               Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_RX_BYTE2                     8            /*  8-bit receive buffer, used only in DMA modes */
#define BITP_SPI_RX_BYTE1                     0            /*  8-bit receive buffer */
#define BITM_SPI_RX_BYTE2                    (_ADI_MSK(0x0000FF00,uint16_t))    /*  8-bit receive buffer, used only in DMA modes */
#define BITM_SPI_RX_BYTE1                    (_ADI_MSK(0x000000FF,uint16_t))    /*  8-bit receive buffer */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_TX                               Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_TX_BYTE2                     8            /*  8-bit transmit buffer, used only in DMA modes */
#define BITP_SPI_TX_BYTE1                     0            /*  8-bit transmit buffer */
#define BITM_SPI_TX_BYTE2                    (_ADI_MSK(0x0000FF00,uint16_t))    /*  8-bit transmit buffer, used only in DMA modes */
#define BITM_SPI_TX_BYTE1                    (_ADI_MSK(0x000000FF,uint16_t))    /*  8-bit transmit buffer */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_DIV                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_DIV_VALUE                    0            /*  SPI clock divider */
#define BITM_SPI_DIV_VALUE                   (_ADI_MSK(0x0000003F,uint16_t))    /*  SPI clock divider */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_CTL                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_CTL_CSRST                   14            /*  Reset Mode for CS Error bit */
#define BITP_SPI_CTL_TFLUSH                  13            /*  SPI Tx FIFO Flush enable */
#define BITP_SPI_CTL_RFLUSH                  12            /*  SPI Rx FIFO Flush enable */
#define BITP_SPI_CTL_CON                     11            /*  Continuous transfer enable */
#define BITP_SPI_CTL_LOOPBACK                10            /*  Loopback enable */
#define BITP_SPI_CTL_OEN                      9            /*  Slave MISO output enable */
#define BITP_SPI_CTL_RXOF                     8            /*  RX overflow overwrite enable */
#define BITP_SPI_CTL_ZEN                      7            /*  Transmit zeros enable */
#define BITP_SPI_CTL_TIM                      6            /*  SPI transfer and interrupt mode */
#define BITP_SPI_CTL_LSB                      5            /*  LSB first transfer enable */
#define BITP_SPI_CTL_WOM                      4            /*  SPI Wired Or mode */
#define BITP_SPI_CTL_CPOL                     3            /*  Serial Clock Polarity */
#define BITP_SPI_CTL_CPHA                     2            /*  Serial clock phase mode */
#define BITP_SPI_CTL_MASEN                    1            /*  Master mode enable */
#define BITP_SPI_CTL_SPIEN                    0            /*  SPI enable */
#define BITM_SPI_CTL_CSRST                   (_ADI_MSK(0x00004000,uint16_t))    /*  Reset Mode for CS Error bit */
#define BITM_SPI_CTL_TFLUSH                  (_ADI_MSK(0x00002000,uint16_t))    /*  SPI Tx FIFO Flush enable */
#define BITM_SPI_CTL_RFLUSH                  (_ADI_MSK(0x00001000,uint16_t))    /*  SPI Rx FIFO Flush enable */
#define BITM_SPI_CTL_CON                     (_ADI_MSK(0x00000800,uint16_t))    /*  Continuous transfer enable */
#define BITM_SPI_CTL_LOOPBACK                (_ADI_MSK(0x00000400,uint16_t))    /*  Loopback enable */
#define BITM_SPI_CTL_OEN                     (_ADI_MSK(0x00000200,uint16_t))    /*  Slave MISO output enable */
#define BITM_SPI_CTL_RXOF                    (_ADI_MSK(0x00000100,uint16_t))    /*  RX overflow overwrite enable */
#define BITM_SPI_CTL_ZEN                     (_ADI_MSK(0x00000080,uint16_t))    /*  Transmit zeros enable */
#define BITM_SPI_CTL_TIM                     (_ADI_MSK(0x00000040,uint16_t))    /*  SPI transfer and interrupt mode */

#define BITM_SPI_CTL_LSB                     (_ADI_MSK(0x00000020,uint16_t))    /*  LSB first transfer enable */

#define BITM_SPI_CTL_WOM                     (_ADI_MSK(0x00000010,uint16_t))    /*  SPI Wired Or mode */

#define BITM_SPI_CTL_CPOL                    (_ADI_MSK(0x00000008,uint16_t))    /*  Serial Clock Polarity */

#define BITM_SPI_CTL_CPHA                    (_ADI_MSK(0x00000004,uint16_t))    /*  Serial clock phase mode */

#define BITM_SPI_CTL_MASEN                   (_ADI_MSK(0x00000002,uint16_t))    /*  Master mode enable */

#define BITM_SPI_CTL_SPIEN                   (_ADI_MSK(0x00000001,uint16_t))    /*  SPI enable */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_IEN                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_IEN_TXEMPTY                 14            /*  Tx-FIFO Empty interrupt enable */
#define BITP_SPI_IEN_XFRDONE                 13            /*  SPI transfer completion interrupt enable */
#define BITP_SPI_IEN_TXDONE                  12            /*  SPI transmit done interrupt enable */
#define BITP_SPI_IEN_RDY                     11            /*  Ready signal edge interrupt enable */
#define BITP_SPI_IEN_RXOVR                   10            /*  Rx-overflow interrupt enable */
#define BITP_SPI_IEN_TXUNDR                   9            /*  Tx-underflow interrupt enable */
#define BITP_SPI_IEN_CS                       8            /*  Enable interrupt on every CS edge in slave CON mode */
#define BITP_SPI_IEN_IRQMODE                  0            /*  SPI IRQ mode bits */

#define BITM_SPI_IEN_TXEMPTY                 (_ADI_MSK(0x00004000,uint16_t))    /*  Tx-FIFO Empty interrupt enable */

#define BITM_SPI_IEN_XFRDONE                 (_ADI_MSK(0x00002000,uint16_t))    /*  SPI transfer completion interrupt enable */

#define BITM_SPI_IEN_TXDONE                  (_ADI_MSK(0x00001000,uint16_t))    /*  SPI transmit done interrupt enable */

#define BITM_SPI_IEN_RDY                     (_ADI_MSK(0x00000800,uint16_t))    /*  Ready signal edge interrupt enable */

#define BITM_SPI_IEN_RXOVR                   (_ADI_MSK(0x00000400,uint16_t))    /*  Rx-overflow interrupt enable */

#define BITM_SPI_IEN_TXUNDR                  (_ADI_MSK(0x00000200,uint16_t))    /*  Tx-underflow interrupt enable */
#define BITM_SPI_IEN_CS                      (_ADI_MSK(0x00000100,uint16_t))    /*  Enable interrupt on every CS edge in slave CON mode */

#define BITM_SPI_IEN_IRQMODE                 (_ADI_MSK(0x00000007,uint16_t))    /*  SPI IRQ mode bits */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_CNT                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_CNT_FRAMECONT               15            /*  Continue frame */
#define BITP_SPI_CNT_VALUE                    0            /*  Transfer byte count */

#define BITM_SPI_CNT_FRAMECONT               (_ADI_MSK(0x00008000,uint16_t))    /*  Continue frame */
#define BITM_SPI_CNT_VALUE                   (_ADI_MSK(0x00003FFF,uint16_t))    /*  Transfer byte count */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_DMA                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_DMA_RXEN                     2            /*  Enable receive DMA request */
#define BITP_SPI_DMA_TXEN                     1            /*  Enable transmit DMA request */
#define BITP_SPI_DMA_EN                       0            /*  Enable DMA for data transfer */
#define BITM_SPI_DMA_RXEN                    (_ADI_MSK(0x00000004,uint16_t))    /*  Enable receive DMA request */
#define BITM_SPI_DMA_TXEN                    (_ADI_MSK(0x00000002,uint16_t))    /*  Enable transmit DMA request */
#define BITM_SPI_DMA_EN                      (_ADI_MSK(0x00000001,uint16_t))    /*  Enable DMA for data transfer */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_FIFO_STAT                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_FIFO_STAT_RX                 8            /*  SPI Rx FIFO status */
#define BITP_SPI_FIFO_STAT_TX                 0            /*  SPI Tx FIFO status */

#define BITM_SPI_FIFO_STAT_RX                (_ADI_MSK(0x00000F00,uint16_t))    /*  SPI Rx FIFO status */

#define BITM_SPI_FIFO_STAT_TX                (_ADI_MSK(0x0000000F,uint16_t))    /*  SPI Tx FIFO status */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_RD_CTL                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_RD_CTL_THREEPIN              8            /*  Three pin SPI mode */
#define BITP_SPI_RD_CTL_TXBYTES               2            /*  Transmit byte count minus 1 for read command */
#define BITP_SPI_RD_CTL_OVERLAP               1            /*  Tx/Rx Overlap mode */
#define BITP_SPI_RD_CTL_CMDEN                 0            /*  Read command enable */

#define BITM_SPI_RD_CTL_THREEPIN             (_ADI_MSK(0x00000100,uint16_t))    /*  Three pin SPI mode */
#define BITM_SPI_RD_CTL_TXBYTES              (_ADI_MSK(0x0000003C,uint16_t))    /*  Transmit byte count minus 1 for read command */

#define BITM_SPI_RD_CTL_OVERLAP              (_ADI_MSK(0x00000002,uint16_t))    /*  Tx/Rx Overlap mode */

#define BITM_SPI_RD_CTL_CMDEN                (_ADI_MSK(0x00000001,uint16_t))    /*  Read command enable */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_FLOW_CTL                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_FLOW_CTL_RDBURSTSZ           8            /*  Read data burst size minus 1 */
#define BITP_SPI_FLOW_CTL_RDYPOL              4            /*  Polarity of RDY/MISO line */
#define BITP_SPI_FLOW_CTL_MODE                0            /*  Flow control mode */
#define BITM_SPI_FLOW_CTL_RDBURSTSZ          (_ADI_MSK(0x00000F00,uint16_t))    /*  Read data burst size minus 1 */

#define BITM_SPI_FLOW_CTL_RDYPOL             (_ADI_MSK(0x00000010,uint16_t))    /*  Polarity of RDY/MISO line */

#define BITM_SPI_FLOW_CTL_MODE               (_ADI_MSK(0x00000003,uint16_t))    /*  Flow control mode */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_CS_CTL                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_CS_CTL_SEL                   0            /*  Chip-Select control */
#define BITM_SPI_CS_CTL_SEL                  (_ADI_MSK(0x0000000F,uint16_t))    /*  Chip-Select control */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_CS_OVERRIDE                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_CS_OVERRIDE_CTL              0            /*  CS Override Control */

#define BITM_SPI_CS_OVERRIDE_CTL             (_ADI_MSK(0x00000003,uint16_t))    /*  CS Override Control */

/* -------------------------------------------------------------------------------------------------------------------------
          SPI_XFR_CNT                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPI_XFR_CNT_BYTES                0            /*  Number of bytes transferred over SPI. */
#define BITM_SPI_XFR_CNT_BYTES               (_ADI_MSK(0x00003FFF,uint16_t))    /*  Number of bytes transferred over SPI. */


/* ============================================================================================================================
        UART
   ============================================================================================================================ */

/* ============================================================================================================================
        UART0
   ============================================================================================================================ */
#define REG_UART0_RX                         0x40005000            /*  UART0 Receive Buffer Register */
#define REG_UART0_TX                         0x40005000            /*  UART0 Transmit Holding Register */
#define REG_UART0_IEN                        0x40005004            /*  UART0 Interrupt Enable */
#define REG_UART0_IIR                        0x40005008            /*  UART0 Interrupt ID */
#define REG_UART0_LCR                        0x4000500C            /*  UART0 Line Control */
#define REG_UART0_MCR                        0x40005010            /*  UART0 Modem Control */
#define REG_UART0_LSR                        0x40005014            /*  UART0 Line Status */
#define REG_UART0_MSR                        0x40005018            /*  UART0 Modem Status */
#define REG_UART0_SCR                        0x4000501C            /*  UART0 Scratch buffer */
#define REG_UART0_FBR                        0x40005024            /*  UART0 Fractional Baud Rate */
#define REG_UART0_DIV                        0x40005028            /*  UART0 Baudrate divider */
#define REG_UART0_CTL                        0x40005030            /*  UART0 UART control register */

/* ============================================================================================================================
        UART Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          UART_RX                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_RX_RBR                      0            /*  Receive Buffer Register */
#define BITM_UART_RX_RBR                     (_ADI_MSK(0x000000FF,uint16_t))    /*  Receive Buffer Register */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_TX                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_TX_THR                      0            /*  Transmit Holding Register */
#define BITM_UART_TX_THR                     (_ADI_MSK(0x000000FF,uint16_t))    /*  Transmit Holding Register */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_IEN                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_IEN_EDMAR                   5            /*  DMA requests in receive mode */
#define BITP_UART_IEN_EDMAT                   4            /*  DMA requests in transmit mode */
#define BITP_UART_IEN_EDSSI                   3            /*  Modem status interrupt */
#define BITP_UART_IEN_ELSI                    2            /*  Rx status interrupt */
#define BITP_UART_IEN_ETBEI                   1            /*  Transmit buffer empty interrupt */
#define BITP_UART_IEN_ERBFI                   0            /*  Receive buffer full interrupt */

#define BITM_UART_IEN_EDMAR                  (_ADI_MSK(0x00000020,uint16_t))    /*  DMA requests in receive mode */

#define BITM_UART_IEN_EDMAT                  (_ADI_MSK(0x00000010,uint16_t))    /*  DMA requests in transmit mode */

#define BITM_UART_IEN_EDSSI                  (_ADI_MSK(0x00000008,uint16_t))    /*  Modem status interrupt */

#define BITM_UART_IEN_ELSI                   (_ADI_MSK(0x00000004,uint16_t))    /*  Rx status interrupt */

#define BITM_UART_IEN_ETBEI                  (_ADI_MSK(0x00000002,uint16_t))    /*  Transmit buffer empty interrupt */

#define BITM_UART_IEN_ERBFI                  (_ADI_MSK(0x00000001,uint16_t))    /*  Receive buffer full interrupt */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_IIR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_IIR_STAT                    1            /*  Interrupt status */
#define BITP_UART_IIR_NIRQ                    0            /*  Interrupt flag */

#define BITM_UART_IIR_STAT                   (_ADI_MSK(0x00000006,uint16_t))    /*  Interrupt status */
#define BITM_UART_IIR_NIRQ                   (_ADI_MSK(0x00000001,uint16_t))    /*  Interrupt flag */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_LCR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_LCR_BRK                     6            /*  Set Break */
#define BITP_UART_LCR_SP                      5            /*  Stick Parity */
#define BITP_UART_LCR_EPS                     4            /*  Parity Select */
#define BITP_UART_LCR_PEN                     3            /*  Parity Enable */
#define BITP_UART_LCR_STOP                    2            /*  Stop Bit */
#define BITP_UART_LCR_WLS                     0            /*  Word Length Select */

#define BITM_UART_LCR_BRK                    (_ADI_MSK(0x00000040,uint16_t))    /*  Set Break */

#define BITM_UART_LCR_SP                     (_ADI_MSK(0x00000020,uint16_t))    /*  Stick Parity */

#define BITM_UART_LCR_EPS                    (_ADI_MSK(0x00000010,uint16_t))    /*  Parity Select */

#define BITM_UART_LCR_PEN                    (_ADI_MSK(0x00000008,uint16_t))    /*  Parity Enable */

#define BITM_UART_LCR_STOP                   (_ADI_MSK(0x00000004,uint16_t))    /*  Stop Bit */

#define BITM_UART_LCR_WLS                    (_ADI_MSK(0x00000003,uint16_t))    /*  Word Length Select */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_MCR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_MCR_LOOPBACK                4            /*  Loopback mode */
#define BITP_UART_MCR_OUT2                    3            /*  Output 2 */
#define BITP_UART_MCR_OUT1                    2            /*  Output 1 */
#define BITP_UART_MCR_RTS                     1            /*  Request to send */
#define BITP_UART_MCR_DTR                     0            /*  Data Terminal Ready */

#define BITM_UART_MCR_LOOPBACK               (_ADI_MSK(0x00000010,uint16_t))    /*  Loopback mode */

#define BITM_UART_MCR_OUT2                   (_ADI_MSK(0x00000008,uint16_t))    /*  Output 2 */

#define BITM_UART_MCR_OUT1                   (_ADI_MSK(0x00000004,uint16_t))    /*  Output 1 */

#define BITM_UART_MCR_RTS                    (_ADI_MSK(0x00000002,uint16_t))    /*  Request to send */

#define BITM_UART_MCR_DTR                    (_ADI_MSK(0x00000001,uint16_t))    /*  Data Terminal Ready */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_LSR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_LSR_TEMT                    6            /*  Transmit and Shift Register Empty Status */
#define BITP_UART_LSR_THRE                    5            /*  Transmit Register Empty */
#define BITP_UART_LSR_BI                      4            /*  Break Indicator */
#define BITP_UART_LSR_FE                      3            /*  Framing Error */
#define BITP_UART_LSR_PE                      2            /*  Parity Error */
#define BITP_UART_LSR_OE                      1            /*  Overrun Error */
#define BITP_UART_LSR_DR                      0            /*  Data Ready */

#define BITM_UART_LSR_TEMT                   (_ADI_MSK(0x00000040,uint16_t))    /*  Transmit and Shift Register Empty Status */

#define BITM_UART_LSR_THRE                   (_ADI_MSK(0x00000020,uint16_t))    /*  Transmit Register Empty */

#define BITM_UART_LSR_BI                     (_ADI_MSK(0x00000010,uint16_t))    /*  Break Indicator */

#define BITM_UART_LSR_FE                     (_ADI_MSK(0x00000008,uint16_t))    /*  Framing Error */

#define BITM_UART_LSR_PE                     (_ADI_MSK(0x00000004,uint16_t))    /*  Parity Error */

#define BITM_UART_LSR_OE                     (_ADI_MSK(0x00000002,uint16_t))    /*  Overrun Error */

#define BITM_UART_LSR_DR                     (_ADI_MSK(0x00000001,uint16_t))    /*  Data Ready */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_MSR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_MSR_DCD                     7            /*  Data Carrier Detect */
#define BITP_UART_MSR_RI                      6            /*  Ring Indicator */
#define BITP_UART_MSR_DSR                     5            /*  Data Set Ready */
#define BITP_UART_MSR_CTS                     4            /*  Clear To Send */
#define BITP_UART_MSR_DDCD                    3            /*  Delta DCD */
#define BITP_UART_MSR_TERI                    2            /*  Trailing Edge RI */
#define BITP_UART_MSR_DDSR                    1            /*  Delta DSR */
#define BITP_UART_MSR_DCTS                    0            /*  Delta CTS */

#define BITM_UART_MSR_DCD                    (_ADI_MSK(0x00000080,uint16_t))    /*  Data Carrier Detect */

#define BITM_UART_MSR_RI                     (_ADI_MSK(0x00000040,uint16_t))    /*  Ring Indicator */

#define BITM_UART_MSR_DSR                    (_ADI_MSK(0x00000020,uint16_t))    /*  Data Set Ready */

#define BITM_UART_MSR_CTS                    (_ADI_MSK(0x00000010,uint16_t))    /*  Clear To Send */

#define BITM_UART_MSR_DDCD                   (_ADI_MSK(0x00000008,uint16_t))    /*  Delta DCD */

#define BITM_UART_MSR_TERI                   (_ADI_MSK(0x00000004,uint16_t))    /*  Trailing Edge RI */

#define BITM_UART_MSR_DDSR                   (_ADI_MSK(0x00000002,uint16_t))    /*  Delta DSR */

#define BITM_UART_MSR_DCTS                   (_ADI_MSK(0x00000001,uint16_t))    /*  Delta CTS */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_SCR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_SCR_SCR                     0            /*  Scratch */
#define BITM_UART_SCR_SCR                    (_ADI_MSK(0x000000FF,uint16_t))    /*  Scratch */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_FBR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_FBR_FBEN                   15            /*  Fractional baud rate generator enable */
#define BITP_UART_FBR_DIVM                   11            /*  Fractional baud rate M divide bits 1 to 3 */
#define BITP_UART_FBR_DIVN                    0            /*  Fractional baud rate N divide bits 0 to 2047. */
#define BITM_UART_FBR_FBEN                   (_ADI_MSK(0x00008000,uint16_t))    /*  Fractional baud rate generator enable */
#define BITM_UART_FBR_DIVM                   (_ADI_MSK(0x00001800,uint16_t))    /*  Fractional baud rate M divide bits 1 to 3 */
#define BITM_UART_FBR_DIVN                   (_ADI_MSK(0x000007FF,uint16_t))    /*  Fractional baud rate N divide bits 0 to 2047. */

/* -------------------------------------------------------------------------------------------------------------------------
          UART_CTL                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_UART_CTL_FORCECLK                1            /*  Force UCLK on */

#define BITM_UART_CTL_FORCECLK               (_ADI_MSK(0x00000002,uint16_t))    /*  Force UCLK on */


/* ============================================================================================================================
        Beeper Driver
   ============================================================================================================================ */

/* ============================================================================================================================
        BEEP0
   ============================================================================================================================ */
#define REG_BEEP0_CFG                        0x40005C00            /*  BEEP0 Beeper configuration */
#define REG_BEEP0_STAT                       0x40005C04            /*  BEEP0 Beeper status */
#define REG_BEEP0_TONEA                      0x40005C08            /*  BEEP0 Tone A Data */
#define REG_BEEP0_TONEB                      0x40005C0C            /*  BEEP0 Tone B Data */

/* ============================================================================================================================
        BEEP Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          BEEP_CFG                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BEEP_CFG_SEQATENDIRQ            15            /*  Sequence end IRQ */
#define BITP_BEEP_CFG_SEQNEARENDIRQ          14            /*  Sequence 1 cycle from end IRQ */
#define BITP_BEEP_CFG_BENDIRQ                13            /*  Tone B end IRQ */
#define BITP_BEEP_CFG_BSTARTIRQ              12            /*  Tone B start IRQ */
#define BITP_BEEP_CFG_AENDIRQ                11            /*  Tone A end IRQ */
#define BITP_BEEP_CFG_ASTARTIRQ              10            /*  Tone A start IRQ */
#define BITP_BEEP_CFG_EN                      8            /*  Beeper Enable */
#define BITP_BEEP_CFG_SEQREPEAT               0            /*  Beeper Sequence Repeat value */

#define BITM_BEEP_CFG_SEQATENDIRQ            (_ADI_MSK(0x00008000,uint16_t))    /*  Sequence end IRQ */

#define BITM_BEEP_CFG_SEQNEARENDIRQ          (_ADI_MSK(0x00004000,uint16_t))    /*  Sequence 1 cycle from end IRQ */

#define BITM_BEEP_CFG_BENDIRQ                (_ADI_MSK(0x00002000,uint16_t))    /*  Tone B end IRQ */

#define BITM_BEEP_CFG_BSTARTIRQ              (_ADI_MSK(0x00001000,uint16_t))    /*  Tone B start IRQ */

#define BITM_BEEP_CFG_AENDIRQ                (_ADI_MSK(0x00000800,uint16_t))    /*  Tone A end IRQ */

#define BITM_BEEP_CFG_ASTARTIRQ              (_ADI_MSK(0x00000400,uint16_t))    /*  Tone A start IRQ */

#define BITM_BEEP_CFG_EN                     (_ADI_MSK(0x00000100,uint16_t))    /*  Beeper Enable */

#define BITM_BEEP_CFG_SEQREPEAT              (_ADI_MSK(0x000000FF,uint16_t))    /*  Beeper Sequence Repeat value */

/* -------------------------------------------------------------------------------------------------------------------------
          BEEP_STAT                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BEEP_STAT_SEQENDED              15            /*  Sequencer has ended */
#define BITP_BEEP_STAT_SEQNEAREND            14            /*  Sequencer last tone-pair has started */
#define BITP_BEEP_STAT_BENDED                13            /*  Tone B has ended */
#define BITP_BEEP_STAT_BSTARTED              12            /*  Tone B has started */
#define BITP_BEEP_STAT_AENDED                11            /*  Tone A has ended */
#define BITP_BEEP_STAT_ASTARTED              10            /*  Tone A has started */
#define BITP_BEEP_STAT_BUSY                   8            /*  Beeper is busy */
#define BITP_BEEP_STAT_SEQREMAIN              0            /*  Remaining tone-pair sequence iterations to play in SEQ mode */
#define BITM_BEEP_STAT_SEQENDED              (_ADI_MSK(0x00008000,uint16_t))    /*  Sequencer has ended */
#define BITM_BEEP_STAT_SEQNEAREND            (_ADI_MSK(0x00004000,uint16_t))    /*  Sequencer last tone-pair has started */
#define BITM_BEEP_STAT_BENDED                (_ADI_MSK(0x00002000,uint16_t))    /*  Tone B has ended */
#define BITM_BEEP_STAT_BSTARTED              (_ADI_MSK(0x00001000,uint16_t))    /*  Tone B has started */
#define BITM_BEEP_STAT_AENDED                (_ADI_MSK(0x00000800,uint16_t))    /*  Tone A has ended */
#define BITM_BEEP_STAT_ASTARTED              (_ADI_MSK(0x00000400,uint16_t))    /*  Tone A has started */

#define BITM_BEEP_STAT_BUSY                  (_ADI_MSK(0x00000100,uint16_t))    /*  Beeper is busy */
#define BITM_BEEP_STAT_SEQREMAIN             (_ADI_MSK(0x000000FF,uint16_t))    /*  Remaining tone-pair sequence iterations to play in SEQ mode */

/* -------------------------------------------------------------------------------------------------------------------------
          BEEP_TONEA                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BEEP_TONEA_DIS                  15            /*  Output disable */
#define BITP_BEEP_TONEA_FREQ                  8            /*  Tone frequency */
#define BITP_BEEP_TONEA_DUR                   0            /*  Tone duration */

#define BITM_BEEP_TONEA_DIS                  (_ADI_MSK(0x00008000,uint16_t))    /*  Output disable */

#define BITM_BEEP_TONEA_FREQ                 (_ADI_MSK(0x00007F00,uint16_t))    /*  Tone frequency */

#define BITM_BEEP_TONEA_DUR                  (_ADI_MSK(0x000000FF,uint16_t))    /*  Tone duration */

/* -------------------------------------------------------------------------------------------------------------------------
          BEEP_TONEB                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BEEP_TONEB_DIS                  15            /*  Output disable */
#define BITP_BEEP_TONEB_FREQ                  8            /*  Tone frequency */
#define BITP_BEEP_TONEB_DUR                   0            /*  Tone duration */

#define BITM_BEEP_TONEB_DIS                  (_ADI_MSK(0x00008000,uint16_t))    /*  Output disable */

#define BITM_BEEP_TONEB_FREQ                 (_ADI_MSK(0x00007F00,uint16_t))    /*  Tone frequency */

#define BITM_BEEP_TONEB_DUR                  (_ADI_MSK(0x000000FF,uint16_t))    /*  Tone duration */


/* ============================================================================================================================
        DMA
   ============================================================================================================================ */

/* ============================================================================================================================
        DMA0
   ============================================================================================================================ */
#define REG_DMA0_STAT                        0x40010000            /*  DMA0 DMA Status */
#define REG_DMA0_CFG                         0x40010004            /*  DMA0 DMA Configuration */
#define REG_DMA0_PDBPTR                      0x40010008            /*  DMA0 DMA channel primary control data base pointer */
#define REG_DMA0_ADBPTR                      0x4001000C            /*  DMA0 DMA channel alternate control data base pointer */
#define REG_DMA0_SWREQ                       0x40010014            /*  DMA0 DMA channel software request */
#define REG_DMA0_RMSK_SET                    0x40010020            /*  DMA0 DMA channel request mask set */
#define REG_DMA0_RMSK_CLR                    0x40010024            /*  DMA0 DMA channel request mask clear */
#define REG_DMA0_EN_SET                      0x40010028            /*  DMA0 DMA channel enable set */
#define REG_DMA0_EN_CLR                      0x4001002C            /*  DMA0 DMA channel enable clear */
#define REG_DMA0_ALT_SET                     0x40010030            /*  DMA0 DMA channel primary-alternate set */
#define REG_DMA0_ALT_CLR                     0x40010034            /*  DMA0 DMA channel primary-alternate clear */
#define REG_DMA0_PRI_SET                     0x40010038            /*  DMA0 DMA channel priority set */
#define REG_DMA0_PRI_CLR                     0x4001003C            /*  DMA0 DMA channel priority clear */
#define REG_DMA0_ERRCHNL_CLR                 0x40010048            /*  DMA0 DMA Per Channel Error Clear */
#define REG_DMA0_ERR_CLR                     0x4001004C            /*  DMA0 DMA bus error clear */
#define REG_DMA0_INVALIDDESC_CLR             0x40010050            /*  DMA0 DMA Per Channel Invalid Descriptor Clear */
#define REG_DMA0_BS_SET                      0x40010800            /*  DMA0 DMA channel bytes swap enable set */
#define REG_DMA0_BS_CLR                      0x40010804            /*  DMA0 DMA channel bytes swap enable clear */
#define REG_DMA0_SRCADDR_SET                 0x40010810            /*  DMA0 DMA channel source address decrement enable set */
#define REG_DMA0_SRCADDR_CLR                 0x40010814            /*  DMA0 DMA channel source address decrement enable clear */
#define REG_DMA0_DSTADDR_SET                 0x40010818            /*  DMA0 DMA channel destination address decrement enable set */
#define REG_DMA0_DSTADDR_CLR                 0x4001081C            /*  DMA0 DMA channel destination address decrement enable clear */
#define REG_DMA0_REVID                       0x40010FE0            /*  DMA0 DMA Controller Revision ID */

/* ============================================================================================================================
        DMA Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          DMA_STAT                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_STAT_CHANM1                 16            /*  Number of available DMA channels minus 1 */
#define BITP_DMA_STAT_STATE                   4            /*  Current state of DMA controller */
#define BITP_DMA_STAT_MEN                     0            /*  Enable status of the controller */
#define BITM_DMA_STAT_CHANM1                 (_ADI_MSK(0x001F0000,uint32_t))    /*  Number of available DMA channels minus 1 */

#define BITM_DMA_STAT_STATE                  (_ADI_MSK(0x000000F0,uint32_t))    /*  Current state of DMA controller */

#define BITM_DMA_STAT_MEN                    (_ADI_MSK(0x00000001,uint32_t))    /*  Enable status of the controller */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_CFG                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_CFG_MEN                      0            /*  Controller enable */

#define BITM_DMA_CFG_MEN                     (_ADI_MSK(0x00000001,uint32_t))    /*  Controller enable */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_SWREQ                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_SWREQ_CHAN                   0            /*  Generate software request */
#define BITM_DMA_SWREQ_CHAN                  (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Generate software request */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_RMSK_SET                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_RMSK_SET_CHAN                0            /*  Mask requests from DMA channels */
#define BITM_DMA_RMSK_SET_CHAN               (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Mask requests from DMA channels */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_RMSK_CLR                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_RMSK_CLR_CHAN                0            /*  Clear Request Mask Set bits */
#define BITM_DMA_RMSK_CLR_CHAN               (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Clear Request Mask Set bits */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_EN_SET                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_EN_SET_CHAN                  0            /*  Enable DMA channels */
#define BITM_DMA_EN_SET_CHAN                 (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Enable DMA channels */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_EN_CLR                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_EN_CLR_CHAN                  0            /*  Disable DMA channels */
#define BITM_DMA_EN_CLR_CHAN                 (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Disable DMA channels */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_ALT_SET                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_ALT_SET_CHAN                 0            /*  Control struct status / select alt struct */
#define BITM_DMA_ALT_SET_CHAN                (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Control struct status / select alt struct */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_ALT_CLR                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_ALT_CLR_CHAN                 0            /*  Select primary data struct */
#define BITM_DMA_ALT_CLR_CHAN                (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Select primary data struct */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_PRI_SET                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_PRI_SET_CHAN                 0            /*  Configure channel for high priority */
#define BITM_DMA_PRI_SET_CHAN                (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Configure channel for high priority */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_PRI_CLR                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_PRI_CLR_CHPRICLR             0            /*  Configure channel for default priority level */
#define BITM_DMA_PRI_CLR_CHPRICLR            (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Configure channel for default priority level */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_ERRCHNL_CLR                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_ERRCHNL_CLR_CHAN             0            /*  Per channel Bus error status/ Per channel bus error clear */
#define BITM_DMA_ERRCHNL_CLR_CHAN            (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Per channel Bus error status/ Per channel bus error clear */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_ERR_CLR                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_ERR_CLR_CHAN                 0            /*  Bus error status */
#define BITM_DMA_ERR_CLR_CHAN                (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Bus error status */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_INVALIDDESC_CLR                  Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_INVALIDDESC_CLR_CHAN         0            /*  Per channel Invalid Descriptor status/ Per channel Invalid descriptor status clear */
#define BITM_DMA_INVALIDDESC_CLR_CHAN        (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Per channel Invalid Descriptor status/ Per channel Invalid descriptor status clear */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_BS_SET                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_BS_SET_CHAN                  0            /*  Byte swap status */
#define BITM_DMA_BS_SET_CHAN                 (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Byte swap status */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_BS_CLR                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_BS_CLR_CHAN                  0            /*  Disable byte swap */
#define BITM_DMA_BS_CLR_CHAN                 (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Disable byte swap */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_SRCADDR_SET                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_SRCADDR_SET_CHAN             0            /*  Source Address decrement status / configure Source address decrement */
#define BITM_DMA_SRCADDR_SET_CHAN            (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Source Address decrement status / configure Source address decrement */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_SRCADDR_CLR                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_SRCADDR_CLR_CHAN             0            /*  Disable source address decrement */
#define BITM_DMA_SRCADDR_CLR_CHAN            (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Disable source address decrement */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_DSTADDR_SET                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_DSTADDR_SET_CHAN             0            /*  Destination Address decrement status / configure destination address decrement */
#define BITM_DMA_DSTADDR_SET_CHAN            (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Destination Address decrement status / configure destination address decrement */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_DSTADDR_CLR                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_DSTADDR_CLR_CHAN             0            /*  Disable destination address decrement */
#define BITM_DMA_DSTADDR_CLR_CHAN            (_ADI_MSK(0x00FFFFFF,uint32_t))    /*  Disable destination address decrement */

/* -------------------------------------------------------------------------------------------------------------------------
          DMA_REVID                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_DMA_REVID_VALUE                  0            /*  DMA Controller revision ID */
#define BITM_DMA_REVID_VALUE                 (_ADI_MSK(0x000000FF,uint32_t))    /*  DMA Controller revision ID */


/* ============================================================================================================================
        Flash Controller
   ============================================================================================================================ */

/* ============================================================================================================================
        FLCC0
   ============================================================================================================================ */
#define REG_FLCC0_STAT                       0x40018000            /*  FLCC0 Status */
#define REG_FLCC0_IEN                        0x40018004            /*  FLCC0 Interrupt Enable */
#define REG_FLCC0_CMD                        0x40018008            /*  FLCC0 Command */
#define REG_FLCC0_KH_ADDR                    0x4001800C            /*  FLCC0 WRITE address */
#define REG_FLCC0_KH_DATA0                   0x40018010            /*  FLCC0 WRITE lower data */
#define REG_FLCC0_KH_DATA1                   0x40018014            /*  FLCC0 WRITE upper data */
#define REG_FLCC0_PAGE_ADDR0                 0x40018018            /*  FLCC0 Lower page address */
#define REG_FLCC0_PAGE_ADDR1                 0x4001801C            /*  FLCC0 Upper page address */
#define REG_FLCC0_KEY                        0x40018020            /*  FLCC0 Key */
#define REG_FLCC0_WR_ABORT_ADDR              0x40018024            /*  FLCC0 Write abort address */
#define REG_FLCC0_WRPROT                     0x40018028            /*  FLCC0 Write protection */
#define REG_FLCC0_SIGNATURE                  0x4001802C            /*  FLCC0 Signature */
#define REG_FLCC0_UCFG                       0x40018030            /*  FLCC0 User configuration */
#define REG_FLCC0_TIME_PARAM0                0x40018034            /*  FLCC0 Time parameter 0 */
#define REG_FLCC0_TIME_PARAM1                0x40018038            /*  FLCC0 Time parameter 1 */
#define REG_FLCC0_ABORT_EN_LO                0x4001803C            /*  FLCC0 IRQ Abort Enable (lower bits) */
#define REG_FLCC0_ABORT_EN_HI                0x40018040            /*  FLCC0 IRQ Abort Enable (upper bits) */
#define REG_FLCC0_ECC_CFG                    0x40018044            /*  FLCC0 ECC Config */
#define REG_FLCC0_ECC_ADDR                   0x40018048            /*  FLCC0 ECC Status (address) */

/* ============================================================================================================================
        FLCC Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_STAT                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_STAT_CACHESRAMPERR         29            /*  SRAM parity errors in Cache Controller */
#define BITP_FLCC_STAT_ECCDCODE              27            /*  DCode AHB Bus Error ECC status */
#define BITP_FLCC_STAT_ECCICODE              25            /*  ICode AHB Bus Error ECC status */
#define BITP_FLCC_STAT_ECCERRCNT             17            /*  ECC correction counter */
#define BITP_FLCC_STAT_ECCINFOSIGN           15            /*  ECC status of flash initialization */
#define BITP_FLCC_STAT_INIT                  14            /*  Flash controller initialization in progress */
#define BITP_FLCC_STAT_SIGNERR               13            /*  Signature check failure during initialization */
#define BITP_FLCC_STAT_OVERLAP               11            /*  Overlapping Command */
#define BITP_FLCC_STAT_ECCRDERR               9            /*  ECC IRQ cause */
#define BITP_FLCC_STAT_ECCERRCMD              7            /*  ECC errors detected during user issued SIGN command */
#define BITP_FLCC_STAT_SLEEPING               6            /*  Flash array is in low power (sleep) mode */
#define BITP_FLCC_STAT_CMDFAIL                4            /*  Provides information on command failures */
#define BITP_FLCC_STAT_WRALCOMP               3            /*  Write almost complete */
#define BITP_FLCC_STAT_CMDCOMP                2            /*  Command complete */
#define BITP_FLCC_STAT_WRCLOSE                1            /*  WRITE registers are closed */
#define BITP_FLCC_STAT_CMDBUSY                0            /*  Command busy */
#define BITM_FLCC_STAT_CACHESRAMPERR         (_ADI_MSK(0x20000000,uint32_t))    /*  SRAM parity errors in Cache Controller */

#define BITM_FLCC_STAT_ECCDCODE              (_ADI_MSK(0x18000000,uint32_t))    /*  DCode AHB Bus Error ECC status */

#define BITM_FLCC_STAT_ECCICODE              (_ADI_MSK(0x06000000,uint32_t))    /*  ICode AHB Bus Error ECC status */
#define BITM_FLCC_STAT_ECCERRCNT             (_ADI_MSK(0x000E0000,uint32_t))    /*  ECC correction counter */

#define BITM_FLCC_STAT_ECCINFOSIGN           (_ADI_MSK(0x00018000,uint32_t))    /*  ECC status of flash initialization */
#define BITM_FLCC_STAT_INIT                  (_ADI_MSK(0x00004000,uint32_t))    /*  Flash controller initialization in progress */
#define BITM_FLCC_STAT_SIGNERR               (_ADI_MSK(0x00002000,uint32_t))    /*  Signature check failure during initialization */
#define BITM_FLCC_STAT_OVERLAP               (_ADI_MSK(0x00000800,uint32_t))    /*  Overlapping Command */

#define BITM_FLCC_STAT_ECCRDERR              (_ADI_MSK(0x00000600,uint32_t))    /*  ECC IRQ cause */

#define BITM_FLCC_STAT_ECCERRCMD             (_ADI_MSK(0x00000180,uint32_t))    /*  ECC errors detected during user issued SIGN command */
#define BITM_FLCC_STAT_SLEEPING              (_ADI_MSK(0x00000040,uint32_t))    /*  Flash array is in low power (sleep) mode */

#define BITM_FLCC_STAT_CMDFAIL               (_ADI_MSK(0x00000030,uint32_t))    /*  Provides information on command failures */
#define BITM_FLCC_STAT_WRALCOMP              (_ADI_MSK(0x00000008,uint32_t))    /*  Write almost complete */
#define BITM_FLCC_STAT_CMDCOMP               (_ADI_MSK(0x00000004,uint32_t))    /*  Command complete */
#define BITM_FLCC_STAT_WRCLOSE               (_ADI_MSK(0x00000002,uint32_t))    /*  WRITE registers are closed */
#define BITM_FLCC_STAT_CMDBUSY               (_ADI_MSK(0x00000001,uint32_t))    /*  Command busy */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_IEN                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_IEN_ECC_ERROR               6            /*  Control whether to generate bus errors, interrupts, or neither in response to 2-bit ECC Error events */
#define BITP_FLCC_IEN_ECC_CORRECT             4            /*  Control whether to generate bus errors, interrupts, or neither in response to 1-bit ECC Correction events */
#define BITP_FLCC_IEN_CMDFAIL                 2            /*  Command fail interrupt enable */
#define BITP_FLCC_IEN_WRALCMPLT               1            /*  Write almost complete interrupt enable */
#define BITP_FLCC_IEN_CMDCMPLT                0            /*  Command complete interrupt enable */

#define BITM_FLCC_IEN_ECC_ERROR              (_ADI_MSK(0x000000C0,uint32_t))    /*  Control whether to generate bus errors, interrupts, or neither in response to 2-bit ECC Error events */

#define BITM_FLCC_IEN_ECC_CORRECT            (_ADI_MSK(0x00000030,uint32_t))    /*  Control whether to generate bus errors, interrupts, or neither in response to 1-bit ECC Correction events */
#define BITM_FLCC_IEN_CMDFAIL                (_ADI_MSK(0x00000004,uint32_t))    /*  Command fail interrupt enable */
#define BITM_FLCC_IEN_WRALCMPLT              (_ADI_MSK(0x00000002,uint32_t))    /*  Write almost complete interrupt enable */
#define BITM_FLCC_IEN_CMDCMPLT               (_ADI_MSK(0x00000001,uint32_t))    /*  Command complete interrupt enable */
#define ENUM_FLCC_IEN_NONE_ERR               (_ADI_MSK(0x00000000,uint32_t))    /*  ECC_ERROR: Do not generate a response to ECC events */
#define ENUM_FLCC_IEN_BUS_ERR_ERR            (_ADI_MSK(0x00000040,uint32_t))    /*  ECC_ERROR: Generate Bus Errors in response to ECC events */
#define ENUM_FLCC_IEN_IRQ_ERR                (_ADI_MSK(0x00000080,uint32_t))    /*  ECC_ERROR: Generate IRQs in response to ECC events */
#define ENUM_FLCC_IEN_NONE_COR               (_ADI_MSK(0x00000000,uint32_t))    /*  ECC_CORRECT: Do not generate a response to ECC events */
#define ENUM_FLCC_IEN_BUS_ERR_COR            (_ADI_MSK(0x00000010,uint32_t))    /*  ECC_CORRECT: Generate Bus Errors in response to ECC events */
#define ENUM_FLCC_IEN_IRQ_COR                (_ADI_MSK(0x00000020,uint32_t))    /*  ECC_CORRECT: Generate IRQs in response to ECC events */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_CMD                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_CMD_VALUE                   0            /*  Commands */

#define BITM_FLCC_CMD_VALUE                  (_ADI_MSK(0x0000000F,uint32_t))    /*  Commands */
#define ENUM_FLCC_CMD_IDLE                   (_ADI_MSK(0x00000000,uint32_t))    /*  VALUE: IDLE */
#define ENUM_FLCC_CMD_ABORT                  (_ADI_MSK(0x00000001,uint32_t))    /*  VALUE: ABORT */
#define ENUM_FLCC_CMD_SLEEP                  (_ADI_MSK(0x00000002,uint32_t))    /*  VALUE: Requests flash to enter Sleep mode */
#define ENUM_FLCC_CMD_SIGN                   (_ADI_MSK(0x00000003,uint32_t))    /*  VALUE: SIGN */
#define ENUM_FLCC_CMD_WRITE                  (_ADI_MSK(0x00000004,uint32_t))    /*  VALUE: WRITE */
#define ENUM_FLCC_CMD_BLANK_CHECK            (_ADI_MSK(0x00000005,uint32_t))    /*  VALUE: Checks all of User Space; fails if any bits in user space are cleared */
#define ENUM_FLCC_CMD_ERASEPAGE              (_ADI_MSK(0x00000006,uint32_t))    /*  VALUE: ERASEPAGE */
#define ENUM_FLCC_CMD_MASSERASE              (_ADI_MSK(0x00000007,uint32_t))    /*  VALUE: MASSERASE */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_KH_ADDR                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_KH_ADDR_VALUE               3
#define BITM_FLCC_KH_ADDR_VALUE              (_ADI_MSK(0x0007FFF8,uint32_t))

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_PAGE_ADDR0                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_PAGE_ADDR0_VALUE           10
#define BITM_FLCC_PAGE_ADDR0_VALUE           (_ADI_MSK(0x0007FC00,uint32_t))

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_PAGE_ADDR1                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_PAGE_ADDR1_VALUE           10
#define BITM_FLCC_PAGE_ADDR1_VALUE           (_ADI_MSK(0x0007FC00,uint32_t))

#define ENUM_FLCC_KEY_USERKEY                (_ADI_MSK(0x676C7565,uint32_t))    /*  VALUE: USERKEY */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_UCFG                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_UCFG_AUTOINCEN              1            /*  Auto address increment for Key hole access. */
#define BITP_FLCC_UCFG_KHDMAEN                0            /*  Key-Hole DMA enable. */
#define BITM_FLCC_UCFG_AUTOINCEN             (_ADI_MSK(0x00000002,uint32_t))    /*  Auto address increment for Key hole access. */
#define BITM_FLCC_UCFG_KHDMAEN               (_ADI_MSK(0x00000001,uint32_t))    /*  Key-Hole DMA enable. */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_TIME_PARAM0                     Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_TIME_PARAM0_TNVH1          28            /*  NVSTR Hold time during Mass Erase */
#define BITP_FLCC_TIME_PARAM0_TERASE         24            /*  Erase Time */
#define BITP_FLCC_TIME_PARAM0_TRCV           20            /*  Recovery time */
#define BITP_FLCC_TIME_PARAM0_TNVH           16            /*  NVSTR Hold time */
#define BITP_FLCC_TIME_PARAM0_TPROG          12            /*  Program time */
#define BITP_FLCC_TIME_PARAM0_TPGS            8            /*  NVSTR to Program setup time */
#define BITP_FLCC_TIME_PARAM0_TNVS            4            /*  PROG/ERASE to NVSTR setup time */
#define BITP_FLCC_TIME_PARAM0_DIVREFCLK       0            /*  Divide Reference Clock (by 2) */
#define BITM_FLCC_TIME_PARAM0_TNVH1          (_ADI_MSK(0xF0000000,uint32_t))    /*  NVSTR Hold time during Mass Erase */
#define BITM_FLCC_TIME_PARAM0_TERASE         (_ADI_MSK(0x0F000000,uint32_t))    /*  Erase Time */
#define BITM_FLCC_TIME_PARAM0_TRCV           (_ADI_MSK(0x00F00000,uint32_t))    /*  Recovery time */
#define BITM_FLCC_TIME_PARAM0_TNVH           (_ADI_MSK(0x000F0000,uint32_t))    /*  NVSTR Hold time */
#define BITM_FLCC_TIME_PARAM0_TPROG          (_ADI_MSK(0x0000F000,uint32_t))    /*  Program time */
#define BITM_FLCC_TIME_PARAM0_TPGS           (_ADI_MSK(0x00000F00,uint32_t))    /*  NVSTR to Program setup time */
#define BITM_FLCC_TIME_PARAM0_TNVS           (_ADI_MSK(0x000000F0,uint32_t))    /*  PROG/ERASE to NVSTR setup time */
#define BITM_FLCC_TIME_PARAM0_DIVREFCLK      (_ADI_MSK(0x00000001,uint32_t))    /*  Divide Reference Clock (by 2) */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_TIME_PARAM1                     Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_TIME_PARAM1_TWK             0            /*  Wake up time */
#define BITM_FLCC_TIME_PARAM1_TWK            (_ADI_MSK(0x0000000F,uint32_t))    /*  Wake up time */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_ECC_CFG                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_ECC_CFG_PTR                 8            /*  ECC start page pointer (user should write bits [31:8] of the start page address into bits [31:8] of this register) */
#define BITP_FLCC_ECC_CFG_INFOEN              1            /*  Info space ECC Enable bit */
#define BITP_FLCC_ECC_CFG_EN                  0            /*  ECC Enable */
#define BITM_FLCC_ECC_CFG_PTR                (_ADI_MSK(0xFFFFFF00,uint32_t))    /*  ECC start page pointer (user should write bits [31:8] of the start page address into bits [31:8] of this register) */
#define BITM_FLCC_ECC_CFG_INFOEN             (_ADI_MSK(0x00000002,uint32_t))    /*  Info space ECC Enable bit */
#define BITM_FLCC_ECC_CFG_EN                 (_ADI_MSK(0x00000001,uint32_t))    /*  ECC Enable */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_ECC_ADDR                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_ECC_ADDR_VALUE              0
#define BITM_FLCC_ECC_ADDR_VALUE             (_ADI_MSK(0x0007FFFF,uint32_t))


/* ============================================================================================================================
        Cache Controller
   ============================================================================================================================ */

/* ============================================================================================================================
        FLCC0_CACHE
   ============================================================================================================================ */
#define REG_FLCC0_CACHE_STAT                 0x40018058            /*  FLCC0_CACHE Cache Status register */
#define REG_FLCC0_CACHE_SETUP                0x4001805C            /*  FLCC0_CACHE Cache Setup register */
#define REG_FLCC0_CACHE_KEY                  0x40018060            /*  FLCC0_CACHE Cache Key register */

/* ============================================================================================================================
        FLCC_CACHE Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_CACHE_STAT                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_CACHE_STAT_DCLCK           17            /*  This bit is set when D-Cache is locked and cleared when D-cache is unlocked. */
#define BITP_FLCC_CACHE_STAT_DCEN            16            /*  If this bit is set then D-Cache is enabled and when cleared D-Cache is disabled. */
#define BITP_FLCC_CACHE_STAT_ICLCK            1            /*  This bit is set when I-Cache is locked and cleared when I-cache is unlocked. */
#define BITP_FLCC_CACHE_STAT_ICEN             0            /*  If this bit is set then I-Cache is enabled and when cleared I-Cache is disabled. */
#define BITM_FLCC_CACHE_STAT_DCLCK           (_ADI_MSK(0x00020000,uint32_t))    /*  This bit is set when D-Cache is locked and cleared when D-cache is unlocked. */
#define BITM_FLCC_CACHE_STAT_DCEN            (_ADI_MSK(0x00010000,uint32_t))    /*  If this bit is set then D-Cache is enabled and when cleared D-Cache is disabled. */
#define BITM_FLCC_CACHE_STAT_ICLCK           (_ADI_MSK(0x00000002,uint32_t))    /*  This bit is set when I-Cache is locked and cleared when I-cache is unlocked. */
#define BITM_FLCC_CACHE_STAT_ICEN            (_ADI_MSK(0x00000001,uint32_t))    /*  If this bit is set then I-Cache is enabled and when cleared I-Cache is disabled. */

/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_CACHE_SETUP                     Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_CACHE_SETUP_LCKDC          17            /*  If this bit is set, then D-cache contents are locked. Any new misses won't be replaced in D-Cache. */
#define BITP_FLCC_CACHE_SETUP_DCEN           16            /*  If this bit set, then D-Cache is enabled for AHB accesses. If 0, then D-cache is disabled, and all AHB accesses will be via Flash memory. */
#define BITP_FLCC_CACHE_SETUP_LCKIC           1            /*  If this bit is set, then I-cache contents are locked. Any new misses won't be replaced in I-Cache. */
#define BITP_FLCC_CACHE_SETUP_ICEN            0            /*  If this bit set, then I-Cache is enabled for AHB accesses. If 0, then I-cache is disabled, and all AHB accesses will be via Flash memory. This bit is cleared when Init bit is set. */
#define BITM_FLCC_CACHE_SETUP_LCKDC          (_ADI_MSK(0x00020000,uint32_t))    /*  If this bit is set, then D-cache contents are locked. Any new misses won't be replaced in D-Cache. */
#define BITM_FLCC_CACHE_SETUP_DCEN           (_ADI_MSK(0x00010000,uint32_t))    /*  If this bit set, then D-Cache is enabled for AHB accesses. If 0, then D-cache is disabled, and all AHB accesses will be via Flash memory. */
#define BITM_FLCC_CACHE_SETUP_LCKIC          (_ADI_MSK(0x00000002,uint32_t))    /*  If this bit is set, then I-cache contents are locked. Any new misses won't be replaced in I-Cache. */
#define BITM_FLCC_CACHE_SETUP_ICEN           (_ADI_MSK(0x00000001,uint32_t))    /*  If this bit set, then I-Cache is enabled for AHB accesses. If 0, then I-cache is disabled, and all AHB accesses will be via Flash memory. This bit is cleared when Init bit is set. */


/* ============================================================================================================================
        DFT Controller
   ============================================================================================================================ */

/* ============================================================================================================================
        FLCC0_DFT
   ============================================================================================================================ */
#define REG_FLCC0_DFT_CMD                    0x40018094            /*  FLCC0_DFT Command register */

/* ============================================================================================================================
        FLCC_DFT Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          FLCC_DFT_CMD                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_FLCC_DFT_CMD_VALUE               0            /*  Asserted by user code to select the next command to be executed (may be written at any time) */

#define BITM_FLCC_DFT_CMD_VALUE              (_ADI_MSK(0x00000007,uint32_t))    /*  Asserted by user code to select the next command to be executed (may be written at any time) */
#define ENUM_FLCC_DFT_CMD_IDLE_CMD           (_ADI_MSK(0x00000000,uint32_t))    /*  VALUE: Idle command */
#define ENUM_FLCC_DFT_CMD_READ_CMD           (_ADI_MSK(0x00000001,uint32_t))    /*  VALUE: Read command */
#define ENUM_FLCC_DFT_CMD_PROG_CMD           (_ADI_MSK(0x00000002,uint32_t))    /*  VALUE: Program command */
#define ENUM_FLCC_DFT_CMD_ERASE_CMD          (_ADI_MSK(0x00000003,uint32_t))    /*  VALUE: Erase command */
#define ENUM_FLCC_DFT_CMD_TMRSET_CMD         (_ADI_MSK(0x00000004,uint32_t))    /*  VALUE: Test Mode Register Set command (note: this command provides Erase Reference Cell functionality) */


/* ============================================================================================================================
        Flash Controller
   ============================================================================================================================ */

/* ============================================================================================================================
        FLCC_TEST Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */

/* ============================================================================================================================
        
   ============================================================================================================================ */

/* ============================================================================================================================
        GPIO0
   ============================================================================================================================ */
#define REG_GPIO0_CFG                        0x40020000            /*  GPIO0 Port  Configuration */
#define REG_GPIO0_OEN                        0x40020004            /*  GPIO0 Port output enable */
#define REG_GPIO0_PE                         0x40020008            /*  GPIO0 Port output pullup/pulldown enable */
#define REG_GPIO0_IEN                        0x4002000C            /*  GPIO0 Port  Input Path Enable */
#define REG_GPIO0_IN                         0x40020010            /*  GPIO0 Port  registered data input */
#define REG_GPIO0_OUT                        0x40020014            /*  GPIO0 Port data output */
#define REG_GPIO0_SET                        0x40020018            /*  GPIO0 Port data out set */
#define REG_GPIO0_CLR                        0x4002001C            /*  GPIO0 Port data out clear */
#define REG_GPIO0_TGL                        0x40020020            /*  GPIO0 Port  pin toggle */
#define REG_GPIO0_POL                        0x40020024            /*  GPIO0 Port interrupt polarity */
#define REG_GPIO0_IENA                       0x40020028            /*  GPIO0 Port  interrupt A enable */
#define REG_GPIO0_IENB                       0x4002002C            /*  GPIO0 Port interrupt B enable */
#define REG_GPIO0_INT                        0x40020030            /*  GPIO0 Port interrupt Status */
#define REG_GPIO0_DS                         0x40020034            /*  GPIO0 Port  drive strength select */

/* ============================================================================================================================
        GPIO1
   ============================================================================================================================ */
#define REG_GPIO1_CFG                        0x40020040            /*  GPIO1 Port  Configuration */
#define REG_GPIO1_OEN                        0x40020044            /*  GPIO1 Port output enable */
#define REG_GPIO1_PE                         0x40020048            /*  GPIO1 Port output pullup/pulldown enable */
#define REG_GPIO1_IEN                        0x4002004C            /*  GPIO1 Port  Input Path Enable */
#define REG_GPIO1_IN                         0x40020050            /*  GPIO1 Port  registered data input */
#define REG_GPIO1_OUT                        0x40020054            /*  GPIO1 Port data output */
#define REG_GPIO1_SET                        0x40020058            /*  GPIO1 Port data out set */
#define REG_GPIO1_CLR                        0x4002005C            /*  GPIO1 Port data out clear */
#define REG_GPIO1_TGL                        0x40020060            /*  GPIO1 Port  pin toggle */
#define REG_GPIO1_POL                        0x40020064            /*  GPIO1 Port interrupt polarity */
#define REG_GPIO1_IENA                       0x40020068            /*  GPIO1 Port  interrupt A enable */
#define REG_GPIO1_IENB                       0x4002006C            /*  GPIO1 Port interrupt B enable */
#define REG_GPIO1_INT                        0x40020070            /*  GPIO1 Port interrupt Status */
#define REG_GPIO1_DS                         0x40020074            /*  GPIO1 Port  drive strength select */

/* ============================================================================================================================
        GPIO2
   ============================================================================================================================ */
#define REG_GPIO2_CFG                        0x40020080            /*  GPIO2 Port  Configuration */
#define REG_GPIO2_OEN                        0x40020084            /*  GPIO2 Port output enable */
#define REG_GPIO2_PE                         0x40020088            /*  GPIO2 Port output pullup/pulldown enable */
#define REG_GPIO2_IEN                        0x4002008C            /*  GPIO2 Port  Input Path Enable */
#define REG_GPIO2_IN                         0x40020090            /*  GPIO2 Port  registered data input */
#define REG_GPIO2_OUT                        0x40020094            /*  GPIO2 Port data output */
#define REG_GPIO2_SET                        0x40020098            /*  GPIO2 Port data out set */
#define REG_GPIO2_CLR                        0x4002009C            /*  GPIO2 Port data out clear */
#define REG_GPIO2_TGL                        0x400200A0            /*  GPIO2 Port  pin toggle */
#define REG_GPIO2_POL                        0x400200A4            /*  GPIO2 Port interrupt polarity */
#define REG_GPIO2_IENA                       0x400200A8            /*  GPIO2 Port  interrupt A enable */
#define REG_GPIO2_IENB                       0x400200AC            /*  GPIO2 Port interrupt B enable */
#define REG_GPIO2_INT                        0x400200B0            /*  GPIO2 Port interrupt Status */
#define REG_GPIO2_DS                         0x400200B4            /*  GPIO2 Port  drive strength select */

/* ============================================================================================================================
        GPIO Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          GPIO_CFG                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_GPIO_CFG_PIN15                  30            /*  Pin 15  configuration bits */
#define BITP_GPIO_CFG_PIN14                  28            /*  Pin 14  configuration bits */
#define BITP_GPIO_CFG_PIN13                  26            /*  Pin 13  configuration bits */
#define BITP_GPIO_CFG_PIN12                  24            /*  Pin 12  configuration bits */
#define BITP_GPIO_CFG_PIN11                  22            /*  Pin 11  configuration bits */
#define BITP_GPIO_CFG_PIN10                  20            /*  Pin 10  configuration bits */
#define BITP_GPIO_CFG_PIN09                  18            /*  Pin 9 configuration bits */
#define BITP_GPIO_CFG_PIN08                  16            /*  Pin 8 configuration bits */
#define BITP_GPIO_CFG_PIN07                  14            /*  Pin 7 configuration bits */
#define BITP_GPIO_CFG_PIN06                  12            /*  Pin 6 configuration bits */
#define BITP_GPIO_CFG_PIN05                  10            /*  Pin 5 configuration bits */
#define BITP_GPIO_CFG_PIN04                   8            /*  Pin 4 configuration bits */
#define BITP_GPIO_CFG_PIN03                   6            /*  Pin 3 configuration bits */
#define BITP_GPIO_CFG_PIN02                   4            /*  Pin 2 configuration bits */
#define BITP_GPIO_CFG_PIN01                   2            /*  Pin 1 configuration bits */
#define BITP_GPIO_CFG_PIN00                   0            /*  Pin 0 configuration bits */
#define BITM_GPIO_CFG_PIN15                  (_ADI_MSK(0xC0000000,uint32_t))    /*  Pin 15  configuration bits */
#define BITM_GPIO_CFG_PIN14                  (_ADI_MSK(0x30000000,uint32_t))    /*  Pin 14  configuration bits */
#define BITM_GPIO_CFG_PIN13                  (_ADI_MSK(0x0C000000,uint32_t))    /*  Pin 13  configuration bits */
#define BITM_GPIO_CFG_PIN12                  (_ADI_MSK(0x03000000,uint32_t))    /*  Pin 12  configuration bits */
#define BITM_GPIO_CFG_PIN11                  (_ADI_MSK(0x00C00000,uint32_t))    /*  Pin 11  configuration bits */
#define BITM_GPIO_CFG_PIN10                  (_ADI_MSK(0x00300000,uint32_t))    /*  Pin 10  configuration bits */
#define BITM_GPIO_CFG_PIN09                  (_ADI_MSK(0x000C0000,uint32_t))    /*  Pin 9 configuration bits */
#define BITM_GPIO_CFG_PIN08                  (_ADI_MSK(0x00030000,uint32_t))    /*  Pin 8 configuration bits */
#define BITM_GPIO_CFG_PIN07                  (_ADI_MSK(0x0000C000,uint32_t))    /*  Pin 7 configuration bits */
#define BITM_GPIO_CFG_PIN06                  (_ADI_MSK(0x00003000,uint32_t))    /*  Pin 6 configuration bits */
#define BITM_GPIO_CFG_PIN05                  (_ADI_MSK(0x00000C00,uint32_t))    /*  Pin 5 configuration bits */
#define BITM_GPIO_CFG_PIN04                  (_ADI_MSK(0x00000300,uint32_t))    /*  Pin 4 configuration bits */
#define BITM_GPIO_CFG_PIN03                  (_ADI_MSK(0x000000C0,uint32_t))    /*  Pin 3 configuration bits */
#define BITM_GPIO_CFG_PIN02                  (_ADI_MSK(0x00000030,uint32_t))    /*  Pin 2 configuration bits */
#define BITM_GPIO_CFG_PIN01                  (_ADI_MSK(0x0000000C,uint32_t))    /*  Pin 1 configuration bits */
#define BITM_GPIO_CFG_PIN00                  (_ADI_MSK(0x00000003,uint32_t))    /*  Pin 0 configuration bits */


/* ============================================================================================================================
        Serial Port
   ============================================================================================================================ */

/* ============================================================================================================================
        SPORT0
   ============================================================================================================================ */
#define REG_SPORT0_CTL_A                     0x40038000            /*  SPORT0 Half SPORT 'A' Control Register */
#define REG_SPORT0_DIV_A                     0x40038004            /*  SPORT0 Half SPORT 'A' Divisor Register */
#define REG_SPORT0_IEN_A                     0x40038008            /*  SPORT0 Half SPORT A's Interrupt Enable register */
#define REG_SPORT0_STAT_A                    0x4003800C            /*  SPORT0 Half SPORT A's Status register */
#define REG_SPORT0_NUMTRAN_A                 0x40038010            /*  SPORT0 Half SPORT A Number of transfers register */
#define REG_SPORT0_CNVT_A                    0x40038014            /*  SPORT0 Half SPORT A's register to be programmed with the desired CONVT pulse width */
#define REG_SPORT0_TX_A                      0x40038020            /*  SPORT0 Half SPORT 'A' Tx Buffer Register */
#define REG_SPORT0_RX_A                      0x40038028            /*  SPORT0 Half SPORT 'A' Rx Buffer Register */
#define REG_SPORT0_CTL_B                     0x40038040            /*  SPORT0 Half SPORT 'B' Control Register */
#define REG_SPORT0_DIV_B                     0x40038044            /*  SPORT0 Half SPORT 'B' Divisor Register */
#define REG_SPORT0_IEN_B                     0x40038048            /*  SPORT0 Half SPORT B's Interrupt Enable register */
#define REG_SPORT0_STAT_B                    0x4003804C            /*  SPORT0 Half SPORT B's Status register */
#define REG_SPORT0_NUMTRAN_B                 0x40038050            /*  SPORT0 Half SPORT B Number of transfers register */
#define REG_SPORT0_CNVT_B                    0x40038054            /*  SPORT0 Half SPORT B's register to be programmed with the desired CONVT pulse width */
#define REG_SPORT0_TX_B                      0x40038060            /*  SPORT0 Half SPORT 'B' Tx Buffer Register */
#define REG_SPORT0_RX_B                      0x40038068            /*  SPORT0 Half SPORT 'B' Rx Buffer Register */

/* ============================================================================================================================
        SPORT Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_CTL_A                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_CTL_A_DMAEN               26            /*  DMA Enable */
#define BITP_SPORT_CTL_A_SPTRAN              25            /*  Serial Port Transfer Direction */
#define BITP_SPORT_CTL_A_GCLKEN              21            /*  Gated Clock Enable */
#define BITP_SPORT_CTL_A_FSERRMODE           20            /*  Frame Sync Error Operation */
#define BITP_SPORT_CTL_A_PACK                18            /*  Packing Enable */
#define BITP_SPORT_CTL_A_LAFS                17            /*  Late Frame Sync */
#define BITP_SPORT_CTL_A_LFS                 16            /*  Active-Low Frame Sync */
#define BITP_SPORT_CTL_A_DIFS                15            /*  Data-Independent Frame Sync */
#define BITP_SPORT_CTL_A_IFS                 14            /*  Internal Frame Sync */
#define BITP_SPORT_CTL_A_FSR                 13            /*  Frame Sync Required */
#define BITP_SPORT_CTL_A_CKRE                12            /*  Clock Rising Edge */
#define BITP_SPORT_CTL_A_OPMODE              11            /*  Operation mode */
#define BITP_SPORT_CTL_A_ICLK                10            /*  Internal Clock */
#define BITP_SPORT_CTL_A_SLEN                 4            /*  Serial Word Length */
#define BITP_SPORT_CTL_A_LSBF                 3            /*  Least-Significant Bit First */
#define BITP_SPORT_CTL_A_CKMUXSEL             2            /*  Clock Multiplexer Select */
#define BITP_SPORT_CTL_A_FSMUXSEL             1            /*  Frame Sync Multiplexer Select */
#define BITP_SPORT_CTL_A_SPEN                 0            /*  Serial Port Enable */

#define BITM_SPORT_CTL_A_DMAEN               (_ADI_MSK(0x04000000,uint32_t))    /*  DMA Enable */

#define BITM_SPORT_CTL_A_SPTRAN              (_ADI_MSK(0x02000000,uint32_t))    /*  Serial Port Transfer Direction */

#define BITM_SPORT_CTL_A_GCLKEN              (_ADI_MSK(0x00200000,uint32_t))    /*  Gated Clock Enable */

#define BITM_SPORT_CTL_A_FSERRMODE           (_ADI_MSK(0x00100000,uint32_t))    /*  Frame Sync Error Operation */

#define BITM_SPORT_CTL_A_PACK                (_ADI_MSK(0x000C0000,uint32_t))    /*  Packing Enable */

#define BITM_SPORT_CTL_A_LAFS                (_ADI_MSK(0x00020000,uint32_t))    /*  Late Frame Sync */

#define BITM_SPORT_CTL_A_LFS                 (_ADI_MSK(0x00010000,uint32_t))    /*  Active-Low Frame Sync */

#define BITM_SPORT_CTL_A_DIFS                (_ADI_MSK(0x00008000,uint32_t))    /*  Data-Independent Frame Sync */

#define BITM_SPORT_CTL_A_IFS                 (_ADI_MSK(0x00004000,uint32_t))    /*  Internal Frame Sync */

#define BITM_SPORT_CTL_A_FSR                 (_ADI_MSK(0x00002000,uint32_t))    /*  Frame Sync Required */

#define BITM_SPORT_CTL_A_CKRE                (_ADI_MSK(0x00001000,uint32_t))    /*  Clock Rising Edge */

#define BITM_SPORT_CTL_A_OPMODE              (_ADI_MSK(0x00000800,uint32_t))    /*  Operation mode */

#define BITM_SPORT_CTL_A_ICLK                (_ADI_MSK(0x00000400,uint32_t))    /*  Internal Clock */
#define BITM_SPORT_CTL_A_SLEN                (_ADI_MSK(0x000001F0,uint32_t))    /*  Serial Word Length */

#define BITM_SPORT_CTL_A_LSBF                (_ADI_MSK(0x00000008,uint32_t))    /*  Least-Significant Bit First */

#define BITM_SPORT_CTL_A_CKMUXSEL            (_ADI_MSK(0x00000004,uint32_t))    /*  Clock Multiplexer Select */

#define BITM_SPORT_CTL_A_FSMUXSEL            (_ADI_MSK(0x00000002,uint32_t))    /*  Frame Sync Multiplexer Select */

#define BITM_SPORT_CTL_A_SPEN                (_ADI_MSK(0x00000001,uint32_t))    /*  Serial Port Enable */
#define ENUM_SPORT_CTL_A_CTL_RX              (_ADI_MSK(0x00000000,uint32_t))    /*  SPTRAN: Receive */
#define ENUM_SPORT_CTL_A_CTL_TX              (_ADI_MSK(0x02000000,uint32_t))    /*  SPTRAN: Transmit */
#define ENUM_SPORT_CTL_A_CTL_GCLK_DIS        (_ADI_MSK(0x00000000,uint32_t))    /*  GCLKEN: Disable */
#define ENUM_SPORT_CTL_A_CTL_GCLK_EN         (_ADI_MSK(0x00200000,uint32_t))    /*  GCLKEN: Enable */
#define ENUM_SPORT_CTL_A_CTL_PACK_DIS        (_ADI_MSK(0x00000000,uint32_t))    /*  PACK: Disable */
#define ENUM_SPORT_CTL_A_CTL_PACK_8BIT       (_ADI_MSK(0x00040000,uint32_t))    /*  PACK: 8-bit packing enable */
#define ENUM_SPORT_CTL_A_CTL_PACK_16BIT      (_ADI_MSK(0x00080000,uint32_t))    /*  PACK: 16-bit packing enable */
#define ENUM_SPORT_CTL_A_CTL_EARLY_FS        (_ADI_MSK(0x00000000,uint32_t))    /*  LAFS: Early frame sync */
#define ENUM_SPORT_CTL_A_CTL_LATE_FS         (_ADI_MSK(0x00020000,uint32_t))    /*  LAFS: Late frame sync */
#define ENUM_SPORT_CTL_A_CTL_FS_LO           (_ADI_MSK(0x00000000,uint32_t))    /*  LFS: Active high frame sync */
#define ENUM_SPORT_CTL_A_CTL_FS_HI           (_ADI_MSK(0x00010000,uint32_t))    /*  LFS: Active low frame sync */
#define ENUM_SPORT_CTL_A_CTL_DATA_DEP_FS     (_ADI_MSK(0x00000000,uint32_t))    /*  DIFS: Data-dependent frame sync */
#define ENUM_SPORT_CTL_A_CTL_DATA_INDP_FS    (_ADI_MSK(0x00008000,uint32_t))    /*  DIFS: Data-independent frame sync */
#define ENUM_SPORT_CTL_A_CTL_EXTERNAL_FS     (_ADI_MSK(0x00000000,uint32_t))    /*  IFS: External frame sync */
#define ENUM_SPORT_CTL_A_CTL_INTERNAL_FS     (_ADI_MSK(0x00004000,uint32_t))    /*  IFS: Internal frame sync */
#define ENUM_SPORT_CTL_A_CTL_FS_NOT_REQ      (_ADI_MSK(0x00000000,uint32_t))    /*  FSR: No frame sync required */
#define ENUM_SPORT_CTL_A_CTL_FS_REQ          (_ADI_MSK(0x00002000,uint32_t))    /*  FSR: Frame sync required */
#define ENUM_SPORT_CTL_A_CTL_CLK_FALL_EDGE   (_ADI_MSK(0x00000000,uint32_t))    /*  CKRE: Clock falling edge */
#define ENUM_SPORT_CTL_A_CTL_CLK_RISE_EDGE   (_ADI_MSK(0x00001000,uint32_t))    /*  CKRE: Clock rising edge */
#define ENUM_SPORT_CTL_A_CTL_SERIAL          (_ADI_MSK(0x00000000,uint32_t))    /*  OPMODE: DSP standard */
#define ENUM_SPORT_CTL_A_CTL_TIMER_EN_MODE   (_ADI_MSK(0x00000800,uint32_t))    /*  OPMODE: Timer_enable mode */
#define ENUM_SPORT_CTL_A_CTL_EXTERNAL_CLK    (_ADI_MSK(0x00000000,uint32_t))    /*  ICLK: External clock */
#define ENUM_SPORT_CTL_A_CTL_INTERNAL_CLK    (_ADI_MSK(0x00000400,uint32_t))    /*  ICLK: Internal clock */
#define ENUM_SPORT_CTL_A_CTL_MSB_FIRST       (_ADI_MSK(0x00000000,uint32_t))    /*  LSBF: MSB first sent/received */
#define ENUM_SPORT_CTL_A_CTL_LSB_FIRST       (_ADI_MSK(0x00000008,uint32_t))    /*  LSBF: LSB first sent/received */
#define ENUM_SPORT_CTL_A_CTL_CLK_MUX_DIS     (_ADI_MSK(0x00000000,uint32_t))    /*  CKMUXSEL: Disable serial clock multiplexing */
#define ENUM_SPORT_CTL_A_CTL_CLK_MUX_EN      (_ADI_MSK(0x00000004,uint32_t))    /*  CKMUXSEL: Enable serial clock multiplexing */
#define ENUM_SPORT_CTL_A_CTL_FS_MUX_DIS      (_ADI_MSK(0x00000000,uint32_t))    /*  FSMUXSEL: Disable frame sync multiplexing */
#define ENUM_SPORT_CTL_A_CTL_FS_MUX_EN       (_ADI_MSK(0x00000002,uint32_t))    /*  FSMUXSEL: Enable frame sync multiplexing */
#define ENUM_SPORT_CTL_A_CTL_DIS             (_ADI_MSK(0x00000000,uint32_t))    /*  SPEN: Disable */
#define ENUM_SPORT_CTL_A_CTL_EN              (_ADI_MSK(0x00000001,uint32_t))    /*  SPEN: Enable */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_DIV_A                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_DIV_A_FSDIV               16            /*  Frame Sync Divisor */
#define BITP_SPORT_DIV_A_CLKDIV               0            /*  Clock Divisor */
#define BITM_SPORT_DIV_A_FSDIV               (_ADI_MSK(0x00FF0000,uint32_t))    /*  Frame Sync Divisor */
#define BITM_SPORT_DIV_A_CLKDIV              (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Clock Divisor */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_IEN_A                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_IEN_A_SYSDATERR            4            /*  Data error for system writes or reads */
#define BITP_SPORT_IEN_A_DATA                 3            /*  Data request interrupt to the core */
#define BITP_SPORT_IEN_A_FSERRMSK             2            /*  Frame Sync Error (Interrupt) Mask */
#define BITP_SPORT_IEN_A_DERRMSK              1            /*  Data Error (Interrupt) Mask */
#define BITP_SPORT_IEN_A_TF                   0            /*  Transfer Finish Interrupt Enable */

#define BITM_SPORT_IEN_A_SYSDATERR           (_ADI_MSK(0x00000010,uint32_t))    /*  Data error for system writes or reads */

#define BITM_SPORT_IEN_A_DATA                (_ADI_MSK(0x00000008,uint32_t))    /*  Data request interrupt to the core */

#define BITM_SPORT_IEN_A_FSERRMSK            (_ADI_MSK(0x00000004,uint32_t))    /*  Frame Sync Error (Interrupt) Mask */

#define BITM_SPORT_IEN_A_DERRMSK             (_ADI_MSK(0x00000002,uint32_t))    /*  Data Error (Interrupt) Mask */

#define BITM_SPORT_IEN_A_TF                  (_ADI_MSK(0x00000001,uint32_t))    /*  Transfer Finish Interrupt Enable */
#define ENUM_SPORT_IEN_A_CTL_TXFIN_DIS       (_ADI_MSK(0x00000000,uint32_t))    /*  TF: Transfer finish Interrupt is disabled */
#define ENUM_SPORT_IEN_A_CTL_TXFIN_EN        (_ADI_MSK(0x00000001,uint32_t))    /*  TF: Transfer Finish Interrupt is Enabled */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_STAT_A                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_STAT_A_DXS                 8            /*  Data Transfer Buffer Status */
#define BITP_SPORT_STAT_A_SYSDATERR           4            /*  System Data Error Status */
#define BITP_SPORT_STAT_A_DATA                3            /*  Data Buffer status */
#define BITP_SPORT_STAT_A_FSERR               2            /*  Frame Sync Error Status */
#define BITP_SPORT_STAT_A_DERR                1            /*  Data Error Status */
#define BITP_SPORT_STAT_A_TFI                 0            /*  Transmit Finish Interrupt Status */

#define BITM_SPORT_STAT_A_DXS                (_ADI_MSK(0x00000300,uint32_t))    /*  Data Transfer Buffer Status */

#define BITM_SPORT_STAT_A_SYSDATERR          (_ADI_MSK(0x00000010,uint32_t))    /*  System Data Error Status */

#define BITM_SPORT_STAT_A_DATA               (_ADI_MSK(0x00000008,uint32_t))    /*  Data Buffer status */

#define BITM_SPORT_STAT_A_FSERR              (_ADI_MSK(0x00000004,uint32_t))    /*  Frame Sync Error Status */

#define BITM_SPORT_STAT_A_DERR               (_ADI_MSK(0x00000002,uint32_t))    /*  Data Error Status */

#define BITM_SPORT_STAT_A_TFI                (_ADI_MSK(0x00000001,uint32_t))    /*  Transmit Finish Interrupt Status */
#define ENUM_SPORT_STAT_A_CTL_EMPTY          (_ADI_MSK(0x00000000,uint32_t))    /*  DXS: Empty */
#define ENUM_SPORT_STAT_A_CTL_PART_FULL      (_ADI_MSK(0x00000200,uint32_t))    /*  DXS: Partially full */
#define ENUM_SPORT_STAT_A_CTL_FULL           (_ADI_MSK(0x00000300,uint32_t))    /*  DXS: Full */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_NUMTRAN_A                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_NUMTRAN_A_VALUE            0            /*  Number of transfers (Half SPORT A) */
#define BITM_SPORT_NUMTRAN_A_VALUE           (_ADI_MSK(0x00000FFF,uint32_t))    /*  Number of transfers (Half SPORT A) */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_CNVT_A                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_CNVT_A_CNVT2FS            16            /*  CONVT to FS duration: Half SPORT A */
#define BITP_SPORT_CNVT_A_POL                 8            /*  Polarity of the Convt signal */
#define BITP_SPORT_CNVT_A_WID                 0            /*  CONVT signal width: Half SPORT A */
#define BITM_SPORT_CNVT_A_CNVT2FS            (_ADI_MSK(0x00FF0000,uint32_t))    /*  CONVT to FS duration: Half SPORT A */

#define BITM_SPORT_CNVT_A_POL                (_ADI_MSK(0x00000100,uint32_t))    /*  Polarity of the Convt signal */
#define BITM_SPORT_CNVT_A_WID                (_ADI_MSK(0x0000000F,uint32_t))    /*  CONVT signal width: Half SPORT A */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_CTL_B                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_CTL_B_DMAEN               26            /*  DMA Enable */
#define BITP_SPORT_CTL_B_SPTRAN              25            /*  Serial Port Transfer Direction */
#define BITP_SPORT_CTL_B_GCLKEN              21            /*  Gated Clock Enable */
#define BITP_SPORT_CTL_B_FSERRMODE           20            /*  Frame Sync Error Operation */
#define BITP_SPORT_CTL_B_PACK                18            /*  Packing Enable */
#define BITP_SPORT_CTL_B_LAFS                17            /*  Late Frame Sync */
#define BITP_SPORT_CTL_B_LFS                 16            /*  Active-Low Frame Sync */
#define BITP_SPORT_CTL_B_DIFS                15            /*  Data-Independent Frame Sync */
#define BITP_SPORT_CTL_B_IFS                 14            /*  Internal Frame Sync */
#define BITP_SPORT_CTL_B_FSR                 13            /*  Frame Sync Required */
#define BITP_SPORT_CTL_B_CKRE                12            /*  Clock Rising Edge */
#define BITP_SPORT_CTL_B_OPMODE              11            /*  Operation mode */
#define BITP_SPORT_CTL_B_ICLK                10            /*  Internal Clock */
#define BITP_SPORT_CTL_B_SLEN                 4            /*  Serial Word Length */
#define BITP_SPORT_CTL_B_LSBF                 3            /*  Least-Significant Bit First */
#define BITP_SPORT_CTL_B_SPEN                 0            /*  Serial Port Enable */

#define BITM_SPORT_CTL_B_DMAEN               (_ADI_MSK(0x04000000,uint32_t))    /*  DMA Enable */

#define BITM_SPORT_CTL_B_SPTRAN              (_ADI_MSK(0x02000000,uint32_t))    /*  Serial Port Transfer Direction */

#define BITM_SPORT_CTL_B_GCLKEN              (_ADI_MSK(0x00200000,uint32_t))    /*  Gated Clock Enable */

#define BITM_SPORT_CTL_B_FSERRMODE           (_ADI_MSK(0x00100000,uint32_t))    /*  Frame Sync Error Operation */

#define BITM_SPORT_CTL_B_PACK                (_ADI_MSK(0x000C0000,uint32_t))    /*  Packing Enable */

#define BITM_SPORT_CTL_B_LAFS                (_ADI_MSK(0x00020000,uint32_t))    /*  Late Frame Sync */

#define BITM_SPORT_CTL_B_LFS                 (_ADI_MSK(0x00010000,uint32_t))    /*  Active-Low Frame Sync */

#define BITM_SPORT_CTL_B_DIFS                (_ADI_MSK(0x00008000,uint32_t))    /*  Data-Independent Frame Sync */

#define BITM_SPORT_CTL_B_IFS                 (_ADI_MSK(0x00004000,uint32_t))    /*  Internal Frame Sync */

#define BITM_SPORT_CTL_B_FSR                 (_ADI_MSK(0x00002000,uint32_t))    /*  Frame Sync Required */

#define BITM_SPORT_CTL_B_CKRE                (_ADI_MSK(0x00001000,uint32_t))    /*  Clock Rising Edge */

#define BITM_SPORT_CTL_B_OPMODE              (_ADI_MSK(0x00000800,uint32_t))    /*  Operation mode */

#define BITM_SPORT_CTL_B_ICLK                (_ADI_MSK(0x00000400,uint32_t))    /*  Internal Clock */
#define BITM_SPORT_CTL_B_SLEN                (_ADI_MSK(0x000001F0,uint32_t))    /*  Serial Word Length */

#define BITM_SPORT_CTL_B_LSBF                (_ADI_MSK(0x00000008,uint32_t))    /*  Least-Significant Bit First */

#define BITM_SPORT_CTL_B_SPEN                (_ADI_MSK(0x00000001,uint32_t))    /*  Serial Port Enable */
#define ENUM_SPORT_CTL_B_CTL_PACK_DIS        (_ADI_MSK(0x00000000,uint32_t))    /*  PACK: Disable */
#define ENUM_SPORT_CTL_B_CTL_PACK_8BIT       (_ADI_MSK(0x00040000,uint32_t))    /*  PACK: 8-bit packing enable */
#define ENUM_SPORT_CTL_B_CTL_PACK_16BIT      (_ADI_MSK(0x00080000,uint32_t))    /*  PACK: 16-bit packing enable */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_DIV_B                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_DIV_B_FSDIV               16            /*  Frame Sync Divisor */
#define BITP_SPORT_DIV_B_CLKDIV               0            /*  Clock Divisor */
#define BITM_SPORT_DIV_B_FSDIV               (_ADI_MSK(0x00FF0000,uint32_t))    /*  Frame Sync Divisor */
#define BITM_SPORT_DIV_B_CLKDIV              (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Clock Divisor */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_IEN_B                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_IEN_B_SYSDATERR            4            /*  Data error for system writes or reads */
#define BITP_SPORT_IEN_B_DATA                 3            /*  Data request interrupt to the core */
#define BITP_SPORT_IEN_B_FSERRMSK             2            /*  Frame Sync Error (Interrupt) Mask */
#define BITP_SPORT_IEN_B_DERRMSK              1            /*  Data Error (Interrupt) Mask */
#define BITP_SPORT_IEN_B_TF                   0            /*  Transmit Finish Interrupt Enable */

#define BITM_SPORT_IEN_B_SYSDATERR           (_ADI_MSK(0x00000010,uint32_t))    /*  Data error for system writes or reads */

#define BITM_SPORT_IEN_B_DATA                (_ADI_MSK(0x00000008,uint32_t))    /*  Data request interrupt to the core */

#define BITM_SPORT_IEN_B_FSERRMSK            (_ADI_MSK(0x00000004,uint32_t))    /*  Frame Sync Error (Interrupt) Mask */

#define BITM_SPORT_IEN_B_DERRMSK             (_ADI_MSK(0x00000002,uint32_t))    /*  Data Error (Interrupt) Mask */

#define BITM_SPORT_IEN_B_TF                  (_ADI_MSK(0x00000001,uint32_t))    /*  Transmit Finish Interrupt Enable */
#define ENUM_SPORT_IEN_B_CTL_TXFIN_DIS       (_ADI_MSK(0x00000000,uint32_t))    /*  TF: Transfer Finish Interrupt is disabled */
#define ENUM_SPORT_IEN_B_CTL_TXFIN_EN        (_ADI_MSK(0x00000001,uint32_t))    /*  TF: Transfer Finish Interrupt is Enabled */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_STAT_B                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_STAT_B_DXS                 8            /*  Data Transfer Buffer Status */
#define BITP_SPORT_STAT_B_SYSDATERR           4            /*  System Data Error Status */
#define BITP_SPORT_STAT_B_DATA                3            /*  Data Buffer status */
#define BITP_SPORT_STAT_B_FSERR               2            /*  Frame Sync Error Status */
#define BITP_SPORT_STAT_B_DERR                1            /*  Data Error Status */
#define BITP_SPORT_STAT_B_TFI                 0            /*  Transmit Finish Interrupt Status */

#define BITM_SPORT_STAT_B_DXS                (_ADI_MSK(0x00000300,uint32_t))    /*  Data Transfer Buffer Status */

#define BITM_SPORT_STAT_B_SYSDATERR          (_ADI_MSK(0x00000010,uint32_t))    /*  System Data Error Status */

#define BITM_SPORT_STAT_B_DATA               (_ADI_MSK(0x00000008,uint32_t))    /*  Data Buffer status */

#define BITM_SPORT_STAT_B_FSERR              (_ADI_MSK(0x00000004,uint32_t))    /*  Frame Sync Error Status */

#define BITM_SPORT_STAT_B_DERR               (_ADI_MSK(0x00000002,uint32_t))    /*  Data Error Status */

#define BITM_SPORT_STAT_B_TFI                (_ADI_MSK(0x00000001,uint32_t))    /*  Transmit Finish Interrupt Status */
#define ENUM_SPORT_STAT_B_CTL_EMPTY          (_ADI_MSK(0x00000000,uint32_t))    /*  DXS: Empty */
#define ENUM_SPORT_STAT_B_CTL_PART_FULL      (_ADI_MSK(0x00000200,uint32_t))    /*  DXS: Partially full */
#define ENUM_SPORT_STAT_B_CTL_FULL           (_ADI_MSK(0x00000300,uint32_t))    /*  DXS: Full */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_NUMTRAN_B                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_NUMTRAN_B_VALUE            0            /*  Number of transfers (Half SPORT A) */
#define BITM_SPORT_NUMTRAN_B_VALUE           (_ADI_MSK(0x00000FFF,uint32_t))    /*  Number of transfers (Half SPORT A) */

/* -------------------------------------------------------------------------------------------------------------------------
          SPORT_CNVT_B                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_SPORT_CNVT_B_CNVT2FS            16            /*  CONVT to FS duration: Half SPORT B */
#define BITP_SPORT_CNVT_B_POL                 8            /*  Polarity of the Convt signal */
#define BITP_SPORT_CNVT_B_WID                 0            /*  CONVT signal width: Half SPORT B */
#define BITM_SPORT_CNVT_B_CNVT2FS            (_ADI_MSK(0x00FF0000,uint32_t))    /*  CONVT to FS duration: Half SPORT B */

#define BITM_SPORT_CNVT_B_POL                (_ADI_MSK(0x00000100,uint32_t))    /*  Polarity of the Convt signal */
#define BITM_SPORT_CNVT_B_WID                (_ADI_MSK(0x0000000F,uint32_t))    /*  CONVT signal width: Half SPORT B */


/* ============================================================================================================================
        CRC
   ============================================================================================================================ */

/* ============================================================================================================================
        CRC0
   ============================================================================================================================ */
#define REG_CRC0_CTL                         0x40040000            /*  CRC0 CRC Control Register */
#define REG_CRC0_IPDATA                      0x40040004            /*  CRC0 Input Data Register */
#define REG_CRC0_RESULT                      0x40040008            /*  CRC0 CRC Result Register */
#define REG_CRC0_POLY                        0x4004000C            /*  CRC0 Programmable CRC Polynomial */

/* ============================================================================================================================
        CRC Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          CRC_CTL                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRC_CTL_W16SWP                   4            /*  Word16 Swap */
#define BITP_CRC_CTL_BYTMIRR                  3            /*  Byte Mirroring */
#define BITP_CRC_CTL_BITMIRR                  2            /*  Bit Mirroring */
#define BITP_CRC_CTL_LSBFIRST                 1            /*  LSB First CRC calculation enable */
#define BITP_CRC_CTL_EN                       0            /*  CRC peripheral enable */

#define BITM_CRC_CTL_W16SWP                  (_ADI_MSK(0x00000010,uint32_t))    /*  Word16 Swap */

#define BITM_CRC_CTL_BYTMIRR                 (_ADI_MSK(0x00000008,uint32_t))    /*  Byte Mirroring */

#define BITM_CRC_CTL_BITMIRR                 (_ADI_MSK(0x00000004,uint32_t))    /*  Bit Mirroring */

#define BITM_CRC_CTL_LSBFIRST                (_ADI_MSK(0x00000002,uint32_t))    /*  LSB First CRC calculation enable */

#define BITM_CRC_CTL_EN                      (_ADI_MSK(0x00000001,uint32_t))    /*  CRC peripheral enable */


/* ============================================================================================================================
        Random Number Generator
   ============================================================================================================================ */

/* ============================================================================================================================
        RNG0
   ============================================================================================================================ */
#define REG_RNG0_CTL                         0x40040400            /*  RNG0 RNG Control Register */
#define REG_RNG0_LEN                         0x40040404            /*  RNG0 RNG Sample Length Register */
#define REG_RNG0_STAT                        0x40040408            /*  RNG0 RNG Status Register */
#define REG_RNG0_DATA                        0x4004040C            /*  RNG0 RNG Data Register */
#define REG_RNG0_OSCCNT                      0x40040410            /*  RNG0 Oscillator Count */
#define REG_RNG0_OSCDIFF0                    0x40040414            /*  RNG0 Oscillator Difference */
#define REG_RNG0_OSCDIFF1                    0x40040415            /*  RNG0 Oscillator Difference */
#define REG_RNG0_OSCDIFF2                    0x40040416            /*  RNG0 Oscillator Difference */
#define REG_RNG0_OSCDIFF3                    0x40040417            /*  RNG0 Oscillator Difference */
#define REG_RNG0_OSCDIFFn(i)                 (REG_RNG0_OSCDIFF0 + ((i) * 1))
#define REG_RNG0_OSCDIFFn_COUNT              4

/* ============================================================================================================================
        RNG Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          RNG_CTL                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RNG_CTL_SINGLE                   3            /*  Generate a single number */
#define BITP_RNG_CTL_EN                       0            /*  RNG Enable */

#define BITM_RNG_CTL_SINGLE                  (_ADI_MSK(0x00000008,uint16_t))    /*  Generate a single number */

#define BITM_RNG_CTL_EN                      (_ADI_MSK(0x00000001,uint16_t))    /*  RNG Enable */
#define ENUM_RNG_CTL_WORD                    (_ADI_MSK(0x00000000,uint16_t))    /*  SINGLE: Buffer Word */
#define ENUM_RNG_CTL_SINGLE                  (_ADI_MSK(0x00000008,uint16_t))    /*  SINGLE: Single Byte */
#define ENUM_RNG_CTL_DISABLE                 (_ADI_MSK(0x00000000,uint16_t))    /*  EN: Disable the RNG */
#define ENUM_RNG_CTL_ENABLE                  (_ADI_MSK(0x00000001,uint16_t))    /*  EN: Enable the RNG */

/* -------------------------------------------------------------------------------------------------------------------------
          RNG_LEN                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RNG_LEN_PRESCALE                12            /*  Prescaler for the sample counter */
#define BITP_RNG_LEN_RELOAD                   0            /*  Reload value for the sample counter */
#define BITM_RNG_LEN_PRESCALE                (_ADI_MSK(0x0000F000,uint16_t))    /*  Prescaler for the sample counter */
#define BITM_RNG_LEN_RELOAD                  (_ADI_MSK(0x00000FFF,uint16_t))    /*  Reload value for the sample counter */

/* -------------------------------------------------------------------------------------------------------------------------
          RNG_STAT                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RNG_STAT_STUCK                   1            /*  Sampled data stuck high or low */
#define BITP_RNG_STAT_RNRDY                   0            /*  Random number ready */
#define BITM_RNG_STAT_STUCK                  (_ADI_MSK(0x00000002,uint16_t))    /*  Sampled data stuck high or low */

#define BITM_RNG_STAT_RNRDY                  (_ADI_MSK(0x00000001,uint16_t))    /*  Random number ready */

/* -------------------------------------------------------------------------------------------------------------------------
          RNG_DATA                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RNG_DATA_BUFF                    8            /*  Buffer for RNG data */
#define BITP_RNG_DATA_VALUE                   0            /*  Value of the CRC accumulator */
#define BITM_RNG_DATA_BUFF                   (_ADI_MSK(0xFFFFFF00,uint32_t))    /*  Buffer for RNG data */
#define BITM_RNG_DATA_VALUE                  (_ADI_MSK(0x000000FF,uint32_t))    /*  Value of the CRC accumulator */

/* -------------------------------------------------------------------------------------------------------------------------
          RNG_OSCCNT                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_RNG_OSCCNT_VALUE                 0            /*  Oscillator count */
#define BITM_RNG_OSCCNT_VALUE                (_ADI_MSK(0x0FFFFFFF,uint32_t))    /*  Oscillator count */


/* ============================================================================================================================
        crypto
   ============================================================================================================================ */

/* ============================================================================================================================
        CRYPT0
   ============================================================================================================================ */
#define REG_CRYPT0_CFG                       0x40044000            /*  CRYPT0 Configuration Register */
#define REG_CRYPT0_DATALEN                   0x40044004            /*  CRYPT0 Payload Data Length */
#define REG_CRYPT0_PREFIXLEN                 0x40044008            /*  CRYPT0 Authentication Data Length */
#define REG_CRYPT0_INTEN                     0x4004400C            /*  CRYPT0 Interrupt Enable Register */
#define REG_CRYPT0_STAT                      0x40044010            /*  CRYPT0 Status Register */
#define REG_CRYPT0_INBUF                     0x40044014            /*  CRYPT0 Input Buffer */
#define REG_CRYPT0_OUTBUF                    0x40044018            /*  CRYPT0 Output Buffer */
#define REG_CRYPT0_NONCE0                    0x4004401C            /*  CRYPT0 Nonce Bits [31:0] */
#define REG_CRYPT0_NONCE1                    0x40044020            /*  CRYPT0 Nonce Bits [63:32] */
#define REG_CRYPT0_NONCE2                    0x40044024            /*  CRYPT0 Nonce Bits [95:64] */
#define REG_CRYPT0_NONCE3                    0x40044028            /*  CRYPT0 Nonce Bits [127:96] */
#define REG_CRYPT0_AESKEY0                   0x4004402C            /*  CRYPT0 Key Bits[ 31:0 ] */
#define REG_CRYPT0_AESKEY1                   0x40044030            /*  CRYPT0 Key Bits [ 63:32 ] */
#define REG_CRYPT0_AESKEY2                   0x40044034            /*  CRYPT0 Key Bits [ 95:64 ] */
#define REG_CRYPT0_AESKEY3                   0x40044038            /*  CRYPT0 Key Bits [ 127:96 ] */
#define REG_CRYPT0_AESKEY4                   0x4004403C            /*  CRYPT0 Key Bits [ 159:128 ] */
#define REG_CRYPT0_AESKEY5                   0x40044040            /*  CRYPT0 Key Bits [ 191:160 ] */
#define REG_CRYPT0_AESKEY6                   0x40044044            /*  CRYPT0 Key Bits [ 223:192 ] */
#define REG_CRYPT0_AESKEY7                   0x40044048            /*  CRYPT0 Key Bits [ 255:224 ] */
#define REG_CRYPT0_CNTRINIT                  0x4004404C            /*  CRYPT0 Counter Initialization Vector */
#define REG_CRYPT0_SHAH0                     0x40044050            /*  CRYPT0 SHA Bits [ 31:0 ] */
#define REG_CRYPT0_SHAH1                     0x40044054            /*  CRYPT0 SHA Bits [ 63:32 ] */
#define REG_CRYPT0_SHAH2                     0x40044058            /*  CRYPT0 SHA Bits [ 95:64 ] */
#define REG_CRYPT0_SHAH3                     0x4004405C            /*  CRYPT0 SHA Bits [ 127:96 ] */
#define REG_CRYPT0_SHAH4                     0x40044060            /*  CRYPT0 SHA Bits [ 159:128 ] */
#define REG_CRYPT0_SHAH5                     0x40044064            /*  CRYPT0 SHA Bits [ 191:160 ] */
#define REG_CRYPT0_SHAH6                     0x40044068            /*  CRYPT0 SHA Bits [ 223:192] */
#define REG_CRYPT0_SHAH7                     0x4004406C            /*  CRYPT0 SHA Bits [ 255:224 ] */

/* ============================================================================================================================
        CRYPT Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          CRYPT_CFG                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRYPT_CFG_SHADATSRC             27            /*  Select data input to SHA block */
#define BITP_CRYPT_CFG_SHAINIT               26            /*  Restarts SHA Computation */
#define BITP_CRYPT_CFG_SHA256EN              25            /*  Enable SHA-256 Operation */
#define BITP_CRYPT_CFG_SHA224EN              24            /*  Enable SHA-224 Operation */
#define BITP_CRYPT_CFG_CMACEN                20            /*  Enable CMAC Mode Operation */
#define BITP_CRYPT_CFG_CCMEN                 19            /*  Enable CCM/CCM* Mode Operation */
#define BITP_CRYPT_CFG_CBCEN                 18            /*  Enable CBC Mode Operation */
#define BITP_CRYPT_CFG_CTREN                 17            /*  Enable CTR Mode Operation */
#define BITP_CRYPT_CFG_ECBEN                 16            /*  Enable ECB Mode Operation */
#define BITP_CRYPT_CFG_KEYLEN                 8            /*  Select Key Length for AES Cipher */
#define BITP_CRYPT_CFG_KEYSEL                 7            /*  Select Key Source */
#define BITP_CRYPT_CFG_ENDIAN                 6            /*  Endianness */
#define BITP_CRYPT_CFG_OUTFLUSH               5            /*  Output Buffer Flush */
#define BITP_CRYPT_CFG_INFLUSH                4            /*  Input Buffer Flush */
#define BITP_CRYPT_CFG_OUTDMAEN               3            /*  Enable DMA for Output Buffer */
#define BITP_CRYPT_CFG_INDMAEN                2            /*  Enable DMA for Input Buffer */
#define BITP_CRYPT_CFG_ENCR                   1            /*  Encrypt or Decrypt */
#define BITP_CRYPT_CFG_BLKEN                  0            /*  Enable bit for the Crypto Block */

#define BITM_CRYPT_CFG_SHADATSRC             (_ADI_MSK(0x08000000,uint32_t))    /*  Select data input to SHA block */
#define BITM_CRYPT_CFG_SHAINIT               (_ADI_MSK(0x04000000,uint32_t))    /*  Restarts SHA Computation */
#define BITM_CRYPT_CFG_SHA256EN              (_ADI_MSK(0x02000000,uint32_t))    /*  Enable SHA-256 Operation */
#define BITM_CRYPT_CFG_SHA224EN              (_ADI_MSK(0x01000000,uint32_t))    /*  Enable SHA-224 Operation */
#define BITM_CRYPT_CFG_CMACEN                (_ADI_MSK(0x00100000,uint32_t))    /*  Enable CMAC Mode Operation */
#define BITM_CRYPT_CFG_CCMEN                 (_ADI_MSK(0x00080000,uint32_t))    /*  Enable CCM/CCM* Mode Operation */
#define BITM_CRYPT_CFG_CBCEN                 (_ADI_MSK(0x00040000,uint32_t))    /*  Enable CBC Mode Operation */
#define BITM_CRYPT_CFG_CTREN                 (_ADI_MSK(0x00020000,uint32_t))    /*  Enable CTR Mode Operation */
#define BITM_CRYPT_CFG_ECBEN                 (_ADI_MSK(0x00010000,uint32_t))    /*  Enable ECB Mode Operation */

#define BITM_CRYPT_CFG_KEYLEN                (_ADI_MSK(0x00000300,uint32_t))    /*  Select Key Length for AES Cipher */

#define BITM_CRYPT_CFG_KEYSEL                (_ADI_MSK(0x00000080,uint32_t))    /*  Select Key Source */

#define BITM_CRYPT_CFG_ENDIAN                (_ADI_MSK(0x00000040,uint32_t))    /*  Endianness */
#define BITM_CRYPT_CFG_OUTFLUSH              (_ADI_MSK(0x00000020,uint32_t))    /*  Output Buffer Flush */
#define BITM_CRYPT_CFG_INFLUSH               (_ADI_MSK(0x00000010,uint32_t))    /*  Input Buffer Flush */

#define BITM_CRYPT_CFG_OUTDMAEN              (_ADI_MSK(0x00000008,uint32_t))    /*  Enable DMA for Output Buffer */

#define BITM_CRYPT_CFG_INDMAEN               (_ADI_MSK(0x00000004,uint32_t))    /*  Enable DMA for Input Buffer */

#define BITM_CRYPT_CFG_ENCR                  (_ADI_MSK(0x00000002,uint32_t))    /*  Encrypt or Decrypt */

#define BITM_CRYPT_CFG_BLKEN                 (_ADI_MSK(0x00000001,uint32_t))    /*  Enable bit for the Crypto Block */
#define ENUM_CRYPT_CFG_INBUF                 (_ADI_MSK(0x00000000,uint32_t))    /*  SHADATSRC: SHA takes input from input buffer */
#define ENUM_CRYPT_CFG_OPBUF                 (_ADI_MSK(0x08000000,uint32_t))    /*  SHADATSRC: SHA takes input from output buffer */
#define ENUM_CRYPT_CFG_LITTLE_ENDIAN         (_ADI_MSK(0x00000000,uint32_t))    /*  ENDIAN: Little Endian Format */
#define ENUM_CRYPT_CFG_BIG_ENDIAN            (_ADI_MSK(0x00000040,uint32_t))    /*  ENDIAN: Big Endian Format */
#define ENUM_CRYPT_CFG_DMA_DISABLE_OUTBUF    (_ADI_MSK(0x00000000,uint32_t))    /*  OUTDMAEN: Disable DMA Requesting for Output Buffer */
#define ENUM_CRYPT_CFG_DMA_ENABLE_OUTBUF     (_ADI_MSK(0x00000008,uint32_t))    /*  OUTDMAEN: Enable DMA Requesting for Output Buffer */
#define ENUM_CRYPT_CFG_DMA_DISABLE_INBUF     (_ADI_MSK(0x00000000,uint32_t))    /*  INDMAEN: Disable DMA Requesting for Input Buffer */
#define ENUM_CRYPT_CFG_DMA_ENABLE_INBUF      (_ADI_MSK(0x00000004,uint32_t))    /*  INDMAEN: Enable DMA Requesting for Input Buffer */
#define ENUM_CRYPT_CFG_ENABLE                (_ADI_MSK(0x00000000,uint32_t))    /*  BLKEN: Enable Crypto Block */
#define ENUM_CRYPT_CFG_DISABLE               (_ADI_MSK(0x00000001,uint32_t))    /*  BLKEN: Disable Crypto Block */

/* -------------------------------------------------------------------------------------------------------------------------
          CRYPT_DATALEN                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRYPT_DATALEN_VALUE              0            /*  Length of Payload Data */
#define BITM_CRYPT_DATALEN_VALUE             (_ADI_MSK(0x000FFFFF,uint32_t))    /*  Length of Payload Data */

/* -------------------------------------------------------------------------------------------------------------------------
          CRYPT_PREFIXLEN                      Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRYPT_PREFIXLEN_VALUE            0            /*  Length of Associated Data */
#define BITM_CRYPT_PREFIXLEN_VALUE           (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Length of Associated Data */

/* -------------------------------------------------------------------------------------------------------------------------
          CRYPT_INTEN                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRYPT_INTEN_SHADONEN             5            /*  Enable SHA_Done Interrupt */
#define BITP_CRYPT_INTEN_OUTUNDREN            3            /*  Enable the Output Underflow Interrupt */
#define BITP_CRYPT_INTEN_INOVREN              2            /*  Enable Input Overflow Interrupt. */
#define BITP_CRYPT_INTEN_OUTRDYEN             1            /*  Enables the Output Ready Interrupt. */
#define BITP_CRYPT_INTEN_INRDYEN              0            /*  Enable Input Ready Interrupt */
#define BITM_CRYPT_INTEN_SHADONEN            (_ADI_MSK(0x00000020,uint32_t))    /*  Enable SHA_Done Interrupt */
#define BITM_CRYPT_INTEN_OUTUNDREN           (_ADI_MSK(0x00000008,uint32_t))    /*  Enable the Output Underflow Interrupt */
#define BITM_CRYPT_INTEN_INOVREN             (_ADI_MSK(0x00000004,uint32_t))    /*  Enable Input Overflow Interrupt. */
#define BITM_CRYPT_INTEN_OUTRDYEN            (_ADI_MSK(0x00000002,uint32_t))    /*  Enables the Output Ready Interrupt. */
#define BITM_CRYPT_INTEN_INRDYEN             (_ADI_MSK(0x00000001,uint32_t))    /*  Enable Input Ready Interrupt */

/* -------------------------------------------------------------------------------------------------------------------------
          CRYPT_STAT                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRYPT_STAT_OUTWORDS             10            /*  Number of words in the Output Buffer */
#define BITP_CRYPT_STAT_INWORDS               7            /*  Number of words in the Input Buffer */
#define BITP_CRYPT_STAT_SHABUSY               6            /*  SHA Busy. Computations are ongoing */
#define BITP_CRYPT_STAT_SHADONE               5            /*  SHA Computation complete */
#define BITP_CRYPT_STAT_OUTUNDR               3            /*  Underflow Interrupt in the Output */
#define BITP_CRYPT_STAT_INOVR                 2            /*  Overflow in the input buffer. */
#define BITP_CRYPT_STAT_OUTRDY                1            /*  Output Data ready to be read */
#define BITP_CRYPT_STAT_INRDY                 0            /*  Input buffer requires more data before computation can begin. Remains set till the Buffer is filled. */
#define BITM_CRYPT_STAT_OUTWORDS             (_ADI_MSK(0x00001C00,uint32_t))    /*  Number of words in the Output Buffer */
#define BITM_CRYPT_STAT_INWORDS              (_ADI_MSK(0x00000380,uint32_t))    /*  Number of words in the Input Buffer */
#define BITM_CRYPT_STAT_SHABUSY              (_ADI_MSK(0x00000040,uint32_t))    /*  SHA Busy. Computations are ongoing */
#define BITM_CRYPT_STAT_SHADONE              (_ADI_MSK(0x00000020,uint32_t))    /*  SHA Computation complete */
#define BITM_CRYPT_STAT_OUTUNDR              (_ADI_MSK(0x00000008,uint32_t))    /*  Underflow Interrupt in the Output */
#define BITM_CRYPT_STAT_INOVR                (_ADI_MSK(0x00000004,uint32_t))    /*  Overflow in the input buffer. */
#define BITM_CRYPT_STAT_OUTRDY               (_ADI_MSK(0x00000002,uint32_t))    /*  Output Data ready to be read */
#define BITM_CRYPT_STAT_INRDY                (_ADI_MSK(0x00000001,uint32_t))    /*  Input buffer requires more data before computation can begin. Remains set till the Buffer is filled. */

/* -------------------------------------------------------------------------------------------------------------------------
          CRYPT_CNTRINIT                       Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CRYPT_CNTRINIT_VALUE             0            /*  Counter Initialization Value */
#define BITM_CRYPT_CNTRINIT_VALUE            (_ADI_MSK(0x000FFFFF,uint32_t))    /*  Counter Initialization Value */


/* ============================================================================================================================
        Power Management Registers
   ============================================================================================================================ */

/* ============================================================================================================================
        PMG0
   ============================================================================================================================ */
#define REG_PMG0_IEN                         0x4004C000            /*  PMG0 Power Supply Monitor Interrupt Enable */
#define REG_PMG0_PSM_STAT                    0x4004C004            /*  PMG0 Power supply monitor status */
#define REG_PMG0_PWRMOD                      0x4004C008            /*  PMG0 Power Mode Register */
#define REG_PMG0_PWRKEY                      0x4004C00C            /*  PMG0 Key protection for PWRMOD */
#define REG_PMG0_SHDN_STAT                   0x4004C010            /*  PMG0 SHUTDOWN Status Register */
#define REG_PMG0_SRAMRET                     0x4004C014            /*  PMG0 Control for Retention SRAM during HIBERNATE Mode */
#define REG_PMG0_RST_STAT                    0x4004C040            /*  PMG0 Reset status */
#define REG_PMG0_CTL1                        0x4004C044            /*  PMG0 HPBUCK control */

/* ============================================================================================================================
        PMG Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          PMG_IEN                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_IEN_IENBAT                  10            /*  Interrupt enable for VBAT range */
#define BITP_PMG_IEN_RANGEBAT                 8            /*  Battery Monitor Range */
#define BITP_PMG_IEN_VREGOVR                  2            /*  Enable Interrupt when VREG over-voltage: over- 1.32V */
#define BITP_PMG_IEN_VREGUNDR                 1            /*  Enable Interrupt when VREG under-voltage: below 1V */
#define BITP_PMG_IEN_VBAT                     0            /*  Enable Interrupt for VBAT */
#define BITM_PMG_IEN_IENBAT                  (_ADI_MSK(0x00000400,uint32_t))    /*  Interrupt enable for VBAT range */

#define BITM_PMG_IEN_RANGEBAT                (_ADI_MSK(0x00000300,uint32_t))    /*  Battery Monitor Range */
#define BITM_PMG_IEN_VREGOVR                 (_ADI_MSK(0x00000004,uint32_t))    /*  Enable Interrupt when VREG over-voltage: over- 1.32V */
#define BITM_PMG_IEN_VREGUNDR                (_ADI_MSK(0x00000002,uint32_t))    /*  Enable Interrupt when VREG under-voltage: below 1V */
#define BITM_PMG_IEN_VBAT                    (_ADI_MSK(0x00000001,uint32_t))    /*  Enable Interrupt for VBAT */
#define ENUM_PMG_IEN_REGION1                 (_ADI_MSK(0x00000000,uint32_t))    /*  RANGEBAT: Configure to generate interrupt if VBAT > 2.75V */
#define ENUM_PMG_IEN_REGION2                 (_ADI_MSK(0x00000100,uint32_t))    /*  RANGEBAT: Configure to generate interrupt if VBAT between 2.75V - 1.6V */
#define ENUM_PMG_IEN_REGION3                 (_ADI_MSK(0x00000200,uint32_t))    /*  RANGEBAT: Configure to generate interrupt if VBAT between 2.3V - 1.6V */
#define ENUM_PMG_IEN_NA                      (_ADI_MSK(0x00000300,uint32_t))    /*  RANGEBAT: N/A */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_PSM_STAT                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_PSM_STAT_RORANGE3           15            /*  VBAT range3 (2.3v - 1.6v) */
#define BITP_PMG_PSM_STAT_RORANGE2           14            /*  VBAT range2 (2.75v - 2.3v) */
#define BITP_PMG_PSM_STAT_RORANGE1           13            /*  VBAT range1 (> 2.75v) */
#define BITP_PMG_PSM_STAT_RANGE3             10            /*  VBAT range3 (2.3v - 1.6v) */
#define BITP_PMG_PSM_STAT_RANGE2              9            /*  VBAT range2 (2.75v - 2.3v) */
#define BITP_PMG_PSM_STAT_RANGE1              8            /*  VBAT range1 (> 2.75v) */
#define BITP_PMG_PSM_STAT_WICENACK            7            /*  WIC Enable Acknowledge from Cortex */
#define BITP_PMG_PSM_STAT_VREGOVR             2            /*  Status bit for alarm indicating Overvoltage for VREG */
#define BITP_PMG_PSM_STAT_VREGUNDR            1            /*  Status bit for Alarm indicating VREG is below 1V. */
#define BITP_PMG_PSM_STAT_VBATUNDR            0            /*  Status bit indicating an Alarm that battery is below 1.8V. */

#define BITM_PMG_PSM_STAT_RORANGE3           (_ADI_MSK(0x00008000,uint32_t))    /*  VBAT range3 (2.3v - 1.6v) */

#define BITM_PMG_PSM_STAT_RORANGE2           (_ADI_MSK(0x00004000,uint32_t))    /*  VBAT range2 (2.75v - 2.3v) */

#define BITM_PMG_PSM_STAT_RORANGE1           (_ADI_MSK(0x00002000,uint32_t))    /*  VBAT range1 (> 2.75v) */

#define BITM_PMG_PSM_STAT_RANGE3             (_ADI_MSK(0x00000400,uint32_t))    /*  VBAT range3 (2.3v - 1.6v) */

#define BITM_PMG_PSM_STAT_RANGE2             (_ADI_MSK(0x00000200,uint32_t))    /*  VBAT range2 (2.75v - 2.3v) */

#define BITM_PMG_PSM_STAT_RANGE1             (_ADI_MSK(0x00000100,uint32_t))    /*  VBAT range1 (> 2.75v) */
#define BITM_PMG_PSM_STAT_WICENACK           (_ADI_MSK(0x00000080,uint32_t))    /*  WIC Enable Acknowledge from Cortex */
#define BITM_PMG_PSM_STAT_VREGOVR            (_ADI_MSK(0x00000004,uint32_t))    /*  Status bit for alarm indicating Overvoltage for VREG */
#define BITM_PMG_PSM_STAT_VREGUNDR           (_ADI_MSK(0x00000002,uint32_t))    /*  Status bit for Alarm indicating VREG is below 1V. */
#define BITM_PMG_PSM_STAT_VBATUNDR           (_ADI_MSK(0x00000001,uint32_t))    /*  Status bit indicating an Alarm that battery is below 1.8V. */
#define ENUM_PMG_PSM_STAT_BATSTAT1           (_ADI_MSK(0x00000000,uint32_t))    /*  RORANGE1: VBAT NOT in the range specified */
#define ENUM_PMG_PSM_STAT_BATSTAT0           (_ADI_MSK(0x00002000,uint32_t))    /*  RORANGE1: VBAT in the range specified */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_PWRMOD                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_PWRMOD_MONVBATN              3            /*  Monitor VBAT during HIBERNATE Mode. Monitors VBAT by default */
#define BITP_PMG_PWRMOD_MODE                  0            /*  Power Mode Bits */
#define BITM_PMG_PWRMOD_MONVBATN             (_ADI_MSK(0x00000008,uint32_t))    /*  Monitor VBAT during HIBERNATE Mode. Monitors VBAT by default */

#define BITM_PMG_PWRMOD_MODE                 (_ADI_MSK(0x00000003,uint32_t))    /*  Power Mode Bits */
#define ENUM_PMG_PWRMOD_FLEXI                (_ADI_MSK(0x00000000,uint32_t))    /*  MODE: FLEXI Mode */
#define ENUM_PMG_PWRMOD_HIBERNATE            (_ADI_MSK(0x00000002,uint32_t))    /*  MODE: HIBERNATE Mode */
#define ENUM_PMG_PWRMOD_SHUTDOWN             (_ADI_MSK(0x00000003,uint32_t))    /*  MODE: SHUTDOWN Mode */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_PWRKEY                           Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_PWRKEY_VALUE                 0            /*  Power control key register */
#define BITM_PMG_PWRKEY_VALUE                (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Power control key register */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_SHDN_STAT                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_SHDN_STAT_RTC                3            /*  Interrupt from RTC */
#define BITP_PMG_SHDN_STAT_EXTINT2            2            /*  Interrupt from External Interrupt 2 */
#define BITP_PMG_SHDN_STAT_EXTINT1            1            /*  Interrupt from External Interrupt 1 */
#define BITP_PMG_SHDN_STAT_EXTINT0            0            /*  Interrupt from External Interrupt 0 */
#define BITM_PMG_SHDN_STAT_RTC               (_ADI_MSK(0x00000008,uint32_t))    /*  Interrupt from RTC */
#define BITM_PMG_SHDN_STAT_EXTINT2           (_ADI_MSK(0x00000004,uint32_t))    /*  Interrupt from External Interrupt 2 */
#define BITM_PMG_SHDN_STAT_EXTINT1           (_ADI_MSK(0x00000002,uint32_t))    /*  Interrupt from External Interrupt 1 */
#define BITM_PMG_SHDN_STAT_EXTINT0           (_ADI_MSK(0x00000001,uint32_t))    /*  Interrupt from External Interrupt 0 */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_SRAMRET                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_SRAMRET_BNK2EN               1            /*  Enable retention bank 2 (16kB) */
#define BITP_PMG_SRAMRET_BNK1EN               0            /*  Enable retention bank 1 (8kB) */
#define BITM_PMG_SRAMRET_BNK2EN              (_ADI_MSK(0x00000002,uint32_t))    /*  Enable retention bank 2 (16kB) */
#define BITM_PMG_SRAMRET_BNK1EN              (_ADI_MSK(0x00000001,uint32_t))    /*  Enable retention bank 1 (8kB) */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_RST_STAT                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_RST_STAT_PORSRC              4            /*  Power on reset Source (pmg_rst_src) */
#define BITP_PMG_RST_STAT_SWRST               3            /*  Software reset */
#define BITP_PMG_RST_STAT_WDRST               2            /*  Watchdog timeout */
#define BITP_PMG_RST_STAT_EXTRST              1            /*  External reset */
#define BITP_PMG_RST_STAT_POR                 0            /*  Power-on reset */

#define BITM_PMG_RST_STAT_PORSRC             (_ADI_MSK(0x00000030,uint32_t))    /*  Power on reset Source (pmg_rst_src) */
#define BITM_PMG_RST_STAT_SWRST              (_ADI_MSK(0x00000008,uint32_t))    /*  Software reset */
#define BITM_PMG_RST_STAT_WDRST              (_ADI_MSK(0x00000004,uint32_t))    /*  Watchdog timeout */
#define BITM_PMG_RST_STAT_EXTRST             (_ADI_MSK(0x00000002,uint32_t))    /*  External reset */
#define BITM_PMG_RST_STAT_POR                (_ADI_MSK(0x00000001,uint32_t))    /*  Power-on reset */
#define ENUM_PMG_RST_STAT_FAILSAFE_HV        (_ADI_MSK(0x00000000,uint32_t))    /*  PORSRC: POR triggered because VBAT drops below Fail Safe */
#define ENUM_PMG_RST_STAT_RST_VBAT           (_ADI_MSK(0x00000010,uint32_t))    /*  PORSRC: POR trigger because VBAT supply (VBAT<1.7v) */
#define ENUM_PMG_RST_STAT_RST_VREG           (_ADI_MSK(0x00000020,uint32_t))    /*  PORSRC: POR triggered because VDD supply (VDD < 1.08v) */
#define ENUM_PMG_RST_STAT_FAILSAFE_LV        (_ADI_MSK(0x00000030,uint32_t))    /*  PORSRC: POR triggered because VREG drops below Fail Safe */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_CTL1                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_CTL1_HPBUCKEN                0            /*  Enable HP Buck */
#define BITM_PMG_CTL1_HPBUCKEN               (_ADI_MSK(0x00000001,uint32_t))    /*  Enable HP Buck */


/* ============================================================================================================================
        External interrupt configuration
   ============================================================================================================================ */

/* ============================================================================================================================
        XINT0
   ============================================================================================================================ */
#define REG_XINT0_CFG0                       0x4004C080            /*  XINT0 External Interrupt configuration 0 */
#define REG_XINT0_CLR                        0x4004C090            /*  XINT0 External Interrupt clear */
#define REG_XINT0_NMICLR                     0x4004C094            /*  XINT0 Non-maskable interrupt clear */

/* ============================================================================================================================
        XINT Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          XINT_CFG0                            Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_XINT_CFG0_SIPMDE                17            /*  External Interrupt using SIP_UPDATE Mode registers */
#define BITP_XINT_CFG0_SIPEN                 16            /*  External Interrupt using SIP_UPDATE enable bit */
#define BITP_XINT_CFG0_IRQ3EN                15            /*  External Interrupt 3 enable bit */
#define BITP_XINT_CFG0_IRQ3MDE               12            /*  External Interrupt 3 Mode registers */
#define BITP_XINT_CFG0_IRQ2EN                11            /*  External Interrupt 2 Enable bit */
#define BITP_XINT_CFG0_IRQ2MDE                8            /*  External Interrupt 2 Mode registers */
#define BITP_XINT_CFG0_IRQ1EN                 7            /*  External Interrupt 1 Enable bit */
#define BITP_XINT_CFG0_IRQ1MDE                4            /*  External Interrupt 1 Mode registers */
#define BITP_XINT_CFG0_IRQ0EN                 3            /*  External Interrupt 0 Enable bit */
#define BITP_XINT_CFG0_IRQ0MDE                0            /*  External Interrupt 0 Mode registers */

#define BITM_XINT_CFG0_SIPMDE                (_ADI_MSK(0x000E0000,uint32_t))    /*  External Interrupt using SIP_UPDATE Mode registers */

#define BITM_XINT_CFG0_SIPEN                 (_ADI_MSK(0x00010000,uint32_t))    /*  External Interrupt using SIP_UPDATE enable bit */

#define BITM_XINT_CFG0_IRQ3EN                (_ADI_MSK(0x00008000,uint32_t))    /*  External Interrupt 3 enable bit */

#define BITM_XINT_CFG0_IRQ3MDE               (_ADI_MSK(0x00007000,uint32_t))    /*  External Interrupt 3 Mode registers */

#define BITM_XINT_CFG0_IRQ2EN                (_ADI_MSK(0x00000800,uint32_t))    /*  External Interrupt 2 Enable bit */

#define BITM_XINT_CFG0_IRQ2MDE               (_ADI_MSK(0x00000700,uint32_t))    /*  External Interrupt 2 Mode registers */

#define BITM_XINT_CFG0_IRQ1EN                (_ADI_MSK(0x00000080,uint32_t))    /*  External Interrupt 1 Enable bit */

#define BITM_XINT_CFG0_IRQ1MDE               (_ADI_MSK(0x00000070,uint32_t))    /*  External Interrupt 1 Mode registers */

#define BITM_XINT_CFG0_IRQ0EN                (_ADI_MSK(0x00000008,uint32_t))    /*  External Interrupt 0 Enable bit */

#define BITM_XINT_CFG0_IRQ0MDE               (_ADI_MSK(0x00000007,uint32_t))    /*  External Interrupt 0 Mode registers */

/* -------------------------------------------------------------------------------------------------------------------------
          XINT_CLR                             Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_XINT_CLR_SIPCLR                  4            /*  External interrupt Clear for SIP_UPDATE */
#define BITP_XINT_CLR_IRQ3                    3            /*  External interrupt 3 */
#define BITP_XINT_CLR_IRQ2                    2            /*  External interrupt 2 */
#define BITP_XINT_CLR_IRQ1                    1            /*  External interrupt 1 */
#define BITP_XINT_CLR_IRQ0                    0            /*  External interrupt 0 */
#define BITM_XINT_CLR_SIPCLR                 (_ADI_MSK(0x00000010,uint32_t))    /*  External interrupt Clear for SIP_UPDATE */
#define BITM_XINT_CLR_IRQ3                   (_ADI_MSK(0x00000008,uint32_t))    /*  External interrupt 3 */
#define BITM_XINT_CLR_IRQ2                   (_ADI_MSK(0x00000004,uint32_t))    /*  External interrupt 2 */
#define BITM_XINT_CLR_IRQ1                   (_ADI_MSK(0x00000002,uint32_t))    /*  External interrupt 1 */
#define BITM_XINT_CLR_IRQ0                   (_ADI_MSK(0x00000001,uint32_t))    /*  External interrupt 0 */

/* -------------------------------------------------------------------------------------------------------------------------
          XINT_NMICLR                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_XINT_NMICLR_CLR                  0            /*  NMI clear */
#define BITM_XINT_NMICLR_CLR                 (_ADI_MSK(0x00000001,uint32_t))    /*  NMI clear */


/* ============================================================================================================================
        Clocking registers
   ============================================================================================================================ */

/* ============================================================================================================================
        CLKG0_OSC
   ============================================================================================================================ */
#define REG_CLKG0_OSC_KEY                    0x4004C10C            /*  CLKG0_OSC Key Protection for OSCCTRL */
#define REG_CLKG0_OSC_CTL                    0x4004C110            /*  CLKG0_OSC Oscillator Control */

/* ============================================================================================================================
        CLKG_OSC Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_OSC_KEY                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_OSC_KEY_VALUE               0            /*  Oscillator key */
#define BITM_CLKG_OSC_KEY_VALUE              (_ADI_MSK(0x0000FFFF,uint32_t))    /*  Oscillator key */

/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_OSC_CTL                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_OSC_CTL_HFXTALOK           11            /*  Status of HFXTAL oscillator */
#define BITP_CLKG_OSC_CTL_LFXTALOK           10            /*  Status of LFXTAL oscillator */
#define BITP_CLKG_OSC_CTL_HFOSCOK             9            /*  Status of HFOSC oscillator */
#define BITP_CLKG_OSC_CTL_LFOSCOK             8            /*  Status of LFOSC oscillator */
#define BITP_CLKG_OSC_CTL_HFXTALEN            3            /*  High frequency crystal oscillator enable */
#define BITP_CLKG_OSC_CTL_LFXTALEN            2            /*  Low frequency crystal oscillator enable */
#define BITP_CLKG_OSC_CTL_HFOSCEN             1            /*  High frequency internal oscillator enable */
#define BITP_CLKG_OSC_CTL_LFCLKMUX            0            /*  32 KHz clock select mux */

#define BITM_CLKG_OSC_CTL_HFXTALOK           (_ADI_MSK(0x00000800,uint32_t))    /*  Status of HFXTAL oscillator */

#define BITM_CLKG_OSC_CTL_LFXTALOK           (_ADI_MSK(0x00000400,uint32_t))    /*  Status of LFXTAL oscillator */

#define BITM_CLKG_OSC_CTL_HFOSCOK            (_ADI_MSK(0x00000200,uint32_t))    /*  Status of HFOSC oscillator */

#define BITM_CLKG_OSC_CTL_LFOSCOK            (_ADI_MSK(0x00000100,uint32_t))    /*  Status of LFOSC oscillator */

#define BITM_CLKG_OSC_CTL_HFXTALEN           (_ADI_MSK(0x00000008,uint32_t))    /*  High frequency crystal oscillator enable */

#define BITM_CLKG_OSC_CTL_LFXTALEN           (_ADI_MSK(0x00000004,uint32_t))    /*  Low frequency crystal oscillator enable */

#define BITM_CLKG_OSC_CTL_HFOSCEN            (_ADI_MSK(0x00000002,uint32_t))    /*  High frequency internal oscillator enable */

#define BITM_CLKG_OSC_CTL_LFCLKMUX           (_ADI_MSK(0x00000001,uint32_t))    /*  32 KHz clock select mux */


/* ============================================================================================================================
        Power Management Registers
   ============================================================================================================================ */

/* ============================================================================================================================
        PMG0_TST
   ============================================================================================================================ */
#define REG_PMG0_TST_SRAM_CTL                0x4004C260            /*  PMG0_TST Control for SRAM parity and instruction SRAM */
#define REG_PMG0_TST_SRAM_INITSTAT           0x4004C264            /*  PMG0_TST Initialization Status Register */
#define REG_PMG0_TST_CLR_LATCH_GPIOS         0x4004C268            /*  PMG0_TST CLEAR GPIO AFTER SHUTDOWN MODE */
#define REG_PMG0_TST_SCRPAD_IMG              0x4004C26C            /*  PMG0_TST SCRATCH PAD IMAGE */
#define REG_PMG0_TST_SCRPAD_3V_RD            0x4004C270            /*  PMG0_TST SCRATCH PAD SAVED IN BATTERY DOMAIN */

/* ============================================================================================================================
        PMG_TST Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          PMG_TST_SRAM_CTL                     Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_TST_SRAM_CTL_INSTREN        31            /*  Enables instruction SRAM */
#define BITP_PMG_TST_SRAM_CTL_PENBNK5        21            /*  Enable parity check */
#define BITP_PMG_TST_SRAM_CTL_PENBNK4        20            /*  Enable parity check */
#define BITP_PMG_TST_SRAM_CTL_PENBNK3        19            /*  Enable parity check */
#define BITP_PMG_TST_SRAM_CTL_PENBNK2        18            /*  Enable parity check */
#define BITP_PMG_TST_SRAM_CTL_PENBNK1        17            /*  Enable parity check */
#define BITP_PMG_TST_SRAM_CTL_PENBNK0        16            /*  Enable parity check */
#define BITP_PMG_TST_SRAM_CTL_ABTINIT        15            /*  Abort current initialization. Self-cleared */
#define BITP_PMG_TST_SRAM_CTL_AUTOINIT       14            /*  Automatic initialization on wake up from Hibernate mode */
#define BITP_PMG_TST_SRAM_CTL_STARTINIT      13            /*  Write one to trigger initialization. Self-cleared */
#define BITP_PMG_TST_SRAM_CTL_BNK5EN          5            /*  Enable initialization */
#define BITP_PMG_TST_SRAM_CTL_BNK4EN          4            /*  Enable initialization */
#define BITP_PMG_TST_SRAM_CTL_BNK3EN          3            /*  Enable initialization */
#define BITP_PMG_TST_SRAM_CTL_BNK2EN          2            /*  Enable initialization */
#define BITP_PMG_TST_SRAM_CTL_BNK1EN          1            /*  Enable initialization */
#define BITP_PMG_TST_SRAM_CTL_BNK0EN          0            /*  Enable initialization */
#define BITM_PMG_TST_SRAM_CTL_INSTREN        (_ADI_MSK(0x80000000,uint32_t))    /*  Enables instruction SRAM */
#define BITM_PMG_TST_SRAM_CTL_PENBNK5        (_ADI_MSK(0x00200000,uint32_t))    /*  Enable parity check */
#define BITM_PMG_TST_SRAM_CTL_PENBNK4        (_ADI_MSK(0x00100000,uint32_t))    /*  Enable parity check */
#define BITM_PMG_TST_SRAM_CTL_PENBNK3        (_ADI_MSK(0x00080000,uint32_t))    /*  Enable parity check */
#define BITM_PMG_TST_SRAM_CTL_PENBNK2        (_ADI_MSK(0x00040000,uint32_t))    /*  Enable parity check */
#define BITM_PMG_TST_SRAM_CTL_PENBNK1        (_ADI_MSK(0x00020000,uint32_t))    /*  Enable parity check */
#define BITM_PMG_TST_SRAM_CTL_PENBNK0        (_ADI_MSK(0x00010000,uint32_t))    /*  Enable parity check */
#define BITM_PMG_TST_SRAM_CTL_ABTINIT        (_ADI_MSK(0x00008000,uint32_t))    /*  Abort current initialization. Self-cleared */
#define BITM_PMG_TST_SRAM_CTL_AUTOINIT       (_ADI_MSK(0x00004000,uint32_t))    /*  Automatic initialization on wake up from Hibernate mode */
#define BITM_PMG_TST_SRAM_CTL_STARTINIT      (_ADI_MSK(0x00002000,uint32_t))    /*  Write one to trigger initialization. Self-cleared */
#define BITM_PMG_TST_SRAM_CTL_BNK5EN         (_ADI_MSK(0x00000020,uint32_t))    /*  Enable initialization */
#define BITM_PMG_TST_SRAM_CTL_BNK4EN         (_ADI_MSK(0x00000010,uint32_t))    /*  Enable initialization */
#define BITM_PMG_TST_SRAM_CTL_BNK3EN         (_ADI_MSK(0x00000008,uint32_t))    /*  Enable initialization */
#define BITM_PMG_TST_SRAM_CTL_BNK2EN         (_ADI_MSK(0x00000004,uint32_t))    /*  Enable initialization */
#define BITM_PMG_TST_SRAM_CTL_BNK1EN         (_ADI_MSK(0x00000002,uint32_t))    /*  Enable initialization */
#define BITM_PMG_TST_SRAM_CTL_BNK0EN         (_ADI_MSK(0x00000001,uint32_t))    /*  Enable initialization */

/* -------------------------------------------------------------------------------------------------------------------------
          PMG_TST_SRAM_INITSTAT                Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PMG_TST_SRAM_INITSTAT_BNK5       5            /*  0:Not initialized; 1:Initialization completed */
#define BITP_PMG_TST_SRAM_INITSTAT_BNK4       4            /*  0:Not initialized; 1:Initialization completed */
#define BITP_PMG_TST_SRAM_INITSTAT_BNK3       3            /*  0:Not initialized; 1:Initialization completed */
#define BITP_PMG_TST_SRAM_INITSTAT_BNK2       2            /*  0:Not initialized; 1:Initialization completed */
#define BITP_PMG_TST_SRAM_INITSTAT_BNK1       1            /*  0:Not initialized; 1:Initialization completed */
#define BITP_PMG_TST_SRAM_INITSTAT_BNK0       0            /*  0:Not initialized; 1:Initialization completed */
#define BITM_PMG_TST_SRAM_INITSTAT_BNK5      (_ADI_MSK(0x00000020,uint32_t))    /*  0:Not initialized; 1:Initialization completed */
#define BITM_PMG_TST_SRAM_INITSTAT_BNK4      (_ADI_MSK(0x00000010,uint32_t))    /*  0:Not initialized; 1:Initialization completed */
#define BITM_PMG_TST_SRAM_INITSTAT_BNK3      (_ADI_MSK(0x00000008,uint32_t))    /*  0:Not initialized; 1:Initialization completed */
#define BITM_PMG_TST_SRAM_INITSTAT_BNK2      (_ADI_MSK(0x00000004,uint32_t))    /*  0:Not initialized; 1:Initialization completed */
#define BITM_PMG_TST_SRAM_INITSTAT_BNK1      (_ADI_MSK(0x00000002,uint32_t))    /*  0:Not initialized; 1:Initialization completed */
#define BITM_PMG_TST_SRAM_INITSTAT_BNK0      (_ADI_MSK(0x00000001,uint32_t))    /*  0:Not initialized; 1:Initialization completed */


/* ============================================================================================================================
        Clocking registers
   ============================================================================================================================ */

/* ============================================================================================================================
        CLKG0_CLK
   ============================================================================================================================ */
#define REG_CLKG0_CLK_CTL0                   0x4004C300            /*  CLKG0_CLK Misc clock settings */
#define REG_CLKG0_CLK_CTL1                   0x4004C304            /*  CLKG0_CLK Clock dividers */
#define REG_CLKG0_CLK_CTL3                   0x4004C30C            /*  CLKG0_CLK System PLL */
#define REG_CLKG0_CLK_CTL5                   0x4004C314            /*  CLKG0_CLK User clock gating control */
#define REG_CLKG0_CLK_STAT0                  0x4004C318            /*  CLKG0_CLK Clocking status */

/* ============================================================================================================================
        CLKG_CLK Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_CLK_CTL0                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_CLK_CTL0_HFXTALIE          15            /*  High frequency crystal interrupt enable */
#define BITP_CLKG_CLK_CTL0_LFXTALIE          14            /*  Low frequency crystal interrupt enable */
#define BITP_CLKG_CLK_CTL0_SPLLIPSEL         11            /*  SPLL source select mux */
#define BITP_CLKG_CLK_CTL0_RCLKMUX            8            /*  Flash reference clock and HPBUCK clock source mux */
#define BITP_CLKG_CLK_CTL0_CLKMUX             0            /*  Clock mux select */

#define BITM_CLKG_CLK_CTL0_HFXTALIE          (_ADI_MSK(0x00008000,uint32_t))    /*  High frequency crystal interrupt enable */

#define BITM_CLKG_CLK_CTL0_LFXTALIE          (_ADI_MSK(0x00004000,uint32_t))    /*  Low frequency crystal interrupt enable */

#define BITM_CLKG_CLK_CTL0_SPLLIPSEL         (_ADI_MSK(0x00000800,uint32_t))    /*  SPLL source select mux */

#define BITM_CLKG_CLK_CTL0_RCLKMUX           (_ADI_MSK(0x00000300,uint32_t))    /*  Flash reference clock and HPBUCK clock source mux */

#define BITM_CLKG_CLK_CTL0_CLKMUX            (_ADI_MSK(0x00000003,uint32_t))    /*  Clock mux select */

/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_CLK_CTL1                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_CLK_CTL1_ACLKDIVCNT        16            /*  ACLK Divide Count. */
#define BITP_CLKG_CLK_CTL1_PCLKDIVCNT         8            /*  PCLK divide count */
#define BITP_CLKG_CLK_CTL1_HCLKDIVCNT         0            /*  HCLK divide count */
#define BITM_CLKG_CLK_CTL1_ACLKDIVCNT        (_ADI_MSK(0x003F0000,uint32_t))    /*  ACLK Divide Count. */
#define BITM_CLKG_CLK_CTL1_PCLKDIVCNT        (_ADI_MSK(0x00003F00,uint32_t))    /*  PCLK divide count */
#define BITM_CLKG_CLK_CTL1_HCLKDIVCNT        (_ADI_MSK(0x0000003F,uint32_t))    /*  HCLK divide count */

/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_CLK_CTL3                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_CLK_CTL3_SPLLMUL2          16            /*  system PLL multiply by 2 */
#define BITP_CLKG_CLK_CTL3_SPLLMSEL          11            /*  System PLL M Divider */
#define BITP_CLKG_CLK_CTL3_SPLLIE            10            /*  System PLL interrupt enable */
#define BITP_CLKG_CLK_CTL3_SPLLEN             9            /*  System PLL enable */
#define BITP_CLKG_CLK_CTL3_SPLLDIV2           8            /*  System PLL division by 2 */
#define BITP_CLKG_CLK_CTL3_SPLLNSEL           0            /*  System PLL N multiplier */

#define BITM_CLKG_CLK_CTL3_SPLLMUL2          (_ADI_MSK(0x00010000,uint32_t))    /*  system PLL multiply by 2 */
#define BITM_CLKG_CLK_CTL3_SPLLMSEL          (_ADI_MSK(0x00007800,uint32_t))    /*  System PLL M Divider */

#define BITM_CLKG_CLK_CTL3_SPLLIE            (_ADI_MSK(0x00000400,uint32_t))    /*  System PLL interrupt enable */

#define BITM_CLKG_CLK_CTL3_SPLLEN            (_ADI_MSK(0x00000200,uint32_t))    /*  System PLL enable */

#define BITM_CLKG_CLK_CTL3_SPLLDIV2          (_ADI_MSK(0x00000100,uint32_t))    /*  System PLL division by 2 */
#define BITM_CLKG_CLK_CTL3_SPLLNSEL          (_ADI_MSK(0x0000001F,uint32_t))    /*  System PLL N multiplier */

/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_CLK_CTL5                        Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_CLK_CTL5_PERCLKOFF          5            /*  This bit is used to disable all clocks connected to all peripherals */
#define BITP_CLKG_CLK_CTL5_ACLKOFF            4            /*  ADC clock user control. */
#define BITP_CLKG_CLK_CTL5_UCLKI2COFF         3            /*  I2C clock user control */
#define BITP_CLKG_CLK_CTL5_GPTCLK2OFF         2            /*  GP timer 2 user control */
#define BITP_CLKG_CLK_CTL5_GPTCLK1OFF         1            /*  GP timer 1 user control */
#define BITP_CLKG_CLK_CTL5_GPTCLK0OFF         0            /*  GP timer 0 user control */
#define BITM_CLKG_CLK_CTL5_PERCLKOFF         (_ADI_MSK(0x00000020,uint32_t))    /*  This bit is used to disable all clocks connected to all peripherals */

#define BITM_CLKG_CLK_CTL5_ACLKOFF           (_ADI_MSK(0x00000010,uint32_t))    /*  ADC clock user control. */

#define BITM_CLKG_CLK_CTL5_UCLKI2COFF        (_ADI_MSK(0x00000008,uint32_t))    /*  I2C clock user control */

#define BITM_CLKG_CLK_CTL5_GPTCLK2OFF        (_ADI_MSK(0x00000004,uint32_t))    /*  GP timer 2 user control */

#define BITM_CLKG_CLK_CTL5_GPTCLK1OFF        (_ADI_MSK(0x00000002,uint32_t))    /*  GP timer 1 user control */

#define BITM_CLKG_CLK_CTL5_GPTCLK0OFF        (_ADI_MSK(0x00000001,uint32_t))    /*  GP timer 0 user control */

/* -------------------------------------------------------------------------------------------------------------------------
          CLKG_CLK_STAT0                       Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_CLKG_CLK_STAT0_HFXTALNOK        14            /*  HF crystal not stable */
#define BITP_CLKG_CLK_STAT0_HFXTALOK         13            /*  HF crystal stable */
#define BITP_CLKG_CLK_STAT0_HFXTAL           12            /*  HF crystal status */
#define BITP_CLKG_CLK_STAT0_LFXTALNOK        10            /*  LF crystal not stable */
#define BITP_CLKG_CLK_STAT0_LFXTALOK          9            /*  LF crystal stable */
#define BITP_CLKG_CLK_STAT0_LFXTAL            8            /*  LF crystal status */
#define BITP_CLKG_CLK_STAT0_SPLLUNLK          2            /*  System PLL unlock */
#define BITP_CLKG_CLK_STAT0_SPLLLK            1            /*  System PLL lock */
#define BITP_CLKG_CLK_STAT0_SPLL              0            /*  System PLL status */

#define BITM_CLKG_CLK_STAT0_HFXTALNOK        (_ADI_MSK(0x00004000,uint32_t))    /*  HF crystal not stable */

#define BITM_CLKG_CLK_STAT0_HFXTALOK         (_ADI_MSK(0x00002000,uint32_t))    /*  HF crystal stable */

#define BITM_CLKG_CLK_STAT0_HFXTAL           (_ADI_MSK(0x00001000,uint32_t))    /*  HF crystal status */

#define BITM_CLKG_CLK_STAT0_LFXTALNOK        (_ADI_MSK(0x00000400,uint32_t))    /*  LF crystal not stable */

#define BITM_CLKG_CLK_STAT0_LFXTALOK         (_ADI_MSK(0x00000200,uint32_t))    /*  LF crystal stable */

#define BITM_CLKG_CLK_STAT0_LFXTAL           (_ADI_MSK(0x00000100,uint32_t))    /*  LF crystal status */

#define BITM_CLKG_CLK_STAT0_SPLLUNLK         (_ADI_MSK(0x00000004,uint32_t))    /*  System PLL unlock */

#define BITM_CLKG_CLK_STAT0_SPLLLK           (_ADI_MSK(0x00000002,uint32_t))    /*  System PLL lock */

#define BITM_CLKG_CLK_STAT0_SPLL             (_ADI_MSK(0x00000001,uint32_t))    /*  System PLL status */


/* ============================================================================================================================
        Bus matrix
   ============================================================================================================================ */

/* ============================================================================================================================
        BUSM0
   ============================================================================================================================ */
#define REG_BUSM0_ARBIT0                     0x4004C800            /*  BUSM0 Arbitration Priority Configuration for FLASH and SRAM0 */
#define REG_BUSM0_ARBIT1                     0x4004C804            /*  BUSM0 Arbitration Priority Configuration for SRAM1 and SIP */
#define REG_BUSM0_ARBIT2                     0x4004C808            /*  BUSM0 Arbitration Priority Configuration for APB32 and APB16 */
#define REG_BUSM0_ARBIT3                     0x4004C80C            /*  BUSM0 Arbitration Priority Configuration for APB16 priority for core and for DMA1 */

/* ============================================================================================================================
        BUSM Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          BUSM_ARBIT0                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BUSM_ARBIT0_SRAM0_DMA0          20            /*  SRAM0 priority for DMA0 */
#define BITP_BUSM_ARBIT0_SRAM0_SBUS          18            /*  SRAM0 priority for SBUS */
#define BITP_BUSM_ARBIT0_SRAM0_DCODE         16            /*  SRAM0 priority for Dcode */
#define BITP_BUSM_ARBIT0_FLSH_DMA0            4            /*  Flash priority for DMA0 */
#define BITP_BUSM_ARBIT0_FLSH_SBUS            2            /*  Flash priority for SBUS */
#define BITP_BUSM_ARBIT0_FLSH_DCODE           0            /*  Flash priority for DCODE */
#define BITM_BUSM_ARBIT0_SRAM0_DMA0          (_ADI_MSK(0x00300000,uint32_t))    /*  SRAM0 priority for DMA0 */
#define BITM_BUSM_ARBIT0_SRAM0_SBUS          (_ADI_MSK(0x000C0000,uint32_t))    /*  SRAM0 priority for SBUS */
#define BITM_BUSM_ARBIT0_SRAM0_DCODE         (_ADI_MSK(0x00030000,uint32_t))    /*  SRAM0 priority for Dcode */
#define BITM_BUSM_ARBIT0_FLSH_DMA0           (_ADI_MSK(0x00000030,uint32_t))    /*  Flash priority for DMA0 */
#define BITM_BUSM_ARBIT0_FLSH_SBUS           (_ADI_MSK(0x0000000C,uint32_t))    /*  Flash priority for SBUS */
#define BITM_BUSM_ARBIT0_FLSH_DCODE          (_ADI_MSK(0x00000003,uint32_t))    /*  Flash priority for DCODE */

/* -------------------------------------------------------------------------------------------------------------------------
          BUSM_ARBIT1                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BUSM_ARBIT1_SIP_DMA0            20            /*  SIP priority for DMA0 */
#define BITP_BUSM_ARBIT1_SIP_SBUS            18            /*  SIP priority for SBUS */
#define BITP_BUSM_ARBIT1_SIP_DCODE           16            /*  SIP priority for DCODE */
#define BITP_BUSM_ARBIT1_SRAM1_DMA0           4            /*  SRAM1 priority for DMA0 */
#define BITP_BUSM_ARBIT1_SRAM1_SBUS           2            /*  SRAM1 priority for SBUS */
#define BITP_BUSM_ARBIT1_SRAM1_DCODE          0            /*  SRAM1 priority for Dcode */
#define BITM_BUSM_ARBIT1_SIP_DMA0            (_ADI_MSK(0x00300000,uint32_t))    /*  SIP priority for DMA0 */
#define BITM_BUSM_ARBIT1_SIP_SBUS            (_ADI_MSK(0x000C0000,uint32_t))    /*  SIP priority for SBUS */
#define BITM_BUSM_ARBIT1_SIP_DCODE           (_ADI_MSK(0x00030000,uint32_t))    /*  SIP priority for DCODE */
#define BITM_BUSM_ARBIT1_SRAM1_DMA0          (_ADI_MSK(0x00000030,uint32_t))    /*  SRAM1 priority for DMA0 */
#define BITM_BUSM_ARBIT1_SRAM1_SBUS          (_ADI_MSK(0x0000000C,uint32_t))    /*  SRAM1 priority for SBUS */
#define BITM_BUSM_ARBIT1_SRAM1_DCODE         (_ADI_MSK(0x00000003,uint32_t))    /*  SRAM1 priority for Dcode */

/* -------------------------------------------------------------------------------------------------------------------------
          BUSM_ARBIT2                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BUSM_ARBIT2_APB16_DMA0          20            /*  APB16 priority for DMA0 */
#define BITP_BUSM_ARBIT2_APB16_SBUS          18            /*  APB16 priority for SBUS */
#define BITP_BUSM_ARBIT2_APB16_DCODE         16            /*  APB16 priority for DCODE */
#define BITP_BUSM_ARBIT2_APB32_DMA0           4            /*  APB32 priority for DMA0 */
#define BITP_BUSM_ARBIT2_APB32_SBUS           2            /*  APB32 priority for SBUS */
#define BITP_BUSM_ARBIT2_APB32_DCODE          0            /*  APB32 priority for DCODE */
#define BITM_BUSM_ARBIT2_APB16_DMA0          (_ADI_MSK(0x00300000,uint32_t))    /*  APB16 priority for DMA0 */
#define BITM_BUSM_ARBIT2_APB16_SBUS          (_ADI_MSK(0x000C0000,uint32_t))    /*  APB16 priority for SBUS */
#define BITM_BUSM_ARBIT2_APB16_DCODE         (_ADI_MSK(0x00030000,uint32_t))    /*  APB16 priority for DCODE */
#define BITM_BUSM_ARBIT2_APB32_DMA0          (_ADI_MSK(0x00000030,uint32_t))    /*  APB32 priority for DMA0 */
#define BITM_BUSM_ARBIT2_APB32_SBUS          (_ADI_MSK(0x0000000C,uint32_t))    /*  APB32 priority for SBUS */
#define BITM_BUSM_ARBIT2_APB32_DCODE         (_ADI_MSK(0x00000003,uint32_t))    /*  APB32 priority for DCODE */

/* -------------------------------------------------------------------------------------------------------------------------
          BUSM_ARBIT3                          Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_BUSM_ARBIT3_APB16_4DMA_DMA1     17            /*  APB16 for dma priority for DMA1 */
#define BITP_BUSM_ARBIT3_APB16_4DMA_CORE     16            /*  APB16 for dma priority for CORE */
#define BITP_BUSM_ARBIT3_APB16_DMA1           1            /*  APB16 priority for DMA1 */
#define BITP_BUSM_ARBIT3_APB16_CORE           0            /*  APB16 priority for CORE */
#define BITM_BUSM_ARBIT3_APB16_4DMA_DMA1     (_ADI_MSK(0x00020000,uint32_t))    /*  APB16 for dma priority for DMA1 */
#define BITM_BUSM_ARBIT3_APB16_4DMA_CORE     (_ADI_MSK(0x00010000,uint32_t))    /*  APB16 for dma priority for CORE */
#define BITM_BUSM_ARBIT3_APB16_DMA1          (_ADI_MSK(0x00000002,uint32_t))    /*  APB16 priority for DMA1 */
#define BITM_BUSM_ARBIT3_APB16_CORE          (_ADI_MSK(0x00000001,uint32_t))    /*  APB16 priority for CORE */


/* ============================================================================================================================
        Parallel Test Interface
   ============================================================================================================================ */

/* ============================================================================================================================
        PTI0
   ============================================================================================================================ */
#define REG_PTI0_RST_ISR_STARTADDR           0x4004CD00            /*  PTI0 Reset ISR Start Address */
#define REG_PTI0_RST_STACK_PTR               0x4004CD04            /*  PTI0 Reset Stack Pointer */
#define REG_PTI0_CTL                         0x4004CD08            /*  PTI0 Parallel Test Interface Control Register */

/* ============================================================================================================================
        PTI Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          PTI_CTL                              Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_PTI_CTL_EN                       0
#define BITM_PTI_CTL_EN                      (_ADI_MSK(0x00000001,uint32_t))


/* ============================================================================================================================
        Cortex-M3 Interrupt Controller
   ============================================================================================================================ */

/* ============================================================================================================================
        NVIC0
   ============================================================================================================================ */
#define REG_NVIC0_INTNUM                     0xE000E004            /*  NVIC0 Interrupt Control Type */
#define REG_NVIC0_STKSTA                     0xE000E010            /*  NVIC0 Systick Control and Status */
#define REG_NVIC0_STKLD                      0xE000E014            /*  NVIC0 Systick Reload Value */
#define REG_NVIC0_STKVAL                     0xE000E018            /*  NVIC0 Systick Current Value */
#define REG_NVIC0_STKCAL                     0xE000E01C            /*  NVIC0 Systick Calibration Value */
#define REG_NVIC0_INTSETE0                   0xE000E100            /*  NVIC0 IRQ0..31 Set_Enable */
#define REG_NVIC0_INTSETE1                   0xE000E104            /*  NVIC0 IRQ32..63 Set_Enable */
#define REG_NVIC0_INTCLRE0                   0xE000E180            /*  NVIC0 IRQ0..31 Clear_Enable */
#define REG_NVIC0_INTCLRE1                   0xE000E184            /*  NVIC0 IRQ32..63 Clear_Enable */
#define REG_NVIC0_INTSETP0                   0xE000E200            /*  NVIC0 IRQ0..31 Set_Pending */
#define REG_NVIC0_INTSETP1                   0xE000E204            /*  NVIC0 IRQ32..63 Set_Pending */
#define REG_NVIC0_INTCLRP0                   0xE000E280            /*  NVIC0 IRQ0..31 Clear_Pending */
#define REG_NVIC0_INTCLRP1                   0xE000E284            /*  NVIC0 IRQ32..63 Clear_Pending */
#define REG_NVIC0_INTACT0                    0xE000E300            /*  NVIC0 IRQ0..31 Active Bit */
#define REG_NVIC0_INTACT1                    0xE000E304            /*  NVIC0 IRQ32..63 Active Bit */
#define REG_NVIC0_INTPRI0                    0xE000E400            /*  NVIC0 IRQ0..3 Priority */
#define REG_NVIC0_INTPRI1                    0xE000E404            /*  NVIC0 IRQ4..7 Priority */
#define REG_NVIC0_INTPRI2                    0xE000E408            /*  NVIC0 IRQ8..11 Priority */
#define REG_NVIC0_INTPRI3                    0xE000E40C            /*  NVIC0 IRQ12..15 Priority */
#define REG_NVIC0_INTPRI4                    0xE000E410            /*  NVIC0 IRQ16..19 Priority */
#define REG_NVIC0_INTPRI5                    0xE000E414            /*  NVIC0 IRQ20..23 Priority */
#define REG_NVIC0_INTPRI6                    0xE000E418            /*  NVIC0 IRQ24..27 Priority */
#define REG_NVIC0_INTPRI7                    0xE000E41C            /*  NVIC0 IRQ28..31 Priority */
#define REG_NVIC0_INTPRI8                    0xE000E420            /*  NVIC0 IRQ32..35 Priority */
#define REG_NVIC0_INTPRI9                    0xE000E424            /*  NVIC0 IRQ36..39 Priority */
#define REG_NVIC0_INTPRI10                   0xE000E428            /*  NVIC0 IRQ40..43 Priority */
#define REG_NVIC0_INTCPID                    0xE000ED00            /*  NVIC0 CPUID Base */
#define REG_NVIC0_INTSTA                     0xE000ED04            /*  NVIC0 Interrupt Control State */
#define REG_NVIC0_INTVEC                     0xE000ED08            /*  NVIC0 Vector Table Offset */
#define REG_NVIC0_INTAIRC                    0xE000ED0C            /*  NVIC0 Application Interrupt/Reset Control */
#define REG_NVIC0_INTCON0                    0xE000ED10            /*  NVIC0 System Control */
#define REG_NVIC0_INTCON1                    0xE000ED14            /*  NVIC0 Configuration Control */
#define REG_NVIC0_INTSHPRIO0                 0xE000ED18            /*  NVIC0 System Handlers 4-7 Priority */
#define REG_NVIC0_INTSHPRIO1                 0xE000ED1C            /*  NVIC0 System Handlers 8-11 Priority */
#define REG_NVIC0_INTSHPRIO3                 0xE000ED20            /*  NVIC0 System Handlers 12-15 Priority */
#define REG_NVIC0_INTSHCSR                   0xE000ED24            /*  NVIC0 System Handler Control and State */
#define REG_NVIC0_INTCFSR                    0xE000ED28            /*  NVIC0 Configurable Fault Status */
#define REG_NVIC0_INTHFSR                    0xE000ED2C            /*  NVIC0 Hard Fault Status */
#define REG_NVIC0_INTDFSR                    0xE000ED30            /*  NVIC0 Debug Fault Status */
#define REG_NVIC0_INTMMAR                    0xE000ED34            /*  NVIC0 Mem Manage Address */
#define REG_NVIC0_INTBFAR                    0xE000ED38            /*  NVIC0 Bus Fault Address */
#define REG_NVIC0_INTAFSR                    0xE000ED3C            /*  NVIC0 Auxiliary Fault Status */
#define REG_NVIC0_INTPFR0                    0xE000ED40            /*  NVIC0 Processor Feature Register 0 */
#define REG_NVIC0_INTPFR1                    0xE000ED44            /*  NVIC0 Processor Feature Register 1 */
#define REG_NVIC0_INTDFR0                    0xE000ED48            /*  NVIC0 Debug Feature Register 0 */
#define REG_NVIC0_INTAFR0                    0xE000ED4C            /*  NVIC0 Auxiliary Feature Register 0 */
#define REG_NVIC0_INTMMFR0                   0xE000ED50            /*  NVIC0 Memory Model Feature Register 0 */
#define REG_NVIC0_INTMMFR1                   0xE000ED54            /*  NVIC0 Memory Model Feature Register 1 */
#define REG_NVIC0_INTMMFR2                   0xE000ED58            /*  NVIC0 Memory Model Feature Register 2 */
#define REG_NVIC0_INTMMFR3                   0xE000ED5C            /*  NVIC0 Memory Model Feature Register 3 */
#define REG_NVIC0_INTISAR0                   0xE000ED60            /*  NVIC0 ISA Feature Register 0 */
#define REG_NVIC0_INTISAR1                   0xE000ED64            /*  NVIC0 ISA Feature Register 1 */
#define REG_NVIC0_INTISAR2                   0xE000ED68            /*  NVIC0 ISA Feature Register 2 */
#define REG_NVIC0_INTISAR3                   0xE000ED6C            /*  NVIC0 ISA Feature Register 3 */
#define REG_NVIC0_INTISAR4                   0xE000ED70            /*  NVIC0 ISA Feature Register 4 */
#define REG_NVIC0_INTTRGI                    0xE000EF00            /*  NVIC0 Software Trigger Interrupt Register */
#define REG_NVIC0_INTPID4                    0xE000EFD0            /*  NVIC0 Peripheral Identification Register 4 */
#define REG_NVIC0_INTPID5                    0xE000EFD4            /*  NVIC0 Peripheral Identification Register 5 */
#define REG_NVIC0_INTPID6                    0xE000EFD8            /*  NVIC0 Peripheral Identification Register 6 */
#define REG_NVIC0_INTPID7                    0xE000EFDC            /*  NVIC0 Peripheral Identification Register 7 */
#define REG_NVIC0_INTPID0                    0xE000EFE0            /*  NVIC0 Peripheral Identification Bits7:0 */
#define REG_NVIC0_INTPID1                    0xE000EFE4            /*  NVIC0 Peripheral Identification Bits15:8 */
#define REG_NVIC0_INTPID2                    0xE000EFE8            /*  NVIC0 Peripheral Identification Bits16:23 */
#define REG_NVIC0_INTPID3                    0xE000EFEC            /*  NVIC0 Peripheral Identification Bits24:31 */
#define REG_NVIC0_INTCID0                    0xE000EFF0            /*  NVIC0 Component Identification Bits7:0 */
#define REG_NVIC0_INTCID1                    0xE000EFF4            /*  NVIC0 Component Identification Bits15:8 */
#define REG_NVIC0_INTCID2                    0xE000EFF8            /*  NVIC0 Component Identification Bits16:23 */
#define REG_NVIC0_INTCID3                    0xE000EFFC            /*  NVIC0 Component Identification Bits24:31 */

/* ============================================================================================================================
        NVIC Register BitMasks, Positions & Enumerations 
   ============================================================================================================================ */
/* -------------------------------------------------------------------------------------------------------------------------
          NVIC_INTCON0                         Pos/Masks         Description
   ------------------------------------------------------------------------------------------------------------------------- */
#define BITP_NVIC_INTCON0_SLEEPDEEP           2            /*  deep sleep flag for HIBERNATE mode */
#define BITP_NVIC_INTCON0_SLEEPONEXIT         1            /*  Sleeps the core on exit from an ISR */

#define BITM_NVIC_INTCON0_SLEEPDEEP          (_ADI_MSK(0x00000004,uint16_t))    /*  deep sleep flag for HIBERNATE mode */

#define BITM_NVIC_INTCON0_SLEEPONEXIT        (_ADI_MSK(0x00000002,uint16_t))    /*  Sleeps the core on exit from an ISR */


/* Serial Peripheral Interface Parameters */

/***** SPI0  */
/***** SPI1  */
/***** SPI2  */

/* DMA Parameters */

/***** DMA0  */

/* Flash Controller Parameters */

/***** FLCC0  */

/* FLCC Parameters */

/***** FLCC0  */

/* Cache Controller Parameters */

/***** FLCC0_CACHE  */

/* DFT Controller Parameters */

/***** FLCC0_DFT  */

/* Flash Controller Parameters */

/***** FLCC0_TEST  */

/* Serial Port Parameters */

/***** SPORT0  */
#define PARAM_SPORT0_REVID_REV              0
#define PARAM_SPORT0_REVID_MAJOR            0
#define PARAM_SPORT0_APB_DELAY_CFG          0

/* crypto Parameters */

/***** CRYPT0  */
#define PARAM_CRYPT0_INCLUDE_CNTR           1
#define PARAM_CRYPT0_INCLUDE_CBC            1
#define PARAM_CRYPT0_INCLUDE_CCM            1
#define PARAM_CRYPT0_INCLUDE_MAC            1
#define PARAM_CRYPT0_INCLUDE_SHA            1
#define PARAM_CRYPT0_INCLUDE_192KEY          1
#define PARAM_CRYPT0_INCLUDE_256KEY          1
#define PARAM_CRYPT0_INCLUDE_EXTKEY          1

/* PMG Parameters */

/***** PMG0  */

/* Power Management Registers Parameters */

/***** PMG0  */

/* External interrupt configuration Parameters */

/***** XINT0  */

/* Clocking registers Parameters */

/***** CLKG0_OSC  */

/* CLKG Parameters */

/***** CLKG0  */

/* Power Management Registers Parameters */

/***** PMG0_TST  */

/* Clocking registers Parameters */

/***** CLKG0_CLK  */
/* -------------------------------------------------------------------------------------------------------------------------
 *
 *  ADuCM302x Parameters
 *
 * ------------------------------------------------------------------------------------------------------------------------- */

/* ====================================================================================================
 *    Interrupt Definitions
 * ==================================================================================================== */
#define INTR_SYS_RESET                       -15		/* Cortex-M3 Reset */
#define INTR_SYS_NonMaskableint              -14		/* Cortex-M3 Non-maskable Interrupt */
#define INTR_SYS_HardFault                   -13		/* Cortex-M3 Hardware Fault */
#define INTR_SYS_MemoryManagement            -12		/* Cortex-M3 Memory Management Interrupt */
#define INTR_SYS_BusFault                    -11		/* Cortex-M3 Bus Fault */
#define INTR_SYS_UsageFault                  -10		/* Cortex-M3 Usage Fault */
#define INTR_SYS_SVCall                       -5		/* Cortex-M3 SVCall Interrupt */
#define INTR_SYS_DebugMonitor                 -4		/* Cortex-M3 Debug Monitor */
#define INTR_SYS_PendSV                       -2		/* Cortex-M3 PendSV Interrupt */
#define INTR_SYS_SysTick                      -1		/* Cortex-M3 SysTick Interrupt */
#define INTR_RTC1_EVT                          0		/* Event */
#define INTR_XINT0_EVT0                        1		/* External Wakeup Interrupt n */
#define INTR_XINT0_EVT1                        2		/* External Wakeup Interrupt n */
#define INTR_XINT0_EVT2                        3		/* External Wakeup Interrupt n */
#define INTR_XINT0_EVT3                        4		/* External Wakeup Interrupt n */
#define INTR_WDT0_EXP                          5		/* Expiration */
#define INTR_PMG0_VREG_OVR                     6		/* Voltage Regulator (VREG) Overvoltage */
#define INTR_PMG0_BATT_RANGE                   7		/* Battery Voltage (VBAT) Out of Range */
#define INTR_RTC0_EVT                          8		/* Event */
#define INTR_SYS_GPIO_INTA                     9		/* GPIO Interrupt A */
#define INTR_SYS_GPIO_INTB                    10		/* GPIO Interrupt B */
#define INTR_TMR0_EVT                         11		/* Event */
#define INTR_TMR1_EVT                         12		/* Event */
#define INTR_FLCC0_EVT                        13		/* Event */
#define INTR_UART0_EVT                        14		/* Event */
#define INTR_SPI0_EVT                         15		/* Event */
#define INTR_SPI2_EVT                         16		/* Event */
#define INTR_I2C0_SLV_EVT                     17		/* Slave Event */
#define INTR_I2C0_MST_EVT                     18		/* Master Event */
#define INTR_DMA0_CHAN_ERR                    19		/* Channel Error */
#define INTR_DMA0_CH0_DONE                    20		/* Channel 0 Done */
#define INTR_DMA0_CH1_DONE                    21		/* Channel 1 Done */
#define INTR_DMA0_CH2_DONE                    22		/* Channel 2 Done */
#define INTR_DMA0_CH3_DONE                    23		/* Channel 3 Done */
#define INTR_DMA0_CH4_DONE                    24		/* Channel 4 Done */
#define INTR_DMA0_CH5_DONE                    25		/* Channel 5 Done */
#define INTR_DMA0_CH6_DONE                    26		/* Channel 6 Done */
#define INTR_DMA0_CH7_DONE                    27		/* Channel 7 Done */
#define INTR_DMA0_CH8_DONE                    28		/* Channel 8 Done */
#define INTR_DMA0_CH9_DONE                    29		/* Channel 9 Done */
#define INTR_DMA0_CH10_DONE                   30		/* Channel 10 Done */
#define INTR_DMA0_CH11_DONE                   31		/* Channel 11 Done */
#define INTR_DMA0_CH12_DONE                   32		/* Channel 12 Done */
#define INTR_DMA0_CH13_DONE                   33		/* Channel 13 Done */
#define INTR_DMA0_CH14_DONE                   34		/* Channel 14 Done */
#define INTR_DMA0_CH15_DONE                   35		/* Channel 15 Done */
#define INTR_SPORT0_A_EVT                     36		/* Channel A Event */
#define INTR_SPORT0_B_EVT                     37		/* Channel B Event */
#define INTR_CRYPT0_EVT                       38		/* Event */
#define INTR_TMR2_EVT                         40		/* Event */
#define INTR_CLKG0_XTAL_OSC_EVT               41		/* Crystal Oscillator Event */
#define INTR_SPI1_EVT                         42		/* Event */
#define INTR_CLKG0_PLL_EVT                    43		/* PLL Event */
#define INTR_RNG0_EVT                         44		/*  */
#define INTR_BEEP0_EVT                        45		/* Event */
#define INTR_DMA0_CH16_DONE                   56		/* Channel 16 Done */
#define INTR_DMA0_CH17_DONE                   57		/* Channel 17 Done */
#define INTR_DMA0_CH18_DONE                   58		/* Channel 18 Done */
#define INTR_DMA0_CH19_DONE                   59		/* Channel 19 Done */
#define INTR_DMA0_CH20_DONE                   60		/* Channel 20 Done */
#define INTR_DMA0_CH21_DONE                   61		/* Channel 21 Done */
#define INTR_DMA0_CH22_DONE                   62		/* Channel 22 Done */
#define INTR_DMA0_CH23_DONE                   63		/* Channel 23 Done */


#endif	/* end ifndef _DEF_ADUCM302X_H */
