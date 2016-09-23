/* ================================================================================

     Project      :   ADuCM302x
     File         :   ADuCM302x_typedefs.h
     Description  :   C Register Structures

     Date         :   Oct 21, 2014

     Copyright (c) Analog Devices, Inc.  All Rights Reserved.
     This software is proprietary and confidential to Analog Devices, Inc. and
     its licensors.

     This file was auto-generated. Do not make local changes to this file.

   ================================================================================ */

/** @defgroup LOAD 16-bit load value (LOAD) Register
 *  16-bit load value (LOAD) Register.
 *
 *  16-bit load value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_LOAD_Struct
 *! \brief  16-bit load value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_LOAD_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Load value */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_LOAD_t;

/*@}*/

/** @defgroup CURCNT 16-bit timer value (CURCNT) Register
 *  16-bit timer value (CURCNT) Register.
 *
 *  16-bit timer value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_CURCNT_Struct
 *! \brief  16-bit timer value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_CURCNT_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Current count */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_CURCNT_t;

/*@}*/

/** @defgroup CTL Control (CTL) Register
 *  Control (CTL) Register.
 *
 *  Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_CTL_Struct
 *! \brief  Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_CTL_t {
  union {
    struct {
      unsigned int PRE        :  2;  /**< Prescaler */
      unsigned int UP         :  1;  /**< Count up */
      unsigned int MODE       :  1;  /**< Timer mode */
      unsigned int EN         :  1;  /**< Timer enable */
      unsigned int CLK        :  2;  /**< Clock select */
      unsigned int RLD        :  1;  /**< Reload control */
      unsigned int EVTRANGE   :  5;  /**< Event select range */
      unsigned int EVTEN      :  1;  /**< Event select */
      unsigned int RSTEN      :  1;  /**< Counter and prescale reset enable */
      unsigned int SYNCBYP    :  1;  /**< Synchronization bypass */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_CTL_t;

/*@}*/

/** @defgroup CLRINT Clear Interrupt (CLRINT) Register
 *  Clear Interrupt (CLRINT) Register.
 *
 *  Clear Interrupt
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_CLRINT_Struct
 *! \brief  Clear Interrupt Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_CLRINT_t {
  union {
    struct {
      unsigned int TIMEOUT    :  1;  /**< Clear timeout interrupt */
      unsigned int EVTCAPT    :  1;  /**< Clear captured event interrupt */
      unsigned int reserved2  : 14;
    };
    uint16_t VALUE16;
  };
} ADI_TMR_CLRINT_t;

/*@}*/

/** @defgroup CAPTURE Capture (CAPTURE) Register
 *  Capture (CAPTURE) Register.
 *
 *  Capture
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_CAPTURE_Struct
 *! \brief  Capture Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_CAPTURE_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< 16-bit captured value */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_CAPTURE_t;

/*@}*/

/** @defgroup ALOAD 16-bit load value, asynchronous (ALOAD) Register
 *  16-bit load value, asynchronous (ALOAD) Register.
 *
 *  16-bit load value, asynchronous
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_ALOAD_Struct
 *! \brief  16-bit load value, asynchronous Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_ALOAD_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Load value, asynchronous */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_ALOAD_t;

/*@}*/

/** @defgroup ACURCNT 16-bit timer value, asynchronous (ACURCNT) Register
 *  16-bit timer value, asynchronous (ACURCNT) Register.
 *
 *  16-bit timer value, asynchronous
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_ACURCNT_Struct
 *! \brief  16-bit timer value, asynchronous Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_ACURCNT_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Counter value */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_ACURCNT_t;

/*@}*/

/** @defgroup STAT Status (STAT) Register
 *  Status (STAT) Register.
 *
 *  Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_STAT_Struct
 *! \brief  Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_STAT_t {
  union {
    struct {
      unsigned int TIMEOUT    :  1;  /**< Timeout event occurred */
      unsigned int CAPTURE    :  1;  /**< Capture event pending */
      unsigned int reserved2  :  4;
      unsigned int BUSY       :  1;  /**< Timer Busy */
      unsigned int PDOK       :  1;  /**< Clear Interrupt Register synchronization */
      unsigned int CNTRST     :  1;  /**< Counter reset occurring */
      unsigned int reserved9  :  7;
    };
    uint16_t VALUE16;
  };
} ADI_TMR_STAT_t;

/*@}*/

/** @defgroup PWMCTL PWM Control Register (PWMCTL) Register
 *  PWM Control Register (PWMCTL) Register.
 *
 *  PWM Control Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_PWMCTL_Struct
 *! \brief  PWM Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_PWMCTL_t {
  union {
    struct {
      unsigned int MATCH      :  1;  /**< PWM Match enabled */
      unsigned int IDLESTATE  :  1;  /**< PWM Idle State */
      unsigned int reserved2  : 14;
    };
    uint16_t VALUE16;
  };
} ADI_TMR_PWMCTL_t;

/*@}*/

/** @defgroup PWMMATCH PWM Match Value (PWMMATCH) Register
 *  PWM Match Value (PWMMATCH) Register.
 *
 *  PWM Match Value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_TMR_PWMMATCH_Struct
 *! \brief  PWM Match Value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_TMR_PWMMATCH_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< PWM Match Value */
    };
    uint16_t VALUE16;
  };
} ADI_TMR_PWMMATCH_t;

/*@}*/

/** @defgroup CR0 RTC Control 0 (CR0) Register
 *  RTC Control 0 (CR0) Register.
 *
 *  RTC Control 0
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_RTC_CR0_MOD60ALM
 *! \brief  Periodic, modulo-60 alarm time in prescaled RTC time units beyond a modulo-60 boundary (MOD60ALM) Enumerations
 *  ========================================================================= */
typedef enum
{
  RTC_CR0_EXAMPLE_1_THIRTY_TIME_UNITS_DECIMAL    = 30,  /**< Example of setting a modulo-60 periodic interrupt from the RTC to be issued to the CPU at 30 time units past a modulo-60 boundary. */
  RTC_CR0_EXAMPLE_2_FIFTYFIVE_TIME_UNITS_DECIMAL = 55   /**< Example of setting a modulo-60 periodic interrupt from the RTC to be issued to the CPU at 55 time units past a modulo-60 boundary. */
} ADI_RTC_CR0_MOD60ALM;


/* ==========================================================================
 *! \struct ADI_RTC_CR0_Struct
 *! \brief  RTC Control 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_CR0_t {
  union {
    struct {
      unsigned int CNTEN      :  1;  /**< Global enable for the RTC */
      unsigned int ALMEN      :  1;  /**< Enable the RTC alarm (absolute) operation */
      unsigned int ALMINTEN   :  1;  /**< Enable ALMINT-sourced alarm interrupts to the CPU */
      unsigned int TRMEN      :  1;  /**< Enable RTC digital trimming */
      unsigned int MOD60ALMEN :  1;  /**< Enable RTC modulo-60 counting of time past a modulo-60 boundary */
      unsigned int MOD60ALM   :  6;  /**< Periodic, modulo-60 alarm time in prescaled RTC time units beyond a modulo-60 boundary */
      unsigned int MOD60ALMINTEN :  1;  /**< Enable periodic MOD60ALMINT-sourced interrupts to the CPU */
      unsigned int ISOINTEN      :  1;  /**< Enable ISOINT-sourced interrupts to the CPU when isolation of the RTC power domain is activated and subsequently de-activated */
      unsigned int WPNDERRINTEN  :  1;  /**< Enable WPENDERRINT-sourced interrupts to the CPU when an RTC register-write pending error occurs */
      unsigned int WSYNCINTEN    :  1;  /**< Enable WSYNCINT-sourced interrupts to the CPU */
      unsigned int WPNDINTEN     :  1;  /**< Enable WPENDINT-sourced interrupts to the CPU */
      unsigned int reserved16    : 16;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_CR0_t;

/*@}*/

/** @defgroup SR0 RTC Status 0 (SR0) Register
 *  RTC Status 0 (SR0) Register.
 *
 *  RTC Status 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_SR0_Struct
 *! \brief  RTC Status 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_SR0_t {
  union {
    struct {
      unsigned int FAIL       :  1;  /**< RTC failure interrupt source */
      unsigned int ALMINT     :  1;  /**< Alarm interrupt source */
      unsigned int MOD60ALMINT :  1;  /**< Modulo-60 RTC alarm interrupt source */
      unsigned int ISOINT      :  1;  /**< RTC power-domain isolation interrupt source */
      unsigned int WPNDERRINT  :  1;  /**< Write pending error interrupt source */
      unsigned int WSYNCINT    :  1;  /**< Write synchronisation interrupt */
      unsigned int WPNDINT     :  1;  /**< Write pending interrupt */
      unsigned int WSYNCCR0    :  1;  /**< Synchronisation status of posted writes to RTCCR0 */
      unsigned int WSYNCSR0    :  1;  /**< Synchronisation status of posted clearances to interrupt sources in RTCSR0 */
      unsigned int WSYNCCNT0   :  1;  /**< Synchronisation status of posted writes to RTCCNT0 */
      unsigned int WSYNCCNT1   :  1;  /**< Synchronisation status of posted writes to RTCCNT1 */
      unsigned int WSYNCALM0   :  1;  /**< Synchronisation status of posted writes to RTCALM0 */
      unsigned int WSYNCALM1   :  1;  /**< Synchronisation status of posted writes to RTCALM1 */
      unsigned int WSYNCTRM    :  1;  /**< Synchronisation status of posted writes to RTCTRM */
      unsigned int ISOENB      :  1;  /**< Master enable for isolation of the always-on RTC power domain */
      unsigned int LOCK32KHZ   :  1;  /**< Lock status of the 32kHz crystal oscillator output clock */
      unsigned int WSYNCCR1    :  1;  /**< Synchronisation status of posted writes to RTCCR1 */
      unsigned int WSYNCALM2   :  1;  /**< Synchronisation status of posted writes to RTCALM2 */
      unsigned int reserved18  : 14;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_SR0_t;

/*@}*/

/** @defgroup SR1 RTC Status 1 (SR1) Register
 *  RTC Status 1 (SR1) Register.
 *
 *  RTC Status 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_SR1_Struct
 *! \brief  RTC Status 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_SR1_t {
  union {
    struct {
      unsigned int DBGPTR     :  3;  /**< Debug address pointer */
      unsigned int WERRCODE   :  4;  /**< Identifier for the source of a write synchronisation error */
      unsigned int WPNDCR0    :  1;  /**< Pending status of posted writes to RTCCR0 */
      unsigned int WPNDSR0    :  1;  /**< Pending status of posted clearances of interrupt sources in RTCSR0 */
      unsigned int WPNDCNT0   :  1;  /**< Pending status of posted writes to RTCCNT0 */
      unsigned int WPNDCNT1   :  1;  /**< Pending status of posted writes to RTCCNT1 */
      unsigned int WPNDALM0   :  1;  /**< Pending status of posted writes to RTCALM0 */
      unsigned int WPNDALM1   :  1;  /**< Pending status of posted writes to RTCALM1 */
      unsigned int WPNDTRM    :  1;  /**< Pending status of posted writes to RTCTRM */
      unsigned int DISPRSCLMIR :  1;  /**< Mirror of the DISPRESCALE field of RTCCR0 */
      unsigned int DBGWDOG     :  1;  /**< Indicator that the transaction lock watchdog is currently active */
      unsigned int WPNDCR1     :  1;  /**< Pending status of posted writes to RTCCR1 */
      unsigned int WPNDALM2    :  1;  /**< Pending status of posted writes to RTCALM2 */
      unsigned int reserved18  : 14;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_SR1_t;

/*@}*/

/** @defgroup CNT0 RTC Count 0 (CNT0) Register
 *  RTC Count 0 (CNT0) Register.
 *
 *  RTC Count 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_CNT0_Struct
 *! \brief  RTC Count 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_CNT0_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Lower 16 prescaled (non-fractional) bits of the RTC real-time count. */
      unsigned int CNT1MIR    : 16;  /**< Mirror of RTCCNT1. */
    };
    uint32_t VALUE32;
  };
} ADI_RTC_CNT0_t;

/*@}*/

/** @defgroup CNT1 RTC Count 1 (CNT1) Register
 *  RTC Count 1 (CNT1) Register.
 *
 *  RTC Count 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_CNT1_Struct
 *! \brief  RTC Count 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_CNT1_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Upper 16 prescaled (non-fractional) bits of the RTC real-time count. */
    };
    uint16_t VALUE16;
  };
} ADI_RTC_CNT1_t;

/*@}*/

/** @defgroup ALM0 RTC Alarm 0 (ALM0) Register
 *  RTC Alarm 0 (ALM0) Register.
 *
 *  RTC Alarm 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_ALM0_Struct
 *! \brief  RTC Alarm 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_ALM0_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Lower 16 prescaled (i.e. non-fractional) bits of the RTC alarm target time. */
      unsigned int ALM1MIR    : 16;  /**< Mirror of RTCALM1. */
    };
    uint32_t VALUE32;
  };
} ADI_RTC_ALM0_t;

/*@}*/

/** @defgroup ALM1 RTC Alarm 1 (ALM1) Register
 *  RTC Alarm 1 (ALM1) Register.
 *
 *  RTC Alarm 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_ALM1_Struct
 *! \brief  RTC Alarm 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_ALM1_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Upper 16 prescaled (i.e. non-fractional) bits of the RTC alarm target time. */
    };
    uint16_t VALUE16;
  };
} ADI_RTC_ALM1_t;

/*@}*/

/** @defgroup TRM RTC Trim (TRM) Register
 *  RTC Trim (TRM) Register.
 *
 *  RTC Trim
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_TRM_Struct
 *! \brief  RTC Trim Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_TRM_t {
  union {
    struct {
      unsigned int VALUE      :  3;  /**< Trim value in prescaled RTC time units to be added or subtracted from the RTC count at the end of a periodic interval selected by TRMIVL */
      unsigned int ADD        :  1;  /**< Trim polarity */
      unsigned int IVL        :  2;  /**< Trim interval in prescaled RTC time units */
      unsigned int IVL2EXPMIN :  4;  /**< Minimum power-of-two interval of prescaled RTC time units which TRMIVL can select */
      unsigned int reserved10 :  6;
    };
    uint16_t VALUE16;
  };
} ADI_RTC_TRM_t;

/*@}*/

/** @defgroup GWY RTC Gateway (GWY) Register
 *  RTC Gateway (GWY) Register.
 *
 *  RTC Gateway
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_GWY_Struct
 *! \brief  RTC Gateway Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_GWY_t {
  union {
    struct {
      unsigned int SWKEY      : 16;  /**< Software-keyed command issued by the CPU. */
    };
    uint16_t VALUE16;
  };
} ADI_RTC_GWY_t;

/*@}*/

/** @defgroup CR1 RTC Control 1 (CR1) Register
 *  RTC Control 1 (CR1) Register.
 *
 *  RTC Control 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_CR1_Struct
 *! \brief  RTC Control 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_CR1_t {
  union {
    struct {
      unsigned int CNTINTEN   :  1;  /**< Enable for the RTC count interrupt source, RTCCNTINT in RTCSR2 */
      unsigned int PSINTEN    :  1;  /**< Enable for the prescaled, modulo-1 interrupt source, RTCPSINT in RTCSR2 */
      unsigned int TRMINTEN   :  1;  /**< Enable for the RTC trim interrupt source, RTCTRMINT in RTCSR2 */
      unsigned int CNTROLLINTEN :  1;  /**< Enable for the RTC count roll-over interrupt source, RTCCNTROLLINT in RTCSR2 */
      unsigned int CNTMOD60ROLLINTEN :  1;  /**< Enable for the RTC modulo-60 count roll-over interrupt source, RTCMOD60ROLLINT in RTCSR2 */
      unsigned int PRESCALE2EXP      :  4;  /**< Prescale power of 2 division factor for the RTC base clock */
      unsigned int reserved9         :  7;
    };
    uint16_t VALUE16;
  };
} ADI_RTC_CR1_t;

/*@}*/

/** @defgroup SR2 RTC Status 2 (SR2) Register
 *  RTC Status 2 (SR2) Register.
 *
 *  RTC Status 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_SR2_Struct
 *! \brief  RTC Status 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_SR2_t {
  union {
    struct {
      unsigned int CNTINT     :  1;  /**< RTC count interrupt source */
      unsigned int PSINT      :  1;  /**< RTC prescaled, modulo-1 boundary interrupt source */
      unsigned int TRMINT     :  1;  /**< RTC trim interrupt source */
      unsigned int CNTROLLINT :  1;  /**< RTC count roll-over interrupt source */
      unsigned int CNTMOD60ROLLINT :  1;  /**< RTC modulo-60 count roll-over interrupt source */
      unsigned int CNTROLL         :  1;  /**< RTC count roll-over */
      unsigned int CNTMOD60ROLL    :  1;  /**< RTC count modulo-60 roll-over */
      unsigned int TRMBDYMIR       :  1;  /**< Mirror of the RTCTRMBDY field of RTCMOD */
      unsigned int reserved8       :  4;
      unsigned int WPNDCR1MIR      :  1;  /**< Mirror of the WPENDCR1 field of RTCSR1 */
      unsigned int WPNDALM2MIR     :  1;  /**< Mirror of the WPENDALM2 field of RTCSR1 */
      unsigned int WSYNCCR1MIR     :  1;  /**< Mirror of the WSYNCCR1 field of RTCSR0 */
      unsigned int WSYNCALM2MIR    :  1;  /**< Mirror of the WSYNCALM2 field of RTCSR0 */
    };
    uint16_t VALUE16;
  };
} ADI_RTC_SR2_t;

/*@}*/

/** @defgroup SNAP0 RTC Snapshot 0 (SNAP0) Register
 *  RTC Snapshot 0 (SNAP0) Register.
 *
 *  RTC Snapshot 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_SNAP0_Struct
 *! \brief  RTC Snapshot 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_SNAP0_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Sticky snapshot of RTCCNT0 */
      unsigned int reserved16 : 16;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_SNAP0_t;

/*@}*/

/** @defgroup SNAP1 RTC Snapshot 1 (SNAP1) Register
 *  RTC Snapshot 1 (SNAP1) Register.
 *
 *  RTC Snapshot 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_SNAP1_Struct
 *! \brief  RTC Snapshot 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_SNAP1_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Sticky snapshot of RTCCNT1 */
    };
    uint16_t VALUE16;
  };
} ADI_RTC_SNAP1_t;

/*@}*/

/** @defgroup SNAP2 RTC Snapshot 2 (SNAP2) Register
 *  RTC Snapshot 2 (SNAP2) Register.
 *
 *  RTC Snapshot 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_SNAP2_Struct
 *! \brief  RTC Snapshot 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_SNAP2_t {
  union {
    struct {
      unsigned int VALUE      : 15;  /**< Sticky snapshot of RTCCNT2 */
      unsigned int reserved15 : 17;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_SNAP2_t;

/*@}*/

/** @defgroup MOD RTC Modulo (MOD) Register
 *  RTC Modulo (MOD) Register.
 *
 *  RTC Modulo
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_MOD_Struct
 *! \brief  RTC Modulo Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_MOD_t {
  union {
    struct {
      unsigned int CNTMOD60   :  6;  /**< Modulo-60 value of the RTC count, {RTCCNT1, RTCCNT0} */
      unsigned int INCR       :  4;  /**< Most recent increment value which has been added to the RTC count in {RTCCNT1, RTCCNT0} */
      unsigned int TRMBDY     :  1;  /**< Trim boundary indicator that the most recent RTC count increment has coincided with trimming of the count value */
      unsigned int CNT0_4TOZERO :  5;  /**< Mirror of RTCCNT0[5:0]. */
      unsigned int reserved16   : 16;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_MOD_t;

/*@}*/

/** @defgroup CNT2 RTC Count 2 (CNT2) Register
 *  RTC Count 2 (CNT2) Register.
 *
 *  RTC Count 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_CNT2_Struct
 *! \brief  RTC Count 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_CNT2_t {
  union {
    struct {
      unsigned int VALUE      : 15;  /**< Fractional bits of the RTC real-time count */
      unsigned int reserved15 : 17;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_CNT2_t;

/*@}*/

/** @defgroup ALM2 RTC Alarm 2 (ALM2) Register
 *  RTC Alarm 2 (ALM2) Register.
 *
 *  RTC Alarm 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RTC_ALM2_Struct
 *! \brief  RTC Alarm 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RTC_ALM2_t {
  union {
    struct {
      unsigned int VALUE      : 15;  /**< Fractional (non-prescaled) bits of the RTC alarm target time */
      unsigned int reserved15 : 17;
    };
    uint32_t VALUE32;
  };
} ADI_RTC_ALM2_t;

/*@}*/

/** @defgroup ADIID ADI Identification (ADIID) Register
 *  ADI Identification (ADIID) Register.
 *
 *  ADI Identification
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SYS_ADIID_Struct
 *! \brief  ADI Identification Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SYS_ADIID_t {
  union {
    struct {
      unsigned int VALUE      : 16;
    };
    uint16_t VALUE16;
  };
} ADI_SYS_ADIID_t;

/*@}*/

/** @defgroup CHIPID Chip Identifier (CHIPID) Register
 *  Chip Identifier (CHIPID) Register.
 *
 *  Chip Identifier
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SYS_CHIPID_Struct
 *! \brief  Chip Identifier Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SYS_CHIPID_t {
  union {
    struct {
      unsigned int REV        :  4;
      unsigned int PARTID     : 12;
    };
    uint16_t VALUE16;
  };
} ADI_SYS_CHIPID_t;

/*@}*/

/** @defgroup SWDEN Serial Wire Debug Enable (SWDEN) Register
 *  Serial Wire Debug Enable (SWDEN) Register.
 *
 *  Serial Wire Debug Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SYS_SWDEN_Struct
 *! \brief  Serial Wire Debug Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SYS_SWDEN_t {
  union {
    struct {
      unsigned int VALUE      : 16;
    };
    uint16_t VALUE16;
  };
} ADI_SYS_SWDEN_t;

/*@}*/

/** @defgroup LOAD Load value (LOAD) Register
 *  Load value (LOAD) Register.
 *
 *  Load value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_WDT_LOAD_Struct
 *! \brief  Load value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_WDT_LOAD_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Load value */
    };
    uint16_t VALUE16;
  };
} ADI_WDT_LOAD_t;

/*@}*/

/** @defgroup CCNT Current count value (CCNT) Register
 *  Current count value (CCNT) Register.
 *
 *  Current count value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_WDT_CCNT_Struct
 *! \brief  Current count value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_WDT_CCNT_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Current count value */
    };
    uint16_t VALUE16;
  };
} ADI_WDT_CCNT_t;

/*@}*/

/** @defgroup CTL Control (CTL) Register
 *  Control (CTL) Register.
 *
 *  Control
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_WDT_CTL_PMD
 *! \brief  Power Mode Disable (PMD) Enumerations
 *  ========================================================================= */
typedef enum
{
  WDT_CTL_RUN     = 0,  /**< The watchdog timer will continue its count down when Cortex is in halted state.                                                                                                                                 */
  WDT_CTL_SUSPEND = 1   /**< When hibernate or halted mode is entered, the watchdog counter will suspend its count down. As hibernate mode is exited, the count down will resume from its current count value (the count will not be reset). */
} ADI_WDT_CTL_PMD;


/*  =========================================================================
 *! \enum   ADI_WDT_CTL_PRE
 *! \brief  Prescaler (PRE) Enumerations
 *  ========================================================================= */
typedef enum
{
  WDT_CTL_DIV1     = 0,  /**< source clock/1.             */
  WDT_CTL_DIV16    = 1,  /**< source clock/16.            */
  WDT_CTL_DIV256   = 2,  /**< source clock/256 (default). */
  WDT_CTL_RESERVED = 3   /**< Reserved                    */
} ADI_WDT_CTL_PRE;


/* ==========================================================================
 *! \struct ADI_WDT_CTL_Struct
 *! \brief  Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_WDT_CTL_t {
  union {
    struct {
      unsigned int PMD        :  1;  /**< Power Mode Disable */
      unsigned int IRQ        :  1;  /**< Timer interrupt */
      unsigned int PRE        :  2;  /**< Prescaler */
      unsigned int reserved4  :  1;
      unsigned int EN         :  1;  /**< Timer enable */
      unsigned int MODE       :  1;  /**< Timer mode */
      unsigned int SPARE      :  1;  /**< Unused spare bit */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_WDT_CTL_t;

/*@}*/

/** @defgroup RESTART Clear interrupt (RESTART) Register
 *  Clear interrupt (RESTART) Register.
 *
 *  Clear interrupt
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_WDT_RESTART_Struct
 *! \brief  Clear interrupt Register bit field structure
 * ========================================================================== */
typedef struct _ADI_WDT_RESTART_t {
  union {
    struct {
      unsigned int CLRWORD    : 16;  /**< Clear watchdog */
    };
    uint16_t VALUE16;
  };
} ADI_WDT_RESTART_t;

/*@}*/

/** @defgroup STAT Status (STAT) Register
 *  Status (STAT) Register.
 *
 *  Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_WDT_STAT_Struct
 *! \brief  Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_WDT_STAT_t {
  union {
    struct {
      unsigned int IRQ        :  1;  /**< WDT Interrupt */
      unsigned int CLRIRQ     :  1;  /**< Clear Interrupt Register write sync in progress */
      unsigned int LOADING    :  1;  /**< Load Register write sync in progress */
      unsigned int COUNTING   :  1;  /**< Control Register write sync in progress */
      unsigned int LOCKED     :  1;  /**< Lock status bit */
      unsigned int RSTCTL     :  1;  /**< Reset Control Register written and locked */
      unsigned int reserved6  : 10;
    };
    uint16_t VALUE16;
  };
} ADI_WDT_STAT_t;

/*@}*/

/** @defgroup MCTL Master control (MCTL) Register
 *  Master control (MCTL) Register.
 *
 *  Master control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_MCTL_Struct
 *! \brief  Master control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_MCTL_t {
  union {
    struct {
      unsigned int MASEN      :  1;  /**< Master enable */
      unsigned int COMPLETE   :  1;  /**< Start back-off disable */
      unsigned int LOOPBACK   :  1;  /**< Internal loopback enable */
      unsigned int STRETCHSCL :  1;  /**< Stretch SCL enable */
      unsigned int IENMRX     :  1;  /**< Receive request interrupt enable */
      unsigned int IENMTX     :  1;  /**< Transmit request interrupt enable. */
      unsigned int IENALOST   :  1;  /**< Arbitration lost interrupt enable */
      unsigned int IENACK     :  1;  /**< ACK not received interrupt enable */
      unsigned int IENCMP     :  1;  /**< Transaction completed (or stop detected) interrupt enable */
      unsigned int MXMITDEC   :  1;  /**< Decrement master TX FIFO status when a byte has been transmitted */
      unsigned int MRXDMA     :  1;  /**< Enable master Rx DMA request */
      unsigned int MTXDMA     :  1;  /**< Enable master Tx DMA request */
      unsigned int BUSCLR     :  1;  /**< Bus-Clear Enable */
      unsigned int STOPBUSCLR :  1;  /**< Prestop Bus-Clear */
      unsigned int reserved14 :  2;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_MCTL_t;

/*@}*/

/** @defgroup MSTAT Master status (MSTAT) Register
 *  Master status (MSTAT) Register.
 *
 *  Master status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_MSTAT_Struct
 *! \brief  Master status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_MSTAT_t {
  union {
    struct {
      unsigned int MTXF       :  2;  /**< Master Transmit FIFO status */
      unsigned int MTXREQ     :  1;  /**< When read is master Transmit request; when write is Clear master transmit interrupt bit */
      unsigned int MRXREQ     :  1;  /**< Master Receive request */
      unsigned int NACKADDR   :  1;  /**< ACK not received in response to an address */
      unsigned int ALOST      :  1;  /**< Arbitration lost */
      unsigned int MBUSY      :  1;  /**< Master busy */
      unsigned int NACKDATA   :  1;  /**< ACK not received in response to data write */
      unsigned int TCOMP      :  1;  /**< Transaction complete or stop detected */
      unsigned int MRXOVR     :  1;  /**< Master Receive FIFO overflow */
      unsigned int LINEBUSY   :  1;  /**< Line is busy */
      unsigned int MSTOP      :  1;  /**< STOP driven by this I2C Master */
      unsigned int MTXUNDR    :  1;  /**< Master Transmit Underflow */
      unsigned int SDAFILT    :  1;  /**< State of SDA Line */
      unsigned int SCLFILT    :  1;  /**< State of SCL Line */
      unsigned int reserved15 :  1;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_MSTAT_t;

/*@}*/

/** @defgroup MRX Master receive data (MRX) Register
 *  Master receive data (MRX) Register.
 *
 *  Master receive data
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_MRX_Struct
 *! \brief  Master receive data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_MRX_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Master receive register */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_MRX_t;

/*@}*/

/** @defgroup MTX Master transmit data (MTX) Register
 *  Master transmit data (MTX) Register.
 *
 *  Master transmit data
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_MTX_Struct
 *! \brief  Master transmit data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_MTX_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Master transmit register */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_MTX_t;

/*@}*/

/** @defgroup MRXCNT Master receive data count (MRXCNT) Register
 *  Master receive data count (MRXCNT) Register.
 *
 *  Master receive data count
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_MRXCNT_Struct
 *! \brief  Master receive data count Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_MRXCNT_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Receive count */
      unsigned int EXTEND     :  1;  /**< Extended read */
      unsigned int reserved9  :  7;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_MRXCNT_t;

/*@}*/

/** @defgroup MCRXCNT Master current receive data count (MCRXCNT) Register
 *  Master current receive data count (MCRXCNT) Register.
 *
 *  Master current receive data count
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_MCRXCNT_Struct
 *! \brief  Master current receive data count Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_MCRXCNT_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Current receive count */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_MCRXCNT_t;

/*@}*/

/** @defgroup ADDR1 1st master address byte (ADDR1) Register
 *  1st master address byte (ADDR1) Register.
 *
 *  1st master address byte
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ADDR1_Struct
 *! \brief  1st master address byte Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ADDR1_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Address byte 1 */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ADDR1_t;

/*@}*/

/** @defgroup ADDR2 2nd master address byte (ADDR2) Register
 *  2nd master address byte (ADDR2) Register.
 *
 *  2nd master address byte
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ADDR2_Struct
 *! \brief  2nd master address byte Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ADDR2_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Address byte 2 */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ADDR2_t;

/*@}*/

/** @defgroup BYT Start byte (BYT) Register
 *  Start byte (BYT) Register.
 *
 *  Start byte
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_BYT_Struct
 *! \brief  Start byte Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_BYT_t {
  union {
    struct {
      unsigned int SBYTE      :  8;  /**< Start byte */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_BYT_t;

/*@}*/

/** @defgroup DIV Serial clock period divisor (DIV) Register
 *  Serial clock period divisor (DIV) Register.
 *
 *  Serial clock period divisor
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_DIV_Struct
 *! \brief  Serial clock period divisor Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_DIV_t {
  union {
    struct {
      unsigned int LOW        :  8;  /**< Serial clock low time */
      unsigned int HIGH       :  8;  /**< Serial clock high time */
    };
    uint16_t VALUE16;
  };
} ADI_I2C_DIV_t;

/*@}*/

/** @defgroup SCTL Slave control (SCTL) Register
 *  Slave control (SCTL) Register.
 *
 *  Slave control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_SCTL_Struct
 *! \brief  Slave control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_SCTL_t {
  union {
    struct {
      unsigned int SLVEN      :  1;  /**< Slave enable */
      unsigned int ADR10EN    :  1;  /**< Enabled 10-bit addressing */
      unsigned int GCEN       :  1;  /**< General call enable */
      unsigned int HGCEN      :  1;  /**< Hardware general call enable */
      unsigned int GCSBCLR    :  1;  /**< General call status bit clear */
      unsigned int EARLYTXR   :  1;  /**< Early transmit request mode */
      unsigned int STRETCHSCL :  1;  /**< Stretch SCL enable */
      unsigned int NACK       :  1;  /**< NACK next communication */
      unsigned int IENSTOP    :  1;  /**< Stop condition detected interrupt enable */
      unsigned int IENSRX     :  1;  /**< Slave Receive request interrupt enable */
      unsigned int IENSTX     :  1;  /**< Slave Transmit request interrupt enable */
      unsigned int STXDEC     :  1;  /**< Decrement Slave Tx FIFO status when a byte has been transmitted */
      unsigned int IENREPST   :  1;  /**< Repeated start interrupt enable */
      unsigned int SRXDMA     :  1;  /**< Enable slave Rx DMA request */
      unsigned int STXDMA     :  1;  /**< Enable slave Tx DMA request */
      unsigned int reserved15 :  1;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_SCTL_t;

/*@}*/

/** @defgroup SSTAT Slave I2C Status/Error/IRQ (SSTAT) Register
 *  Slave I2C Status/Error/IRQ (SSTAT) Register.
 *
 *  Slave I2C Status/Error/IRQ
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_SSTAT_Struct
 *! \brief  Slave I2C Status/Error/IRQ Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_SSTAT_t {
  union {
    struct {
      unsigned int STXFSEREQ  :  1;  /**< Slave Tx FIFO Status or early request */
      unsigned int STXUNDR    :  1;  /**< Slave Transmit FIFO underflow */
      unsigned int STXREQ     :  1;  /**< When read is slave transmit request; when write is clear slave transmit interrupt bit */
      unsigned int SRXREQ     :  1;  /**< Slave Receive request */
      unsigned int SRXOVR     :  1;  /**< Slave Receive FIFO overflow */
      unsigned int NOACK      :  1;  /**< Ack not generated by the slave */
      unsigned int SBUSY      :  1;  /**< Slave busy */
      unsigned int GCINT      :  1;  /**< General call interrupt */
      unsigned int GCID       :  2;  /**< General ID */
      unsigned int STOP       :  1;  /**< Stop after start and matching address */
      unsigned int IDMAT      :  2;  /**< Device ID matched */
      unsigned int REPSTART   :  1;  /**< Repeated start and matching address */
      unsigned int START      :  1;  /**< Start and matching address */
      unsigned int reserved15 :  1;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_SSTAT_t;

/*@}*/

/** @defgroup SRX Slave receive (SRX) Register
 *  Slave receive (SRX) Register.
 *
 *  Slave receive
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_SRX_Struct
 *! \brief  Slave receive Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_SRX_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Slave receive register */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_SRX_t;

/*@}*/

/** @defgroup STX Slave transmit (STX) Register
 *  Slave transmit (STX) Register.
 *
 *  Slave transmit
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_STX_Struct
 *! \brief  Slave transmit Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_STX_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Slave transmit register */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_STX_t;

/*@}*/

/** @defgroup ALT Hardware general call ID (ALT) Register
 *  Hardware general call ID (ALT) Register.
 *
 *  Hardware general call ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ALT_Struct
 *! \brief  Hardware general call ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ALT_t {
  union {
    struct {
      unsigned int ID         :  8;  /**< Slave Alt */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ALT_t;

/*@}*/

/** @defgroup ID0 1st slave address device ID (ID0) Register
 *  1st slave address device ID (ID0) Register.
 *
 *  1st slave address device ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ID0_Struct
 *! \brief  1st slave address device ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ID0_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Slave device ID 0 */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ID0_t;

/*@}*/

/** @defgroup ID1 2nd slave address device ID (ID1) Register
 *  2nd slave address device ID (ID1) Register.
 *
 *  2nd slave address device ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ID1_Struct
 *! \brief  2nd slave address device ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ID1_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Slave device ID 1 */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ID1_t;

/*@}*/

/** @defgroup ID2 3rd slave address device ID (ID2) Register
 *  3rd slave address device ID (ID2) Register.
 *
 *  3rd slave address device ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ID2_Struct
 *! \brief  3rd slave address device ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ID2_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Slave device ID 2 */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ID2_t;

/*@}*/

/** @defgroup ID3 4th slave address device ID (ID3) Register
 *  4th slave address device ID (ID3) Register.
 *
 *  4th slave address device ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ID3_Struct
 *! \brief  4th slave address device ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ID3_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Slave device ID 3 */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ID3_t;

/*@}*/

/** @defgroup STAT Master and slave FIFO status (STAT) Register
 *  Master and slave FIFO status (STAT) Register.
 *
 *  Master and slave FIFO status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_STAT_Struct
 *! \brief  Master and slave FIFO status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_STAT_t {
  union {
    struct {
      unsigned int STXF       :  2;  /**< Slave transmit FIFO status */
      unsigned int SRXF       :  2;  /**< Slave receive FIFO status */
      unsigned int MTXF       :  2;  /**< Master transmit FIFO status */
      unsigned int MRXF       :  2;  /**< Master receive FIFO status */
      unsigned int SFLUSH     :  1;  /**< Flush the slave transmit FIFO */
      unsigned int MFLUSH     :  1;  /**< Flush the master transmit FIFO */
      unsigned int reserved10 :  6;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_STAT_t;

/*@}*/

/** @defgroup SHCTL Shared control (SHCTL) Register
 *  Shared control (SHCTL) Register.
 *
 *  Shared control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_SHCTL_Struct
 *! \brief  Shared control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_SHCTL_t {
  union {
    struct {
      unsigned int RST        :  1;  /**< Reset START STOP detect circuit */
      unsigned int reserved1  : 15;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_SHCTL_t;

/*@}*/

/** @defgroup TCTL Timing Control Register (TCTL) Register
 *  Timing Control Register (TCTL) Register.
 *
 *  Timing Control Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_TCTL_Struct
 *! \brief  Timing Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_TCTL_t {
  union {
    struct {
      unsigned int THDATIN    :  5;  /**< Data In Hold Start */
      unsigned int reserved5  :  3;
      unsigned int FILTEROFF  :  1;  /**< Input Filter Control */
      unsigned int reserved9  :  7;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_TCTL_t;

/*@}*/

/** @defgroup ASTRETCH_SCL Automatic stretch SCL register (ASTRETCH_SCL) Register
 *  Automatic stretch SCL register (ASTRETCH_SCL) Register.
 *
 *  Automatic stretch SCL register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_I2C_ASTRETCH_SCL_Struct
 *! \brief  Automatic stretch SCL Register bit field structure
 * ========================================================================== */
typedef struct _ADI_I2C_ASTRETCH_SCL_t {
  union {
    struct {
      unsigned int MST        :  4;  /**< master automatic stretch mode */
      unsigned int SLV        :  4;  /**< slave automatic stretch mode */
      unsigned int MSTTMO     :  1;  /**< master automatic stretch timeout */
      unsigned int SLVTMO     :  1;  /**< slave automatic stretch timeout */
      unsigned int reserved10 :  6;
    };
    uint16_t VALUE16;
  };
} ADI_I2C_ASTRETCH_SCL_t;

/*@}*/

/** @defgroup STAT Status (STAT) Register
 *  Status (STAT) Register.
 *
 *  Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_STAT_Struct
 *! \brief  Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_STAT_t {
  union {
    struct {
      unsigned int IRQ        :  1;  /**< SPI Interrupt status */
      unsigned int XFRDONE    :  1;  /**< SPI transfer completion */
      unsigned int TXEMPTY    :  1;  /**< SPI Tx FIFO empty interrupt */
      unsigned int TXDONE     :  1;  /**< SPI Tx Done in read command mode */
      unsigned int TXUNDR     :  1;  /**< SPI Tx FIFO underflow */
      unsigned int TXIRQ      :  1;  /**< SPI Tx IRQ */
      unsigned int RXIRQ      :  1;  /**< SPI Rx IRQ */
      unsigned int RXOVR      :  1;  /**< SPI Rx FIFO overflow */
      unsigned int reserved8  :  3;
      unsigned int CS         :  1;  /**< CS Status */
      unsigned int CSERR      :  1;  /**< Detected a CS error condition in slave mode */
      unsigned int CSFALL     :  1;  /**< Detected a falling edge on CS, in slave CON mode */
      unsigned int CSRISE     :  1;  /**< Detected a rising edge on CS, in slave CON mode */
      unsigned int RDY        :  1;  /**< Detected an edge on Ready indicator for flow-control */
    };
    uint16_t VALUE16;
  };
} ADI_SPI_STAT_t;

/*@}*/

/** @defgroup RX Receive (RX) Register
 *  Receive (RX) Register.
 *
 *  Receive
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_RX_Struct
 *! \brief  Receive Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_RX_t {
  union {
    struct {
      unsigned int BYTE1      :  8;  /**< 8-bit receive buffer */
      unsigned int BYTE2      :  8;  /**< 8-bit receive buffer, used only in DMA modes */
    };
    uint16_t VALUE16;
  };
} ADI_SPI_RX_t;

/*@}*/

/** @defgroup TX Transmit (TX) Register
 *  Transmit (TX) Register.
 *
 *  Transmit
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_TX_Struct
 *! \brief  Transmit Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_TX_t {
  union {
    struct {
      unsigned int BYTE1      :  8;  /**< 8-bit transmit buffer */
      unsigned int BYTE2      :  8;  /**< 8-bit transmit buffer, used only in DMA modes */
    };
    uint16_t VALUE16;
  };
} ADI_SPI_TX_t;

/*@}*/

/** @defgroup DIV SPI baud rate selection (DIV) Register
 *  SPI baud rate selection (DIV) Register.
 *
 *  SPI baud rate selection
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_DIV_Struct
 *! \brief  SPI baud rate selection Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_DIV_t {
  union {
    struct {
      unsigned int VALUE      :  6;  /**< SPI clock divider */
      unsigned int reserved6  : 10;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_DIV_t;

/*@}*/

/** @defgroup CTL SPI configuration 1 (CTL) Register
 *  SPI configuration 1 (CTL) Register.
 *
 *  SPI configuration 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_CTL_Struct
 *! \brief  SPI configuration 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_CTL_t {
  union {
    struct {
      unsigned int SPIEN      :  1;  /**< SPI enable */
      unsigned int MASEN      :  1;  /**< Master mode enable */
      unsigned int CPHA       :  1;  /**< Serial clock phase mode */
      unsigned int CPOL       :  1;  /**< Serial Clock Polarity */
      unsigned int WOM        :  1;  /**< SPI Wired Or mode */
      unsigned int LSB        :  1;  /**< LSB first transfer enable */
      unsigned int TIM        :  1;  /**< SPI transfer and interrupt mode */
      unsigned int ZEN        :  1;  /**< Transmit zeros enable */
      unsigned int RXOF       :  1;  /**< RX overflow overwrite enable */
      unsigned int OEN        :  1;  /**< Slave MISO output enable */
      unsigned int LOOPBACK   :  1;  /**< Loopback enable */
      unsigned int CON        :  1;  /**< Continuous transfer enable */
      unsigned int RFLUSH     :  1;  /**< SPI Rx FIFO Flush enable */
      unsigned int TFLUSH     :  1;  /**< SPI Tx FIFO Flush enable */
      unsigned int CSRST      :  1;  /**< Reset Mode for CS Error bit */
      unsigned int reserved15 :  1;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_CTL_t;

/*@}*/

/** @defgroup IEN SPI configuration 2 (IEN) Register
 *  SPI configuration 2 (IEN) Register.
 *
 *  SPI configuration 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_IEN_Struct
 *! \brief  SPI configuration 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_IEN_t {
  union {
    struct {
      unsigned int IRQMODE    :  3;  /**< SPI IRQ mode bits */
      unsigned int reserved3  :  5;
      unsigned int CS         :  1;  /**< Enable interrupt on every CS edge in slave CON mode */
      unsigned int TXUNDR     :  1;  /**< Tx-underflow interrupt enable */
      unsigned int RXOVR      :  1;  /**< Rx-overflow interrupt enable */
      unsigned int RDY        :  1;  /**< Ready signal edge interrupt enable */
      unsigned int TXDONE     :  1;  /**< SPI transmit done interrupt enable */
      unsigned int XFRDONE    :  1;  /**< SPI transfer completion interrupt enable */
      unsigned int TXEMPTY    :  1;  /**< Tx-FIFO Empty interrupt enable */
      unsigned int reserved15 :  1;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_IEN_t;

/*@}*/

/** @defgroup CNT Transfer byte count (CNT) Register
 *  Transfer byte count (CNT) Register.
 *
 *  Transfer byte count
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_CNT_Struct
 *! \brief  Transfer byte count Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_CNT_t {
  union {
    struct {
      unsigned int VALUE      : 14;  /**< Transfer byte count */
      unsigned int reserved14 :  1;
      unsigned int FRAMECONT  :  1;  /**< Continue frame */
    };
    uint16_t VALUE16;
  };
} ADI_SPI_CNT_t;

/*@}*/

/** @defgroup DMA SPI DMA enable (DMA) Register
 *  SPI DMA enable (DMA) Register.
 *
 *  SPI DMA enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_DMA_Struct
 *! \brief  SPI DMA enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_DMA_t {
  union {
    struct {
      unsigned int EN         :  1;  /**< Enable DMA for data transfer */
      unsigned int TXEN       :  1;  /**< Enable transmit DMA request */
      unsigned int RXEN       :  1;  /**< Enable receive DMA request */
      unsigned int reserved3  : 13;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_DMA_t;

/*@}*/

/** @defgroup FIFO_STAT FIFO Status (FIFO_STAT) Register
 *  FIFO Status (FIFO_STAT) Register.
 *
 *  FIFO Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_FIFO_STAT_Struct
 *! \brief  FIFO Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_FIFO_STAT_t {
  union {
    struct {
      unsigned int TX         :  4;  /**< SPI Tx FIFO status */
      unsigned int reserved4  :  4;
      unsigned int RX         :  4;  /**< SPI Rx FIFO status */
      unsigned int reserved12 :  4;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_FIFO_STAT_t;

/*@}*/

/** @defgroup RD_CTL Read Control (RD_CTL) Register
 *  Read Control (RD_CTL) Register.
 *
 *  Read Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_RD_CTL_Struct
 *! \brief  Read Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_RD_CTL_t {
  union {
    struct {
      unsigned int CMDEN      :  1;  /**< Read command enable */
      unsigned int OVERLAP    :  1;  /**< Tx/Rx Overlap mode */
      unsigned int TXBYTES    :  4;  /**< Transmit byte count minus 1 for read command */
      unsigned int reserved6  :  2;
      unsigned int THREEPIN   :  1;  /**< Three pin SPI mode */
      unsigned int reserved9  :  7;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_RD_CTL_t;

/*@}*/

/** @defgroup FLOW_CTL Flow Control (FLOW_CTL) Register
 *  Flow Control (FLOW_CTL) Register.
 *
 *  Flow Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_FLOW_CTL_Struct
 *! \brief  Flow Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_FLOW_CTL_t {
  union {
    struct {
      unsigned int MODE       :  2;  /**< Flow control mode */
      unsigned int reserved2  :  2;
      unsigned int RDYPOL     :  1;  /**< Polarity of RDY/MISO line */
      unsigned int reserved5  :  3;
      unsigned int RDBURSTSZ  :  4;  /**< Read data burst size minus 1 */
      unsigned int reserved12 :  4;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_FLOW_CTL_t;

/*@}*/

/** @defgroup WAIT_TMR Wait timer for flow control (WAIT_TMR) Register
 *  Wait timer for flow control (WAIT_TMR) Register.
 *
 *  Wait timer for flow control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_WAIT_TMR_Struct
 *! \brief  Wait timer for flow control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_WAIT_TMR_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Wait timer for flow-control */
    };
    uint16_t VALUE16;
  };
} ADI_SPI_WAIT_TMR_t;

/*@}*/

/** @defgroup CS_CTL Chip-Select control for multi-slave connections (CS_CTL) Register
 *  Chip-Select control for multi-slave connections (CS_CTL) Register.
 *
 *  Chip-Select control for multi-slave connections
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_CS_CTL_Struct
 *! \brief  Chip-Select control for multi-slave connections Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_CS_CTL_t {
  union {
    struct {
      unsigned int SEL        :  4;  /**< Chip-Select control */
      unsigned int reserved4  : 12;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_CS_CTL_t;

/*@}*/

/** @defgroup CS_OVERRIDE Chip-Select Override (CS_OVERRIDE) Register
 *  Chip-Select Override (CS_OVERRIDE) Register.
 *
 *  Chip-Select Override
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_CS_OVERRIDE_Struct
 *! \brief  Chip-Select Override Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_CS_OVERRIDE_t {
  union {
    struct {
      unsigned int CTL        :  2;  /**< CS Override Control */
      unsigned int reserved2  : 14;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_CS_OVERRIDE_t;

/*@}*/

/** @defgroup XFR_CNT Transferred Count (XFR_CNT) Register
 *  Transferred Count (XFR_CNT) Register.
 *
 *  Transferred Count
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPI_XFR_CNT_Struct
 *! \brief  Transferred Count Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPI_XFR_CNT_t {
  union {
    struct {
      unsigned int BYTES      : 14;  /**< Number of bytes transferred over SPI. */
      unsigned int reserved14 :  2;
    };
    uint16_t VALUE16;
  };
} ADI_SPI_XFR_CNT_t;

/*@}*/

/** @defgroup TX Transmit Holding Register (TX) Register
 *  Transmit Holding Register (TX) Register.
 *
 *  Transmit Holding Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_TX_Struct
 *! \brief  Transmit Holding Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_TX_t {
  union {
    struct {
      unsigned int THR        :  8;  /**< Transmit Holding Register */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_UART_TX_t;

/*@}*/

/** @defgroup RX Receive Buffer Register (RX) Register
 *  Receive Buffer Register (RX) Register.
 *
 *  Receive Buffer Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_RX_Struct
 *! \brief  Receive Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_RX_t {
  union {
    struct {
      unsigned int RBR        :  8;  /**< Receive Buffer Register */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_UART_RX_t;

/*@}*/

/** @defgroup IEN Interrupt Enable (IEN) Register
 *  Interrupt Enable (IEN) Register.
 *
 *  Interrupt Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_IEN_Struct
 *! \brief  Interrupt Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_IEN_t {
  union {
    struct {
      unsigned int ERBFI      :  1;  /**< Receive buffer full interrupt */
      unsigned int ETBEI      :  1;  /**< Transmit buffer empty interrupt */
      unsigned int ELSI       :  1;  /**< Rx status interrupt */
      unsigned int EDSSI      :  1;  /**< Modem status interrupt */
      unsigned int EDMAT      :  1;  /**< DMA requests in transmit mode */
      unsigned int EDMAR      :  1;  /**< DMA requests in receive mode */
      unsigned int reserved6  : 10;
    };
    uint16_t VALUE16;
  };
} ADI_UART_IEN_t;

/*@}*/

/** @defgroup IIR Interrupt ID (IIR) Register
 *  Interrupt ID (IIR) Register.
 *
 *  Interrupt ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_IIR_Struct
 *! \brief  Interrupt ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_IIR_t {
  union {
    struct {
      unsigned int NIRQ       :  1;  /**< Interrupt flag */
      unsigned int STAT       :  2;  /**< Interrupt status */
      unsigned int reserved3  : 13;
    };
    uint16_t VALUE16;
  };
} ADI_UART_IIR_t;

/*@}*/

/** @defgroup LCR Line Control (LCR) Register
 *  Line Control (LCR) Register.
 *
 *  Line Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_LCR_Struct
 *! \brief  Line Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_LCR_t {
  union {
    struct {
      unsigned int WLS        :  2;  /**< Word Length Select */
      unsigned int STOP       :  1;  /**< Stop Bit */
      unsigned int PEN        :  1;  /**< Parity Enable */
      unsigned int EPS        :  1;  /**< Parity Select */
      unsigned int SP         :  1;  /**< Stick Parity */
      unsigned int BRK        :  1;  /**< Set Break */
      unsigned int reserved7  :  9;
    };
    uint16_t VALUE16;
  };
} ADI_UART_LCR_t;

/*@}*/

/** @defgroup MCR Modem Control (MCR) Register
 *  Modem Control (MCR) Register.
 *
 *  Modem Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_MCR_Struct
 *! \brief  Modem Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_MCR_t {
  union {
    struct {
      unsigned int DTR        :  1;  /**< Data Terminal Ready */
      unsigned int RTS        :  1;  /**< Request to send */
      unsigned int OUT1       :  1;  /**< Output 1 */
      unsigned int OUT2       :  1;  /**< Output 2 */
      unsigned int LOOPBACK   :  1;  /**< Loopback mode */
      unsigned int reserved5  : 11;
    };
    uint16_t VALUE16;
  };
} ADI_UART_MCR_t;

/*@}*/

/** @defgroup LSR Line Status (LSR) Register
 *  Line Status (LSR) Register.
 *
 *  Line Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_LSR_Struct
 *! \brief  Line Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_LSR_t {
  union {
    struct {
      unsigned int DR         :  1;  /**< Data Ready */
      unsigned int OE         :  1;  /**< Overrun Error */
      unsigned int PE         :  1;  /**< Parity Error */
      unsigned int FE         :  1;  /**< Framing Error */
      unsigned int BI         :  1;  /**< Break Indicator */
      unsigned int THRE       :  1;  /**< Transmit Register Empty */
      unsigned int TEMT       :  1;  /**< Transmit and Shift Register Empty Status */
      unsigned int reserved7  :  9;
    };
    uint16_t VALUE16;
  };
} ADI_UART_LSR_t;

/*@}*/

/** @defgroup MSR Modem Status (MSR) Register
 *  Modem Status (MSR) Register.
 *
 *  Modem Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_MSR_Struct
 *! \brief  Modem Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_MSR_t {
  union {
    struct {
      unsigned int DCTS       :  1;  /**< Delta CTS */
      unsigned int DDSR       :  1;  /**< Delta DSR */
      unsigned int TERI       :  1;  /**< Trailing Edge RI */
      unsigned int DDCD       :  1;  /**< Delta DCD */
      unsigned int CTS        :  1;  /**< Clear To Send */
      unsigned int DSR        :  1;  /**< Data Set Ready */
      unsigned int RI         :  1;  /**< Ring Indicator */
      unsigned int DCD        :  1;  /**< Data Carrier Detect */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_UART_MSR_t;

/*@}*/

/** @defgroup SCR Scratch buffer (SCR) Register
 *  Scratch buffer (SCR) Register.
 *
 *  Scratch buffer
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_SCR_Struct
 *! \brief  Scratch buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_SCR_t {
  union {
    struct {
      unsigned int SCR        :  8;  /**< Scratch */
      unsigned int reserved8  :  8;
    };
    uint16_t VALUE16;
  };
} ADI_UART_SCR_t;

/*@}*/

/** @defgroup FBR Fractional Baud Rate (FBR) Register
 *  Fractional Baud Rate (FBR) Register.
 *
 *  Fractional Baud Rate
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_FBR_Struct
 *! \brief  Fractional Baud Rate Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_FBR_t {
  union {
    struct {
      unsigned int DIVN       : 11;  /**< Fractional baud rate N divide bits 0 to 2047. */
      unsigned int DIVM       :  2;  /**< Fractional baud rate M divide bits 1 to 3 */
      unsigned int reserved13 :  2;
      unsigned int FBEN       :  1;  /**< Fractional baud rate generator enable */
    };
    uint16_t VALUE16;
  };
} ADI_UART_FBR_t;

/*@}*/

/** @defgroup DIV Baudrate divider (DIV) Register
 *  Baudrate divider (DIV) Register.
 *
 *  Baudrate divider
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_DIV_Struct
 *! \brief  Baudrate divider Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_DIV_t {
  union {
    struct {
      unsigned int DIV        : 16;  /**< Baud rate divider */
    };
    uint16_t VALUE16;
  };
} ADI_UART_DIV_t;

/*@}*/

/** @defgroup CTL UART control register (CTL) Register
 *  UART control register (CTL) Register.
 *
 *  UART control register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_UART_CTL_Struct
 *! \brief  UART control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_UART_CTL_t {
  union {
    struct {
      unsigned int reserved0  :  1;
      unsigned int FORCECLK   :  1;  /**< Force UCLK on */
      unsigned int reserved2  : 14;
    };
    uint16_t VALUE16;
  };
} ADI_UART_CTL_t;

/*@}*/

/** @defgroup CFG Beeper configuration (CFG) Register
 *  Beeper configuration (CFG) Register.
 *
 *  Beeper configuration
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BEEP_CFG_Struct
 *! \brief  Beeper configuration Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BEEP_CFG_t {
  union {
    struct {
      unsigned int SEQREPEAT  :  8;  /**< Beeper Sequence Repeat value */
      unsigned int EN         :  1;  /**< Beeper Enable */
      unsigned int reserved9  :  1;
      unsigned int ASTARTIRQ  :  1;  /**< Tone A start IRQ */
      unsigned int AENDIRQ    :  1;  /**< Tone A end IRQ */
      unsigned int BSTARTIRQ  :  1;  /**< Tone B start IRQ */
      unsigned int BENDIRQ    :  1;  /**< Tone B end IRQ */
      unsigned int SEQNEARENDIRQ :  1;  /**< Sequence 1 cycle from end IRQ */
      unsigned int SEQATENDIRQ   :  1;  /**< Sequence end IRQ */
    };
    uint16_t VALUE16;
  };
} ADI_BEEP_CFG_t;

/*@}*/

/** @defgroup STAT Beeper status (STAT) Register
 *  Beeper status (STAT) Register.
 *
 *  Beeper status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BEEP_STAT_Struct
 *! \brief  Beeper status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BEEP_STAT_t {
  union {
    struct {
      unsigned int SEQREMAIN  :  8;  /**< Remaining tone-pair sequence iterations to play in SEQ mode */
      unsigned int BUSY       :  1;  /**< Beeper is busy */
      unsigned int reserved9  :  1;
      unsigned int ASTARTED   :  1;  /**< Tone A has started */
      unsigned int AENDED     :  1;  /**< Tone A has ended */
      unsigned int BSTARTED   :  1;  /**< Tone B has started */
      unsigned int BENDED     :  1;  /**< Tone B has ended */
      unsigned int SEQNEAREND :  1;  /**< Sequencer last tone-pair has started */
      unsigned int SEQENDED   :  1;  /**< Sequencer has ended */
    };
    uint16_t VALUE16;
  };
} ADI_BEEP_STAT_t;

/*@}*/

/** @defgroup TONEA Tone A Data (TONEA) Register
 *  Tone A Data (TONEA) Register.
 *
 *  Tone A Data
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BEEP_TONEA_Struct
 *! \brief  Tone A Data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BEEP_TONEA_t {
  union {
    struct {
      unsigned int DUR        :  8;  /**< Tone duration */
      unsigned int FREQ       :  7;  /**< Tone frequency */
      unsigned int DIS        :  1;  /**< Output disable */
    };
    uint16_t VALUE16;
  };
} ADI_BEEP_TONEA_t;

/*@}*/

/** @defgroup TONEB Tone B Data (TONEB) Register
 *  Tone B Data (TONEB) Register.
 *
 *  Tone B Data
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BEEP_TONEB_Struct
 *! \brief  Tone B Data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BEEP_TONEB_t {
  union {
    struct {
      unsigned int DUR        :  8;  /**< Tone duration */
      unsigned int FREQ       :  7;  /**< Tone frequency */
      unsigned int DIS        :  1;  /**< Output disable */
    };
    uint16_t VALUE16;
  };
} ADI_BEEP_TONEB_t;

/*@}*/

/** @defgroup STAT DMA Status (STAT) Register
 *  DMA Status (STAT) Register.
 *
 *  DMA Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_STAT_Struct
 *! \brief  DMA Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_STAT_t {
  union {
    struct {
      unsigned int MEN        :  1;  /**< Enable status of the controller */
      unsigned int reserved1  :  3;
      unsigned int STATE      :  4;  /**< Current state of DMA controller */
      unsigned int reserved8  :  8;
      unsigned int CHANM1     :  5;  /**< Number of available DMA channels minus 1 */
      unsigned int reserved21 : 11;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_STAT_t;

/*@}*/

/** @defgroup CFG DMA Configuration (CFG) Register
 *  DMA Configuration (CFG) Register.
 *
 *  DMA Configuration
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_CFG_Struct
 *! \brief  DMA Configuration Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_CFG_t {
  union {
    struct {
      unsigned int MEN        :  1;  /**< Controller enable */
      unsigned int reserved1  : 31;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_CFG_t;

/*@}*/

/** @defgroup PDBPTR DMA channel primary control data base pointer (PDBPTR) Register
 *  DMA channel primary control data base pointer (PDBPTR) Register.
 *
 *  DMA channel primary control data base pointer
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_PDBPTR_Struct
 *! \brief  DMA channel primary control data base pointer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_PDBPTR_t {
  union {
    struct {
      unsigned int ADDR       : 32;  /**< Pointer to the base address of the primary data structure */
    };
    uint32_t VALUE32;
  };
} ADI_DMA_PDBPTR_t;

/*@}*/

/** @defgroup ADBPTR DMA channel alternate control data base pointer (ADBPTR) Register
 *  DMA channel alternate control data base pointer (ADBPTR) Register.
 *
 *  DMA channel alternate control data base pointer
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_ADBPTR_Struct
 *! \brief  DMA channel alternate control data base pointer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_ADBPTR_t {
  union {
    struct {
      unsigned int ADDR       : 32;  /**< Base address of the alternate data structure */
    };
    uint32_t VALUE32;
  };
} ADI_DMA_ADBPTR_t;

/*@}*/

/** @defgroup SWREQ DMA channel software request (SWREQ) Register
 *  DMA channel software request (SWREQ) Register.
 *
 *  DMA channel software request
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_SWREQ_Struct
 *! \brief  DMA channel software request Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_SWREQ_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Generate software request */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_SWREQ_t;

/*@}*/

/** @defgroup RMSK_SET DMA channel request mask set (RMSK_SET) Register
 *  DMA channel request mask set (RMSK_SET) Register.
 *
 *  DMA channel request mask set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_RMSK_SET_Struct
 *! \brief  DMA channel request mask set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_RMSK_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Mask requests from DMA channels */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_RMSK_SET_t;

/*@}*/

/** @defgroup RMSK_CLR DMA channel request mask clear (RMSK_CLR) Register
 *  DMA channel request mask clear (RMSK_CLR) Register.
 *
 *  DMA channel request mask clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_RMSK_CLR_Struct
 *! \brief  DMA channel request mask clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_RMSK_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Clear Request Mask Set bits */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_RMSK_CLR_t;

/*@}*/

/** @defgroup EN_SET DMA channel enable set (EN_SET) Register
 *  DMA channel enable set (EN_SET) Register.
 *
 *  DMA channel enable set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_EN_SET_Struct
 *! \brief  DMA channel enable set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_EN_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Enable DMA channels */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_EN_SET_t;

/*@}*/

/** @defgroup EN_CLR DMA channel enable clear (EN_CLR) Register
 *  DMA channel enable clear (EN_CLR) Register.
 *
 *  DMA channel enable clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_EN_CLR_Struct
 *! \brief  DMA channel enable clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_EN_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Disable DMA channels */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_EN_CLR_t;

/*@}*/

/** @defgroup ALT_SET DMA channel primary-alternate set (ALT_SET) Register
 *  DMA channel primary-alternate set (ALT_SET) Register.
 *
 *  DMA channel primary-alternate set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_ALT_SET_Struct
 *! \brief  DMA channel primary-alternate set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_ALT_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Control struct status / select alt struct */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_ALT_SET_t;

/*@}*/

/** @defgroup ALT_CLR DMA channel primary-alternate clear (ALT_CLR) Register
 *  DMA channel primary-alternate clear (ALT_CLR) Register.
 *
 *  DMA channel primary-alternate clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_ALT_CLR_Struct
 *! \brief  DMA channel primary-alternate clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_ALT_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Select primary data struct */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_ALT_CLR_t;

/*@}*/

/** @defgroup PRI_SET DMA channel priority set (PRI_SET) Register
 *  DMA channel priority set (PRI_SET) Register.
 *
 *  DMA channel priority set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_PRI_SET_Struct
 *! \brief  DMA channel priority set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_PRI_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Configure channel for high priority */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_PRI_SET_t;

/*@}*/

/** @defgroup PRI_CLR DMA channel priority clear (PRI_CLR) Register
 *  DMA channel priority clear (PRI_CLR) Register.
 *
 *  DMA channel priority clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_PRI_CLR_Struct
 *! \brief  DMA channel priority clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_PRI_CLR_t {
  union {
    struct {
      unsigned int CHPRICLR   : 24;  /**< Configure channel for default priority level */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_PRI_CLR_t;

/*@}*/

/** @defgroup ERRCHNL_CLR DMA Per Channel Error Clear (ERRCHNL_CLR) Register
 *  DMA Per Channel Error Clear (ERRCHNL_CLR) Register.
 *
 *  DMA Per Channel Error Clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_ERRCHNL_CLR_Struct
 *! \brief  DMA Per Channel Error Clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_ERRCHNL_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Per channel Bus error status/ Per channel bus error clear */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_ERRCHNL_CLR_t;

/*@}*/

/** @defgroup ERR_CLR DMA bus error clear (ERR_CLR) Register
 *  DMA bus error clear (ERR_CLR) Register.
 *
 *  DMA bus error clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_ERR_CLR_Struct
 *! \brief  DMA bus error clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_ERR_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Bus error status */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_ERR_CLR_t;

/*@}*/

/** @defgroup INVALIDDESC_CLR DMA Per Channel Invalid Descriptor Clear (INVALIDDESC_CLR) Register
 *  DMA Per Channel Invalid Descriptor Clear (INVALIDDESC_CLR) Register.
 *
 *  DMA Per Channel Invalid Descriptor Clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_INVALIDDESC_CLR_Struct
 *! \brief  DMA Per Channel Invalid Descriptor Clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_INVALIDDESC_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Per channel Invalid Descriptor status/ Per channel Invalid descriptor status clear */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_INVALIDDESC_CLR_t;

/*@}*/

/** @defgroup BS_SET DMA channel bytes swap enable set (BS_SET) Register
 *  DMA channel bytes swap enable set (BS_SET) Register.
 *
 *  DMA channel bytes swap enable set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_BS_SET_Struct
 *! \brief  DMA channel bytes swap enable set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_BS_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Byte swap status */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_BS_SET_t;

/*@}*/

/** @defgroup BS_CLR DMA channel bytes swap enable clear (BS_CLR) Register
 *  DMA channel bytes swap enable clear (BS_CLR) Register.
 *
 *  DMA channel bytes swap enable clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_BS_CLR_Struct
 *! \brief  DMA channel bytes swap enable clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_BS_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Disable byte swap */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_BS_CLR_t;

/*@}*/

/** @defgroup SRCADDR_SET DMA channel source address decrement enable set (SRCADDR_SET) Register
 *  DMA channel source address decrement enable set (SRCADDR_SET) Register.
 *
 *  DMA channel source address decrement enable set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_SRCADDR_SET_Struct
 *! \brief  DMA channel source address decrement enable set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_SRCADDR_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Source Address decrement status / configure Source address decrement */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_SRCADDR_SET_t;

/*@}*/

/** @defgroup SRCADDR_CLR DMA channel source address decrement enable clear (SRCADDR_CLR) Register
 *  DMA channel source address decrement enable clear (SRCADDR_CLR) Register.
 *
 *  DMA channel source address decrement enable clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_SRCADDR_CLR_Struct
 *! \brief  DMA channel source address decrement enable clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_SRCADDR_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Disable source address decrement */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_SRCADDR_CLR_t;

/*@}*/

/** @defgroup DSTADDR_SET DMA channel destination address decrement enable set (DSTADDR_SET) Register
 *  DMA channel destination address decrement enable set (DSTADDR_SET) Register.
 *
 *  DMA channel destination address decrement enable set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_DSTADDR_SET_Struct
 *! \brief  DMA channel destination address decrement enable set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_DSTADDR_SET_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Destination Address decrement status / configure destination address decrement */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_DSTADDR_SET_t;

/*@}*/

/** @defgroup DSTADDR_CLR DMA channel destination address decrement enable clear (DSTADDR_CLR) Register
 *  DMA channel destination address decrement enable clear (DSTADDR_CLR) Register.
 *
 *  DMA channel destination address decrement enable clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_DSTADDR_CLR_Struct
 *! \brief  DMA channel destination address decrement enable clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_DSTADDR_CLR_t {
  union {
    struct {
      unsigned int CHAN       : 24;  /**< Disable destination address decrement */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_DSTADDR_CLR_t;

/*@}*/

/** @defgroup REVID DMA Controller Revision ID (REVID) Register
 *  DMA Controller Revision ID (REVID) Register.
 *
 *  DMA Controller Revision ID
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_DMA_REVID_Struct
 *! \brief  DMA Controller Revision ID Register bit field structure
 * ========================================================================== */
typedef struct _ADI_DMA_REVID_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< DMA Controller revision ID */
      unsigned int reserved8  : 24;
    };
    uint32_t VALUE32;
  };
} ADI_DMA_REVID_t;

/*@}*/

/** @defgroup STAT Status (STAT) Register
 *  Status (STAT) Register.
 *
 *  Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_STAT_Struct
 *! \brief  Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_STAT_t {
  union {
    struct {
      unsigned int CMDBUSY    :  1;  /**< Command busy */
      unsigned int WRCLOSE    :  1;  /**< WRITE registers are closed */
      unsigned int CMDCOMP    :  1;  /**< Command complete */
      unsigned int WRALCOMP   :  1;  /**< Write almost complete */
      unsigned int CMDFAIL    :  2;  /**< Provides information on command failures */
      unsigned int SLEEPING   :  1;  /**< Flash array is in low power (sleep) mode */
      unsigned int ECCERRCMD  :  2;  /**< ECC errors detected during user issued SIGN command */
      unsigned int ECCRDERR   :  2;  /**< ECC IRQ cause */
      unsigned int OVERLAP    :  1;  /**< Overlapping Command */
      unsigned int reserved12 :  1;
      unsigned int SIGNERR    :  1;  /**< Signature check failure during initialization */
      unsigned int INIT       :  1;  /**< Flash controller initialization in progress */
      unsigned int ECCINFOSIGN :  2;  /**< ECC status of flash initialization */
      unsigned int ECCERRCNT   :  3;  /**< ECC correction counter */
      unsigned int reserved20  :  5;
      unsigned int ECCICODE    :  2;  /**< ICode AHB Bus Error ECC status */
      unsigned int ECCDCODE    :  2;  /**< DCode AHB Bus Error ECC status */
      unsigned int CACHESRAMPERR :  1;  /**< SRAM parity errors in Cache Controller */
      unsigned int reserved30    :  2;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_STAT_t;

/*@}*/

/** @defgroup IEN Interrupt Enable (IEN) Register
 *  Interrupt Enable (IEN) Register.
 *
 *  Interrupt Enable
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_FLCC_IEN_ECC_CORRECT
 *! \brief  Control whether to generate bus errors, interrupts, or neither in response to 1-bit ECC Correction events (ECC_CORRECT) Enumerations
 *  ========================================================================= */
typedef enum
{
  FLCC_IEN_NONE_COR    = 0,  /**< Do not generate a response to ECC events      */
  FLCC_IEN_BUS_ERR_COR = 1,  /**< Generate Bus Errors in response to ECC events */
  FLCC_IEN_IRQ_COR     = 2   /**< Generate IRQs in response to ECC events       */
} ADI_FLCC_IEN_ECC_CORRECT;


/*  =========================================================================
 *! \enum   ADI_FLCC_IEN_ECC_ERROR
 *! \brief  Control whether to generate bus errors, interrupts, or neither in response to 2-bit ECC Error events (ECC_ERROR) Enumerations
 *  ========================================================================= */
typedef enum
{
  FLCC_IEN_NONE_ERR    = 0,  /**< Do not generate a response to ECC events      */
  FLCC_IEN_BUS_ERR_ERR = 1,  /**< Generate Bus Errors in response to ECC events */
  FLCC_IEN_IRQ_ERR     = 2   /**< Generate IRQs in response to ECC events       */
} ADI_FLCC_IEN_ECC_ERROR;


/* ==========================================================================
 *! \struct ADI_FLCC_IEN_Struct
 *! \brief  Interrupt Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_IEN_t {
  union {
    struct {
      unsigned int CMDCMPLT   :  1;  /**< Command complete interrupt enable */
      unsigned int WRALCMPLT  :  1;  /**< Write almost complete interrupt enable */
      unsigned int CMDFAIL    :  1;  /**< Command fail interrupt enable */
      unsigned int reserved3   :  1;
      unsigned int ECC_CORRECT :  2;  /**< Control whether to generate bus errors, interrupts, or neither in response to 1-bit ECC Correction events */
      unsigned int ECC_ERROR   :  2;  /**< Control whether to generate bus errors, interrupts, or neither in response to 2-bit ECC Error events */
      unsigned int reserved8   : 24;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_IEN_t;

/*@}*/

/** @defgroup CMD Command (CMD) Register
 *  Command (CMD) Register.
 *
 *  Command
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_FLCC_CMD_VALUE
 *! \brief  Commands (VALUE) Enumerations
 *  ========================================================================= */
typedef enum
{
  FLCC_CMD_IDLE        = 0,  /**< IDLE                                                                  */
  FLCC_CMD_ABORT       = 1,  /**< ABORT                                                                 */
  FLCC_CMD_SLEEP       = 2,  /**< Requests flash to enter Sleep mode                                    */
  FLCC_CMD_SIGN        = 3,  /**< SIGN                                                                  */
  FLCC_CMD_WRITE       = 4,  /**< WRITE                                                                 */
  FLCC_CMD_BLANK_CHECK = 5,  /**< Checks all of User Space; fails if any bits in user space are cleared */
  FLCC_CMD_ERASEPAGE   = 6,  /**< ERASEPAGE                                                             */
  FLCC_CMD_MASSERASE   = 7   /**< MASSERASE                                                             */
} ADI_FLCC_CMD_VALUE;


/* ==========================================================================
 *! \struct ADI_FLCC_CMD_Struct
 *! \brief  Command Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_CMD_t {
  union {
    struct {
      unsigned int VALUE      :  4;  /**< Commands */
      unsigned int reserved4  : 28;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_CMD_t;

/*@}*/

/** @defgroup KH_ADDR WRITE address (KH_ADDR) Register
 *  WRITE address (KH_ADDR) Register.
 *
 *  WRITE address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_KH_ADDR_Struct
 *! \brief  WRITE address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_KH_ADDR_t {
  union {
    struct {
      unsigned int reserved0  :  3;
      unsigned int VALUE      : 16;
      unsigned int reserved19 : 13;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_KH_ADDR_t;

/*@}*/

/** @defgroup KH_DATA0 WRITE lower data (KH_DATA0) Register
 *  WRITE lower data (KH_DATA0) Register.
 *
 *  WRITE lower data
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_KH_DATA0_Struct
 *! \brief  WRITE lower data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_KH_DATA0_t {
  union {
    struct {
      unsigned int VALUE      : 32;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_KH_DATA0_t;

/*@}*/

/** @defgroup KH_DATA1 WRITE upper data (KH_DATA1) Register
 *  WRITE upper data (KH_DATA1) Register.
 *
 *  WRITE upper data
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_KH_DATA1_Struct
 *! \brief  WRITE upper data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_KH_DATA1_t {
  union {
    struct {
      unsigned int VALUE      : 32;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_KH_DATA1_t;

/*@}*/

/** @defgroup PAGE_ADDR0 Lower page address (PAGE_ADDR0) Register
 *  Lower page address (PAGE_ADDR0) Register.
 *
 *  Lower page address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_PAGE_ADDR0_Struct
 *! \brief  Lower page address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_PAGE_ADDR0_t {
  union {
    struct {
      unsigned int reserved0  : 10;
      unsigned int VALUE      :  9;
      unsigned int reserved19 : 13;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_PAGE_ADDR0_t;

/*@}*/

/** @defgroup PAGE_ADDR1 Upper page address (PAGE_ADDR1) Register
 *  Upper page address (PAGE_ADDR1) Register.
 *
 *  Upper page address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_PAGE_ADDR1_Struct
 *! \brief  Upper page address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_PAGE_ADDR1_t {
  union {
    struct {
      unsigned int reserved0  : 10;
      unsigned int VALUE      :  9;
      unsigned int reserved19 : 13;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_PAGE_ADDR1_t;

/*@}*/

/** @defgroup KEY Key (KEY) Register
 *  Key (KEY) Register.
 *
 *  Key
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_FLCC_KEY_VALUE
 *! \brief  Key register (VALUE) Enumerations
 *  ========================================================================= */
typedef enum
{
  FLCC_KEY_USERKEY = 1735161189   /**< USERKEY */
} ADI_FLCC_KEY_VALUE;


/* ==========================================================================
 *! \struct ADI_FLCC_KEY_Struct
 *! \brief  Key Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_KEY_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key register */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_KEY_t;

/*@}*/

/** @defgroup WR_ABORT_ADDR Write abort address (WR_ABORT_ADDR) Register
 *  Write abort address (WR_ABORT_ADDR) Register.
 *
 *  Write abort address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_WR_ABORT_ADDR_Struct
 *! \brief  Write abort address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_WR_ABORT_ADDR_t {
  union {
    struct {
      unsigned int VALUE      : 32;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_WR_ABORT_ADDR_t;

/*@}*/

/** @defgroup WRPROT Write protection (WRPROT) Register
 *  Write protection (WRPROT) Register.
 *
 *  Write protection
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_WRPROT_Struct
 *! \brief  Write protection Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_WRPROT_t {
  union {
    struct {
      unsigned int WORD       : 32;  /**< Clear bits to write protect related groups of user space pages. Once cleared these bits can only be set again by resetting the part */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_WRPROT_t;

/*@}*/

/** @defgroup SIGNATURE Signature (SIGNATURE) Register
 *  Signature (SIGNATURE) Register.
 *
 *  Signature
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_SIGNATURE_Struct
 *! \brief  Signature Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_SIGNATURE_t {
  union {
    struct {
      unsigned int VALUE      : 32;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_SIGNATURE_t;

/*@}*/

/** @defgroup UCFG User configuration (UCFG) Register
 *  User configuration (UCFG) Register.
 *
 *  User configuration
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_UCFG_Struct
 *! \brief  User configuration Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_UCFG_t {
  union {
    struct {
      unsigned int KHDMAEN    :  1;  /**< Key-Hole DMA enable. */
      unsigned int AUTOINCEN  :  1;  /**< Auto address increment for Key hole access. */
      unsigned int reserved2  : 30;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_UCFG_t;

/*@}*/

/** @defgroup TIME_PARAM0 Time parameter 0 (TIME_PARAM0) Register
 *  Time parameter 0 (TIME_PARAM0) Register.
 *
 *  Time parameter 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_TIME_PARAM0_Struct
 *! \brief  Time parameter 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_TIME_PARAM0_t {
  union {
    struct {
      unsigned int DIVREFCLK  :  1;  /**< Divide Reference Clock (by 2) */
      unsigned int reserved1  :  3;
      unsigned int TNVS       :  4;  /**< PROG/ERASE to NVSTR setup time */
      unsigned int TPGS       :  4;  /**< NVSTR to Program setup time */
      unsigned int TPROG      :  4;  /**< Program time */
      unsigned int TNVH       :  4;  /**< NVSTR Hold time */
      unsigned int TRCV       :  4;  /**< Recovery time */
      unsigned int TERASE     :  4;  /**< Erase Time */
      unsigned int TNVH1      :  4;  /**< NVSTR Hold time during Mass Erase */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_TIME_PARAM0_t;

/*@}*/

/** @defgroup TIME_PARAM1 Time parameter 1 (TIME_PARAM1) Register
 *  Time parameter 1 (TIME_PARAM1) Register.
 *
 *  Time parameter 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_TIME_PARAM1_Struct
 *! \brief  Time parameter 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_TIME_PARAM1_t {
  union {
    struct {
      unsigned int TWK        :  4;  /**< Wake up time */
      unsigned int reserved4  : 28;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_TIME_PARAM1_t;

/*@}*/

/** @defgroup ABORT_EN_LO IRQ Abort Enable (lower bits) (ABORT_EN_LO) Register
 *  IRQ Abort Enable (lower bits) (ABORT_EN_LO) Register.
 *
 *  IRQ Abort Enable (lower bits)
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_ABORT_EN_LO_Struct
 *! \brief  IRQ Abort Enable (lower bits) Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_ABORT_EN_LO_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< VALUE[31:0] Sys IRQ abort enable */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_ABORT_EN_LO_t;

/*@}*/

/** @defgroup ABORT_EN_HI IRQ Abort Enable (upper bits) (ABORT_EN_HI) Register
 *  IRQ Abort Enable (upper bits) (ABORT_EN_HI) Register.
 *
 *  IRQ Abort Enable (upper bits)
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_ABORT_EN_HI_Struct
 *! \brief  IRQ Abort Enable (upper bits) Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_ABORT_EN_HI_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< VALUE[63:32] Sys IRQ abort enable */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_ABORT_EN_HI_t;

/*@}*/

/** @defgroup ECC_CFG ECC Config (ECC_CFG) Register
 *  ECC Config (ECC_CFG) Register.
 *
 *  ECC Config
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_ECC_CFG_Struct
 *! \brief  ECC Config Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_ECC_CFG_t {
  union {
    struct {
      unsigned int EN         :  1;  /**< ECC Enable */
      unsigned int INFOEN     :  1;  /**< Info space ECC Enable bit */
      unsigned int reserved2  :  6;
      unsigned int PTR        : 24;  /**< ECC start page pointer (user should write bits [31:8] of the start page address into bits [31:8] of this register) */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_ECC_CFG_t;

/*@}*/

/** @defgroup ECC_ADDR ECC Status (address) (ECC_ADDR) Register
 *  ECC Status (address) (ECC_ADDR) Register.
 *
 *  ECC Status (address)
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_ECC_ADDR_Struct
 *! \brief  ECC Status (address) Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_ECC_ADDR_t {
  union {
    struct {
      unsigned int VALUE      : 19;
      unsigned int reserved19 : 13;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_ECC_ADDR_t;

/*@}*/

/** @defgroup STAT Cache Status register (STAT) Register
 *  Cache Status register (STAT) Register.
 *
 *  Cache Status register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_CACHE_STAT_Struct
 *! \brief  Cache Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_CACHE_STAT_t {
  union {
    struct {
      unsigned int ICEN       :  1;  /**< If this bit is set then I-Cache is enabled and when cleared I-Cache is disabled. */
      unsigned int ICLCK      :  1;  /**< This bit is set when I-Cache is locked and cleared when I-cache is unlocked. */
      unsigned int reserved2  : 14;
      unsigned int DCEN       :  1;  /**< If this bit is set then D-Cache is enabled and when cleared D-Cache is disabled. */
      unsigned int DCLCK      :  1;  /**< This bit is set when D-Cache is locked and cleared when D-cache is unlocked. */
      unsigned int reserved18 : 14;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_CACHE_STAT_t;

/*@}*/

/** @defgroup SETUP Cache Setup register (SETUP) Register
 *  Cache Setup register (SETUP) Register.
 *
 *  Cache Setup register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_CACHE_SETUP_Struct
 *! \brief  Cache Setup Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_CACHE_SETUP_t {
  union {
    struct {
      unsigned int ICEN       :  1;  /**< If this bit set, then I-Cache is enabled for AHB accesses. If 0, then I-cache is disabled, and all AHB accesses will be via Flash memory. This bit is cleared when Init bit is set. */
      unsigned int LCKIC      :  1;  /**< If this bit is set, then I-cache contents are locked. Any new misses won't be replaced in I-Cache. */
      unsigned int reserved2  : 14;
      unsigned int DCEN       :  1;  /**< If this bit set, then D-Cache is enabled for AHB accesses. If 0, then D-cache is disabled, and all AHB accesses will be via Flash memory. */
      unsigned int LCKDC      :  1;  /**< If this bit is set, then D-cache contents are locked. Any new misses won't be replaced in D-Cache. */
      unsigned int reserved18 : 14;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_CACHE_SETUP_t;

/*@}*/

/** @defgroup KEY Cache Key register (KEY) Register
 *  Cache Key register (KEY) Register.
 *
 *  Cache Key register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_FLCC_CACHE_KEY_Struct
 *! \brief  Cache Key Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_CACHE_KEY_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Cache Key register */
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_CACHE_KEY_t;

/*@}*/

/** @defgroup CMD Command register (CMD) Register
 *  Command register (CMD) Register.
 *
 *  Command register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_FLCC_DFT_CMD_VALUE
 *! \brief  Asserted by user code to select the next command to be executed (may be written at any time) (VALUE) Enumerations
 *  ========================================================================= */
typedef enum
{
  FLCC_DFT_CMD_IDLE_CMD   = 0,  /**< Idle command                                                                                    */
  FLCC_DFT_CMD_READ_CMD   = 1,  /**< Read command                                                                                    */
  FLCC_DFT_CMD_PROG_CMD   = 2,  /**< Program command                                                                                 */
  FLCC_DFT_CMD_ERASE_CMD  = 3,  /**< Erase command                                                                                   */
  FLCC_DFT_CMD_TMRSET_CMD = 4   /**< Test Mode Register Set command (note: this command provides Erase Reference Cell functionality) */
} ADI_FLCC_DFT_CMD_VALUE;


/* ==========================================================================
 *! \struct ADI_FLCC_DFT_CMD_Struct
 *! \brief  Command Register bit field structure
 * ========================================================================== */
typedef struct _ADI_FLCC_DFT_CMD_t {
  union {
    struct {
      unsigned int VALUE      :  3;  /**< Asserted by user code to select the next command to be executed (may be written at any time) */
      unsigned int reserved3  : 29;
    };
    uint32_t VALUE32;
  };
} ADI_FLCC_DFT_CMD_t;

/*@}*/

/** @defgroup CFG Port  Configuration (CFG) Register
 *  Port  Configuration (CFG) Register.
 *
 *  Port  Configuration
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_CFG_Struct
 *! \brief  Port  Configuration Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_CFG_t {
  union {
    struct {
      unsigned int PIN00      :  2;  /**< Pin 0 configuration bits */
      unsigned int PIN01      :  2;  /**< Pin 1 configuration bits */
      unsigned int PIN02      :  2;  /**< Pin 2 configuration bits */
      unsigned int PIN03      :  2;  /**< Pin 3 configuration bits */
      unsigned int PIN04      :  2;  /**< Pin 4 configuration bits */
      unsigned int PIN05      :  2;  /**< Pin 5 configuration bits */
      unsigned int PIN06      :  2;  /**< Pin 6 configuration bits */
      unsigned int PIN07      :  2;  /**< Pin 7 configuration bits */
      unsigned int PIN08      :  2;  /**< Pin 8 configuration bits */
      unsigned int PIN09      :  2;  /**< Pin 9 configuration bits */
      unsigned int PIN10      :  2;  /**< Pin 10  configuration bits */
      unsigned int PIN11      :  2;  /**< Pin 11  configuration bits */
      unsigned int PIN12      :  2;  /**< Pin 12  configuration bits */
      unsigned int PIN13      :  2;  /**< Pin 13  configuration bits */
      unsigned int PIN14      :  2;  /**< Pin 14  configuration bits */
      unsigned int PIN15      :  2;  /**< Pin 15  configuration bits */
    };
    uint32_t VALUE32;
  };
} ADI_GPIO_CFG_t;

/*@}*/

/** @defgroup OEN Port output enable (OEN) Register
 *  Port output enable (OEN) Register.
 *
 *  Port output enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_OEN_Struct
 *! \brief  Port output enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_OEN_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Pin Output Drive enable */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_OEN_t;

/*@}*/

/** @defgroup PE Port output pullup/pulldown enable (PE) Register
 *  Port output pullup/pulldown enable (PE) Register.
 *
 *  Port output pullup/pulldown enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_PE_Struct
 *! \brief  Port output pullup/pulldown enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_PE_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Pin Pull enable */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_PE_t;

/*@}*/

/** @defgroup IEN Port  Input Path Enable (IEN) Register
 *  Port  Input Path Enable (IEN) Register.
 *
 *  Port  Input Path Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_IEN_Struct
 *! \brief  Port  Input Path Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_IEN_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Input path enable */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_IEN_t;

/*@}*/

/** @defgroup IN Port  registered data input (IN) Register
 *  Port  registered data input (IN) Register.
 *
 *  Port  registered data input
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_IN_Struct
 *! \brief  Port  registered data input Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_IN_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Registered data input */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_IN_t;

/*@}*/

/** @defgroup OUT Port data output (OUT) Register
 *  Port data output (OUT) Register.
 *
 *  Port data output
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_OUT_Struct
 *! \brief  Port data output Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_OUT_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Data out */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_OUT_t;

/*@}*/

/** @defgroup SET Port data out set (SET) Register
 *  Port data out set (SET) Register.
 *
 *  Port data out set
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_SET_Struct
 *! \brief  Port data out set Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_SET_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Set the output HIGH for the pin */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_SET_t;

/*@}*/

/** @defgroup CLR Port data out clear (CLR) Register
 *  Port data out clear (CLR) Register.
 *
 *  Port data out clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_CLR_Struct
 *! \brief  Port data out clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_CLR_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Set the output low  for the port pin */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_CLR_t;

/*@}*/

/** @defgroup TGL Port  pin toggle (TGL) Register
 *  Port  pin toggle (TGL) Register.
 *
 *  Port  pin toggle
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_TGL_Struct
 *! \brief  Port  pin toggle Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_TGL_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Toggle the output of the port pin */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_TGL_t;

/*@}*/

/** @defgroup POL Port interrupt polarity (POL) Register
 *  Port interrupt polarity (POL) Register.
 *
 *  Port interrupt polarity
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_POL_Struct
 *! \brief  Port interrupt polarity Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_POL_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Interrupt polarity. */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_POL_t;

/*@}*/

/** @defgroup IENA Port  interrupt A enable (IENA) Register
 *  Port  interrupt A enable (IENA) Register.
 *
 *  Port  interrupt A enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_IENA_Struct
 *! \brief  Port  interrupt A enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_IENA_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Interrupt A enable. */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_IENA_t;

/*@}*/

/** @defgroup IENB Port interrupt B enable (IENB) Register
 *  Port interrupt B enable (IENB) Register.
 *
 *  Port interrupt B enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_IENB_Struct
 *! \brief  Port interrupt B enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_IENB_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Interrupt B enable. */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_IENB_t;

/*@}*/

/** @defgroup INT Port interrupt Status (INT) Register
 *  Port interrupt Status (INT) Register.
 *
 *  Port interrupt Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_INT_Struct
 *! \brief  Port interrupt Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_INT_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Interrupt Status */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_INT_t;

/*@}*/

/** @defgroup DS Port  drive strength select (DS) Register
 *  Port  drive strength select (DS) Register.
 *
 *  Port  drive strength select
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_GPIO_DS_Struct
 *! \brief  Port  drive strength select Register bit field structure
 * ========================================================================== */
typedef struct _ADI_GPIO_DS_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Drive strength select */
    };
    uint16_t VALUE16;
  };
} ADI_GPIO_DS_t;

/*@}*/

/** @defgroup CTL_A Half SPORT 'A' Control Register (CTL_A) Register
 *  Half SPORT 'A' Control Register (CTL_A) Register.
 *
 *  Half SPORT 'A' Control Register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_SPEN
 *! \brief  Serial Port Enable (SPEN) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_DIS = 0,  /**< Disable */
  SPORT_CTL_A_CTL_EN  = 1   /**< Enable  */
} ADI_SPORT_CTL_A_SPEN;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_FSMUXSEL
 *! \brief  Frame Sync Multiplexer Select (FSMUXSEL) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_FS_MUX_DIS = 0,  /**< Disable frame sync multiplexing */
  SPORT_CTL_A_CTL_FS_MUX_EN  = 1   /**< Enable frame sync multiplexing  */
} ADI_SPORT_CTL_A_FSMUXSEL;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_CKMUXSEL
 *! \brief  Clock Multiplexer Select (CKMUXSEL) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_CLK_MUX_DIS = 0,  /**< Disable serial clock multiplexing */
  SPORT_CTL_A_CTL_CLK_MUX_EN  = 1   /**< Enable serial clock multiplexing  */
} ADI_SPORT_CTL_A_CKMUXSEL;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_LSBF
 *! \brief  Least-Significant Bit First (LSBF) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_MSB_FIRST = 0,  /**< MSB first sent/received */
  SPORT_CTL_A_CTL_LSB_FIRST = 1   /**< LSB first sent/received */
} ADI_SPORT_CTL_A_LSBF;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_ICLK
 *! \brief  Internal Clock (ICLK) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_EXTERNAL_CLK = 0,  /**< External clock */
  SPORT_CTL_A_CTL_INTERNAL_CLK = 1   /**< Internal clock */
} ADI_SPORT_CTL_A_ICLK;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_OPMODE
 *! \brief  Operation mode (OPMODE) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_SERIAL        = 0,  /**< DSP standard      */
  SPORT_CTL_A_CTL_TIMER_EN_MODE = 1   /**< Timer_enable mode */
} ADI_SPORT_CTL_A_OPMODE;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_CKRE
 *! \brief  Clock Rising Edge (CKRE) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_CLK_FALL_EDGE = 0,  /**< Clock falling edge */
  SPORT_CTL_A_CTL_CLK_RISE_EDGE = 1   /**< Clock rising edge  */
} ADI_SPORT_CTL_A_CKRE;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_FSR
 *! \brief  Frame Sync Required (FSR) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_FS_NOT_REQ = 0,  /**< No frame sync required */
  SPORT_CTL_A_CTL_FS_REQ     = 1   /**< Frame sync required    */
} ADI_SPORT_CTL_A_FSR;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_IFS
 *! \brief  Internal Frame Sync (IFS) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_EXTERNAL_FS = 0,  /**< External frame sync */
  SPORT_CTL_A_CTL_INTERNAL_FS = 1   /**< Internal frame sync */
} ADI_SPORT_CTL_A_IFS;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_DIFS
 *! \brief  Data-Independent Frame Sync (DIFS) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_DATA_DEP_FS  = 0,  /**< Data-dependent frame sync   */
  SPORT_CTL_A_CTL_DATA_INDP_FS = 1   /**< Data-independent frame sync */
} ADI_SPORT_CTL_A_DIFS;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_LFS
 *! \brief  Active-Low Frame Sync (LFS) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_FS_LO = 0,  /**< Active high frame sync */
  SPORT_CTL_A_CTL_FS_HI = 1   /**< Active low frame sync  */
} ADI_SPORT_CTL_A_LFS;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_LAFS
 *! \brief  Late Frame Sync (LAFS) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_EARLY_FS = 0,  /**< Early frame sync */
  SPORT_CTL_A_CTL_LATE_FS  = 1   /**< Late frame sync  */
} ADI_SPORT_CTL_A_LAFS;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_PACK
 *! \brief  Packing Enable (PACK) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_PACK_DIS   = 0,  /**< Disable               */
  SPORT_CTL_A_CTL_PACK_8BIT  = 1,  /**< 8-bit packing enable  */
  SPORT_CTL_A_CTL_PACK_16BIT = 2   /**< 16-bit packing enable */
} ADI_SPORT_CTL_A_PACK;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_GCLKEN
 *! \brief  Gated Clock Enable (GCLKEN) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_GCLK_DIS = 0,  /**< Disable */
  SPORT_CTL_A_CTL_GCLK_EN  = 1   /**< Enable  */
} ADI_SPORT_CTL_A_GCLKEN;


/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_A_SPTRAN
 *! \brief  Serial Port Transfer Direction (SPTRAN) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_A_CTL_RX = 0,  /**< Receive  */
  SPORT_CTL_A_CTL_TX = 1   /**< Transmit */
} ADI_SPORT_CTL_A_SPTRAN;


/* ==========================================================================
 *! \struct ADI_SPORT_CTL_A_Struct
 *! \brief  Half SPORT 'A' Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_CTL_A_t {
  union {
    struct {
      unsigned int SPEN       :  1;  /**< Serial Port Enable */
      unsigned int FSMUXSEL   :  1;  /**< Frame Sync Multiplexer Select */
      unsigned int CKMUXSEL   :  1;  /**< Clock Multiplexer Select */
      unsigned int LSBF       :  1;  /**< Least-Significant Bit First */
      unsigned int SLEN       :  5;  /**< Serial Word Length */
      unsigned int reserved9  :  1;
      unsigned int ICLK       :  1;  /**< Internal Clock */
      unsigned int OPMODE     :  1;  /**< Operation mode */
      unsigned int CKRE       :  1;  /**< Clock Rising Edge */
      unsigned int FSR        :  1;  /**< Frame Sync Required */
      unsigned int IFS        :  1;  /**< Internal Frame Sync */
      unsigned int DIFS       :  1;  /**< Data-Independent Frame Sync */
      unsigned int LFS        :  1;  /**< Active-Low Frame Sync */
      unsigned int LAFS       :  1;  /**< Late Frame Sync */
      unsigned int PACK       :  2;  /**< Packing Enable */
      unsigned int FSERRMODE  :  1;  /**< Frame Sync Error Operation */
      unsigned int GCLKEN     :  1;  /**< Gated Clock Enable */
      unsigned int reserved22 :  3;
      unsigned int SPTRAN     :  1;  /**< Serial Port Transfer Direction */
      unsigned int DMAEN      :  1;  /**< DMA Enable */
      unsigned int reserved27 :  5;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_CTL_A_t;

/*@}*/

/** @defgroup DIV_A Half SPORT 'A' Divisor Register (DIV_A) Register
 *  Half SPORT 'A' Divisor Register (DIV_A) Register.
 *
 *  Half SPORT 'A' Divisor Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_DIV_A_Struct
 *! \brief  Half SPORT 'A' Divisor Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_DIV_A_t {
  union {
    struct {
      unsigned int CLKDIV     : 16;  /**< Clock Divisor */
      unsigned int FSDIV      :  8;  /**< Frame Sync Divisor */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_DIV_A_t;

/*@}*/

/** @defgroup IEN_A Half SPORT A's Interrupt Enable register (IEN_A) Register
 *  Half SPORT A's Interrupt Enable register (IEN_A) Register.
 *
 *  Half SPORT A's Interrupt Enable register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_SPORT_IEN_A_TF
 *! \brief  Transfer Finish Interrupt Enable (TF) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_IEN_A_CTL_TXFIN_DIS = 0,  /**< Transfer finish Interrupt is disabled */
  SPORT_IEN_A_CTL_TXFIN_EN  = 1   /**< Transfer Finish Interrupt is Enabled  */
} ADI_SPORT_IEN_A_TF;


/* ==========================================================================
 *! \struct ADI_SPORT_IEN_A_Struct
 *! \brief  Half SPORT A's Interrupt Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_IEN_A_t {
  union {
    struct {
      unsigned int TF         :  1;  /**< Transfer Finish Interrupt Enable */
      unsigned int DERRMSK    :  1;  /**< Data Error (Interrupt) Mask */
      unsigned int FSERRMSK   :  1;  /**< Frame Sync Error (Interrupt) Mask */
      unsigned int DATA       :  1;  /**< Data request interrupt to the core */
      unsigned int SYSDATERR  :  1;  /**< Data error for system writes or reads */
      unsigned int reserved5  : 27;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_IEN_A_t;

/*@}*/

/** @defgroup STAT_A Half SPORT A's Status register (STAT_A) Register
 *  Half SPORT A's Status register (STAT_A) Register.
 *
 *  Half SPORT A's Status register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_SPORT_STAT_A_DXS
 *! \brief  Data Transfer Buffer Status (DXS) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_STAT_A_CTL_EMPTY     = 0,  /**< Empty          */
  SPORT_STAT_A_CTL_PART_FULL = 2,  /**< Partially full */
  SPORT_STAT_A_CTL_FULL      = 3   /**< Full           */
} ADI_SPORT_STAT_A_DXS;


/* ==========================================================================
 *! \struct ADI_SPORT_STAT_A_Struct
 *! \brief  Half SPORT A's Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_STAT_A_t {
  union {
    struct {
      unsigned int TFI        :  1;  /**< Transmit Finish Interrupt Status */
      unsigned int DERR       :  1;  /**< Data Error Status */
      unsigned int FSERR      :  1;  /**< Frame Sync Error Status */
      unsigned int DATA       :  1;  /**< Data Buffer status */
      unsigned int SYSDATERR  :  1;  /**< System Data Error Status */
      unsigned int reserved5  :  3;
      unsigned int DXS        :  2;  /**< Data Transfer Buffer Status */
      unsigned int reserved10 : 22;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_STAT_A_t;

/*@}*/

/** @defgroup NUMTRAN_A Half SPORT A Number of transfers register (NUMTRAN_A) Register
 *  Half SPORT A Number of transfers register (NUMTRAN_A) Register.
 *
 *  Half SPORT A Number of transfers register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_NUMTRAN_A_Struct
 *! \brief  Half SPORT A Number of transfers Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_NUMTRAN_A_t {
  union {
    struct {
      unsigned int VALUE      : 12;  /**< Number of transfers (Half SPORT A) */
      unsigned int reserved12 : 20;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_NUMTRAN_A_t;

/*@}*/

/** @defgroup CNVT_A Half SPORT A's register to be programmed with the desired CONVT pulse width (CNVT_A) Register
 *  Half SPORT A's register to be programmed with the desired CONVT pulse width (CNVT_A) Register.
 *
 *  Half SPORT A's register to be programmed with the desired CONVT pulse width
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_CNVT_A_Struct
 *! \brief  Half SPORT A's register to be programmed with the desired CONVT pulse width Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_CNVT_A_t {
  union {
    struct {
      unsigned int WID        :  4;  /**< CONVT signal width: Half SPORT A */
      unsigned int reserved4  :  4;
      unsigned int POL        :  1;  /**< Polarity of the Convt signal */
      unsigned int reserved9  :  7;
      unsigned int CNVT2FS    :  8;  /**< CONVT to FS duration: Half SPORT A */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_CNVT_A_t;

/*@}*/

/** @defgroup TX_A Half SPORT 'A' Tx Buffer Register (TX_A) Register
 *  Half SPORT 'A' Tx Buffer Register (TX_A) Register.
 *
 *  Half SPORT 'A' Tx Buffer Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_TX_A_Struct
 *! \brief  Half SPORT 'A' Tx Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_TX_A_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Transmit Buffer */
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_TX_A_t;

/*@}*/

/** @defgroup RX_A Half SPORT 'A' Rx Buffer Register (RX_A) Register
 *  Half SPORT 'A' Rx Buffer Register (RX_A) Register.
 *
 *  Half SPORT 'A' Rx Buffer Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_RX_A_Struct
 *! \brief  Half SPORT 'A' Rx Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_RX_A_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Receive Buffer */
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_RX_A_t;

/*@}*/

/** @defgroup CTL_B Half SPORT 'B' Control Register (CTL_B) Register
 *  Half SPORT 'B' Control Register (CTL_B) Register.
 *
 *  Half SPORT 'B' Control Register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_SPORT_CTL_B_PACK
 *! \brief  Packing Enable (PACK) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_CTL_B_CTL_PACK_DIS   = 0,  /**< Disable               */
  SPORT_CTL_B_CTL_PACK_8BIT  = 1,  /**< 8-bit packing enable  */
  SPORT_CTL_B_CTL_PACK_16BIT = 2   /**< 16-bit packing enable */
} ADI_SPORT_CTL_B_PACK;


/* ==========================================================================
 *! \struct ADI_SPORT_CTL_B_Struct
 *! \brief  Half SPORT 'B' Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_CTL_B_t {
  union {
    struct {
      unsigned int SPEN       :  1;  /**< Serial Port Enable */
      unsigned int reserved1  :  2;
      unsigned int LSBF       :  1;  /**< Least-Significant Bit First */
      unsigned int SLEN       :  5;  /**< Serial Word Length */
      unsigned int reserved9  :  1;
      unsigned int ICLK       :  1;  /**< Internal Clock */
      unsigned int OPMODE     :  1;  /**< Operation mode */
      unsigned int CKRE       :  1;  /**< Clock Rising Edge */
      unsigned int FSR        :  1;  /**< Frame Sync Required */
      unsigned int IFS        :  1;  /**< Internal Frame Sync */
      unsigned int DIFS       :  1;  /**< Data-Independent Frame Sync */
      unsigned int LFS        :  1;  /**< Active-Low Frame Sync */
      unsigned int LAFS       :  1;  /**< Late Frame Sync */
      unsigned int PACK       :  2;  /**< Packing Enable */
      unsigned int FSERRMODE  :  1;  /**< Frame Sync Error Operation */
      unsigned int GCLKEN     :  1;  /**< Gated Clock Enable */
      unsigned int reserved22 :  3;
      unsigned int SPTRAN     :  1;  /**< Serial Port Transfer Direction */
      unsigned int DMAEN      :  1;  /**< DMA Enable */
      unsigned int reserved27 :  5;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_CTL_B_t;

/*@}*/

/** @defgroup DIV_B Half SPORT 'B' Divisor Register (DIV_B) Register
 *  Half SPORT 'B' Divisor Register (DIV_B) Register.
 *
 *  Half SPORT 'B' Divisor Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_DIV_B_Struct
 *! \brief  Half SPORT 'B' Divisor Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_DIV_B_t {
  union {
    struct {
      unsigned int CLKDIV     : 16;  /**< Clock Divisor */
      unsigned int FSDIV      :  8;  /**< Frame Sync Divisor */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_DIV_B_t;

/*@}*/

/** @defgroup IEN_B Half SPORT B's Interrupt Enable register (IEN_B) Register
 *  Half SPORT B's Interrupt Enable register (IEN_B) Register.
 *
 *  Half SPORT B's Interrupt Enable register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_SPORT_IEN_B_TF
 *! \brief  Transmit Finish Interrupt Enable (TF) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_IEN_B_CTL_TXFIN_DIS = 0,  /**< Transfer Finish Interrupt is disabled */
  SPORT_IEN_B_CTL_TXFIN_EN  = 1   /**< Transfer Finish Interrupt is Enabled  */
} ADI_SPORT_IEN_B_TF;


/* ==========================================================================
 *! \struct ADI_SPORT_IEN_B_Struct
 *! \brief  Half SPORT B's Interrupt Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_IEN_B_t {
  union {
    struct {
      unsigned int TF         :  1;  /**< Transmit Finish Interrupt Enable */
      unsigned int DERRMSK    :  1;  /**< Data Error (Interrupt) Mask */
      unsigned int FSERRMSK   :  1;  /**< Frame Sync Error (Interrupt) Mask */
      unsigned int DATA       :  1;  /**< Data request interrupt to the core */
      unsigned int SYSDATERR  :  1;  /**< Data error for system writes or reads */
      unsigned int reserved5  : 27;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_IEN_B_t;

/*@}*/

/** @defgroup STAT_B Half SPORT B's Status register (STAT_B) Register
 *  Half SPORT B's Status register (STAT_B) Register.
 *
 *  Half SPORT B's Status register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_SPORT_STAT_B_DXS
 *! \brief  Data Transfer Buffer Status (DXS) Enumerations
 *  ========================================================================= */
typedef enum
{
  SPORT_STAT_B_CTL_EMPTY     = 0,  /**< Empty          */
  SPORT_STAT_B_CTL_PART_FULL = 2,  /**< Partially full */
  SPORT_STAT_B_CTL_FULL      = 3   /**< Full           */
} ADI_SPORT_STAT_B_DXS;


/* ==========================================================================
 *! \struct ADI_SPORT_STAT_B_Struct
 *! \brief  Half SPORT B's Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_STAT_B_t {
  union {
    struct {
      unsigned int TFI        :  1;  /**< Transmit Finish Interrupt Status */
      unsigned int DERR       :  1;  /**< Data Error Status */
      unsigned int FSERR      :  1;  /**< Frame Sync Error Status */
      unsigned int DATA       :  1;  /**< Data Buffer status */
      unsigned int SYSDATERR  :  1;  /**< System Data Error Status */
      unsigned int reserved5  :  3;
      unsigned int DXS        :  2;  /**< Data Transfer Buffer Status */
      unsigned int reserved10 : 22;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_STAT_B_t;

/*@}*/

/** @defgroup NUMTRAN_B Half SPORT B Number of transfers register (NUMTRAN_B) Register
 *  Half SPORT B Number of transfers register (NUMTRAN_B) Register.
 *
 *  Half SPORT B Number of transfers register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_NUMTRAN_B_Struct
 *! \brief  Half SPORT B Number of transfers Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_NUMTRAN_B_t {
  union {
    struct {
      unsigned int VALUE      : 12;  /**< Number of transfers (Half SPORT A) */
      unsigned int reserved12 : 20;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_NUMTRAN_B_t;

/*@}*/

/** @defgroup CNVT_B Half SPORT B's register to be programmed with the desired CONVT pulse width (CNVT_B) Register
 *  Half SPORT B's register to be programmed with the desired CONVT pulse width (CNVT_B) Register.
 *
 *  Half SPORT B's register to be programmed with the desired CONVT pulse width
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_CNVT_B_Struct
 *! \brief  Half SPORT B's register to be programmed with the desired CONVT pulse width Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_CNVT_B_t {
  union {
    struct {
      unsigned int WID        :  4;  /**< CONVT signal width: Half SPORT B */
      unsigned int reserved4  :  4;
      unsigned int POL        :  1;  /**< Polarity of the Convt signal */
      unsigned int reserved9  :  7;
      unsigned int CNVT2FS    :  8;  /**< CONVT to FS duration: Half SPORT B */
      unsigned int reserved24 :  8;
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_CNVT_B_t;

/*@}*/

/** @defgroup TX_B Half SPORT 'B' Tx Buffer Register (TX_B) Register
 *  Half SPORT 'B' Tx Buffer Register (TX_B) Register.
 *
 *  Half SPORT 'B' Tx Buffer Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_TX_B_Struct
 *! \brief  Half SPORT 'B' Tx Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_TX_B_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Transmit Buffer */
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_TX_B_t;

/*@}*/

/** @defgroup RX_B Half SPORT 'B' Rx Buffer Register (RX_B) Register
 *  Half SPORT 'B' Rx Buffer Register (RX_B) Register.
 *
 *  Half SPORT 'B' Rx Buffer Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_SPORT_RX_B_Struct
 *! \brief  Half SPORT 'B' Rx Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_SPORT_RX_B_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Receive Buffer */
    };
    uint32_t VALUE32;
  };
} ADI_SPORT_RX_B_t;

/*@}*/

/** @defgroup CTL CRC Control Register (CTL) Register
 *  CRC Control Register (CTL) Register.
 *
 *  CRC Control Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRC_CTL_Struct
 *! \brief  CRC Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRC_CTL_t {
  union {
    struct {
      unsigned int EN         :  1;  /**< CRC peripheral enable */
      unsigned int LSBFIRST   :  1;  /**< LSB First CRC calculation enable */
      unsigned int BITMIRR    :  1;  /**< Bit Mirroring */
      unsigned int BYTMIRR    :  1;  /**< Byte Mirroring */
      unsigned int W16SWP     :  1;  /**< Word16 Swap */
      unsigned int reserved5  : 27;
    };
    uint32_t VALUE32;
  };
} ADI_CRC_CTL_t;

/*@}*/

/** @defgroup IPDATA Input Data Register (IPDATA) Register
 *  Input Data Register (IPDATA) Register.
 *
 *  Input Data Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRC_IPDATA_Struct
 *! \brief  Input Data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRC_IPDATA_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Input Data Register. Holds 32bit data that can be written by Processor or DMA Controller */
    };
    uint32_t VALUE32;
  };
} ADI_CRC_IPDATA_t;

/*@}*/

/** @defgroup RESULT CRC Result Register (RESULT) Register
 *  CRC Result Register (RESULT) Register.
 *
 *  CRC Result Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRC_RESULT_Struct
 *! \brief  CRC Result Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRC_RESULT_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< CRC result. */
    };
    uint32_t VALUE32;
  };
} ADI_CRC_RESULT_t;

/*@}*/

/** @defgroup POLY Programmable CRC Polynomial (POLY) Register
 *  Programmable CRC Polynomial (POLY) Register.
 *
 *  Programmable CRC Polynomial
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRC_POLY_Struct
 *! \brief  Programmable CRC Polynomial Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRC_POLY_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< 1-32 bit programmable CRC polynomial */
    };
    uint32_t VALUE32;
  };
} ADI_CRC_POLY_t;

/*@}*/

/** @defgroup CTL RNG Control Register (CTL) Register
 *  RNG Control Register (CTL) Register.
 *
 *  RNG Control Register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_RNG_CTL_EN
 *! \brief  RNG Enable (EN) Enumerations
 *  ========================================================================= */
typedef enum
{
  RNG_CTL_DISABLE = 0,  /**< Disable the RNG */
  RNG_CTL_ENABLE  = 1   /**< Enable the RNG  */
} ADI_RNG_CTL_EN;


/*  =========================================================================
 *! \enum   ADI_RNG_CTL_SINGLE
 *! \brief  Generate a single number (SINGLE) Enumerations
 *  ========================================================================= */
typedef enum
{
  RNG_CTL_WORD   = 0,  /**< Buffer Word */
  RNG_CTL_SINGLE = 1   /**< Single Byte */
} ADI_RNG_CTL_SINGLE;


/* ==========================================================================
 *! \struct ADI_RNG_CTL_Struct
 *! \brief  RNG Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RNG_CTL_t {
  union {
    struct {
      unsigned int EN         :  1;  /**< RNG Enable */
      unsigned int reserved1  :  2;
      unsigned int SINGLE     :  1;  /**< Generate a single number */
      unsigned int reserved4  : 12;
    };
    uint16_t VALUE16;
  };
} ADI_RNG_CTL_t;

/*@}*/

/** @defgroup LEN RNG Sample Length Register (LEN) Register
 *  RNG Sample Length Register (LEN) Register.
 *
 *  RNG Sample Length Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RNG_LEN_Struct
 *! \brief  RNG Sample Length Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RNG_LEN_t {
  union {
    struct {
      unsigned int RELOAD     : 12;  /**< Reload value for the sample counter */
      unsigned int PRESCALE   :  4;  /**< Prescaler for the sample counter */
    };
    uint16_t VALUE16;
  };
} ADI_RNG_LEN_t;

/*@}*/

/** @defgroup STAT RNG Status Register (STAT) Register
 *  RNG Status Register (STAT) Register.
 *
 *  RNG Status Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RNG_STAT_Struct
 *! \brief  RNG Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RNG_STAT_t {
  union {
    struct {
      unsigned int RNRDY      :  1;  /**< Random number ready */
      unsigned int STUCK      :  1;  /**< Sampled data stuck high or low */
      unsigned int reserved2  : 14;
    };
    uint16_t VALUE16;
  };
} ADI_RNG_STAT_t;

/*@}*/

/** @defgroup DATA RNG Data Register (DATA) Register
 *  RNG Data Register (DATA) Register.
 *
 *  RNG Data Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RNG_DATA_Struct
 *! \brief  RNG Data Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RNG_DATA_t {
  union {
    struct {
      unsigned int VALUE      :  8;  /**< Value of the CRC accumulator */
      unsigned int BUFF       : 24;  /**< Buffer for RNG data */
    };
    uint32_t VALUE32;
  };
} ADI_RNG_DATA_t;

/*@}*/

/** @defgroup OSCCNT Oscillator Count (OSCCNT) Register
 *  Oscillator Count (OSCCNT) Register.
 *
 *  Oscillator Count
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RNG_OSCCNT_Struct
 *! \brief  Oscillator Count Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RNG_OSCCNT_t {
  union {
    struct {
      unsigned int VALUE      : 28;  /**< Oscillator count */
      unsigned int reserved28 :  4;
    };
    uint32_t VALUE32;
  };
} ADI_RNG_OSCCNT_t;

/*@}*/

/** @defgroup OSCDIFF Oscillator Difference (OSCDIFF) Register
 *  Oscillator Difference (OSCDIFF) Register.
 *
 *  Oscillator Difference
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_RNG_OSCDIFF_Struct
 *! \brief  Oscillator Difference Register bit field structure
 * ========================================================================== */
typedef struct _ADI_RNG_OSCDIFF_t {
  union {
    struct {
      unsigned int DELTA      :  8;  /**< Oscillator Count difference */
    };
    uint8_t VALUE8;
  };
} ADI_RNG_OSCDIFF_t;

/*@}*/

/** @defgroup CFG Configuration Register (CFG) Register
 *  Configuration Register (CFG) Register.
 *
 *  Configuration Register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_CRYPT_CFG_BLKEN
 *! \brief  Enable bit for the Crypto Block (BLKEN) Enumerations
 *  ========================================================================= */
typedef enum
{
  CRYPT_CFG_ENABLE  = 0,  /**< Enable Crypto Block  */
  CRYPT_CFG_DISABLE = 1   /**< Disable Crypto Block */
} ADI_CRYPT_CFG_BLKEN;


/*  =========================================================================
 *! \enum   ADI_CRYPT_CFG_INDMAEN
 *! \brief  Enable DMA for Input Buffer (INDMAEN) Enumerations
 *  ========================================================================= */
typedef enum
{
  CRYPT_CFG_DMA_DISABLE_INBUF = 0,  /**< Disable DMA Requesting for Input Buffer */
  CRYPT_CFG_DMA_ENABLE_INBUF  = 1   /**< Enable DMA Requesting for Input Buffer  */
} ADI_CRYPT_CFG_INDMAEN;


/*  =========================================================================
 *! \enum   ADI_CRYPT_CFG_OUTDMAEN
 *! \brief  Enable DMA for Output Buffer (OUTDMAEN) Enumerations
 *  ========================================================================= */
typedef enum
{
  CRYPT_CFG_DMA_DISABLE_OUTBUF = 0,  /**< Disable DMA Requesting for Output Buffer */
  CRYPT_CFG_DMA_ENABLE_OUTBUF  = 1   /**< Enable DMA Requesting for Output Buffer  */
} ADI_CRYPT_CFG_OUTDMAEN;


/*  =========================================================================
 *! \enum   ADI_CRYPT_CFG_ENDIAN
 *! \brief  Endianness (ENDIAN) Enumerations
 *  ========================================================================= */
typedef enum
{
  CRYPT_CFG_LITTLE_ENDIAN = 0,  /**< Little Endian Format */
  CRYPT_CFG_BIG_ENDIAN    = 1   /**< Big Endian Format    */
} ADI_CRYPT_CFG_ENDIAN;


/*  =========================================================================
 *! \enum   ADI_CRYPT_CFG_SHADATSRC
 *! \brief  Select data input to SHA block (SHADATSRC) Enumerations
 *  ========================================================================= */
typedef enum
{
  CRYPT_CFG_INBUF = 0,  /**< SHA takes input from input buffer  */
  CRYPT_CFG_OPBUF = 1   /**< SHA takes input from output buffer */
} ADI_CRYPT_CFG_SHADATSRC;


/* ==========================================================================
 *! \struct ADI_CRYPT_CFG_Struct
 *! \brief  Configuration Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_CFG_t {
  union {
    struct {
      unsigned int BLKEN      :  1;  /**< Enable bit for the Crypto Block */
      unsigned int ENCR       :  1;  /**< Encrypt or Decrypt */
      unsigned int INDMAEN    :  1;  /**< Enable DMA for Input Buffer */
      unsigned int OUTDMAEN   :  1;  /**< Enable DMA for Output Buffer */
      unsigned int INFLUSH    :  1;  /**< Input Buffer Flush */
      unsigned int OUTFLUSH   :  1;  /**< Output Buffer Flush */
      unsigned int ENDIAN     :  1;  /**< Endianness */
      unsigned int KEYSEL     :  1;  /**< Select Key Source */
      unsigned int KEYLEN     :  2;  /**< Select Key Length for AES Cipher */
      unsigned int reserved10 :  6;
      unsigned int ECBEN      :  1;  /**< Enable ECB Mode Operation */
      unsigned int CTREN      :  1;  /**< Enable CTR Mode Operation */
      unsigned int CBCEN      :  1;  /**< Enable CBC Mode Operation */
      unsigned int CCMEN      :  1;  /**< Enable CCM/CCM* Mode Operation */
      unsigned int CMACEN     :  1;  /**< Enable CMAC Mode Operation */
      unsigned int reserved21 :  3;
      unsigned int SHA224EN   :  1;  /**< Enable SHA-224 Operation */
      unsigned int SHA256EN   :  1;  /**< Enable SHA-256 Operation */
      unsigned int SHAINIT    :  1;  /**< Restarts SHA Computation */
      unsigned int SHADATSRC  :  1;  /**< Select data input to SHA block */
      unsigned int reserved28 :  4;
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_CFG_t;

/*@}*/

/** @defgroup DATALEN Payload Data Length (DATALEN) Register
 *  Payload Data Length (DATALEN) Register.
 *
 *  Payload Data Length
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_DATALEN_Struct
 *! \brief  Payload Data Length Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_DATALEN_t {
  union {
    struct {
      unsigned int VALUE      : 20;  /**< Length of Payload Data */
      unsigned int reserved20 : 12;
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_DATALEN_t;

/*@}*/

/** @defgroup PREFIXLEN Authentication Data Length (PREFIXLEN) Register
 *  Authentication Data Length (PREFIXLEN) Register.
 *
 *  Authentication Data Length
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_PREFIXLEN_Struct
 *! \brief  Authentication Data Length Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_PREFIXLEN_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Length of Associated Data */
      unsigned int reserved16 : 16;
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_PREFIXLEN_t;

/*@}*/

/** @defgroup INTEN Interrupt Enable Register (INTEN) Register
 *  Interrupt Enable Register (INTEN) Register.
 *
 *  Interrupt Enable Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_INTEN_Struct
 *! \brief  Interrupt Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_INTEN_t {
  union {
    struct {
      unsigned int INRDYEN    :  1;  /**< Enable Input Ready Interrupt */
      unsigned int OUTRDYEN   :  1;  /**< Enables the Output Ready Interrupt. */
      unsigned int INOVREN    :  1;  /**< Enable Input Overflow Interrupt. */
      unsigned int OUTUNDREN  :  1;  /**< Enable the Output Underflow Interrupt */
      unsigned int reserved4  :  1;
      unsigned int SHADONEN   :  1;  /**< Enable SHA_Done Interrupt */
      unsigned int reserved6  : 26;
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_INTEN_t;

/*@}*/

/** @defgroup STAT Status Register (STAT) Register
 *  Status Register (STAT) Register.
 *
 *  Status Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_STAT_Struct
 *! \brief  Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_STAT_t {
  union {
    struct {
      unsigned int INRDY      :  1;  /**< Input buffer requires more data before computation can begin. Remains set till the Buffer is filled. */
      unsigned int OUTRDY     :  1;  /**< Output Data ready to be read */
      unsigned int INOVR      :  1;  /**< Overflow in the input buffer. */
      unsigned int OUTUNDR    :  1;  /**< Underflow Interrupt in the Output */
      unsigned int reserved4  :  1;
      unsigned int SHADONE    :  1;  /**< SHA Computation complete */
      unsigned int SHABUSY    :  1;  /**< SHA Busy. Computations are ongoing */
      unsigned int INWORDS    :  3;  /**< Number of words in the Input Buffer */
      unsigned int OUTWORDS   :  3;  /**< Number of words in the Output Buffer */
      unsigned int reserved13 : 19;
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_STAT_t;

/*@}*/

/** @defgroup INBUF Input Buffer (INBUF) Register
 *  Input Buffer (INBUF) Register.
 *
 *  Input Buffer
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_INBUF_Struct
 *! \brief  Input Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_INBUF_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Input Buffer */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_INBUF_t;

/*@}*/

/** @defgroup OUTBUF Output Buffer (OUTBUF) Register
 *  Output Buffer (OUTBUF) Register.
 *
 *  Output Buffer
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_OUTBUF_Struct
 *! \brief  Output Buffer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_OUTBUF_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Output Buffer */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_OUTBUF_t;

/*@}*/

/** @defgroup NONCE0 Nonce Bits [31:0] (NONCE0) Register
 *  Nonce Bits [31:0] (NONCE0) Register.
 *
 *  Nonce Bits [31:0]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_NONCE0_Struct
 *! \brief  Nonce Bits [31:0] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_NONCE0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Word 0: Nonce : Bits [31:0] */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_NONCE0_t;

/*@}*/

/** @defgroup NONCE1 Nonce Bits [63:32] (NONCE1) Register
 *  Nonce Bits [63:32] (NONCE1) Register.
 *
 *  Nonce Bits [63:32]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_NONCE1_Struct
 *! \brief  Nonce Bits [63:32] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_NONCE1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Word 1: Nonce : Bits [63:32] */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_NONCE1_t;

/*@}*/

/** @defgroup NONCE2 Nonce Bits [95:64] (NONCE2) Register
 *  Nonce Bits [95:64] (NONCE2) Register.
 *
 *  Nonce Bits [95:64]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_NONCE2_Struct
 *! \brief  Nonce Bits [95:64] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_NONCE2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Word 2: Nonce : Bits [95:64] */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_NONCE2_t;

/*@}*/

/** @defgroup NONCE3 Nonce Bits [127:96] (NONCE3) Register
 *  Nonce Bits [127:96] (NONCE3) Register.
 *
 *  Nonce Bits [127:96]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_NONCE3_Struct
 *! \brief  Nonce Bits [127:96] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_NONCE3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Word 3: Nonce : Bits [127:96] */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_NONCE3_t;

/*@}*/

/** @defgroup AESKEY0 Key Bits[ 31:0 ] (AESKEY0) Register
 *  Key Bits[ 31:0 ] (AESKEY0) Register.
 *
 *  Key Bits[ 31:0 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY0_Struct
 *! \brief  Key Bits[ 31:0 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [3:0]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY0_t;

/*@}*/

/** @defgroup AESKEY1 Key Bits [ 63:32 ] (AESKEY1) Register
 *  Key Bits [ 63:32 ] (AESKEY1) Register.
 *
 *  Key Bits [ 63:32 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY1_Struct
 *! \brief  Key Bits [ 63:32 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [7:4]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY1_t;

/*@}*/

/** @defgroup AESKEY2 Key Bits [ 95:64 ] (AESKEY2) Register
 *  Key Bits [ 95:64 ] (AESKEY2) Register.
 *
 *  Key Bits [ 95:64 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY2_Struct
 *! \brief  Key Bits [ 95:64 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [11:8]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY2_t;

/*@}*/

/** @defgroup AESKEY3 Key Bits [ 127:96 ] (AESKEY3) Register
 *  Key Bits [ 127:96 ] (AESKEY3) Register.
 *
 *  Key Bits [ 127:96 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY3_Struct
 *! \brief  Key Bits [ 127:96 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [15:12]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY3_t;

/*@}*/

/** @defgroup AESKEY4 Key Bits [ 159:128 ] (AESKEY4) Register
 *  Key Bits [ 159:128 ] (AESKEY4) Register.
 *
 *  Key Bits [ 159:128 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY4_Struct
 *! \brief  Key Bits [ 159:128 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY4_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [3:0]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY4_t;

/*@}*/

/** @defgroup AESKEY5 Key Bits [ 191:160 ] (AESKEY5) Register
 *  Key Bits [ 191:160 ] (AESKEY5) Register.
 *
 *  Key Bits [ 191:160 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY5_Struct
 *! \brief  Key Bits [ 191:160 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY5_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [7:4]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY5_t;

/*@}*/

/** @defgroup AESKEY6 Key Bits [ 223:192 ] (AESKEY6) Register
 *  Key Bits [ 223:192 ] (AESKEY6) Register.
 *
 *  Key Bits [ 223:192 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY6_Struct
 *! \brief  Key Bits [ 223:192 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY6_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [11:8]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY6_t;

/*@}*/

/** @defgroup AESKEY7 Key Bits [ 255:224 ] (AESKEY7) Register
 *  Key Bits [ 255:224 ] (AESKEY7) Register.
 *
 *  Key Bits [ 255:224 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_AESKEY7_Struct
 *! \brief  Key Bits [ 255:224 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_AESKEY7_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Key: Bytes [15:12]; */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_AESKEY7_t;

/*@}*/

/** @defgroup CNTRINIT Counter Initialization Vector (CNTRINIT) Register
 *  Counter Initialization Vector (CNTRINIT) Register.
 *
 *  Counter Initialization Vector
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_CNTRINIT_Struct
 *! \brief  Counter Initialization Vector Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_CNTRINIT_t {
  union {
    struct {
      unsigned int VALUE      : 20;  /**< Counter Initialization Value */
      unsigned int reserved20 : 12;
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_CNTRINIT_t;

/*@}*/

/** @defgroup SHAH0 SHA Bits [ 31:0 ] (SHAH0) Register
 *  SHA Bits [ 31:0 ] (SHAH0) Register.
 *
 *  SHA Bits [ 31:0 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH0_Struct
 *! \brief  SHA Bits [ 31:0 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH0_t {
  union {
    struct {
      unsigned int SHAHASH0   : 32;  /**< Word 0: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH0_t;

/*@}*/

/** @defgroup SHAH1 SHA Bits [ 63:32 ] (SHAH1) Register
 *  SHA Bits [ 63:32 ] (SHAH1) Register.
 *
 *  SHA Bits [ 63:32 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH1_Struct
 *! \brief  SHA Bits [ 63:32 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH1_t {
  union {
    struct {
      unsigned int SHAHASH1   : 32;  /**< Word 1: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH1_t;

/*@}*/

/** @defgroup SHAH2 SHA Bits [ 95:64 ] (SHAH2) Register
 *  SHA Bits [ 95:64 ] (SHAH2) Register.
 *
 *  SHA Bits [ 95:64 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH2_Struct
 *! \brief  SHA Bits [ 95:64 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH2_t {
  union {
    struct {
      unsigned int SHAHASH2   : 32;  /**< Word 2: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH2_t;

/*@}*/

/** @defgroup SHAH3 SHA Bits [ 127:96 ] (SHAH3) Register
 *  SHA Bits [ 127:96 ] (SHAH3) Register.
 *
 *  SHA Bits [ 127:96 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH3_Struct
 *! \brief  SHA Bits [ 127:96 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH3_t {
  union {
    struct {
      unsigned int SHAHASH3   : 32;  /**< Word 3: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH3_t;

/*@}*/

/** @defgroup SHAH4 SHA Bits [ 159:128 ] (SHAH4) Register
 *  SHA Bits [ 159:128 ] (SHAH4) Register.
 *
 *  SHA Bits [ 159:128 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH4_Struct
 *! \brief  SHA Bits [ 159:128 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH4_t {
  union {
    struct {
      unsigned int SHAHASH4   : 32;  /**< Word 4: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH4_t;

/*@}*/

/** @defgroup SHAH5 SHA Bits [ 191:160 ] (SHAH5) Register
 *  SHA Bits [ 191:160 ] (SHAH5) Register.
 *
 *  SHA Bits [ 191:160 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH5_Struct
 *! \brief  SHA Bits [ 191:160 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH5_t {
  union {
    struct {
      unsigned int SHAHASH5   : 32;  /**< Word 5: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH5_t;

/*@}*/

/** @defgroup SHAH6 SHA Bits [ 223:192] (SHAH6) Register
 *  SHA Bits [ 223:192] (SHAH6) Register.
 *
 *  SHA Bits [ 223:192]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH6_Struct
 *! \brief  SHA Bits [ 223:192] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH6_t {
  union {
    struct {
      unsigned int SHAHASH6   : 32;  /**< Word 6: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH6_t;

/*@}*/

/** @defgroup SHAH7 SHA Bits [ 255:224 ] (SHAH7) Register
 *  SHA Bits [ 255:224 ] (SHAH7) Register.
 *
 *  SHA Bits [ 255:224 ]
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CRYPT_SHAH7_Struct
 *! \brief  SHA Bits [ 255:224 ] Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CRYPT_SHAH7_t {
  union {
    struct {
      unsigned int SHAHASH7   : 32;  /**< Word 7: SHA Hash */
    };
    uint32_t VALUE32;
  };
} ADI_CRYPT_SHAH7_t;

/*@}*/

/** @defgroup IEN Power Supply Monitor Interrupt Enable (IEN) Register
 *  Power Supply Monitor Interrupt Enable (IEN) Register.
 *
 *  Power Supply Monitor Interrupt Enable
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_PMG_IEN_RANGEBAT
 *! \brief  Battery Monitor Range (RANGEBAT) Enumerations
 *  ========================================================================= */
typedef enum
{
  PMG_IEN_REGION1 = 0,  /**< Configure to generate interrupt if VBAT > 2.75V              */
  PMG_IEN_REGION2 = 1,  /**< Configure to generate interrupt if VBAT between 2.75V - 1.6V */
  PMG_IEN_REGION3 = 2,  /**< Configure to generate interrupt if VBAT between 2.3V - 1.6V  */
  PMG_IEN_NA      = 3   /**< N/A                                                          */
} ADI_PMG_IEN_RANGEBAT;


/* ==========================================================================
 *! \struct ADI_PMG_IEN_Struct
 *! \brief  Power Supply Monitor Interrupt Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_IEN_t {
  union {
    struct {
      unsigned int VBAT       :  1;  /**< Enable Interrupt for VBAT */
      unsigned int VREGUNDR   :  1;  /**< Enable Interrupt when VREG under-voltage: below 1V */
      unsigned int VREGOVR    :  1;  /**< Enable Interrupt when VREG over-voltage: over- 1.32V */
      unsigned int reserved3  :  5;
      unsigned int RANGEBAT   :  2;  /**< Battery Monitor Range */
      unsigned int IENBAT     :  1;  /**< Interrupt enable for VBAT range */
      unsigned int reserved11 : 21;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_IEN_t;

/*@}*/

/** @defgroup PSM_STAT Power supply monitor status (PSM_STAT) Register
 *  Power supply monitor status (PSM_STAT) Register.
 *
 *  Power supply monitor status
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_PMG_PSM_STAT_RORANGE1
 *! \brief  VBAT range1 (> 2.75v) (RORANGE1) Enumerations
 *  ========================================================================= */
typedef enum
{
  PMG_PSM_STAT_BATSTAT1 = 0,  /**< VBAT NOT in the range specified */
  PMG_PSM_STAT_BATSTAT0 = 1   /**< VBAT in the range specified     */
} ADI_PMG_PSM_STAT_RORANGE1;


/* ==========================================================================
 *! \struct ADI_PMG_PSM_STAT_Struct
 *! \brief  Power supply monitor status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_PSM_STAT_t {
  union {
    struct {
      unsigned int VBATUNDR   :  1;  /**< Status bit indicating an Alarm that battery is below 1.8V. */
      unsigned int VREGUNDR   :  1;  /**< Status bit for Alarm indicating VREG is below 1V. */
      unsigned int VREGOVR    :  1;  /**< Status bit for alarm indicating Overvoltage for VREG */
      unsigned int reserved3  :  4;
      unsigned int WICENACK   :  1;  /**< WIC Enable Acknowledge from Cortex */
      unsigned int RANGE1     :  1;  /**< VBAT range1 (> 2.75v) */
      unsigned int RANGE2     :  1;  /**< VBAT range2 (2.75v - 2.3v) */
      unsigned int RANGE3     :  1;  /**< VBAT range3 (2.3v - 1.6v) */
      unsigned int reserved11 :  2;
      unsigned int RORANGE1   :  1;  /**< VBAT range1 (> 2.75v) */
      unsigned int RORANGE2   :  1;  /**< VBAT range2 (2.75v - 2.3v) */
      unsigned int RORANGE3   :  1;  /**< VBAT range3 (2.3v - 1.6v) */
      unsigned int reserved16 : 16;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_PSM_STAT_t;

/*@}*/

/** @defgroup PWRMOD Power Mode Register (PWRMOD) Register
 *  Power Mode Register (PWRMOD) Register.
 *
 *  Power Mode Register
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_PMG_PWRMOD_MODE
 *! \brief  Power Mode Bits (MODE) Enumerations
 *  ========================================================================= */
typedef enum
{
  PMG_PWRMOD_FLEXI     = 0,  /**< FLEXI Mode     */
  PMG_PWRMOD_HIBERNATE = 2,  /**< HIBERNATE Mode */
  PMG_PWRMOD_SHUTDOWN  = 3   /**< SHUTDOWN Mode  */
} ADI_PMG_PWRMOD_MODE;


/* ==========================================================================
 *! \struct ADI_PMG_PWRMOD_Struct
 *! \brief  Power Mode Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_PWRMOD_t {
  union {
    struct {
      unsigned int MODE       :  2;  /**< Power Mode Bits */
      unsigned int reserved2  :  1;
      unsigned int MONVBATN   :  1;  /**< Monitor VBAT during HIBERNATE Mode. Monitors VBAT by default */
      unsigned int reserved4  : 28;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_PWRMOD_t;

/*@}*/

/** @defgroup PWRKEY Key protection for PWRMOD (PWRKEY) Register
 *  Key protection for PWRMOD (PWRKEY) Register.
 *
 *  Key protection for PWRMOD
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_PWRKEY_Struct
 *! \brief  Key protection for PWRMOD Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_PWRKEY_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Power control key register */
      unsigned int reserved16 : 16;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_PWRKEY_t;

/*@}*/

/** @defgroup SHDN_STAT SHUTDOWN Status Register (SHDN_STAT) Register
 *  SHUTDOWN Status Register (SHDN_STAT) Register.
 *
 *  SHUTDOWN Status Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_SHDN_STAT_Struct
 *! \brief  SHUTDOWN Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_SHDN_STAT_t {
  union {
    struct {
      unsigned int EXTINT0    :  1;  /**< Interrupt from External Interrupt 0 */
      unsigned int EXTINT1    :  1;  /**< Interrupt from External Interrupt 1 */
      unsigned int EXTINT2    :  1;  /**< Interrupt from External Interrupt 2 */
      unsigned int RTC        :  1;  /**< Interrupt from RTC */
      unsigned int reserved4  : 28;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_SHDN_STAT_t;

/*@}*/

/** @defgroup SRAMRET Control for Retention SRAM during HIBERNATE Mode (SRAMRET) Register
 *  Control for Retention SRAM during HIBERNATE Mode (SRAMRET) Register.
 *
 *  Control for Retention SRAM during HIBERNATE Mode
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_SRAMRET_Struct
 *! \brief  Control for Retention SRAM during HIBERNATE Mode Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_SRAMRET_t {
  union {
    struct {
      unsigned int BNK1EN     :  1;  /**< Enable retention bank 1 (8kB) */
      unsigned int BNK2EN     :  1;  /**< Enable retention bank 2 (16kB) */
      unsigned int reserved2  : 30;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_SRAMRET_t;

/*@}*/

/** @defgroup RST_STAT Reset status (RST_STAT) Register
 *  Reset status (RST_STAT) Register.
 *
 *  Reset status
 *  @{
 */

/*  =========================================================================
 *! \enum   ADI_PMG_RST_STAT_PORSRC
 *! \brief  Power on reset Source (pmg_rst_src) (PORSRC) Enumerations
 *  ========================================================================= */
typedef enum
{
  PMG_RST_STAT_FAILSAFE_HV = 0,  /**< POR triggered because VBAT drops below Fail Safe */
  PMG_RST_STAT_RST_VBAT    = 1,  /**< POR trigger because VBAT supply (VBAT<1.7v)      */
  PMG_RST_STAT_RST_VREG    = 2,  /**< POR triggered because VDD supply (VDD < 1.08v)   */
  PMG_RST_STAT_FAILSAFE_LV = 3   /**< POR triggered because VREG drops below Fail Safe */
} ADI_PMG_RST_STAT_PORSRC;


/* ==========================================================================
 *! \struct ADI_PMG_RST_STAT_Struct
 *! \brief  Reset status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_RST_STAT_t {
  union {
    struct {
      unsigned int POR        :  1;  /**< Power-on reset */
      unsigned int EXTRST     :  1;  /**< External reset */
      unsigned int WDRST      :  1;  /**< Watchdog timeout */
      unsigned int SWRST      :  1;  /**< Software reset */
      unsigned int PORSRC     :  2;  /**< Power on reset Source (pmg_rst_src) */
      unsigned int reserved6  : 26;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_RST_STAT_t;

/*@}*/

/** @defgroup CTL1 HPBUCK control (CTL1) Register
 *  HPBUCK control (CTL1) Register.
 *
 *  HPBUCK control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_CTL1_Struct
 *! \brief  HPBUCK control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_CTL1_t {
  union {
    struct {
      unsigned int HPBUCKEN   :  1;  /**< Enable HP Buck */
      unsigned int reserved1  : 31;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_CTL1_t;

/*@}*/

/** @defgroup CFG0 External Interrupt configuration 0 (CFG0) Register
 *  External Interrupt configuration 0 (CFG0) Register.
 *
 *  External Interrupt configuration 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_XINT_CFG0_Struct
 *! \brief  External Interrupt configuration 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_XINT_CFG0_t {
  union {
    struct {
      unsigned int IRQ0MDE    :  3;  /**< External Interrupt 0 Mode registers */
      unsigned int IRQ0EN     :  1;  /**< External Interrupt 0 Enable bit */
      unsigned int IRQ1MDE    :  3;  /**< External Interrupt 1 Mode registers */
      unsigned int IRQ1EN     :  1;  /**< External Interrupt 1 Enable bit */
      unsigned int IRQ2MDE    :  3;  /**< External Interrupt 2 Mode registers */
      unsigned int IRQ2EN     :  1;  /**< External Interrupt 2 Enable bit */
      unsigned int IRQ3MDE    :  3;  /**< External Interrupt 3 Mode registers */
      unsigned int IRQ3EN     :  1;  /**< External Interrupt 3 enable bit */
      unsigned int SIPEN      :  1;  /**< External Interrupt using SIP_UPDATE enable bit */
      unsigned int SIPMDE     :  3;  /**< External Interrupt using SIP_UPDATE Mode registers */
      unsigned int reserved20 : 12;
    };
    uint32_t VALUE32;
  };
} ADI_XINT_CFG0_t;

/*@}*/

/** @defgroup CLR External Interrupt clear (CLR) Register
 *  External Interrupt clear (CLR) Register.
 *
 *  External Interrupt clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_XINT_CLR_Struct
 *! \brief  External Interrupt clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_XINT_CLR_t {
  union {
    struct {
      unsigned int IRQ0       :  1;  /**< External interrupt 0 */
      unsigned int IRQ1       :  1;  /**< External interrupt 1 */
      unsigned int IRQ2       :  1;  /**< External interrupt 2 */
      unsigned int IRQ3       :  1;  /**< External interrupt 3 */
      unsigned int SIPCLR     :  1;  /**< External interrupt Clear for SIP_UPDATE */
      unsigned int reserved5  : 27;
    };
    uint32_t VALUE32;
  };
} ADI_XINT_CLR_t;

/*@}*/

/** @defgroup NMICLR Non-maskable interrupt clear (NMICLR) Register
 *  Non-maskable interrupt clear (NMICLR) Register.
 *
 *  Non-maskable interrupt clear
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_XINT_NMICLR_Struct
 *! \brief  Non-maskable interrupt clear Register bit field structure
 * ========================================================================== */
typedef struct _ADI_XINT_NMICLR_t {
  union {
    struct {
      unsigned int CLR        :  1;  /**< NMI clear */
      unsigned int reserved1  : 31;
    };
    uint32_t VALUE32;
  };
} ADI_XINT_NMICLR_t;

/*@}*/

/** @defgroup KEY Key Protection for OSCCTRL (KEY) Register
 *  Key Protection for OSCCTRL (KEY) Register.
 *
 *  Key Protection for OSCCTRL
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_OSC_KEY_Struct
 *! \brief  Key Protection for OSCCTRL Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_OSC_KEY_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Oscillator key */
      unsigned int reserved16 : 16;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_OSC_KEY_t;

/*@}*/

/** @defgroup CTL Oscillator Control (CTL) Register
 *  Oscillator Control (CTL) Register.
 *
 *  Oscillator Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_OSC_CTL_Struct
 *! \brief  Oscillator Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_OSC_CTL_t {
  union {
    struct {
      unsigned int LFCLKMUX   :  1;  /**< 32 KHz clock select mux */
      unsigned int HFOSCEN    :  1;  /**< High frequency internal oscillator enable */
      unsigned int LFXTALEN   :  1;  /**< Low frequency crystal oscillator enable */
      unsigned int HFXTALEN   :  1;  /**< High frequency crystal oscillator enable */
      unsigned int reserved4  :  4;
      unsigned int LFOSCOK    :  1;  /**< Status of LFOSC oscillator */
      unsigned int HFOSCOK    :  1;  /**< Status of HFOSC oscillator */
      unsigned int LFXTALOK   :  1;  /**< Status of LFXTAL oscillator */
      unsigned int HFXTALOK   :  1;  /**< Status of HFXTAL oscillator */
      unsigned int reserved12 : 20;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_OSC_CTL_t;

/*@}*/

/** @defgroup SRAM_CTL Control for SRAM parity and instruction SRAM (SRAM_CTL) Register
 *  Control for SRAM parity and instruction SRAM (SRAM_CTL) Register.
 *
 *  Control for SRAM parity and instruction SRAM
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_TST_SRAM_CTL_Struct
 *! \brief  Control for SRAM parity and instruction SRAM Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_TST_SRAM_CTL_t {
  union {
    struct {
      unsigned int BNK0EN     :  1;  /**< Enable initialization */
      unsigned int BNK1EN     :  1;  /**< Enable initialization */
      unsigned int BNK2EN     :  1;  /**< Enable initialization */
      unsigned int BNK3EN     :  1;  /**< Enable initialization */
      unsigned int BNK4EN     :  1;  /**< Enable initialization */
      unsigned int BNK5EN     :  1;  /**< Enable initialization */
      unsigned int reserved6  :  7;
      unsigned int STARTINIT  :  1;  /**< Write one to trigger initialization. Self-cleared */
      unsigned int AUTOINIT   :  1;  /**< Automatic initialization on wake up from Hibernate mode */
      unsigned int ABTINIT    :  1;  /**< Abort current initialization. Self-cleared */
      unsigned int PENBNK0    :  1;  /**< Enable parity check */
      unsigned int PENBNK1    :  1;  /**< Enable parity check */
      unsigned int PENBNK2    :  1;  /**< Enable parity check */
      unsigned int PENBNK3    :  1;  /**< Enable parity check */
      unsigned int PENBNK4    :  1;  /**< Enable parity check */
      unsigned int PENBNK5    :  1;  /**< Enable parity check */
      unsigned int reserved22 :  9;
      unsigned int INSTREN    :  1;  /**< Enables instruction SRAM */
    };
    uint32_t VALUE32;
  };
} ADI_PMG_TST_SRAM_CTL_t;

/*@}*/

/** @defgroup SRAM_INITSTAT Initialization Status Register (SRAM_INITSTAT) Register
 *  Initialization Status Register (SRAM_INITSTAT) Register.
 *
 *  Initialization Status Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_TST_SRAM_INITSTAT_Struct
 *! \brief  Initialization Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_TST_SRAM_INITSTAT_t {
  union {
    struct {
      unsigned int BNK0       :  1;  /**< 0:Not initialized; 1:Initialization completed */
      unsigned int BNK1       :  1;  /**< 0:Not initialized; 1:Initialization completed */
      unsigned int BNK2       :  1;  /**< 0:Not initialized; 1:Initialization completed */
      unsigned int BNK3       :  1;  /**< 0:Not initialized; 1:Initialization completed */
      unsigned int BNK4       :  1;  /**< 0:Not initialized; 1:Initialization completed */
      unsigned int BNK5       :  1;  /**< 0:Not initialized; 1:Initialization completed */
      unsigned int reserved6  : 26;
    };
    uint32_t VALUE32;
  };
} ADI_PMG_TST_SRAM_INITSTAT_t;

/*@}*/

/** @defgroup CLR_LATCH_GPIOS CLEAR GPIO AFTER SHUTDOWN MODE (CLR_LATCH_GPIOS) Register
 *  CLEAR GPIO AFTER SHUTDOWN MODE (CLR_LATCH_GPIOS) Register.
 *
 *  CLEAR GPIO AFTER SHUTDOWN MODE
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_TST_CLR_LATCH_GPIOS_Struct
 *! \brief  CLEAR GPIO AFTER SHUTDOWN MODE Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_TST_CLR_LATCH_GPIOS_t {
  union {
    struct {
      unsigned int VALUE      : 16;  /**< Writing 0x58FA creates a pulse to clear the latches for the GPIOs */
    };
    uint16_t VALUE16;
  };
} ADI_PMG_TST_CLR_LATCH_GPIOS_t;

/*@}*/

/** @defgroup SCRPAD_IMG SCRATCH PAD IMAGE (SCRPAD_IMG) Register
 *  SCRATCH PAD IMAGE (SCRPAD_IMG) Register.
 *
 *  SCRATCH PAD IMAGE
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_TST_SCRPAD_IMG_Struct
 *! \brief  SCRATCH PAD IMAGE Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_TST_SCRPAD_IMG_t {
  union {
    struct {
      unsigned int DATA       : 32;  /**< Anything written to this register will be saved in 3V when going to shutdown mode */
    };
    uint32_t VALUE32;
  };
} ADI_PMG_TST_SCRPAD_IMG_t;

/*@}*/

/** @defgroup SCRPAD_3V_RD SCRATCH PAD SAVED IN BATTERY DOMAIN (SCRPAD_3V_RD) Register
 *  SCRATCH PAD SAVED IN BATTERY DOMAIN (SCRPAD_3V_RD) Register.
 *
 *  SCRATCH PAD SAVED IN BATTERY DOMAIN
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PMG_TST_SCRPAD_3V_RD_Struct
 *! \brief  SCRATCH PAD SAVED IN BATTERY DOMAIN Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PMG_TST_SCRPAD_3V_RD_t {
  union {
    struct {
      unsigned int DATA       : 32;  /**< Read Only register. Reading the scratch pad stored in shutdown mode */
    };
    uint32_t VALUE32;
  };
} ADI_PMG_TST_SCRPAD_3V_RD_t;

/*@}*/

/** @defgroup CTL0 Misc clock settings (CTL0) Register
 *  Misc clock settings (CTL0) Register.
 *
 *  Misc clock settings
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_CLK_CTL0_Struct
 *! \brief  Misc clock settings Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_CLK_CTL0_t {
  union {
    struct {
      unsigned int CLKMUX     :  2;  /**< Clock mux select */
      unsigned int reserved2  :  6;
      unsigned int RCLKMUX    :  2;  /**< Flash reference clock and HPBUCK clock source mux */
      unsigned int reserved10 :  1;
      unsigned int SPLLIPSEL  :  1;  /**< SPLL source select mux */
      unsigned int reserved12 :  2;
      unsigned int LFXTALIE   :  1;  /**< Low frequency crystal interrupt enable */
      unsigned int HFXTALIE   :  1;  /**< High frequency crystal interrupt enable */
      unsigned int reserved16 : 16;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_CLK_CTL0_t;

/*@}*/

/** @defgroup CTL1 Clock dividers (CTL1) Register
 *  Clock dividers (CTL1) Register.
 *
 *  Clock dividers
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_CLK_CTL1_Struct
 *! \brief  Clock dividers Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_CLK_CTL1_t {
  union {
    struct {
      unsigned int HCLKDIVCNT :  6;  /**< HCLK divide count */
      unsigned int reserved6  :  2;
      unsigned int PCLKDIVCNT :  6;  /**< PCLK divide count */
      unsigned int reserved14 :  2;
      unsigned int ACLKDIVCNT :  6;  /**< ACLK Divide Count. */
      unsigned int reserved22 : 10;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_CLK_CTL1_t;

/*@}*/

/** @defgroup CTL3 System PLL (CTL3) Register
 *  System PLL (CTL3) Register.
 *
 *  System PLL
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_CLK_CTL3_Struct
 *! \brief  System PLL Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_CLK_CTL3_t {
  union {
    struct {
      unsigned int SPLLNSEL   :  5;  /**< System PLL N multiplier */
      unsigned int reserved5  :  3;
      unsigned int SPLLDIV2   :  1;  /**< System PLL division by 2 */
      unsigned int SPLLEN     :  1;  /**< System PLL enable */
      unsigned int SPLLIE     :  1;  /**< System PLL interrupt enable */
      unsigned int SPLLMSEL   :  4;  /**< System PLL M Divider */
      unsigned int reserved15 :  1;
      unsigned int SPLLMUL2   :  1;  /**< system PLL multiply by 2 */
      unsigned int reserved17 : 15;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_CLK_CTL3_t;

/*@}*/

/** @defgroup CTL5 User clock gating control (CTL5) Register
 *  User clock gating control (CTL5) Register.
 *
 *  User clock gating control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_CLK_CTL5_Struct
 *! \brief  User clock gating control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_CLK_CTL5_t {
  union {
    struct {
      unsigned int GPTCLK0OFF :  1;  /**< GP timer 0 user control */
      unsigned int GPTCLK1OFF :  1;  /**< GP timer 1 user control */
      unsigned int GPTCLK2OFF :  1;  /**< GP timer 2 user control */
      unsigned int UCLKI2COFF :  1;  /**< I2C clock user control */
      unsigned int ACLKOFF    :  1;  /**< ADC clock user control. */
      unsigned int PERCLKOFF  :  1;  /**< This bit is used to disable all clocks connected to all peripherals */
      unsigned int reserved6  : 26;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_CLK_CTL5_t;

/*@}*/

/** @defgroup STAT0 Clocking status (STAT0) Register
 *  Clocking status (STAT0) Register.
 *
 *  Clocking status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_CLKG_CLK_STAT0_Struct
 *! \brief  Clocking status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_CLKG_CLK_STAT0_t {
  union {
    struct {
      unsigned int SPLL       :  1;  /**< System PLL status */
      unsigned int SPLLLK     :  1;  /**< System PLL lock */
      unsigned int SPLLUNLK   :  1;  /**< System PLL unlock */
      unsigned int reserved3  :  5;
      unsigned int LFXTAL     :  1;  /**< LF crystal status */
      unsigned int LFXTALOK   :  1;  /**< LF crystal stable */
      unsigned int LFXTALNOK  :  1;  /**< LF crystal not stable */
      unsigned int reserved11 :  1;
      unsigned int HFXTAL     :  1;  /**< HF crystal status */
      unsigned int HFXTALOK   :  1;  /**< HF crystal stable */
      unsigned int HFXTALNOK  :  1;  /**< HF crystal not stable */
      unsigned int reserved15 : 17;
    };
    uint32_t VALUE32;
  };
} ADI_CLKG_CLK_STAT0_t;

/*@}*/

/** @defgroup ARBIT0 Arbitration Priority Configuration for FLASH and SRAM0 (ARBIT0) Register
 *  Arbitration Priority Configuration for FLASH and SRAM0 (ARBIT0) Register.
 *
 *  Arbitration Priority Configuration for FLASH and SRAM0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BUSM_ARBIT0_Struct
 *! \brief  Arbitration Priority Configuration for FLASH and SRAM0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BUSM_ARBIT0_t {
  union {
    struct {
      unsigned int FLSH_DCODE :  2;  /**< Flash priority for DCODE */
      unsigned int FLSH_SBUS  :  2;  /**< Flash priority for SBUS */
      unsigned int FLSH_DMA0  :  2;  /**< Flash priority for DMA0 */
      unsigned int reserved6   : 10;
      unsigned int SRAM0_DCODE :  2;  /**< SRAM0 priority for Dcode */
      unsigned int SRAM0_SBUS  :  2;  /**< SRAM0 priority for SBUS */
      unsigned int SRAM0_DMA0  :  2;  /**< SRAM0 priority for DMA0 */
      unsigned int reserved22  : 10;
    };
    uint32_t VALUE32;
  };
} ADI_BUSM_ARBIT0_t;

/*@}*/

/** @defgroup ARBIT1 Arbitration Priority Configuration for SRAM1 and SIP (ARBIT1) Register
 *  Arbitration Priority Configuration for SRAM1 and SIP (ARBIT1) Register.
 *
 *  Arbitration Priority Configuration for SRAM1 and SIP
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BUSM_ARBIT1_Struct
 *! \brief  Arbitration Priority Configuration for SRAM1 and SIP Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BUSM_ARBIT1_t {
  union {
    struct {
      unsigned int SRAM1_DCODE :  2;  /**< SRAM1 priority for Dcode */
      unsigned int SRAM1_SBUS  :  2;  /**< SRAM1 priority for SBUS */
      unsigned int SRAM1_DMA0  :  2;  /**< SRAM1 priority for DMA0 */
      unsigned int reserved6   : 10;
      unsigned int SIP_DCODE   :  2;  /**< SIP priority for DCODE */
      unsigned int SIP_SBUS    :  2;  /**< SIP priority for SBUS */
      unsigned int SIP_DMA0    :  2;  /**< SIP priority for DMA0 */
      unsigned int reserved22  : 10;
    };
    uint32_t VALUE32;
  };
} ADI_BUSM_ARBIT1_t;

/*@}*/

/** @defgroup ARBIT2 Arbitration Priority Configuration for APB32 and APB16 (ARBIT2) Register
 *  Arbitration Priority Configuration for APB32 and APB16 (ARBIT2) Register.
 *
 *  Arbitration Priority Configuration for APB32 and APB16
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BUSM_ARBIT2_Struct
 *! \brief  Arbitration Priority Configuration for APB32 and APB16 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BUSM_ARBIT2_t {
  union {
    struct {
      unsigned int APB32_DCODE :  2;  /**< APB32 priority for DCODE */
      unsigned int APB32_SBUS  :  2;  /**< APB32 priority for SBUS */
      unsigned int APB32_DMA0  :  2;  /**< APB32 priority for DMA0 */
      unsigned int reserved6   : 10;
      unsigned int APB16_DCODE :  2;  /**< APB16 priority for DCODE */
      unsigned int APB16_SBUS  :  2;  /**< APB16 priority for SBUS */
      unsigned int APB16_DMA0  :  2;  /**< APB16 priority for DMA0 */
      unsigned int reserved22  : 10;
    };
    uint32_t VALUE32;
  };
} ADI_BUSM_ARBIT2_t;

/*@}*/

/** @defgroup ARBIT3 Arbitration Priority Configuration for APB16 priority for core and for DMA1 (ARBIT3) Register
 *  Arbitration Priority Configuration for APB16 priority for core and for DMA1 (ARBIT3) Register.
 *
 *  Arbitration Priority Configuration for APB16 priority for core and for DMA1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_BUSM_ARBIT3_Struct
 *! \brief  Arbitration Priority Configuration for APB16 priority for core and for DMA1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_BUSM_ARBIT3_t {
  union {
    struct {
      unsigned int APB16_CORE :  1;  /**< APB16 priority for CORE */
      unsigned int APB16_DMA1 :  1;  /**< APB16 priority for DMA1 */
      unsigned int reserved2       : 14;
      unsigned int APB16_4DMA_CORE :  1;  /**< APB16 for dma priority for CORE */
      unsigned int APB16_4DMA_DMA1 :  1;  /**< APB16 for dma priority for DMA1 */
      unsigned int reserved18      : 14;
    };
    uint32_t VALUE32;
  };
} ADI_BUSM_ARBIT3_t;

/*@}*/

/** @defgroup RST_ISR_STARTADDR Reset ISR Start Address (RST_ISR_STARTADDR) Register
 *  Reset ISR Start Address (RST_ISR_STARTADDR) Register.
 *
 *  Reset ISR Start Address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PTI_RST_ISR_STARTADDR_Struct
 *! \brief  Reset ISR Start Address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PTI_RST_ISR_STARTADDR_t {
  union {
    struct {
      unsigned int VALUE      : 32;
    };
    uint32_t VALUE32;
  };
} ADI_PTI_RST_ISR_STARTADDR_t;

/*@}*/

/** @defgroup RST_STACK_PTR Reset Stack Pointer (RST_STACK_PTR) Register
 *  Reset Stack Pointer (RST_STACK_PTR) Register.
 *
 *  Reset Stack Pointer
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PTI_RST_STACK_PTR_Struct
 *! \brief  Reset Stack Pointer Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PTI_RST_STACK_PTR_t {
  union {
    struct {
      unsigned int VALUE      : 32;
    };
    uint32_t VALUE32;
  };
} ADI_PTI_RST_STACK_PTR_t;

/*@}*/

/** @defgroup CTL Parallel Test Interface Control Register (CTL) Register
 *  Parallel Test Interface Control Register (CTL) Register.
 *
 *  Parallel Test Interface Control Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_PTI_CTL_Struct
 *! \brief  Parallel Test Interface Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_PTI_CTL_t {
  union {
    struct {
      unsigned int EN         :  1;
      unsigned int reserved1  : 31;
    };
    uint32_t VALUE32;
  };
} ADI_PTI_CTL_t;

/*@}*/

/** @defgroup INTNUM Interrupt Control Type (INTNUM) Register
 *  Interrupt Control Type (INTNUM) Register.
 *
 *  Interrupt Control Type
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTNUM_Struct
 *! \brief  Interrupt Control Type Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTNUM_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Interrupt Control Type */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTNUM_t;

/*@}*/

/** @defgroup STKSTA Systick Control and Status (STKSTA) Register
 *  Systick Control and Status (STKSTA) Register.
 *
 *  Systick Control and Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_STKSTA_Struct
 *! \brief  Systick Control and Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_STKSTA_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Systick Control and Status */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_STKSTA_t;

/*@}*/

/** @defgroup STKLD Systick Reload Value (STKLD) Register
 *  Systick Reload Value (STKLD) Register.
 *
 *  Systick Reload Value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_STKLD_Struct
 *! \brief  Systick Reload Value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_STKLD_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Systick Reload Value */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_STKLD_t;

/*@}*/

/** @defgroup STKVAL Systick Current Value (STKVAL) Register
 *  Systick Current Value (STKVAL) Register.
 *
 *  Systick Current Value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_STKVAL_Struct
 *! \brief  Systick Current Value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_STKVAL_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Systick Current Value */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_STKVAL_t;

/*@}*/

/** @defgroup STKCAL Systick Calibration Value (STKCAL) Register
 *  Systick Calibration Value (STKCAL) Register.
 *
 *  Systick Calibration Value
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_STKCAL_Struct
 *! \brief  Systick Calibration Value Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_STKCAL_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Systick Calibration Value */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_STKCAL_t;

/*@}*/

/** @defgroup INTSETE0 IRQ0..31 Set_Enable (INTSETE0) Register
 *  IRQ0..31 Set_Enable (INTSETE0) Register.
 *
 *  IRQ0..31 Set_Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSETE0_Struct
 *! \brief  IRQ0..31 Set_Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSETE0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ0..31 Set_Enable */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSETE0_t;

/*@}*/

/** @defgroup INTSETE1 IRQ32..63 Set_Enable (INTSETE1) Register
 *  IRQ32..63 Set_Enable (INTSETE1) Register.
 *
 *  IRQ32..63 Set_Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSETE1_Struct
 *! \brief  IRQ32..63 Set_Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSETE1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ32..63 Set_Enable */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSETE1_t;

/*@}*/

/** @defgroup INTCLRE0 IRQ0..31 Clear_Enable (INTCLRE0) Register
 *  IRQ0..31 Clear_Enable (INTCLRE0) Register.
 *
 *  IRQ0..31 Clear_Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCLRE0_Struct
 *! \brief  IRQ0..31 Clear_Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCLRE0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ0..31 Clear_Enable */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCLRE0_t;

/*@}*/

/** @defgroup INTCLRE1 IRQ32..63 Clear_Enable (INTCLRE1) Register
 *  IRQ32..63 Clear_Enable (INTCLRE1) Register.
 *
 *  IRQ32..63 Clear_Enable
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCLRE1_Struct
 *! \brief  IRQ32..63 Clear_Enable Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCLRE1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ32..63 Clear_Enable */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCLRE1_t;

/*@}*/

/** @defgroup INTSETP0 IRQ0..31 Set_Pending (INTSETP0) Register
 *  IRQ0..31 Set_Pending (INTSETP0) Register.
 *
 *  IRQ0..31 Set_Pending
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSETP0_Struct
 *! \brief  IRQ0..31 Set_Pending Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSETP0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ0..31 Set_Pending */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSETP0_t;

/*@}*/

/** @defgroup INTSETP1 IRQ32..63 Set_Pending (INTSETP1) Register
 *  IRQ32..63 Set_Pending (INTSETP1) Register.
 *
 *  IRQ32..63 Set_Pending
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSETP1_Struct
 *! \brief  IRQ32..63 Set_Pending Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSETP1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ32..63 Set_Pending */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSETP1_t;

/*@}*/

/** @defgroup INTCLRP0 IRQ0..31 Clear_Pending (INTCLRP0) Register
 *  IRQ0..31 Clear_Pending (INTCLRP0) Register.
 *
 *  IRQ0..31 Clear_Pending
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCLRP0_Struct
 *! \brief  IRQ0..31 Clear_Pending Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCLRP0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ0..31 Clear_Pending */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCLRP0_t;

/*@}*/

/** @defgroup INTCLRP1 IRQ32..63 Clear_Pending (INTCLRP1) Register
 *  IRQ32..63 Clear_Pending (INTCLRP1) Register.
 *
 *  IRQ32..63 Clear_Pending
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCLRP1_Struct
 *! \brief  IRQ32..63 Clear_Pending Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCLRP1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ32..63 Clear_Pending */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCLRP1_t;

/*@}*/

/** @defgroup INTACT0 IRQ0..31 Active Bit (INTACT0) Register
 *  IRQ0..31 Active Bit (INTACT0) Register.
 *
 *  IRQ0..31 Active Bit
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTACT0_Struct
 *! \brief  IRQ0..31 Active Bit Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTACT0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ0..31 Active Bit */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTACT0_t;

/*@}*/

/** @defgroup INTACT1 IRQ32..63 Active Bit (INTACT1) Register
 *  IRQ32..63 Active Bit (INTACT1) Register.
 *
 *  IRQ32..63 Active Bit
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTACT1_Struct
 *! \brief  IRQ32..63 Active Bit Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTACT1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ32..63 Active Bit */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTACT1_t;

/*@}*/

/** @defgroup INTPRI0 IRQ0..3 Priority (INTPRI0) Register
 *  IRQ0..3 Priority (INTPRI0) Register.
 *
 *  IRQ0..3 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI0_Struct
 *! \brief  IRQ0..3 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ0..3 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI0_t;

/*@}*/

/** @defgroup INTPRI1 IRQ4..7 Priority (INTPRI1) Register
 *  IRQ4..7 Priority (INTPRI1) Register.
 *
 *  IRQ4..7 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI1_Struct
 *! \brief  IRQ4..7 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ4..7 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI1_t;

/*@}*/

/** @defgroup INTPRI2 IRQ8..11 Priority (INTPRI2) Register
 *  IRQ8..11 Priority (INTPRI2) Register.
 *
 *  IRQ8..11 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI2_Struct
 *! \brief  IRQ8..11 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ8..11 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI2_t;

/*@}*/

/** @defgroup INTPRI3 IRQ12..15 Priority (INTPRI3) Register
 *  IRQ12..15 Priority (INTPRI3) Register.
 *
 *  IRQ12..15 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI3_Struct
 *! \brief  IRQ12..15 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ12..15 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI3_t;

/*@}*/

/** @defgroup INTPRI4 IRQ16..19 Priority (INTPRI4) Register
 *  IRQ16..19 Priority (INTPRI4) Register.
 *
 *  IRQ16..19 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI4_Struct
 *! \brief  IRQ16..19 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI4_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ16..19 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI4_t;

/*@}*/

/** @defgroup INTPRI5 IRQ20..23 Priority (INTPRI5) Register
 *  IRQ20..23 Priority (INTPRI5) Register.
 *
 *  IRQ20..23 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI5_Struct
 *! \brief  IRQ20..23 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI5_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ20..23 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI5_t;

/*@}*/

/** @defgroup INTPRI6 IRQ24..27 Priority (INTPRI6) Register
 *  IRQ24..27 Priority (INTPRI6) Register.
 *
 *  IRQ24..27 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI6_Struct
 *! \brief  IRQ24..27 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI6_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ24..27 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI6_t;

/*@}*/

/** @defgroup INTPRI7 IRQ28..31 Priority (INTPRI7) Register
 *  IRQ28..31 Priority (INTPRI7) Register.
 *
 *  IRQ28..31 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI7_Struct
 *! \brief  IRQ28..31 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI7_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ28..31 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI7_t;

/*@}*/

/** @defgroup INTPRI8 IRQ32..35 Priority (INTPRI8) Register
 *  IRQ32..35 Priority (INTPRI8) Register.
 *
 *  IRQ32..35 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI8_Struct
 *! \brief  IRQ32..35 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI8_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ32..35 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI8_t;

/*@}*/

/** @defgroup INTPRI9 IRQ36..39 Priority (INTPRI9) Register
 *  IRQ36..39 Priority (INTPRI9) Register.
 *
 *  IRQ36..39 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI9_Struct
 *! \brief  IRQ36..39 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI9_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ36..39 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI9_t;

/*@}*/

/** @defgroup INTPRI10 IRQ40..43 Priority (INTPRI10) Register
 *  IRQ40..43 Priority (INTPRI10) Register.
 *
 *  IRQ40..43 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPRI10_Struct
 *! \brief  IRQ40..43 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPRI10_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< IRQ40..43 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPRI10_t;

/*@}*/

/** @defgroup INTCPID CPUID Base (INTCPID) Register
 *  CPUID Base (INTCPID) Register.
 *
 *  CPUID Base
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCPID_Struct
 *! \brief  CPUID Base Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCPID_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< CPUID Base */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCPID_t;

/*@}*/

/** @defgroup INTSTA Interrupt Control State (INTSTA) Register
 *  Interrupt Control State (INTSTA) Register.
 *
 *  Interrupt Control State
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSTA_Struct
 *! \brief  Interrupt Control State Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSTA_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Interrupt Control State */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSTA_t;

/*@}*/

/** @defgroup INTVEC Vector Table Offset (INTVEC) Register
 *  Vector Table Offset (INTVEC) Register.
 *
 *  Vector Table Offset
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTVEC_Struct
 *! \brief  Vector Table Offset Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTVEC_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Vector Table Offset */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTVEC_t;

/*@}*/

/** @defgroup INTAIRC Application Interrupt/Reset Control (INTAIRC) Register
 *  Application Interrupt/Reset Control (INTAIRC) Register.
 *
 *  Application Interrupt/Reset Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTAIRC_Struct
 *! \brief  Application Interrupt/Reset Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTAIRC_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Application Interrupt/Reset Control */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTAIRC_t;

/*@}*/

/** @defgroup INTCON0 System Control (INTCON0) Register
 *  System Control (INTCON0) Register.
 *
 *  System Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCON0_Struct
 *! \brief  System Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCON0_t {
  union {
    struct {
      unsigned int reserved0   :  1;
      unsigned int SLEEPONEXIT :  1;  /**< Sleeps the core on exit from an ISR */
      unsigned int SLEEPDEEP   :  1;  /**< deep sleep flag for HIBERNATE mode */
      unsigned int reserved3   : 13;
    };
    uint16_t VALUE16;
  };
} ADI_NVIC_INTCON0_t;

/*@}*/

/** @defgroup INTCON1 Configuration Control (INTCON1) Register
 *  Configuration Control (INTCON1) Register.
 *
 *  Configuration Control
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCON1_Struct
 *! \brief  Configuration Control Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCON1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Configuration Control */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCON1_t;

/*@}*/

/** @defgroup INTSHPRIO0 System Handlers 4-7 Priority (INTSHPRIO0) Register
 *  System Handlers 4-7 Priority (INTSHPRIO0) Register.
 *
 *  System Handlers 4-7 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSHPRIO0_Struct
 *! \brief  System Handlers 4-7 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSHPRIO0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< System Handlers 4-7 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSHPRIO0_t;

/*@}*/

/** @defgroup INTSHPRIO1 System Handlers 8-11 Priority (INTSHPRIO1) Register
 *  System Handlers 8-11 Priority (INTSHPRIO1) Register.
 *
 *  System Handlers 8-11 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSHPRIO1_Struct
 *! \brief  System Handlers 8-11 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSHPRIO1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< System Handlers 8-11 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSHPRIO1_t;

/*@}*/

/** @defgroup INTSHPRIO3 System Handlers 12-15 Priority (INTSHPRIO3) Register
 *  System Handlers 12-15 Priority (INTSHPRIO3) Register.
 *
 *  System Handlers 12-15 Priority
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSHPRIO3_Struct
 *! \brief  System Handlers 12-15 Priority Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSHPRIO3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< System Handlers 12-15 Priority */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSHPRIO3_t;

/*@}*/

/** @defgroup INTSHCSR System Handler Control and State (INTSHCSR) Register
 *  System Handler Control and State (INTSHCSR) Register.
 *
 *  System Handler Control and State
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTSHCSR_Struct
 *! \brief  System Handler Control and State Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTSHCSR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< System Handler Control and State */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTSHCSR_t;

/*@}*/

/** @defgroup INTCFSR Configurable Fault Status (INTCFSR) Register
 *  Configurable Fault Status (INTCFSR) Register.
 *
 *  Configurable Fault Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCFSR_Struct
 *! \brief  Configurable Fault Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCFSR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Configurable Fault Status */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCFSR_t;

/*@}*/

/** @defgroup INTHFSR Hard Fault Status (INTHFSR) Register
 *  Hard Fault Status (INTHFSR) Register.
 *
 *  Hard Fault Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTHFSR_Struct
 *! \brief  Hard Fault Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTHFSR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Hard Fault Status */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTHFSR_t;

/*@}*/

/** @defgroup INTDFSR Debug Fault Status (INTDFSR) Register
 *  Debug Fault Status (INTDFSR) Register.
 *
 *  Debug Fault Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTDFSR_Struct
 *! \brief  Debug Fault Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTDFSR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Debug Fault Status */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTDFSR_t;

/*@}*/

/** @defgroup INTMMAR Mem Manage Address (INTMMAR) Register
 *  Mem Manage Address (INTMMAR) Register.
 *
 *  Mem Manage Address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTMMAR_Struct
 *! \brief  Mem Manage Address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTMMAR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Mem Manage Address */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTMMAR_t;

/*@}*/

/** @defgroup INTBFAR Bus Fault Address (INTBFAR) Register
 *  Bus Fault Address (INTBFAR) Register.
 *
 *  Bus Fault Address
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTBFAR_Struct
 *! \brief  Bus Fault Address Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTBFAR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Bus Fault Address */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTBFAR_t;

/*@}*/

/** @defgroup INTAFSR Auxiliary Fault Status (INTAFSR) Register
 *  Auxiliary Fault Status (INTAFSR) Register.
 *
 *  Auxiliary Fault Status
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTAFSR_Struct
 *! \brief  Auxiliary Fault Status Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTAFSR_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Auxiliary Fault Status */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTAFSR_t;

/*@}*/

/** @defgroup INTPFR0 Processor Feature Register 0 (INTPFR0) Register
 *  Processor Feature Register 0 (INTPFR0) Register.
 *
 *  Processor Feature Register 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPFR0_Struct
 *! \brief  Processor Feature Register 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPFR0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Processor Feature Register 0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPFR0_t;

/*@}*/

/** @defgroup INTPFR1 Processor Feature Register 1 (INTPFR1) Register
 *  Processor Feature Register 1 (INTPFR1) Register.
 *
 *  Processor Feature Register 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPFR1_Struct
 *! \brief  Processor Feature Register 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPFR1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Processor Feature Register 1 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPFR1_t;

/*@}*/

/** @defgroup INTDFR0 Debug Feature Register 0 (INTDFR0) Register
 *  Debug Feature Register 0 (INTDFR0) Register.
 *
 *  Debug Feature Register 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTDFR0_Struct
 *! \brief  Debug Feature Register 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTDFR0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Debug Feature Register 0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTDFR0_t;

/*@}*/

/** @defgroup INTAFR0 Auxiliary Feature Register 0 (INTAFR0) Register
 *  Auxiliary Feature Register 0 (INTAFR0) Register.
 *
 *  Auxiliary Feature Register 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTAFR0_Struct
 *! \brief  Auxiliary Feature Register 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTAFR0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Auxiliary Feature Register 0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTAFR0_t;

/*@}*/

/** @defgroup INTMMFR0 Memory Model Feature Register 0 (INTMMFR0) Register
 *  Memory Model Feature Register 0 (INTMMFR0) Register.
 *
 *  Memory Model Feature Register 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTMMFR0_Struct
 *! \brief  Memory Model Feature Register 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTMMFR0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Memory Model Feature Register 0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTMMFR0_t;

/*@}*/

/** @defgroup INTMMFR1 Memory Model Feature Register 1 (INTMMFR1) Register
 *  Memory Model Feature Register 1 (INTMMFR1) Register.
 *
 *  Memory Model Feature Register 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTMMFR1_Struct
 *! \brief  Memory Model Feature Register 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTMMFR1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Memory Model Feature Register 1 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTMMFR1_t;

/*@}*/

/** @defgroup INTMMFR2 Memory Model Feature Register 2 (INTMMFR2) Register
 *  Memory Model Feature Register 2 (INTMMFR2) Register.
 *
 *  Memory Model Feature Register 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTMMFR2_Struct
 *! \brief  Memory Model Feature Register 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTMMFR2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Memory Model Feature Register 2 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTMMFR2_t;

/*@}*/

/** @defgroup INTMMFR3 Memory Model Feature Register 3 (INTMMFR3) Register
 *  Memory Model Feature Register 3 (INTMMFR3) Register.
 *
 *  Memory Model Feature Register 3
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTMMFR3_Struct
 *! \brief  Memory Model Feature Register 3 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTMMFR3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Memory Model Feature Register 3 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTMMFR3_t;

/*@}*/

/** @defgroup INTISAR0 ISA Feature Register 0 (INTISAR0) Register
 *  ISA Feature Register 0 (INTISAR0) Register.
 *
 *  ISA Feature Register 0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTISAR0_Struct
 *! \brief  ISA Feature Register 0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTISAR0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< ISA Feature Register 0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTISAR0_t;

/*@}*/

/** @defgroup INTISAR1 ISA Feature Register 1 (INTISAR1) Register
 *  ISA Feature Register 1 (INTISAR1) Register.
 *
 *  ISA Feature Register 1
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTISAR1_Struct
 *! \brief  ISA Feature Register 1 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTISAR1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< ISA Feature Register 1 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTISAR1_t;

/*@}*/

/** @defgroup INTISAR2 ISA Feature Register 2 (INTISAR2) Register
 *  ISA Feature Register 2 (INTISAR2) Register.
 *
 *  ISA Feature Register 2
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTISAR2_Struct
 *! \brief  ISA Feature Register 2 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTISAR2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< ISA Feature Register 2 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTISAR2_t;

/*@}*/

/** @defgroup INTISAR3 ISA Feature Register 3 (INTISAR3) Register
 *  ISA Feature Register 3 (INTISAR3) Register.
 *
 *  ISA Feature Register 3
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTISAR3_Struct
 *! \brief  ISA Feature Register 3 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTISAR3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< ISA Feature Register 3 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTISAR3_t;

/*@}*/

/** @defgroup INTISAR4 ISA Feature Register 4 (INTISAR4) Register
 *  ISA Feature Register 4 (INTISAR4) Register.
 *
 *  ISA Feature Register 4
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTISAR4_Struct
 *! \brief  ISA Feature Register 4 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTISAR4_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< ISA Feature Register 4 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTISAR4_t;

/*@}*/

/** @defgroup INTTRGI Software Trigger Interrupt Register (INTTRGI) Register
 *  Software Trigger Interrupt Register (INTTRGI) Register.
 *
 *  Software Trigger Interrupt Register
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTTRGI_Struct
 *! \brief  Software Trigger Interrupt Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTTRGI_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Software Trigger Interrupt Register */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTTRGI_t;

/*@}*/

/** @defgroup INTPID4 Peripheral Identification Register 4 (INTPID4) Register
 *  Peripheral Identification Register 4 (INTPID4) Register.
 *
 *  Peripheral Identification Register 4
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID4_Struct
 *! \brief  Peripheral Identification Register 4 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID4_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Register 4 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID4_t;

/*@}*/

/** @defgroup INTPID5 Peripheral Identification Register 5 (INTPID5) Register
 *  Peripheral Identification Register 5 (INTPID5) Register.
 *
 *  Peripheral Identification Register 5
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID5_Struct
 *! \brief  Peripheral Identification Register 5 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID5_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Register 5 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID5_t;

/*@}*/

/** @defgroup INTPID6 Peripheral Identification Register 6 (INTPID6) Register
 *  Peripheral Identification Register 6 (INTPID6) Register.
 *
 *  Peripheral Identification Register 6
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID6_Struct
 *! \brief  Peripheral Identification Register 6 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID6_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Register 6 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID6_t;

/*@}*/

/** @defgroup INTPID7 Peripheral Identification Register 7 (INTPID7) Register
 *  Peripheral Identification Register 7 (INTPID7) Register.
 *
 *  Peripheral Identification Register 7
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID7_Struct
 *! \brief  Peripheral Identification Register 7 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID7_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Register 7 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID7_t;

/*@}*/

/** @defgroup INTPID0 Peripheral Identification Bits7:0 (INTPID0) Register
 *  Peripheral Identification Bits7:0 (INTPID0) Register.
 *
 *  Peripheral Identification Bits7:0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID0_Struct
 *! \brief  Peripheral Identification Bits7:0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Bits7:0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID0_t;

/*@}*/

/** @defgroup INTPID1 Peripheral Identification Bits15:8 (INTPID1) Register
 *  Peripheral Identification Bits15:8 (INTPID1) Register.
 *
 *  Peripheral Identification Bits15:8
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID1_Struct
 *! \brief  Peripheral Identification Bits15:8 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Bits15:8 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID1_t;

/*@}*/

/** @defgroup INTPID2 Peripheral Identification Bits16:23 (INTPID2) Register
 *  Peripheral Identification Bits16:23 (INTPID2) Register.
 *
 *  Peripheral Identification Bits16:23
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID2_Struct
 *! \brief  Peripheral Identification Bits16:23 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Bits16:23 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID2_t;

/*@}*/

/** @defgroup INTPID3 Peripheral Identification Bits24:31 (INTPID3) Register
 *  Peripheral Identification Bits24:31 (INTPID3) Register.
 *
 *  Peripheral Identification Bits24:31
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTPID3_Struct
 *! \brief  Peripheral Identification Bits24:31 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTPID3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Peripheral Identification Bits24:31 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTPID3_t;

/*@}*/

/** @defgroup INTCID0 Component Identification Bits7:0 (INTCID0) Register
 *  Component Identification Bits7:0 (INTCID0) Register.
 *
 *  Component Identification Bits7:0
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCID0_Struct
 *! \brief  Component Identification Bits7:0 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCID0_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Component Identification Bits7:0 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCID0_t;

/*@}*/

/** @defgroup INTCID1 Component Identification Bits15:8 (INTCID1) Register
 *  Component Identification Bits15:8 (INTCID1) Register.
 *
 *  Component Identification Bits15:8
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCID1_Struct
 *! \brief  Component Identification Bits15:8 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCID1_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Component Identification Bits15:8 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCID1_t;

/*@}*/

/** @defgroup INTCID2 Component Identification Bits16:23 (INTCID2) Register
 *  Component Identification Bits16:23 (INTCID2) Register.
 *
 *  Component Identification Bits16:23
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCID2_Struct
 *! \brief  Component Identification Bits16:23 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCID2_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Component Identification Bits16:23 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCID2_t;

/*@}*/

/** @defgroup INTCID3 Component Identification Bits24:31 (INTCID3) Register
 *  Component Identification Bits24:31 (INTCID3) Register.
 *
 *  Component Identification Bits24:31
 *  @{
 */

/* ==========================================================================
 *! \struct ADI_NVIC_INTCID3_Struct
 *! \brief  Component Identification Bits24:31 Register bit field structure
 * ========================================================================== */
typedef struct _ADI_NVIC_INTCID3_t {
  union {
    struct {
      unsigned int VALUE      : 32;  /**< Component Identification Bits24:31 */
    };
    uint32_t VALUE32;
  };
} ADI_NVIC_INTCID3_t;

/*@}*/

