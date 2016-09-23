/*********************************************************************************

Copyright(c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
* @file     temperature_sensor.h
*
* @brief    Primary header file for i2c device driver example.
*
* @details  Primary header file for i2c driver example which contains the
*           processor specific defines.
*
*/

#ifndef _TEMPERATURE_SENSOR_H_
#define _TEMPERATURE_SENSOR_H_

#define TARGETADDR    (0x48u)     /* TODO: hardware address for the Temperature Monitor (ADT7420) */

#define TWIDEVNUM     (0u)        /* TWI device number */
#define BITRATE       (100u)      /* kHz */
#define DUTYCYCLE     (50u)       /* percent */
#define PRESCALEVALUE (125u/10u)  /* fSCLK/10MHz */

#define TEMPREG_MSB   (0x0u)      /* temperature MSB register */
#define TEMPREG_LSB   (0x1u)      /* temperature LSB register */
#define ID_REG        (0x0Bu)     /* manufacture ID register */
#define TEST_VALUE    (0xCBu)     /* expected value */


/* Pin muxing */
extern int32_t adi_initpinmux(void);
void reinit_uart();

#endif /* _TEMPERATURE_SENSOR_H_ */
