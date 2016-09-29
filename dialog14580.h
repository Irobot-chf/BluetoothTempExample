
#ifndef __DIALOG_SPI_M350_H
#define __DIALOG_SPI_M350_H


#include <services/gpio/adi_gpio.h>


#define BLE_RST_PIN     ADI_GPIO_PIN_12
#define BLE_RST_PORT    ADI_GPIO_PORT0
#define BLE_LED_PIN    ADI_GPIO_PIN_4
#define BLE_LED_PORT    ADI_GPIO_PORT2

#define RESET_LENGTH     10 //ms


uint32_t adi_Dialog14580_SPI_Boot(uint8_t const * bin, uint32_t length);
uint8_t calc_crc(uint8_t const * bin, uint32_t length);

#endif