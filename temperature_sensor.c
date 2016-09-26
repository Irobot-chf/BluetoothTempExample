/*!
 *****************************************************************************
 * @file:    temperature_sensor.c
 * @brief:   ADT7420 temperature sensor example using I2C device driver.
 * @version: $Revision: 28843 $
 * @date:    $Date: 2014-11-28 09:10:14 -0500 (Fri, 28 Nov 2014) $
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2011-2014 Analog Devices, Inc.

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
/*!
* @file      temperature_sensor.c
*
* @brief     ADT7420 temperature sensor example using I2C device driver
*
*/

#include <stdio.h>
#include <assert.h>
#include <system.h>
#include <drivers/i2c/adi_i2c.h>
#include <services/pwr/adi_pwr.h>
#include <services/gpio/adi_gpio.h>
#include "common.h"
#include "temperature_sensor.h"
#include "uart_handler.h"
#include "dialog14580.h"

//#include "sps_device_580.h"
#include "sps_device_dialog.h"
//#include "BLE_code_beacon.h"

typedef enum
{
  REG_READ      = 0,
  REG_WRITE		= 1
} REG_RW_MODE;

typedef enum
{
  ADI_ADS_API_FAIL      = -1,
  ADI_ADS_API_SUCCESS   = 0
} ADSENSORAPP_RESULT_TYPE;

uint8_t BootResult = 0;

/* Handle for UART device */
#pragma data_alignment=4
ADI_UART_HANDLE          hUartDevice;

/* Memory for  UART driver */
#pragma data_alignment=4
uint8_t UartDeviceMem[ADI_UART_MEMORY_SIZE];

/* Memory for GPIO callbacks */
static uint8_t GPIOCallbackMem[ADI_GPIO_MEMORY_SIZE];

static ADI_I2C_HANDLE masterDev;

extern void ftoa(float f,char *buf);
void Delay_ms(unsigned int mSec);

static uint8_t counter = 0;
volatile bool_t hbFlag = 0;
static bool_t   wait_flag;

void extInt0Callback(void *pCBParam, uint32_t Event, void *pArg)
{
  *(bool_t*)pCBParam = true;
}

float   SHT25[2];
float   MAX44009;

unsigned char 	RxBuffer[];
char   BLE_Payload[14];
unsigned long   Msg_Count = 0;

unsigned char   BLE_UID[20] = {0x00, 0xEE, 0xAD, 0x14, 0x51, 0xDE, 0x21, 0xD8, 0x91, 0x67, 0x8A, 0xCF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};

//#define BLE_CODE_SIZE 10740
//#define BLE_CODE_SIZE 11820      //Beacon 
//#define BLE_CODE_SIZE 20860        //Paired

extern uint8_t ble_code;
static uint8_t* ble_code_ptr = &ble_code;

uint32_t ADI_Sensor_Delay = 0x10000;	//0x1000;//0x1F0;
void adsAPI_Delay(uint32_t value)  {while(value--);}

/*
 * Read a ADT7420 register value
 */

//==============================================================================
// General delay code. Delays approx. in mSec.
//==============================================================================
void Delay_ms(unsigned int mSec)
{
  int d1, d2;
  d1 = mSec;
  d2 = mSec;
  for ( d1 = 0; d1 < mSec; d1++ )
  {
   for ( d2 = 0; d2 < 3400; d2++ )
   {
   }
  }
}

void UARTCallback( void *pAppHandle, uint32_t nEvent, void *pArg)
{
   uint8_t Index;
}

void InitUART()
{
  /* Variable for storing the return code from UART device */
  ADI_UART_RESULT  eUartResult;
  
 eUartResult = adi_uart_Open(UART_DEVICE_NUM,ADI_UART_DIR_BIDIRECTION,UartDeviceMem,ADI_UART_MEMORY_SIZE,&hUartDevice);
  

  /* Configure  UART device with NO-PARITY, ONE STOP BIT and 8bit word length. */
  eUartResult = adi_uart_SetConfiguration(hUartDevice,
                                              ADI_UART_NO_PARITY,
                                              ADI_UART_ONE_AND_HALF_TWO_STOPBITS,
                                              ADI_UART_WORDLEN_8BITS);
  
  /* Baud rate div values are calcuated for PCLK 26Mhz. Please use the
  host utility UartDivCalculator.exe provided with the installer"
  */
  eUartResult = adi_uart_ConfigBaudRate(hUartDevice,
                                            UART_DIV_C_115200,
                                            UART_DIV_M_115200,
                                            UART_DIV_N_115200,
                                            UART_OSR_115200);
 
  /* Enable the Data flow for Rx */
  eUartResult = adi_uart_EnableRx(hUartDevice,true);

  /* Enable the Data flow for Tx */
  eUartResult = adi_uart_EnableTx(hUartDevice,true); 
  
  eUartResult = adi_uart_RegisterCallback(hUartDevice,UARTCallback,(void*)0);
  eUartResult = adi_uart_SubmitRxBuffer(hUartDevice,RxBuffer,1);
  
}

/*-------------------------------------------------------------------------*/
/* Print a string to the terminal */
void PRINT_C(char* string, int len) {

  int16_t size_l = 0;
  size_l = strlen(string);

  /*if(adi_uart_SubmitTxBuffer(hUartDevice,string,size_l) == ADI_UART_SUCCESS)
    adi_uart_EnableTx(hUartDevice,true);*/
  
  //for(int i = 0; i < size_l; i++) {
  for(int i = 0; i < len; i++) { // for BLE Payload
    pADI_UART0->COMTX = string[i];
    while((pADI_UART0->COMLSR &((uint16_t) BITM_UART_COMLSR_TEMT)) == 0 );
    }
}

void Print_SensData_on_UART()
{
    /*char buffer [8];
    char crlf[2] = {0x0A, 0x0D};
    char *testbuff;
    static int i=0;

    ftoa(MAX44009, &buffer[0]);
    PRINT_C("MAX44009: ");
    PRINT_C(buffer);        
    
    ftoa(SHT25[0],buffer);
    PRINT_C("SHT25_T: ");
    PRINT_C(buffer);
    
    ftoa(SHT25[1],buffer);
    PRINT_C("SHT25_RH: ");
    PRINT_C(buffer);
    PRINT_C(crlf);*/

}


/*
 * Read a ADT7420 register value
 */
static ADI_I2C_RESULT ReadRegister(uint8_t reg, uint8_t *value)
{
    uint8_t WriteBHuffer[1];
    ADI_I2C_RESULT eResult=ADI_I2C_SUCCESS;
    
    /* write register address */
    WriteBHuffer[0] = reg;
    void* pBuffer;
    
    if(ADI_I2C_SUCCESS != (eResult = adi_i2c_SubmitTxBuffer(masterDev, WriteBHuffer, 1u, true)))
    {
        DEBUG_MESSAGE("adi_i2c_SubmitTxBuffer failed\n");
        return eResult;
    }
    
    /* read register value */
    if(ADI_I2C_SUCCESS != (eResult = adi_i2c_SubmitRxBuffer(masterDev, value, 1u, false)))
    {
        DEBUG_MESSAGE("adi_i2c_SubmitRxBuffer failed\n");
        return eResult;
    }
    
    if (ADI_I2C_SUCCESS != (eResult = adi_i2c_Enable(masterDev, true)))
    {
        DEBUG_MESSAGE("adi_i2c_Enable failed\n");
        return eResult;
    }
    if (ADI_I2C_SUCCESS != (eResult = adi_i2c_GetTxBuffer(masterDev, &pBuffer)))
    {
        DEBUG_MESSAGE("adi_i2c_GetTxBuffer failed\n");
        return eResult;
    }
    if (ADI_I2C_SUCCESS != (eResult = adi_i2c_GetRxBuffer(masterDev, &pBuffer)))
    {
        DEBUG_MESSAGE("adi_i2c_GetRxBuffer failed\n");
        return eResult;
    }
    if (ADI_I2C_SUCCESS != (eResult = adi_i2c_Enable(masterDev, false)))
    {
        DEBUG_MESSAGE("adi_i2c_Enable failed\n");
        return eResult;
    }
    
    return eResult;
}

/*
 * main
 */
 int main(void)
{
    ADI_I2C_RESULT eResult=ADI_I2C_SUCCESS;
    uint8_t DevID;
    uint8_t t_msb, t_lsb;
    int16_t Temp;
    float ctemp, ftemp;
    uint8_t deviceMemory[ADI_I2C_MEMORY_SIZE];
    
    /* Clock initialization */
    SystemInit();
    
    //TODO:What does this do? GPIO = Bluetooth?
    adi_gpio_OutputEnable(ADI_GPIO_PORT0, (ADI_GPIO_PIN_4 | ADI_GPIO_PIN_5), true);

    //TODO: Possible Delay?
    
    adi_initpinmux();
    
    /* test system initialization */
    test_Init();
          
    if(adi_pwr_Init()!= ADI_PWR_SUCCESS)
    {
        DEBUG_MESSAGE("\n Failed to intialize the power service \n");
        eResult = ADI_I2C_FAILURE; 
    }
    
    if(ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1))
    {
        DEBUG_MESSAGE("Failed to set clock divider for HCLK\n");
    }
     
    if(ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1))
    {
        DEBUG_MESSAGE("Failed to set clock divider for PCLK\n");
    }  
    
    if(adi_gpio_Init(GPIOCallbackMem, ADI_GPIO_MEMORY_SIZE)!= ADI_GPIO_SUCCESS)
    {
    }
    
    adi_gpio_OutputEnable(BLE_RST_PORT, BLE_RST_PIN, true); //port 2 = BLE RST PORT, pin 4 = BLE RST PIN 
    //DEFAULT = PORT 2, PIN 6
    
    /////////////////////////////////////////////////////////////////////////
    ///////////////////////////////BOOT BLE MODULE///////////////////////////
    /////////////////////////////////////////////////////////////////////////
    //TODO:Boot of BLE is failing
    BootResult = adi_Dialog14580_SPI_Boot(sps_device_dialog_bin,IMAGE_SIZE);
    DEBUG_MESSAGE("Dialog14580 failed to boot\n", BootResult, 0);
    //BootResult = adi_Dialog14580_SPI_Boot(ble_code_ptr,BLE_CODE_SIZE);
    
    adi_gpio_OutputEnable(ADI_GPIO_PORT2,ADI_GPIO_PIN_4,true);
    
    /* I2C INIT */
    
    eResult = adi_i2c_Open(TWIDEVNUM,
                          ADI_I2C_MASTER,
                          &deviceMemory[0],
                          ADI_I2C_MEMORY_SIZE,
                          &masterDev);
    DEBUG_RESULT("adi_i2c_Open failed\n",eResult,ADI_I2C_SUCCESS);
    
    eResult = adi_i2c_SetBitRate(masterDev, BITRATE);
    DEBUG_RESULT("adi_i2c_SetBitRate failed\n",eResult,ADI_I2C_SUCCESS);

    eResult = adi_i2c_SetDutyCycle(masterDev, DUTYCYCLE);
    DEBUG_RESULT("adi_i2c_SetDutyCycle failed\n",eResult,ADI_I2C_SUCCESS);
    
    
    /* Set hardware address width */
    eResult = adi_i2c_SetHWAddressWidth(masterDev, ADI_I2C_HWADDR_WIDTH_7_BITS);
    DEBUG_RESULT("adi_i2c_SetHWAddressWidth failed\n",eResult,ADI_I2C_SUCCESS);

    /* set ADT7420 slave address */
    eResult = adi_i2c_SetHardwareAddress(masterDev, TARGETADDR);
    DEBUG_RESULT("adi_i2c_SetHardwareAddress failed\n",eResult,ADI_I2C_SUCCESS);
    

    uint8_t Data[2];
    uint16_t val;
    
    BLE_Payload[0]  = 0x20;
    BLE_Payload[1]  = 0x41;
    
    
    while(1)
    {

      DevID = 0u;
      eResult = ReadRegister(ID_REG, &DevID);
      DEBUG_RESULT("Failed to read ID register",eResult,ADI_I2C_SUCCESS);
      
      DEBUG_MESSAGE("ADT7420 Manufacture ID: 0x%x\n", DevID, TEST_VALUE);
      
    //eResult = adsAPI_RW_I2C_Sensor_Reg(REG_READ, SHT25_DEV_ADDR, SHT25_TRIG_T_MEAS_HOLDMASTER, &Data[0], 2, 0); //stuck here
    //DEBUG_RESULT("Reading temperature MSB register failed",eResult,ADI_I2C_SUCCESS);
    
      /* Read the temperature MSB register */
        eResult = ReadRegister(TEMPREG_MSB, &t_msb);
        DEBUG_RESULT("Reading temperature MSB register failed",eResult,ADI_I2C_SUCCESS);
        
        /* Read the temperature LSB register */
        eResult = ReadRegister(TEMPREG_LSB, &t_lsb);
        DEBUG_RESULT("Reading temperature LSB register failed",eResult,ADI_I2C_SUCCESS);
        
        /* Get the temperature by discarding the 3 bit flag at the LSB */
        Temp = ((int16_t)((uint16_t)t_msb << 8u) | (uint16_t)t_lsb) >> 3u;
        
        
        /* convert raw to deg C */
        ctemp = (Temp * 1.0)/16.0;

        /* convert raw to deg F */
        ftemp = ctemp * (9.0/5.0) + 32.0;
        
        DEBUG_MESSAGE("Temperature: %5.1f deg C\n",ctemp);
        DEBUG_MESSAGE("Temperature: %5.1f deg F\n",ftemp);
        
//    val = (Data[0]<<8) | Data[1];
//    val = val >> 2;
//    SHT25[0] = (float) (T_MEAS_CONST + (T_MEAS_MULT_CONST14 * val));   //Temparature = (-46.85+176.72*St/2^RES)
    
    /* SHT21 Temperature to BLE Payload */
    BLE_Payload[4]  = Data[0];
    BLE_Payload[5]  = Data[1];
    
    adsAPI_Delay(ADI_Sensor_Delay);
    
   
    ////eResult = ReadRegister(0xE5, &Data[0]);
    //eResult = adsAPI_RW_I2C_Sensor_Reg(REG_READ, SHT25_DEV_ADDR, SHT25_TRIG_RH_MEAS_HOLDMASTER, &Data[0], 2, 0);
    //DEBUG_RESULT("Reading temperature LSB register failed",eResult,ADI_I2C_SUCCESS);
    //
    //val = (Data[0]<<8) | Data[1];
    //val = val >> 4;
    //SHT25[1] = (float) (RH_MEAS_CONST + (RH_MEAS_MULT_CONST12 * val));   //Relative Humidity = (-6 + 125*RHval/2^RES)
    //
    ///* SHT21 Humidity to BLE Payload */
    //BLE_Payload[2]  = Data[0];
    //BLE_Payload[3]  = Data[1];
    //  
    //uint8_t Data[2]={0};
    //uint8_t exp=0, mant=0;
    //
    //eResult = adsAPI_RW_I2C_Sensor_Reg(REG_READ, MAX44009_DEV_ADDR, MAX44009_LUX_HIGH, &Data[0], 2, 0);
    //exp = (Data[0] & 0xF0)>>4;
    //mant = (Data[0]<<4) | (Data[1] & 0x0F);
    //
    ///* MAX44009 Ambient Light to BLE Payload */
    //BLE_Payload[6]  = Data[0];
    //BLE_Payload[7]  = Data[1];
    //
    //uint8_t exp_val=2;
    //for(uint8_t i=exp; i>0; i--) exp_val*=2;
    //MAX44009 = (float) (exp_val * mant* MAX44009_LUX_LSB_VALUE);
    
    /* Nothing to be written to Ambient Light CH1 in BLE Payload */
    BLE_Payload[8]  = 0;
    BLE_Payload[9]  = 0;
    
    /* BLE Message counter */

    BLE_Payload[13] = Msg_Count & 0xFF;
    BLE_Payload[12] = (Msg_Count >> 8) & 0xFF;
    BLE_Payload[11] = (Msg_Count >> 16) & 0xFF;
    BLE_Payload[10] = (Msg_Count >> 24) & 0xFF;
    Msg_Count++;
    
    InitUART();
    //Print_SensData_on_UART();
    PRINT_C(BLE_Payload, 14);
    PRINT_C(BLE_UID, 20);
    Delay_ms(500);
    adi_uart_Close(hUartDevice);
    Delay_ms(1500);
    
    }
}
