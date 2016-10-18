
#ifndef __UART_HANDLER_H__
#define __UART_HANDLER_H__

#include <stdbool.h>
#include <drivers/uart/adi_uart.h>
/*@}*/

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

#define DEMOAPP_USER_INPUT_LENGTH        256
#define STATE_UART_LENGTH                1
#define STATE_CMD_PARAM                  2

#define DEFAULT_PAN_ID       0xABCD

#define ANodeVersion         "AD6LP02  svn_369"


#define DEFAULT_PHY                     0x20                                  // 100kbps and chan 0
#define PHY_NORM_CCA_LEVEL              -70                                   // Default -80dBm normally
#define PHY_NORM_MIN_TX_LEVEL           -85
#define PHY_NORM_REPEAT_NO              3
#define RESETSLOTFLAG                   0
#define INITAWAKESLOT                   1
#define CHAN_INACTIVE                   0xFF
#define NO_OF_PHY_CHANS                 10
#define DEFAULT_SLEEP_TIME              8                                     // Sleep slot = 0,125s => 1s default sleep period
#define UWCHAN_CYC_STARTPOS             0

#define PANID_DEFAULT                  0x1AD1
#define PANID_UNASSIGNED               0xFFFF

#define DEFAULT_PHY_PROFILE         2
#define DAEFULT_CHAN                868000000

#define INNER_ORBIT                 0                                         // top layer end of the network e.g. BS
#define OUTER_ORBIT                 15                                        // bottom of the network e.g. last endpoints

#define STANDARD_REG_WAIT_TIME          1500


typedef struct {
   uint8_t      dtsn;
   uint8_t      sensor_node;
}UART_CONFIG_PARAMETERS;

typedef enum{
 HOP = 0x01,
 DTSN,
 SENSOR_NODE,
 PAN_ID,
 PREFIX,
 UART_READ_COMPLETE_STATE
}API_COMMANDS;


typedef enum{
UART_NOTSET = 0x01,
UART_CONFIG,
UART_TUNSLIP
}UARTMODE;

extern uint16_t accept_dio_rank;
extern uint8_t sensor_end_node;
extern ADI_UART_HANDLE hUartDevice;


#define EUI64LEN                    			8                                         // Up to IPv6 supported
#define NO_OF_PHY_CHANS                 		10
#define FLASH_MEM_BLOCK_LENGTH                  128

#define DESCRIPT_1_STR_LEN                      30
#define MEM_LOC_CONFIG_PARAMS_IN_FLASH          0x800
#define SIZE_OF_CONFIG_PARAMES                  0x180  //size Includes Confing Params,Run Params,URL

#define MEM_LOC_AD6L_RUNPARAMS					MEM_LOC_CONFIG_PARAMS_IN_FLASH
#define MEM_LOC_AD6L_PARAMS						MEM_LOC_CONFIG_PARAMS_IN_FLASH + 0x100
#define MEM_LOC_NODE_DESCRIPT1	                0x880

#define ADINET_DEVICE_ID                		0x01062007 //It is not Flash Memory Location and this code is expecting by WSN Tool

#define SRD_BUFFER_SIZE             			128

#define NO_OF_UPCHANS               			9

#define MEM_LOC_URL			        			MEM_LOC_CONFIG_PARAMS_IN_FLASH + 0x80
#define URL_STR_LEN                             80

/* Memory required by the driver for DMA mode of operation */
#define ADI_UART_MEMORY_SIZE    				(ADI_UART_BIDIR_MEMORY_SIZE)

#define  PRINT_REPORT

#define SIZE_OF_TX_BUFFER  26

#define SIZE_OF_RX_BUFFER  26

#define UART_DEVICE_NUM 0


typedef enum { CONFIG_OKAY = 0, CONFIG_NOT_OKAY } CONFIGSTATE;


typedef enum { AS_NOTREG = 0, AS_NOTATTACHED, AS_ATTACHED }ATTACHEDSTATES;

typedef enum { CS_SENDRDY = 0x01, CS_MESSAGE, CS_CHECKREPLY, CS_TIMEOUT, CS_SENDING }COMMSTAGES;
typedef enum { EXSTART = 0x01, EXTALKING, EXPREDECT, EXLISTEN, EXMONITOR, EXPAUSE, EXFINISH }EXCHANGESTATES;
typedef enum { MODE_A = 0x00, MODE_B, MODE_C, MODE_D, MODE_E, MODE_F }MODE; // Don't change the order of this enum, 08.01.2014 Mode E added for sniffer
typedef enum { COS_RSTSTATE = 0x00, COS_USDATA = 0x01, COS_UPMODE = 0x02, COS_HB = 0x03}COORDSTATES;

typedef enum
    {
    DS_UART = 0x01,
    DS_OTA,
    DS_PROG
    }DATASOURCE;



#define UART_BUFFER_SIZE 	50 //150	//To accommodate more payload for CLUMP
#define UART_HEADER_LEN		4

#endif //__UART_HANDLER_H__