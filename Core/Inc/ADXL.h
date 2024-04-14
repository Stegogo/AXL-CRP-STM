#ifndef ADXL_H   
#define ADXL_H

/**
* @Library for ADXL345 3-axis accelometer 
* @Hardware dependencies: Could be changed very easily.
						STM32L152R uC
						SPI2 
						Some GPIOs
* @Author Iman Hosseinzadeh iman[dot]hosseinzadeh AT gmail
  https://github.com/ImanHz
						
*/
 /**
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* INCLUDES                                                                 */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#include "stm32f1xx_hal.h"
#include "main.h"
#include "spi.h"
#include "stdbool.h"

// SPI handler. Change it if needed.
// NOTE: SPI must be set in High Polarity and 2 Edges Phase mode.

// If you are using libraries other than HAL, i.e. STDPeriph, change writeRegister and readRegister functions.
#define SPIhandler hspi1
extern SPI_HandleTypeDef SPIhandler;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* DEFINES                                                                  */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

// GPIO definition
#define ADXLCS_Pin         SPI1_CS_Pin
#define ADXLCS_GPIO_Port   SPI1_CS_GPIO_Port

// Registers' Address 
#define DEVID 					0x0
#define BW_RATE					0x2C 
#define DATA_FORMAT 		        	0x31
#define FIFO_CTL 				0x38
#define DATA0					0x32
#define POWER_CTL 				0x2D
#define THRESH_TAP				0x1D
#define DUR					0x21
#define TAP_AXES				0x2A
#define INT_ENABLE				0x2E
#define INT_MAP					0x2F
#define LATENT					0x22
#define WINDOW					0x23
#define THRESH_ACT				0x24
#define THRESH_INACT			        0x25
#define TIME_INAT				0x26
#define ACT_INACT_CTL			        0x27
#define THRESH_FF 				0x28
#define TIME_FF					0x29
#define OFFX					0x1E
#define OFFY					0x1F
#define OFFZ					0x20
#define INT_SOURCE				0x30

// Init. Definitions
#define SPIMODE_3WIRE 1
#define SPIMODE_4WIRE 0

#define LPMODE_NORMAL   0
#define LPMODE_LOWPOWER 1

#define BWRATE_6_25 	6
#define BWRATE_12_5 	7
#define BWRATE_25 	8
#define BWRATE_50 	9
#define BWRATE_100	10
#define BWRATE_200	11
#define BWRATE_400	12
#define BWRATE_800	13
#define BWRATE_1600   	14
#define BWRATE_3200   	15

#define INT_ACTIVEHIGH  0
#define INT_ACTIVELOW   1

#define RESOLUTION_FULL  1
#define RESOLUTION_10BIT 0

#define JUSTIFY_MSB 	1
#define JUSTIFY_SIGNED  0

#define	SLEEP_RATE_1HZ 3
#define SLEEP_RATE_2HZ 2
#define SLEEP_RATE_4HZ 1
#define SLEEP_RATE_8HZ 0

#define RANGE_2G  0
#define RANGE_4G  1
#define RANGE_8G  2
#define RANGE_16G 3

#define AUTOSLEEPON  1
#define AUTOSLEEPOFF 0

#define LINKMODEON  1
#define LINKMODEOFF 0

#define INT_DATA_READY   (uint8_t)(1<<7)
#define INT_SINGLE_TAP   (uint8_t)(1<<6)
#define INT_DOUBLE_TAP   (uint8_t)(1<<5)
#define INT_ACTIVITY     (uint8_t)(1<<4)
#define INT_INACTIVITY   (uint8_t)(1<<3)
#define INT_FREE_FALL    (uint8_t)(1<<2)
#define INT_WATERMARK    (uint8_t)(1<<1)
#define INT_OVERRUN      (uint8_t)(1<<0)


// ADXL_getAccel function definitions 
#define OUTPUT_FLOAT  0
#define OUTPUT_SIGNED 1

#define ACTIVITY_AC 1
#define ACTIVITY_DC 0

//////////////////////////////////////////// I N T E R R U P T S //////////////////////
///// Important Note: The interrupt routine in your code should be implemented using ADXL_IntProto()
///// function!
#define X_axes 4
#define Y_axes 2
#define Z_axes 1

#define TYPE_FLOAT

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* ENUMS                                                                    */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

typedef enum 
{
    ADXL_OK,
    ADXL_ERR
} adxlStatus;

typedef enum 
{
    ON,
    OFF
} Switch;

typedef enum 
{
    INT1=0,
    INT2=1
} ADXL_IntOutput;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STRUCTURES                                                               */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

typedef struct 
{
    uint8_t        SPIMode;
    uint8_t        IntMode;
    uint8_t        LPMode;
    uint8_t        Rate;
    uint8_t        Range;
    uint8_t        Resolution;
    uint8_t        Justify;
    uint8_t        AutoSleep;
    uint8_t        LinkMode;
    uint8_t        Interrupt;
    ADXL_IntOutput IntPin;
} ADXL_InitTypeDef;

	
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FUNCTION PROTOTYPES                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

extern adxlStatus ADXL_Init             (ADXL_InitTypeDef * adxl);
extern void       ADXL_getAccel         (void *Data,uint8_t outputType);
extern void       ADXL_Measure          (Switch s);
extern void       ADXL_Sleep            (Switch s,uint8_t rate);
extern void       ADXL_Standby          (Switch s);
extern void       ADXL_test             (uint8_t * regs);
extern void       ADXL_enableSelfTest   (void);
extern void       ADXL_disableSelfTest  (void);				
extern void       ADXL_SetOffset        (int8_t off_x,int8_t off_y,int8_t off_z);
extern void       ADXL_enableSingleTap  (ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold);
extern void       ADXL_disableSingleTap (void);
extern void       ADXL_enableDoubleTap  (ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold, uint8_t Latent, uint8_t Window);
extern void       ADXL_disableDoubleTap (void);
extern void       ADXL_enableActivity   (ADXL_IntOutput out, uint8_t axes, uint8_t Threshold, uint8_t AcDc);
extern void       ADXL_disableActivity  (void);
extern void       ADXL_enableFreeFall   (ADXL_IntOutput out, uint8_t Threshold, uint8_t Time);
extern void       ADXL_disableFreeFall  (void);
extern bool       ADXL_IntProto         (ADXL_InitTypeDef * adxl);

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GLOBAL VARIABLES                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

extern ADXL_InitTypeDef  ADXL_InitStruct;
#ifdef TYPE_FLOAT
extern float ADXL_acc[3];
extern float ADXL_accSquare;
extern float ADXL_accSquareMax;
#else
extern int32_t  ADXL_acc[3];
extern uint32_t ADXL_accSquare;
extern uint32_t ADXL_accSquareMax;
#endif

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#endif  // ADXL_H
