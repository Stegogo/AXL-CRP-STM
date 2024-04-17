/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* INCLUDES                                                                 */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

#include "math.h"
#include "cmsis_os.h"
#include "string.h"
#include "gpio.h"

#include "Accelerometer.h"
      
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* DEFINES                                                                  */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
      
#define ACCEL_THRESHOLD_DEFAULT     19  // g
#define ARRAY_LEN                   50
#define NUM_OF_POINTS_DEFAULT       16  // 10:  6.25 ms, 16: 10ms

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* ENUMS                                                                    */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
      
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STRUCTURES                                                               */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/      

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FUNCTION PROTOTYPES                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//static bool CheckAcceleration (void);

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GLOBAL VARIABLES                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

extern osSemaphoreId AccelDataReadyHandle;
extern UART_HandleTypeDef huart1;
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* LOCAL VARIABLES                                                          */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

//static uint16_t	accelThreshold       	= ACCEL_THRESHOLD_DEFAULT * ACCEL_THRESHOLD_DEFAULT * NUM_OF_POINTS_DEFAULT;
//static uint16_t	accelNumOfPoints     	= NUM_OF_POINTS_DEFAULT;
//static float	accelArray[ARRAY_LEN]  	= {0};
//static uint16_t accelArrayPointer		= 0;
uint8_t double_tap_count = 0;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* INTERFACE                                                                */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void StartReadAccel (void const * argument)
{
	vTaskSetApplicationTaskTag(NULL, (void *) 1);
// ::::::::::::::::::::::::::::: INIT :::::::::::::::::::::::::::::
	ADXL_InitStruct.SPIMode      = SPIMODE_4WIRE;
	ADXL_InitStruct.IntMode      = INT_ACTIVEHIGH;
	ADXL_InitStruct.LPMode       = LPMODE_NORMAL;
	ADXL_InitStruct.Rate         = BWRATE_100;
    ADXL_InitStruct.Range        = RANGE_2G;
    ADXL_InitStruct.Resolution   = RESOLUTION_FULL;
    ADXL_InitStruct.Justify      = JUSTIFY_SIGNED; 
    ADXL_InitStruct.AutoSleep    = AUTOSLEEPOFF;
    ADXL_InitStruct.LinkMode     = LINKMODEOFF;   
    ADXL_InitStruct.Interrupt    = (INT_DATA_READY | INT_DOUBLE_TAP);
    ADXL_InitStruct.IntPin       = INT1;

    while (true)
        {
            adxlStatus status = ADXL_Init (&ADXL_InitStruct);

            if (status == ADXL_OK)
            {


                ADXL_IntProto(&ADXL_InitStruct);
                // ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold, uint8_t Latent, uint8_t Window
				ADXL_enableDoubleTap(INT1, (uint8_t)((1 << D0)), 40, 20, 80, 200);
				ADXL_Measure(ON);
                ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);

                break;
            }
            osDelay(200);
        }
    // ::::::::::::::::::::::::::::: SUPERLOOP :::::::::::::::::::::::::::::
        for (;;)
        {
        	osSemaphoreWait (AccelDataReadyHandle, osWaitForever);

        	int16_t ADXL_out[3]= {0};
        	ADXL_out[0] = (int16_t)(ADXL_acc[0] * 1.0e4);
        	ADXL_out[1] = (int16_t)(ADXL_acc[1] * 1.0e4);
        	ADXL_out[2] = (int16_t)(ADXL_acc[2] * 1.0e4);

        	uint8_t packet[9] = {0};
        	packet[0] = (uint8_t)0xAA;
        	packet[1] = (uint8_t)0x86;
        	packet[2] = (uint8_t)(ADXL_out[0] & 0xFF);
        	packet[3] = (uint8_t)(ADXL_out[0] >> 0x08);
        	packet[4] = (uint8_t)(ADXL_out[1] & 0xFF);
			packet[5] = (uint8_t)(ADXL_out[1] >> 0x08);
			packet[6] = (uint8_t)(ADXL_out[2] & 0xFF);
			packet[7] = (uint8_t)(ADXL_out[2] >> 0x08);
        	packet[8] = (uint8_t)double_tap_count;
        	HAL_UART_Transmit(&huart1, (uint8_t *)packet, sizeof(packet), 10);
        }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (ADXL_IntProto(&ADXL_InitStruct) == true) {
		if (ADXL_getIntSource() & (INT_DOUBLE_TAP)) {	// Double tap interrupt
			double_tap_count++;
		}
		if (ADXL_getIntSource() & (INT_DATA_READY)) {	// Data ready interrupt
			ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);
			osSemaphoreRelease(AccelDataReadyHandle);
		}

    }
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STATIC MEMBERS                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

//static bool CheckAcceleration (void)
//{
//
//}


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
