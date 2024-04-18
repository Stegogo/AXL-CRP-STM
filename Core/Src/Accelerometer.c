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
      
#define TAP_DURATION	20
#define TAP_THRESHOLD	5
#define TAP_LATENT		10
#define TAP_WINDOW		200

#define WINDOW_ARR_LEN 	10

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

uint16_t double_tap_count = 0;
uint16_t tap_windows = 0;
uint16_t successful_tap_windows = 0;
bool cpr_successful = false;

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

            	ADXL_Measure(ON);
                ADXL_IntProto(&ADXL_InitStruct);
				ADXL_enableDoubleTap(INT1, (uint8_t)((1 << D2) | (1 << D1) | (1 << D0)), TAP_DURATION, TAP_THRESHOLD, TAP_LATENT, TAP_WINDOW);
                //ADXL_enableFreeFall(INT2, 20, 5);
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

        	uint8_t packet[10] = {0};
        	packet[0] = (uint8_t)0xAA;
        	packet[1] = (uint8_t)0x86;
        	packet[2] = (uint8_t)(ADXL_out[0] & 0xFF);
        	packet[3] = (uint8_t)(ADXL_out[0] >> 0x08);
        	packet[4] = (uint8_t)(ADXL_out[1] & 0xFF);
			packet[5] = (uint8_t)(ADXL_out[1] >> 0x08);
			packet[6] = (uint8_t)(ADXL_out[2] & 0xFF);
			packet[7] = (uint8_t)(ADXL_out[2] >> 0x08);
			packet[8] = (uint8_t)cpr_successful;
        	packet[9] = (uint8_t)(packet[2] ^ packet[3] ^ packet[4] ^ packet[5] ^ packet[6] ^ packet[7]);
        	HAL_UART_Transmit(&huart1, (uint8_t *)packet, sizeof(packet), 10);
        }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void StartCheckTaps(void const * argument)
{
	vTaskSetApplicationTaskTag(NULL, (void *) 3);
	for(;;)
	{
		if (double_tap_count > 1) {
			tap_windows++;
			double_tap_count = 0;
		} else {
			tap_windows = 0;
			double_tap_count = 0;
			cpr_successful = false;
		}

		if (tap_windows == 10) {
			cpr_successful = true;
		}

		osDelay(1000);
	}
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (ADXL_IntProto(&ADXL_InitStruct) == true) {
		if (GPIO_Pin == 1) {	// Data ready interrupt
			ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);
			osSemaphoreRelease(AccelDataReadyHandle);
		}
		if (GPIO_Pin == 2) {	// Double tap interrupt
			double_tap_count++;
		}
    }
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STATIC MEMBERS                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
