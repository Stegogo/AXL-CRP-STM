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
      
#define TAP_DURATION	40
#define TAP_THRESHOLD	40
#define TAP_LATENT		80
#define TAP_WINDOW		200

#define ACCEL_THRESHOLD			2  // m/s/s
#define ACCEL_ARRAY_LEN 		100	// 100HZ freq rate is 1000 samples per second

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* ENUMS                                                                    */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
      
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STRUCTURES                                                               */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/      

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* FUNCTION PROTOTYPES                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
static void calcDisplacement (void);

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GLOBAL VARIABLES                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

extern osSemaphoreId AccelDataReadyHandle;
extern UART_HandleTypeDef huart1;
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* LOCAL VARIABLES                                                          */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

static uint32_t accelStartTime = 0;
static uint32_t accelEndTime = 0;

static float 	displacement = 0.0f;
static float	accelArray[ACCEL_ARRAY_LEN] = {0};
static uint16_t accelArrayPointer = 0;
static uint16_t	accelNumOfPoints = 1;

static uint16_t	double_tap_count = 0;

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
				ADXL_enableDoubleTap(INT1, (uint8_t)((1 << D0)), TAP_DURATION, TAP_THRESHOLD, TAP_LATENT, TAP_WINDOW);
                ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);

                break;
            }
            osDelay(200);
        }
    // ::::::::::::::::::::::::::::: SUPERLOOP :::::::::::::::::::::::::::::
        for (;;)
        {
        	osSemaphoreWait (AccelDataReadyHandle, osWaitForever);

        	ADXL_accSquare = (ADXL_acc[0]*ADXL_acc[0]) + (ADXL_acc[1]*ADXL_acc[1]) + (ADXL_acc[2]*ADXL_acc[2]);

        	// Detect acceleration start and end; Record timestamps
        	if (ADXL_accSquare >= ACCEL_THRESHOLD) {
        		accelStartTime = HAL_GetTick();
        		accelArray[accelArrayPointer] = ADXL_accSquare;

            	if (++accelArrayPointer >= accelNumOfPoints)  accelArrayPointer = 0;
            	accelNumOfPoints++;
        	}
        	if ((ADXL_accSquare < ACCEL_THRESHOLD) && accelStartTime != 0) {
        		accelEndTime = HAL_GetTick();
        		calcDisplacement();
        		accelStartTime = 0;
        		memset ((uint8_t *)accelArray, 0, ACCEL_ARRAY_LEN * sizeof (float));
        		accelNumOfPoints = 0;
        		accelArrayPointer = 0;
        	}

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
        	packet[8] = (uint8_t)(packet[2] ^ packet[3] ^ packet[4] ^ packet[5] ^ packet[6] ^ packet[7]);
        	HAL_UART_Transmit(&huart1, (uint8_t *)packet, sizeof(packet), 10);
        }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (ADXL_IntProto(&ADXL_InitStruct) == true) {
		if (GPIO_Pin == 1) {	// Data ready interrupt
			ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);
			osSemaphoreRelease(AccelDataReadyHandle);
			//ADXL_getIntSource();
		}
		if (GPIO_Pin == 2) {	// Double tap interrupt
			double_tap_count++;
		}
    }
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STATIC MEMBERS                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

static void calcDisplacement (void)
{
//	/* Calculate Velocity given acceleration and time
//	 * V = V0 + ∫a(t)dt (with integral limits being start and end of acceleraiton)
//	 * V0 is assumed to be 0, as if each compression begins with a stable position
//	 * on the patient's chest.
//	 * V is calculated in meters per second
//	 */
	float y = 0.0f;
	float velocity = 0.0f;
	float start_s = (float)accelStartTime / 1000;	// start timestampt in s
	float end_s = (float)accelEndTime / 1000;		// end timestampt in s
//
//	for (float i = start_s; i < end_s; i += (end_s - start_s) / accelNumOfPoints) {
//		y = accelArray[accelArrayPointer] * (end_s - start_s);
//		velocity += y * (end_s - start_s) / accelNumOfPoints;
//	}

	/* Next, calculate displacement
	 * S = (a * t^2) / 2 (Again, V0 is assumed to be 0).
	 * a here is average acceleration accumulated over time of it being present
	 * S in calculated in meters
	 */
	float accelArrayAvg;
	for (uint8_t i = 0; i < accelNumOfPoints; i++) {
		accelArrayAvg += accelArray[i];
	}
	accelArrayAvg = accelArrayAvg / accelNumOfPoints;
	displacement = (accelArrayAvg * (end_s - start_s) * (end_s - start_s)) / 2;
}


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
