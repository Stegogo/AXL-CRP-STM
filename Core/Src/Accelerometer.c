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
#define CPR_DISPLACEMENT_THRESHOLD 0.5 	// 50 mm
#define CPR_ACCELERATION_THRESHOLD 5	// 2 m/s/s = 0.2g
#define CPR_DURATION_SECONDS 10			// for how long to monitor CPR quality

#define TAP_DURATION	40
#define TAP_THRESHOLD	40
#define TAP_LATENT		40
#define TAP_WINDOW		200

#define ACCEL_THRESHOLD			1.5  // m/s/s
#define ACCEL_ARRAY_LEN 		100	// 100HZ freq rate is 1000 samples per second

#define PACKET_LEN				14

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
static void calcVelocity (void);
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
static float 	velocity = 0.0f;
static float	accelArray[ACCEL_ARRAY_LEN] = {0};
static uint16_t accelArrayPointer = 0;
static uint16_t	accelNumOfPoints = 1;

static uint16_t displacement_out = 0;
static int16_t velocity_out = 0;

uint16_t double_tap_count = 0;
uint16_t tap_windows = 0;
uint16_t successful_tap_windows = 0;
bool cpr_successful = false;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* INTERFACE                                                                */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/*
 * Read accelerometer values and compose a packet to send.
 * Task is blocked; Triggers by semaphore it got from interrupt routine
 * When this task is called first, it configures ADXL and initializes measurements
 */
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
        		if (accelStartTime == 0.0f) {
        			accelStartTime = HAL_GetTick();
        		}
        		accelArray[accelArrayPointer] = ADXL_accSquare;//ADXL_accSquare;


            	if (++accelArrayPointer >= accelNumOfPoints)  accelArrayPointer = 0;
            	accelNumOfPoints++;
        	}
        	if ((ADXL_accSquare < ACCEL_THRESHOLD) && accelStartTime != 0) {
        		accelEndTime = HAL_GetTick();
        		calcDisplacement();
        		calcVelocity();
        		accelStartTime = 0;
        		memset ((uint8_t *)accelArray, 0, ACCEL_ARRAY_LEN * sizeof (float));
        		accelNumOfPoints = 0;
        		accelArrayPointer = 0;
        	}
        	displacement_out = (uint16_t)(displacement * 1.0e6);
        	velocity_out = (int16_t)(velocity * 1.0e6);
        	int16_t ADXL_out[3]= {0};
        	ADXL_out[0] = (int16_t)(ADXL_acc[0] * 1.0e4);
        	ADXL_out[1] = (int16_t)(ADXL_acc[1] * 1.0e4);
        	ADXL_out[2] = (int16_t)(ADXL_acc[2] * 1.0e4);

        	uint8_t packet[PACKET_LEN] = {0};
        	// header
        	packet[0] = (uint8_t)0xAA;
        	packet[1] = (uint8_t)0x86;
        	// accel x, m/s/s * 10000
        	packet[2] = (uint8_t)(ADXL_out[0] & 0xFF);
        	packet[3] = (uint8_t)(ADXL_out[0] >> 0x08);
        	// accel y, m/s/s * 10000
        	packet[4] = (uint8_t)(ADXL_out[1] & 0xFF);
			packet[5] = (uint8_t)(ADXL_out[1] >> 0x08);
			// accel z, m/s/s * 10000
			packet[6] = (uint8_t)(ADXL_out[2] & 0xFF);
			packet[7] = (uint8_t)(ADXL_out[2] >> 0x08);
			// displacement, m * 10000000
			packet[9] = (uint8_t)(displacement_out & 0xFF);
			packet[10] = (uint8_t)(displacement_out >> 0x08);
			// velocity, m/s * 10000000
			packet[11] = (uint8_t)(velocity_out & 0xFF);
			packet[12] = (uint8_t)(velocity_out >> 0x08);
			// checksum
			uint8_t checksum = 0;
			for (uint8_t i = 2; i < PACKET_LEN; i++) {
				checksum ^= packet[i];
			}
        	packet[13] = checksum;
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
			//double_tap_count++;
		}
    }
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STATIC MEMBERS                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/*
 * This task is called once a second to check if the correct taps were
 * made within this second.
 */
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

		if (tap_windows == CPR_DURATION_SECONDS) {
			cpr_successful = true;
		}

		osDelay(1000);
	}
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/* Calculate Velocity given acceleration and time
 * V = V0 + ∫a(t)dt (with integral limits being start and end of acceleration)
 * V0 is assumed to be 0, as if each compression begins with a stable position
 * on the patient's chest.
 * V is calculated in meters per second
 */
static void calcVelocity (void)
{


	velocity = 0.0f; // reset previous
	float start_s = (float)accelStartTime / 1000;					// start timestamp in s
	float end_s = (float)accelEndTime / 1000;						// end timestamp in s
	uint16_t n = accelArrayPointer == 0? 0 : accelArrayPointer - 1;	// number of measurements

   for(int i=0; i <= n; i++) {
	   velocity += (accelArray[i]) * (end_s - start_s) / 2;
   }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/* Calculate displacement
 * S = (a / 2) * t^2 (Again, V0 is assumed to be 0).
 * a here is average acceleration accumulated over time of it being present
 * S in calculated in meters
 */
static void calcDisplacement (void)
{
	displacement = 0.0f; // reset previous
	float start_s = (float)accelStartTime / 1000;					// start timestamp in s
	float end_s = (float)accelEndTime / 1000;						// end timestamp in s
	uint16_t n = accelArrayPointer == 0? 0 : accelArrayPointer - 1;	// number of measurements

	float accelArrayAvg = 0;
	for (uint8_t i = 0; i <= n; i++) {
		accelArrayAvg += accelArray[i];
	}

	accelArrayAvg = accelArrayAvg / n;
	displacement = ((accelArrayAvg / 2)  * (end_s - start_s) * (end_s - start_s)) * 100; // to сm

	if (displacement > CPR_DISPLACEMENT_THRESHOLD && accelArrayAvg > CPR_ACCELERATION_THRESHOLD)
		double_tap_count++;
}


/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
