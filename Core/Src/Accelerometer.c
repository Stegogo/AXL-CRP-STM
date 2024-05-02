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
#define CPR_ACCELERATION_THRESHOLD 2	// 2 m/s/s = 0.2g
#define CPR_SINGLE_TAP_THRESHOLD 20		// how many single taps are allowed
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
static void calcSamples (void);
static void checkCorrectTap (void);
static void resetCPR (void);
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* GLOBAL VARIABLES                                                         */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

extern osSemaphoreId AccelDataReadyHandle;
extern osSemaphoreId ReadyToPackBinHandle;
extern UART_HandleTypeDef huart1;
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* LOCAL VARIABLES                                                          */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

static uint32_t accelStartTime = 0;
static uint32_t accelEndTime = 0;

static float 	displacement = 0.0f;
static float 	velocity = 0.0f;
//static
static float	accelArray[ACCEL_ARRAY_LEN] = {0};
static uint16_t accelArrayPointer = 0;
static uint16_t	accelNumOfPoints = 1;

static uint16_t displacement_out = 0;
static int16_t velocity_out = 0;

uint16_t tap_count = 0;
uint16_t tap_windows = 0;
uint16_t double_tap_windows = 0;
uint16_t single_tap_windows = 0;
uint16_t zero_tap_windows = 0;
bool chest_recoil = false;
bool cpr_successful = false;

uint8_t cycles = 0;

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
	ADXL_InitStruct.Rate         = BWRATE_200;
    ADXL_InitStruct.Range        = RANGE_2G;
    ADXL_InitStruct.Resolution   = RESOLUTION_FULL;
    ADXL_InitStruct.Justify      = JUSTIFY_SIGNED; 
    ADXL_InitStruct.AutoSleep    = AUTOSLEEPOFF;
    ADXL_InitStruct.LinkMode     = LINKMODEOFF;   
    ADXL_InitStruct.Interrupt    = INT_DATA_READY;
    ADXL_InitStruct.IntPin       = INT1;

    while (true) {
		adxlStatus status = ADXL_Init (&ADXL_InitStruct);
		if (status == ADXL_OK) {
			ADXL_Measure(ON);
			ADXL_IntProto(&ADXL_InitStruct);
			//ADXL_enableDoubleTap(INT1, (uint8_t)((1 << D0)), TAP_DURATION, TAP_THRESHOLD, TAP_LATENT, TAP_WINDOW);
			ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);

			break;
		}
		osDelay(200);
	}
	// ::::::::::::::::::::::::::::: SUPERLOOP :::::::::::::::::::::::::::::
	for (;;)
	{
		// Wait for accelerometer to collect data and for driver to read it
		osSemaphoreWait (AccelDataReadyHandle, osWaitForever);
		cycles++;
		ADXL_accSquare = (ADXL_acc[0]*ADXL_acc[0]) + (ADXL_acc[1]*ADXL_acc[1]) + (ADXL_acc[2]*ADXL_acc[2]);

		// Detect acceleration start and end; Record timestamps
		if (ADXL_accSquare >= ACCEL_THRESHOLD) {
			if (accelStartTime == 0.0f) {
				accelStartTime = HAL_GetTick();
			}
			accelArray[accelArrayPointer] = ADXL_accSquare;
			if (++accelArrayPointer >= accelNumOfPoints) accelArrayPointer = 0;
			accelNumOfPoints++;
		}
		if ((ADXL_accSquare < ACCEL_THRESHOLD) && accelStartTime != 0) {
			accelEndTime = HAL_GetTick();
			//calcSamples();
			calcDisplacement();
			calcVelocity();
			checkCorrectTap();
			accelStartTime = 0;
			memset ((uint8_t *)accelArray, 0, ACCEL_ARRAY_LEN * sizeof (float));
			accelNumOfPoints = 0;
			accelArrayPointer = 0;
			//displacement = 0;
			velocity = 0;
		}

		// If we past this point we can compose a valid packet to send, so unlock task
		// guarantee to trigger packet send task half the frequency of this task
		if (cycles == 2) {
			cycles = 0;
			osSemaphoreRelease(ReadyToPackBinHandle);
		}
	}
}

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
		// Keep track of one-second windows total count and
		// how many taps happened within them
		switch (tap_count)
		{
		case 0:
			zero_tap_windows++;
			break;
		case 1:
			single_tap_windows++;
			break;
		case 2:
			double_tap_windows++;
			break;
		default:	// 3 taps per second are clearly a wrong way of doing CPR
			resetCPR();
		}

		// CPR Assessment reset cases:
		// If we record no taps at all for two seconds straight
		// Or if there are more than 20 single taps (so that we do not allow less than 100 taps a minute)
		if ((zero_tap_windows >= 2) || (single_tap_windows >= CPR_SINGLE_TAP_THRESHOLD))
			resetCPR();

		// If CPR lasted for 60 seconds and we received a considerate amount of taps,
		// set success flag with the amount of taps
		if ((double_tap_windows + single_tap_windows + zero_tap_windows) == CPR_DURATION_SECONDS) {
			//uint16_t total_taps = single_tap_windows + (double_tap_windows * 2);
			cpr_successful = true;
		}

		// reset tap count for the next one-second window
		tap_count = 0;
		osDelay(1000);
	}
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/*
 * This task is called once a second to check if the correct taps were
 * made within this second.
 */
void StartSendAccel(void const * argument)
{
	vTaskSetApplicationTaskTag(NULL, (void *) 2);
	for(;;)
	{
		osSemaphoreWait (ReadyToPackBinHandle, osWaitForever);
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
		HAL_UART_Transmit(&huart1, (uint8_t *)packet, sizeof(packet), 1);
		//osDelay(1000);
	}
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (ADXL_IntProto(&ADXL_InitStruct) == true) {
		ADXL_getAccel(ADXL_acc, OUTPUT_FLOAT);
		osSemaphoreRelease(AccelDataReadyHandle);
    }
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* STATIC MEMBERS                                                           */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/* Process accumulated acceleration samples
 * Check for chest recoil -- ensure that the peak acceleration sample is
 * followed by a series of samples of lesser acceleraion
 */
static void calcSamples (void)
{
//	// Calculate array average
//	uint16_t n = accelArrayPointer == 0? 0 : accelArrayPointer - 1;	// number of measurements
//	for (uint8_t i = 0; i <= n; i++) {
//		accelArrayAvg += accelArray[i];
//	}
//	accelArrayAvg = accelArrayAvg / n;

	uint16_t n = accelArrayPointer == 0? 0 : accelArrayPointer - 1;	// number of measurements
	// Chest recoil check
	float accelArrayMax = accelArray[0];
	uint16_t recoil_unlikely = 0;
	uint8_t accelArrayMaxIndex = 0;
	for (uint8_t i = 0; i <= n; i++) {	// Determine sample of maximum value
		if (fabs(accelArray[i]) > accelArrayMax) {
			accelArrayMax = fabs(accelArray[i]);
			accelArrayMaxIndex = i;
		}
	}
	for (uint8_t i = accelArrayMaxIndex + 1; i <= n; i++) {
		if (fabs(accelArray[i]) < fabs(accelArray[i - 1]))
			continue;
		else
			recoil_unlikely++;
	}
	// allow 3 unmatching samples just in case
	if (recoil_unlikely < 3)
		chest_recoil = true;
	else
		chest_recoil = false;
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

	float 	accelArrayAvg = 0;
	uint16_t n = accelArrayPointer == 0? 0 : accelArrayPointer - 1;	// number of measurements
	for (uint8_t i = 0; i <= n; i++) {
		accelArrayAvg += accelArray[i];
	}
	accelArrayAvg = accelArrayAvg / n;

	displacement = ((accelArrayAvg / 2)  * (end_s - start_s) * (end_s - start_s)) * 100; // to сm

	// register a correct tap

}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/* Collect all parameters and decide whether tap was correct if
 * 1 : Average array value is above threshold
 * 2 : Vertical displacement is more than 50 mm
 * 3 : Chest recoil was present
 */
static void checkCorrectTap (void)
{
	float 	accelArrayAvg = 0;
	uint16_t n = accelArrayPointer == 0? 0 : accelArrayPointer - 1;	// number of measurements
	for (uint8_t i = 0; i <= n; i++) {
		accelArrayAvg += accelArray[i];
	}
	accelArrayAvg = accelArrayAvg / n;
	chest_recoil = true;
	if (displacement > CPR_DISPLACEMENT_THRESHOLD && accelArrayAvg > CPR_ACCELERATION_THRESHOLD && chest_recoil == true)
		tap_count++;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/* Reset all parameters that count one-second windows and CPR taps
 * Called in case of bad CPR as to zero all parameters
 */
static void resetCPR (void)
{
	zero_tap_windows = 0;
	single_tap_windows = 0;
	double_tap_windows = 0;

	tap_windows = 0;
	tap_count = 0;
	cpr_successful = false;
	chest_recoil = false;
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/* END                                                                      */
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
