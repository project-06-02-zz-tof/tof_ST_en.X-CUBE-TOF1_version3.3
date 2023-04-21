[#ftl]
#include "53l4a1_ranging_sensor.h"
#include "app_tof_pin_conf.h"
#include "${BoardName}.h"

/*
 * The purpose of this sample application is to showcase the threshold detection
 * functionality of this device. 
 *
 * When the device detects a target that match the configuration an IT is generated
 * and the host will start printing the measurement information on the serial connection (UART).
 *
 * Here is the default configuration:
 * 
 * ITConfig.Criteria = RS_IT_IN_WINDOW;
 * ITConfig.LowThreshold = 200; // distance in mm
 * ITConfig.HighThreshold = 600; // distance in mm
 *
 * Other availables interrupt generation criteria for this device are:
 *
 * - RS_IT_DEFAULT          // IT if new measurement is ready (no thresholds)
 * - RS_IT_IN_WINDOW        // IT if distance >= LowThreshold AND distance <= HighThreshold
 * - RS_IT_OUT_OF_WINDOW    // IT if distance < LowThreshold OR distance > HighThreshold
 * - RS_IT_BELOW_LOW        // IT if distance < LowThreshold
 * - RS_IT_ABOVE_HIGH       // IT if distance > HighThreshold
 */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 10 ms < TimingBudget < 200 ms */
#define RANGING_FREQUENCY (5U) /* Ranging frequency Hz (shall be consistent with TimingBudget value) */

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_Capabilities_t Cap;
static RANGING_SENSOR_ProfileConfig_t Profile;
static int32_t status = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L4A1_ThresholdDetection_Init(void);
static void MX_53L4A1_ThresholdDetection_Process(void);
static void print_result(RANGING_SENSOR_Result_t *Result);
static int32_t decimal_part(float_t x);