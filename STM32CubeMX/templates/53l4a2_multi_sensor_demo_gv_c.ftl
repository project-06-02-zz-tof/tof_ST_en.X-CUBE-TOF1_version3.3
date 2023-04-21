[#ftl]
#include "53l4a2_ranging_sensor.h"
#include "app_tof_pin_conf.h"
#include "${BoardName}.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 8 ms < TimingBudget < 200 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (milliseconds, shall be consistent with TimingBudget value) */

/* Private variables ---------------------------------------------------------*/
static int32_t status = 0;
static uint8_t ToF_Present[RANGING_SENSOR_INSTANCES_NBR] = {0};
volatile uint8_t ToF_EventDetected = 0;

static const char *TofDevStr[] =
{
  [VL53L4A2_DEV_LEFT] = "LEFT",
  [VL53L4A2_DEV_CENTER] = "CENTER",
  [VL53L4A2_DEV_RIGHT] = "RIGHT"
};

/* Private function prototypes -----------------------------------------------*/
static void MX_53L4A2_MultiSensorRanging_Init(void);
static void MX_53L4A2_MultiSensorRanging_Process(void);
static void print_result(RANGING_SENSOR_Result_t *Result);
static void write_lowpower_pin(uint8_t device, GPIO_PinState pin_state);
static void reset_all_sensors(void);
