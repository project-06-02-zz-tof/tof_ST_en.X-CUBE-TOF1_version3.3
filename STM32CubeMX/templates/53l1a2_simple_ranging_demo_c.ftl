[#ftl]

[#list SWIPdatas as SWIP]
[#if SWIP.defines??]
  [#list SWIP.defines as definition]
    [#assign value = definition.value]
    [#assign name = definition.name]

    [#if name.contains("TOF_RANGING_MODE")]
      [#assign flag_53l1a2_ranging_mode = value]
    [/#if]
    [#if name.contains("TOF_OFFSET_CALIBRATION")]
      [#assign flag_53l1a2_offset_calibration = value]
    [/#if]
    [#if name.contains("TOF_XTALK_CALIBRATION")]
      [#assign flag_53l1a2_xtalk_calibration = value]
    [/#if]

  [/#list]
[/#if]
[/#list]

static void MX_53L1A2_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  printf("53L1A2 Simple Ranging demo application\n");
  status = VL53L1A2_RANGING_SENSOR_Init(VL53L1A2_DEV_CENTER);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Init failed\n");
    while(1);
  }
}

static void MX_53L1A2_SimpleRanging_Process(void)
{
[#if flag_53l1a2_offset_calibration == "Yes"]
  uint8_t i;
  uint32_t cal_distance_mm = 100; /* target distance used for offset calibration */
[/#if]
  uint32_t Id;
  
  VL53L1A2_RANGING_SENSOR_ReadID(VL53L1A2_DEV_CENTER, &Id);
  VL53L1A2_RANGING_SENSOR_GetCapabilities(VL53L1A2_DEV_CENTER, &Cap);

[#if flag_53l1a2_xtalk_calibration == "Yes"]
  printf("--- XTALK CALIBRATION ---\n");
  printf("Please remove all objects in front of the sensor\n");
  printf("Press the user button to continue...\n");

  /* wait until the button is pressed */
  while (PushButtonDetected != 1);
  PushButtonDetected = 0;

  printf("--- BEGIN XTALK CALIBRATION ---\n");
  VL53L1A2_RANGING_SENSOR_XTalkCalibration(VL53L1A2_DEV_CENTER);
  printf("--- END OF XTALK CALIBRATION ---\n");
[/#if]

  Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
  Profile.TimingBudget = TIMING_BUDGET; /* 16 ms < TimingBudget < 500 ms */
  Profile.Frequency = 0; /* Induces intermeasurement period, set to ZERO for normal ranging */
  Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  VL53L1A2_RANGING_SENSOR_ConfigProfile(VL53L1A2_DEV_CENTER, &Profile);

[#if flag_53l1a2_offset_calibration == "Yes"]
  printf("--- OFFSET CALIBRATION ---\n");
  printf("Please put a target at %lu mm\n", (unsigned long)cal_distance_mm);
  printf("Press the user button to continue...\n");
  
  /* wait until the button is pressed */
  while (PushButtonDetected != 1);
  PushButtonDetected = 0;

  printf("--- BEGIN OFFSET CALIBRATION ---\n");
  /* make sure that a target is placed at cal_distance_mm (100 mm is the default value in this example) 
   * the application will perform some measure in order to have a log of some pre-calibration values
   */
  status = VL53L1A2_RANGING_SENSOR_Start(VL53L1A2_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Start failed\n");
    while(1);
  }

  /* Pre-calibration ranging */
  for (i = 0; i < 10; i++)
  {
    status = VL53L1A2_RANGING_SENSOR_GetDistance(VL53L1A2_DEV_CENTER, &Result);

    if (status == BSP_ERROR_NONE)
    {
      print_result(&Result);
    }

    HAL_Delay(POLLING_PERIOD);
  }

  /* stop ranging and perform offset calibration */
  VL53L1A2_RANGING_SENSOR_Stop(VL53L1A2_DEV_CENTER);
  VL53L1A2_RANGING_SENSOR_OffsetCalibration(VL53L1A2_DEV_CENTER, cal_distance_mm);
  printf("--- END OF OFFSET CALIBRATION ---\n");
[/#if]
[#if flag_53l1a2_ranging_mode == "Interrupt"]

  status = VL53L1A2_RANGING_SENSOR_Start(VL53L1A2_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Start failed\n");
    while(1);
  }

  while (1)
  {
    /* interrupt mode */
    if (ToF_EventDetected != 0)
    {
      ToF_EventDetected = 0;
      
      status = VL53L1A2_RANGING_SENSOR_GetDistance(VL53L1A2_DEV_CENTER, &Result);
      
      if (status == BSP_ERROR_NONE)
      {
        print_result(&Result);
      }
    }
  }
[/#if]
[#if flag_53l1a2_ranging_mode == "Polling"]

  status = VL53L1A2_RANGING_SENSOR_Start(VL53L1A2_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Start failed\n");
    while(1);
  }

  while (1)
  {
    /* polling mode */
    status = VL53L1A2_RANGING_SENSOR_GetDistance(VL53L1A2_DEV_CENTER, &Result);

    if (status == BSP_ERROR_NONE)
    {
      print_result(&Result);
    }

    HAL_Delay(POLLING_PERIOD);
  }
[/#if]
}

static void print_result(RANGING_SENSOR_Result_t *Result)
{
  uint8_t i, j;

  for (i = 0; i < RANGING_SENSOR_MAX_NB_ZONES; i++)
  {
    printf("\nTargets = %lu", (unsigned long)Result->ZoneResult[i].NumberOfTargets);

    for (j = 0; j < Result->ZoneResult[i].NumberOfTargets; j++)
    {
      printf("\n |---> ");

      printf("Status = %ld, Distance = %5ld mm ",
        (long)Result->ZoneResult[i].Status[j],
        (long)Result->ZoneResult[i].Distance[j]);

      if (Profile.EnableAmbient)
        printf(", Ambient = %ld.%02ld kcps/spad", 
          (long)Result->ZoneResult[i].Ambient[j],
          (long)decimal_part(Result->ZoneResult[i].Ambient[j]));

      if (Profile.EnableSignal)
        printf(", Signal = %ld.%02ld kcps/spad",
          (long)Result->ZoneResult[i].Signal[j],
          (long)decimal_part(Result->ZoneResult[i].Signal[j]));
    }
  }
  printf ("\n");
}

static int32_t decimal_part(float_t x)
{
  int32_t int_part = (int32_t) x;
  return (int32_t)((x - int_part) * 100);
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  PushButtonDetected = 1;
}