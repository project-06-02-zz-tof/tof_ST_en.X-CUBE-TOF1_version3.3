[#ftl]

static void MX_53L4A1_ThresholdDetection_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* reset XSHUT (XSDN) pin */
  HAL_GPIO_WritePin(VL53L4A1_XSHUT_C_PORT, VL53L4A1_XSHUT_C_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(VL53L4A1_XSHUT_C_PORT, VL53L4A1_XSHUT_C_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

  printf("53L4A1 Threshold detection demo application\n");
  status = VL53L4A1_RANGING_SENSOR_Init(VL53L4A1_DEV_CENTER);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L4A1_RANGING_SENSOR_Init failed\n");
    while(1);
  }
}

static void MX_53L4A1_ThresholdDetection_Process(void)
{
  uint32_t Id;

  RANGING_SENSOR_ITConfig_t ITConfig;
  RANGING_SENSOR_Result_t Result;

  VL53L4A1_RANGING_SENSOR_ReadID(VL53L4A1_DEV_CENTER, &Id);
  VL53L4A1_RANGING_SENSOR_GetCapabilities(VL53L4A1_DEV_CENTER, &Cap);

  Profile.RangingProfile = VL53L4CD_PROFILE_AUTONOMOUS;
  Profile.TimingBudget = TIMING_BUDGET; /* 10 ms < TimingBudget < 200 ms */
  Profile.Frequency = RANGING_FREQUENCY; /* Ranging frequency Hz (shall be consistent with TimingBudget value) */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 0; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  VL53L4A1_RANGING_SENSOR_ConfigProfile(VL53L4A1_DEV_CENTER, &Profile);

  /* threshold parameters */
  ITConfig.Criteria = RS_IT_IN_WINDOW;
  ITConfig.LowThreshold = 200; /* mm */
  ITConfig.HighThreshold = 600; /* mm */

  VL53L4A1_RANGING_SENSOR_ConfigIT(VL53L4A1_DEV_CENTER, &ITConfig);

  status = VL53L4A1_RANGING_SENSOR_Start(VL53L4A1_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L4A1_RANGING_SENSOR_Start failed\n");
    while(1);
  }

  while (1)
  {
    /* interrupt mode */
    if (ToF_EventDetected != 0)
    {
      ToF_EventDetected = 0;
      
      status = VL53L4A1_RANGING_SENSOR_GetDistance(VL53L4A1_DEV_CENTER, &Result);
      
      if (status == BSP_ERROR_NONE)
      {
        print_result(&Result);
      }
    }
  }  
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
