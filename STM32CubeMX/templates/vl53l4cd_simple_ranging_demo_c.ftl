[#ftl]

[#list SWIPdatas as SWIP]
[#if SWIP.defines??]
  [#list SWIP.defines as definition]
    [#assign value = definition.value]
    [#assign name = definition.name]

    [#if name.contains("TOF_RANGING_MODE")]
      [#assign flag_VL53L4CD_ranging_mode = value]
    [/#if]

  [/#list]
[/#if]
[/#list]

static void MX_VL53L4CD_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  printf("VL53L4CD Simple Ranging demo application\n");
  status = CUSTOM_RANGING_SENSOR_Init(CUSTOM_VL53L4CD);

  if (status != BSP_ERROR_NONE)
  {
    printf("CUSTOM_RANGING_SENSOR_Init failed\n");
    while(1);
  }
}

static void MX_VL53L4CD_SimpleRanging_Process(void)
{
  uint32_t Id;

  CUSTOM_RANGING_SENSOR_ReadID(CUSTOM_VL53L4CD, &Id);
  CUSTOM_RANGING_SENSOR_GetCapabilities(CUSTOM_VL53L4CD, &Cap);

  Profile.RangingProfile = VL53L4CD_PROFILE_CONTINUOUS;
  Profile.TimingBudget = TIMING_BUDGET; /* 10 ms < TimingBudget < 200 ms */
  Profile.Frequency = 0; /* Induces intermeasurement period, NOT USED for normal ranging */
  Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  CUSTOM_RANGING_SENSOR_ConfigProfile(CUSTOM_VL53L4CD, &Profile);

[#if flag_VL53L4CD_ranging_mode == "Interrupt"]

  status = CUSTOM_RANGING_SENSOR_Start(CUSTOM_VL53L4CD, RS_MODE_ASYNC_CONTINUOUS);

  if (status != BSP_ERROR_NONE)
  {
    printf("CUSTOM_RANGING_SENSOR_Start failed\n");
    while(1);
  }

  while (1)
  {
    /* interrupt mode */
    if (ToF_EventDetected != 0)
    {
      ToF_EventDetected = 0;
      
      status = CUSTOM_RANGING_SENSOR_GetDistance(CUSTOM_VL53L4CD, &Result);
      
      if (status == BSP_ERROR_NONE)
      {
        print_result(&Result);
      }
    }
  }
[/#if]
[#if flag_VL53L4CD_ranging_mode == "Polling"]

  status = CUSTOM_RANGING_SENSOR_Start(CUSTOM_VL53L4CD, RS_MODE_BLOCKING_CONTINUOUS);

  while (1)
  {
    /* polling mode */
    status = CUSTOM_RANGING_SENSOR_GetDistance(CUSTOM_VL53L4CD, &Result);

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
