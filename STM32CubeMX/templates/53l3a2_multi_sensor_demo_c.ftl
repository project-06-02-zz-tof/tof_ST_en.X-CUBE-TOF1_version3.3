[#ftl]

static void MX_53L3A2_MultiSensorRanging_Init(void)
{
  uint8_t device;
  uint16_t i2c_addr;
  uint32_t id;
  
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  printf("53L3A2 Multi Sensor Ranging demo application\n");

  /* put all the devices in shutdown mode */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    VL53L3A2_RANGING_SENSOR_SetPowerMode(device, RANGING_SENSOR_POWERMODE_OFF);
  }

  /* power on the devices one at a time, initialize them and change their address. 
   * once the address is updated, the communication with the devices is checked
   * reading its ID.
   */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    VL53L3A2_RANGING_SENSOR_SetPowerMode(device, RANGING_SENSOR_POWERMODE_ON);
    status = VL53L3A2_RANGING_SENSOR_Init(device);
    
    /* 0: not detected, 1: detected */
    ToF_Present[device] = (status != BSP_ERROR_NONE) ? 0 : 1;
    
    /* skip this device if init not successful */
    if (ToF_Present[device] == 0) continue;

    /* left: 0x54, center: 0x56, right: 0x58 */
    i2c_addr = (RANGING_SENSOR_VL53L3CX_ADDRESS + (device + 1) * 2);
    VL53L3A2_RANGING_SENSOR_SetAddress(device, i2c_addr);
    
    /* check the communication with the device reading the ID */
    VL53L3A2_RANGING_SENSOR_ReadID(device, &id);
    printf("ToF sensor %d - ID: %04lX\n", device, (unsigned long)id);
  }
}

static void MX_53L3A2_MultiSensorRanging_Process(void)
{
  uint8_t i;

  static RANGING_SENSOR_Result_t Result;
  static RANGING_SENSOR_ProfileConfig_t Profile;

  Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
  Profile.TimingBudget = TIMING_BUDGET; /* 16 ms < TimingBudget < 500 ms */
  Profile.Frequency = 0; /* Induces intermeasurement period, set to ZERO for normal ranging */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 0; /* Enable: 1, Disable: 0 */

  for (i = 0; i < RANGING_SENSOR_INSTANCES_NBR; i++)
  {
    /* skip this device if not detected */
    if (ToF_Present[i] != 1) continue;

    VL53L3A2_RANGING_SENSOR_ConfigProfile(i, &Profile);
    status = VL53L3A2_RANGING_SENSOR_Start(i, RS_MODE_BLOCKING_CONTINUOUS);

    if (status != BSP_ERROR_NONE)
    {
      printf("VL53L3A2_RANGING_SENSOR_Start failed\n");
      while(1);
    }
  }

  while (1)
  {
    /* polling mode */
    for (i = 0; i < RANGING_SENSOR_INSTANCES_NBR; i++)
    {
      status = VL53L3A2_RANGING_SENSOR_GetDistance(i, &Result);

      if (status == BSP_ERROR_NONE)
      {
        printf("%s\t - ", TofDevStr[i]);
        print_result(&Result);
        HAL_Delay(POLLING_PERIOD);
      }
    }
    printf ("\n");
  }
}

static void print_result(RANGING_SENSOR_Result_t *Result)
{
  uint8_t i;

  for (i = 0; i < RANGING_SENSOR_MAX_NB_ZONES; i++)
  {
    printf("Status = %2ld, Distance = %5ld mm",
      (long)Result->ZoneResult[i].Status[0],
      (long)Result->ZoneResult[i].Distance[0]);
  }
  printf ("\n");
}
