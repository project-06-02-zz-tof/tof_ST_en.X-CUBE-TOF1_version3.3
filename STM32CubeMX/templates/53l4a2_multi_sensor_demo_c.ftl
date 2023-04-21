[#ftl]

static void MX_53L4A2_MultiSensorRanging_Init(void)
{
  uint8_t device;
  uint16_t i2c_addr;
  uint32_t id;

  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  printf("53L4A2 Multi Sensor Ranging demo application\n");

  reset_all_sensors();

  /* Turn off all the sensors */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    write_lowpower_pin(device, GPIO_PIN_RESET);
  }

  /* initializes each device and put it in low power mode */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    /* enable only one sensor */
    write_lowpower_pin(device, GPIO_PIN_SET);
    HAL_Delay(2);

    status = VL53L4A2_RANGING_SENSOR_Init(device);

    if (status != BSP_ERROR_NONE)
    {
      printf("VL53L4A2_RANGING_SENSOR_Init %d failed\n", device);
      ToF_Present[device] = 0; /* device not detected */
    }
    else
    {
      ToF_Present[device] = 1; /* device detected */
    }

    write_lowpower_pin(device, GPIO_PIN_RESET); /* turn off the device */
  }

  /* power on the devices one at a time, initialize them and change their address.
   * once the address is updated, the communication with the devices is checked
   * reading its ID.
   */
  for (device = 0; device < RANGING_SENSOR_INSTANCES_NBR; device++)
  {
    /* skip the sensor if init not successful */
    if (ToF_Present[device] == 0) continue;

    /* turn on the device */
    write_lowpower_pin(device, GPIO_PIN_SET);
    HAL_Delay(2);

    /* left: 0x54, center: 0x56, right: 0x58 */
    i2c_addr = (RANGING_SENSOR_VL53L4CX_ADDRESS + (device + 1) * 2);
    VL53L4A2_RANGING_SENSOR_SetAddress(device, i2c_addr);

    /* check the communication with the device reading the ID */
    VL53L4A2_RANGING_SENSOR_ReadID(device, &id);
    printf("ToF sensor %d - ID: %04lX\n", device, (unsigned long)id);
  }
}

static void MX_53L4A2_MultiSensorRanging_Process(void)
{
  uint8_t i;

  static RANGING_SENSOR_Result_t Result;
  static RANGING_SENSOR_ProfileConfig_t Profile;

  Profile.RangingProfile = RS_MULTI_TARGET_MEDIUM_RANGE;
  Profile.TimingBudget = TIMING_BUDGET; /* 8 ms < TimingBudget < 200 ms */
  Profile.Frequency = 0; /* Induces intermeasurement period, NOT USED for normal ranging */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 0; /* Enable: 1, Disable: 0 */

  for (i = 0; i < RANGING_SENSOR_INSTANCES_NBR; i++)
  {
    /* skip this device if not detected */
    if (ToF_Present[i] != 1) continue;

    VL53L4A2_RANGING_SENSOR_ConfigProfile(i, &Profile);
    status = VL53L4A2_RANGING_SENSOR_Start(i, RS_MODE_BLOCKING_CONTINUOUS);

    if (status != BSP_ERROR_NONE)
    {
      printf("VL53L4A2_RANGING_SENSOR_Start failed\n");
      while(1);
    }
  }

  while (1)
  {
    /* polling mode */
    for (i = 0; i < RANGING_SENSOR_INSTANCES_NBR; i++)
    {
      if (!ToF_Present[i]) continue;

      status = VL53L4A2_RANGING_SENSOR_GetDistance(i, &Result);

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

static void write_lowpower_pin(uint8_t device, GPIO_PinState pin_state)
{
    switch (device)
    {
      case VL53L4A2_DEV_CENTER:
        HAL_GPIO_WritePin(VL53L4A2_XSHUT_C_PORT, VL53L4A2_XSHUT_C_PIN, pin_state);
        break;

      case VL53L4A2_DEV_LEFT:
        HAL_GPIO_WritePin(VL53L4A2_XSHUT_L_PORT, VL53L4A2_XSHUT_L_PIN, pin_state);
        break;

      case VL53L4A2_DEV_RIGHT:
        HAL_GPIO_WritePin(VL53L4A2_XSHUT_R_PORT, VL53L4A2_XSHUT_R_PIN, pin_state);
        break;

      default:
        break;
    }
}

static void reset_all_sensors(void)
{
  HAL_GPIO_WritePin(VL53L4A2_XSHUT_C_PORT, VL53L4A2_XSHUT_C_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VL53L4A2_XSHUT_L_PORT, VL53L4A2_XSHUT_L_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VL53L4A2_XSHUT_R_PORT, VL53L4A2_XSHUT_R_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);

  HAL_GPIO_WritePin(VL53L4A2_XSHUT_C_PORT, VL53L4A2_XSHUT_C_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VL53L4A2_XSHUT_L_PORT, VL53L4A2_XSHUT_L_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VL53L4A2_XSHUT_R_PORT, VL53L4A2_XSHUT_R_PIN, GPIO_PIN_SET);
  HAL_Delay(2);

}

