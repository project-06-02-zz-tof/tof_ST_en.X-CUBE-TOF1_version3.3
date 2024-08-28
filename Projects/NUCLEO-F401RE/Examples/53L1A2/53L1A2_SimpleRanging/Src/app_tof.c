/**
  ******************************************************************************
  * @file          : app_tof.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the STMicroelectronics.X-CUBE-TOF1.3.3.0 instances.
  ******************************************************************************
  *
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>

#include "53l1a2_ranging_sensor.h"
#include "stm32f4xx_nucleo.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 16 ms < TimingBudget < 500 ms */
#define POLLING_PERIOD (30) /* refresh rate for polling mode (milliseconds, shall be consistent with TimingBudget value) */

/* Private variables ---------------------------------------------------------*/
static RANGING_SENSOR_Capabilities_t Cap;
static RANGING_SENSOR_ProfileConfig_t Profile;
static RANGING_SENSOR_Result_t Result;
static int32_t status = 0;
static volatile uint8_t PushButtonDetected = 0;
volatile uint8_t ToF_EventDetected = 0;
static uint8_t calibration_data_buf_[1024] = {0};

/* Private function prototypes -----------------------------------------------*/
static void MX_53L1A2_SimpleRanging_Init(void);
static void MX_53L1A2_SimpleRanging_Process(void);
static void print_result(RANGING_SENSOR_Result_t *Result);
static void print_result_copy_multi(RANGING_SENSOR_Result_t *Result);
static int32_t decimal_part(float_t x);

void MX_TOF_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */

  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */

  MX_53L1A2_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_TOF_Process(void)
{
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */

  MX_53L1A2_SimpleRanging_Process();

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

static void MX_53L1A2_SimpleRanging_Init(void)
{
  /* Initialize Virtual COM Port */
  BSP_COM_Init(COM1);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  printf("53L1A2 Simple Ranging demo application\n");
  status = VL53L1A2_RANGING_SENSOR_Init(VL53L1A2_DEV_CENTER);

  // l1a2rangeingGetCalibrationData(calibration_data_buf_);
  // for (uint16_t i = 0; i < 0x198; i++) {
  //   printf("0x%x,", calibration_data_buf_[i]);
  // }
  // printf("\r\n");

  if (status != BSP_ERROR_NONE)
  {
    printf("VL53L1A2_RANGING_SENSOR_Init failed\n");
    while(1);
  }
}

static void MX_53L1A2_SimpleRanging_Process(void)
{
  uint32_t Id;

  VL53L1A2_RANGING_SENSOR_ReadID(VL53L1A2_DEV_CENTER, &Id);
  VL53L1A2_RANGING_SENSOR_GetCapabilities(VL53L1A2_DEV_CENTER, &Cap);

  Profile.RangingProfile = RS_MULTI_TARGET_LONG_RANGE;
  Profile.TimingBudget = TIMING_BUDGET; /* 16 ms < TimingBudget < 500 ms */
  Profile.Frequency = 0; /* Induces intermeasurement period, set to ZERO for normal ranging */
  Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  VL53L1A2_RANGING_SENSOR_ConfigProfile(VL53L1A2_DEV_CENTER, &Profile);

  status = VL53L1A2_RANGING_SENSOR_Start(VL53L1A2_DEV_CENTER, RS_MODE_ASYNC_CONTINUOUS);

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
      // print_result(&Result);
      print_result_copy_multi(&Result);
    }

    HAL_Delay(POLLING_PERIOD);
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

// C语言示例代码
// 1.首先定义宏函数简化后期工作（ARMCC/ARMGCC/GCC编译器）：
#define PRINT(window, fmt, args...) printf("{"#window"}"fmt"\n", ##args)
// 2.若要将电压和电流数据进行绘图，并将功率数据分窗显示，可这样调用：
// PRINT(plotter, "%d,%d", current, voltage);
// PRINT(power, "the power is %d W", current * voltage);
// 3.实际显示效果可能如下：
//     {plotter}3,2
//     {power}the power is 6 W

static void print_result_copy_multi(RANGING_SENSOR_Result_t *Result) {
  uint8_t i;
  int distance_invaild = -1;

  for (i = 0; i < RANGING_SENSOR_MAX_NB_ZONES; i++) {
    // printf("Status = %2ld, Distance = %5ld mm",
    //   (long)Result->ZoneResult[i].Status[0],
    //   (long)Result->ZoneResult[i].Distance[0]);

    // printf("{Status is %ld }\n",(long)Result->ZoneResult[i].Status[0]);
    // printf("{Distance is %5ld mm}\n",(long)Result->ZoneResult[i].Distance[0]);
    // printf("{Ambient is %ld.%02ld
    // kcps/spad}\n",(long)Result->ZoneResult[i].Ambient[0],(long)decimal_part(Result->ZoneResult[i].Ambient[0]));
    // printf("{Signal is %ld.%02ld
    // kcps/spad}\n",(long)Result->ZoneResult[i].Signal[0],(long)decimal_part(Result->ZoneResult[i].Signal[0]));
    // printf("{NumberTarget is %ld }\n",(long)Result->ZoneResult[i].NumberOfTargets);


    // if ((long)Result->ZoneResult[i].Status[0] == 0 && Result->ZoneResult[i].NumberOfTargets > 0 &&  (long)Result->ZoneResult[i].Distance[0] > 150)
    // {
    if ((long)Result->ZoneResult[i].Status[0] == 0 && Result->ZoneResult[i].NumberOfTargets > 0)
    {
      printf("{plotter:%ld,%5ld,%ld.%02ld,%ld.%02ld,%ld}\n",
      (long)Result->ZoneResult[i].Status[0],
      (long)Result->ZoneResult[i].Distance[0], 
      (long)Result->ZoneResult[i].Ambient[0],(long)decimal_part(Result->ZoneResult[i].Ambient[0]), 
      (long)Result->ZoneResult[i].Signal[0],(long)decimal_part(Result->ZoneResult[i].Signal[0]),
      (long)Result->ZoneResult[i].NumberOfTargets * 1000);
    }
    else
    {
      // printf("{plotter:%ld,%5ld,%ld.%02ld,%ld.%02ld,%ld}\n",
      // (long)Result->ZoneResult[i].Status[0],
      // distance_invaild, 
      // (long)Result->ZoneResult[i].Ambient[0],(long)decimal_part(Result->ZoneResult[i].Ambient[0]), 
      // (long)Result->ZoneResult[i].Signal[0],(long)decimal_part(Result->ZoneResult[i].Signal[0]),
      // (long)Result->ZoneResult[i].NumberOfTargets * 1000);
    }
    // PRINT(signal,"%ld.%02ld,%ld.%02ld",
    // (long)Result->ZoneResult[i].Ambient[0],(long)decimal_part(Result->ZoneResult[i].Ambient[0]),
    // (long)Result->ZoneResult[i].Signal[0],(long)decimal_part(Result->ZoneResult[i].Signal[0]));
    // PRINT(distance, "%ld,%5ld,%ld", (long)Result->ZoneResult[i].Status[0], (long)Result->ZoneResult[i].Distance[0],
    // (long)Result->ZoneResult[i].NumberOfTargets);
  }
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

#ifdef __cplusplus
}
#endif
