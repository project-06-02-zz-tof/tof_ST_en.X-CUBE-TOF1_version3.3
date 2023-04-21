[#ftl]
[#assign moduleName = "x-cube-tof1"]
[#if ModuleName??]
    [#assign moduleName = ModuleName]
[/#if]
/**
  ******************************************************************************
  * @file          : app_${moduleName?lower_case}.c
  * @author        : IMG SW Application Team
  * @brief         : This file provides code for the configuration
  *                  of the ${name} instances.
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
#include "app_${moduleName?lower_case}.h"
#include "main.h"
#include <stdio.h>
[#if includes??]
[#list includes as include]
#include "${include}"
[/#list]
[/#if]

[#assign useVL53L1CB_SIMPLE_RANGING_DEMO = false]
[#assign useVL53L3CX_SIMPLE_RANGING_DEMO = false]
[#assign useVL53L5CX_SIMPLE_RANGING_DEMO = false]
[#assign useVL53L4CX_SIMPLE_RANGING_DEMO = false]
[#assign useVL53L4CD_SIMPLE_RANGING_DEMO = false]
[#assign useVL53L7CX_SIMPLE_RANGING_DEMO = false]
[#assign useVL53L8CX_SIMPLE_RANGING_DEMO = false]

[#assign use53L4A2_SIMPLE_RANGING_DEMO = false]
[#assign use53L4A2_MULTI_SENSOR_RANGING_DEMO = false]

[#assign use53L4A1_SIMPLE_RANGING_DEMO = false]
[#assign use53L4A1_MULTI_SENSOR_RANGING_DEMO = false]
[#assign use53L4A1_THRESHOLD_DETECTION_DEMO = false]

[#assign use53L1A2_SIMPLE_RANGING_DEMO = false]
[#assign use53L1A2_MULTI_SENSOR_RANGING_DEMO = false]

[#assign use53L3A2_SIMPLE_RANGING_DEMO = false]
[#assign use53L3A2_MULTI_SENSOR_RANGING_DEMO = false]

[#assign use53L5A1_SIMPLE_RANGING_DEMO = false]
[#assign use53L5A1_MULTI_SENSOR_RANGING_DEMO = false]
[#assign use53L5A1_THRESHOLD_DETECTION_DEMO = false]

[#assign use53L7A1_SIMPLE_RANGING_DEMO = false]
[#assign use53L7A1_MULTI_SENSOR_RANGING_DEMO = false]
[#assign use53L7A1_THRESHOLD_DETECTION_DEMO = false]

[#assign use53L8A1_SIMPLE_RANGING_DEMO = false]
[#assign use53L8A1_MULTI_SENSOR_RANGING_DEMO = false]
[#assign use53L8A1_THRESHOLD_DETECTION_DEMO = false]

[#if RTEdatas??]
[#list RTEdatas as define]

[#if define?contains("VL53L1CB_SIMPLE_RANGING")]
[#assign useVL53L1CB_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("VL53L4CX_SIMPLE_RANGING")]
[#assign useVL53L4CX_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("VL53L4CD_SIMPLE_RANGING")]
[#assign useVL53L4CD_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("VL53L3CX_SIMPLE_RANGING")]
[#assign useVL53L3CX_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("VL53L5CX_SIMPLE_RANGING")]
[#assign useVL53L5CX_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("VL53L7CX_SIMPLE_RANGING")]
[#assign useVL53L7CX_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("VL53L8CX_SIMPLE_RANGING")]
[#assign useVL53L8CX_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L1A2_SIMPLE_RANGING")]
[#assign use53L1A2_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L1A2_MULTI_SENSOR_RANGING")]
[#assign use53L1A2_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L3A2_SIMPLE_RANGING")]
[#assign use53L3A2_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L3A2_MULTI_SENSOR_RANGING")]
[#assign use53L3A2_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L4A2_SIMPLE_RANGING")]
[#assign use53L4A2_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L4A2_MULTI_SENSOR_RANGING")]
[#assign use53L4A2_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L4A1_SIMPLE_RANGING")]
[#assign use53L4A1_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L4A1_MULTI_SENSOR_RANGING")]
[#assign use53L4A1_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L4A1_THRESHOLD_DETECTION")]
[#assign use53L4A1_THRESHOLD_DETECTION_DEMO = true]
[/#if]

[#if define?contains("_53L5A1_SIMPLE_RANGING")]
[#assign use53L5A1_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L5A1_MULTI_SENSOR_RANGING")]
[#assign use53L5A1_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L5A1_THRESHOLD_DETECTION")]
[#assign use53L5A1_THRESHOLD_DETECTION_DEMO = true]
[/#if]

[#if define?contains("_53L7A1_SIMPLE_RANGING")]
[#assign use53L7A1_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L7A1_MULTI_SENSOR_RANGING")]
[#assign use53L7A1_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L7A1_THRESHOLD_DETECTION")]
[#assign use53L7A1_THRESHOLD_DETECTION_DEMO = true]
[/#if]

[#if define?contains("_53L8A1_SIMPLE_RANGING")]
[#assign use53L8A1_SIMPLE_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L8A1_MULTI_SENSOR_RANGING")]
[#assign use53L8A1_MULTI_SENSOR_RANGING_DEMO = true]
[/#if]

[#if define?contains("_53L8A1_THRESHOLD_DETECTION")]
[#assign use53L8A1_THRESHOLD_DETECTION_DEMO = true]
[/#if]

[/#list]
[/#if]

[#if useVL53L1CB_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l1cb_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if useVL53L4CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l4cx_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if useVL53L4CD_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l4cd_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if useVL53L3CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l3cx_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if useVL53L5CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l5cx_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if useVL53L7CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l7cx_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if useVL53L8CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l8cx_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L1A2_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l1a2_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L1A2_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l1a2_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L3A2_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l3a2_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L3A2_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l3a2_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L4A2_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a2_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L4A2_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a2_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L4A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a1_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L4A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a1_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L4A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a1_threshold_detection_demo_gv.tmp"/]
[/#if]

[#if use53L5A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l5a1_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L5A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l5a1_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L5A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l5a1_threshold_detection_demo_gv.tmp"/]
[/#if]

[#if use53L7A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l7a1_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L7A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l7a1_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L7A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l7a1_threshold_detection_demo_gv.tmp"/]
[/#if]

[#if use53L8A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l8a1_simple_ranging_demo_gv.tmp"/]
[/#if]

[#if use53L8A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l8a1_multi_sensor_demo_gv.tmp"/]
[/#if]

[#if use53L8A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l8a1_threshold_detection_demo_gv.tmp"/]
[/#if]

void ${fctName}(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN ${fctName?replace("MX_","")}_PreTreatment */

  /* USER CODE END ${fctName?replace("MX_","")}_PreTreatment */

  /* Initialize the peripherals and the TOF components */
  [#if useVL53L1CB_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l1cb_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if useVL53L3CX_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l3cx_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if useVL53L4CX_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l4cx_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if useVL53L4CD_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l4cd_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if useVL53L5CX_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l5cx_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if useVL53L7CX_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l7cx_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if useVL53L8CX_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/vl53l8cx_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L1A2_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l1a2_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L1A2_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l1a2_multi_sensor_demo_init.tmp"/]
  [/#if]
 
  [#if use53L3A2_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l3a2_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L3A2_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l3a2_multi_sensor_demo_init.tmp"/]
  [/#if]

  [#if use53L4A2_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l4a2_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L4A2_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l4a2_multi_sensor_demo_init.tmp"/]
  [/#if]

  [#if use53L4A1_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l4a1_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L4A1_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l4a1_multi_sensor_demo_init.tmp"/]
  [/#if]

  [#if use53L4A1_THRESHOLD_DETECTION_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l4a1_threshold_detection_demo_init.tmp"/]
  [/#if]

  [#if use53L5A1_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l5a1_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L5A1_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l5a1_multi_sensor_demo_init.tmp"/]
  [/#if]

  [#if use53L5A1_THRESHOLD_DETECTION_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l5a1_threshold_detection_demo_init.tmp"/]
  [/#if]

  [#if use53L7A1_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l7a1_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L7A1_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l7a1_multi_sensor_demo_init.tmp"/]
  [/#if]

  [#if use53L7A1_THRESHOLD_DETECTION_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l7a1_threshold_detection_demo_init.tmp"/]
  [/#if]

  [#if use53L8A1_SIMPLE_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l8a1_simple_ranging_demo_init.tmp"/]
  [/#if]

  [#if use53L8A1_MULTI_SENSOR_RANGING_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l8a1_multi_sensor_demo_init.tmp"/]
  [/#if]

  [#if use53L8A1_THRESHOLD_DETECTION_DEMO]
  [@common.optinclude name=mxTmpFolder + "/53l8a1_threshold_detection_demo_init.tmp"/]
  [/#if]

  /* USER CODE BEGIN ${fctName?replace("MX_","")}_PostTreatment */

  /* USER CODE END ${fctName?replace("MX_","")}_PostTreatment */
}

/*
 * LM background task
 */
void ${fctProcessName}(void)
{
  /* USER CODE BEGIN ${fctProcessName?replace("MX_","")}_PreTreatment */

  /* USER CODE END ${fctProcessName?replace("MX_","")}_PreTreatment */

  [#if useVL53L1CB_SIMPLE_RANGING_DEMO]
  MX_VL53L1CB_SimpleRanging_Process();
  [/#if]

  [#if useVL53L3CX_SIMPLE_RANGING_DEMO]
  MX_VL53L3CX_SimpleRanging_Process();
  [/#if]

  [#if useVL53L4CX_SIMPLE_RANGING_DEMO]
  MX_VL53L4CX_SimpleRanging_Process();
  [/#if]

  [#if useVL53L4CD_SIMPLE_RANGING_DEMO]
  MX_VL53L4CD_SimpleRanging_Process();
  [/#if]

  [#if useVL53L5CX_SIMPLE_RANGING_DEMO]
  MX_VL53L5CX_SimpleRanging_Process();
  [/#if]

  [#if useVL53L7CX_SIMPLE_RANGING_DEMO]
  MX_VL53L7CX_SimpleRanging_Process();
  [/#if]

  [#if useVL53L8CX_SIMPLE_RANGING_DEMO]
  MX_VL53L8CX_SimpleRanging_Process();
  [/#if]

  [#if use53L1A2_SIMPLE_RANGING_DEMO]
  MX_53L1A2_SimpleRanging_Process();
  [/#if]

  [#if use53L1A2_MULTI_SENSOR_RANGING_DEMO]
  MX_53L1A2_MultiSensorRanging_Process();
  [/#if]

  [#if use53L3A2_SIMPLE_RANGING_DEMO]
  MX_53L3A2_SimpleRanging_Process();
  [/#if]

  [#if use53L3A2_MULTI_SENSOR_RANGING_DEMO]
  MX_53L3A2_MultiSensorRanging_Process();
  [/#if]

  [#if use53L4A2_SIMPLE_RANGING_DEMO]
  MX_53L4A2_SimpleRanging_Process();
  [/#if]

  [#if use53L4A2_MULTI_SENSOR_RANGING_DEMO]
  MX_53L4A2_MultiSensorRanging_Process();
  [/#if]

  [#if use53L4A1_SIMPLE_RANGING_DEMO]
  MX_53L4A1_SimpleRanging_Process();
  [/#if]

  [#if use53L4A1_MULTI_SENSOR_RANGING_DEMO]
  MX_53L4A1_MultiSensorRanging_Process();
  [/#if]

  [#if use53L4A1_THRESHOLD_DETECTION_DEMO]
  MX_53L4A1_ThresholdDetection_Process();
  [/#if]

  [#if use53L5A1_SIMPLE_RANGING_DEMO]
  MX_53L5A1_SimpleRanging_Process();
  [/#if]

  [#if use53L5A1_MULTI_SENSOR_RANGING_DEMO]
  MX_53L5A1_MultiSensorRanging_Process();
  [/#if]

  [#if use53L5A1_THRESHOLD_DETECTION_DEMO]
  MX_53L5A1_ThresholdDetection_Process();
  [/#if]

  [#if use53L7A1_SIMPLE_RANGING_DEMO]
  MX_53L7A1_SimpleRanging_Process();
  [/#if]

  [#if use53L7A1_MULTI_SENSOR_RANGING_DEMO]
  MX_53L7A1_MultiSensorRanging_Process();
  [/#if]

  [#if use53L7A1_THRESHOLD_DETECTION_DEMO]
  MX_53L7A1_ThresholdDetection_Process();
  [/#if]

  [#if use53L8A1_SIMPLE_RANGING_DEMO]
  MX_53L8A1_SimpleRanging_Process();
  [/#if]

  [#if use53L8A1_THRESHOLD_DETECTION_DEMO]
  MX_53L8A1_ThresholdDetection_Process();
  [/#if]

  /* USER CODE BEGIN ${fctProcessName?replace("MX_","")}_PostTreatment */

  /* USER CODE END ${fctProcessName?replace("MX_","")}_PostTreatment */
}

[#if useVL53L1CB_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l1cb_simple_ranging_demo.tmp"/]
[/#if]

[#if useVL53L3CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l3cx_simple_ranging_demo.tmp"/]
[/#if]

[#if useVL53L4CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l4cx_simple_ranging_demo.tmp"/]
[/#if]

[#if useVL53L4CD_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l4cd_simple_ranging_demo.tmp"/]
[/#if]

[#if useVL53L5CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l5cx_simple_ranging_demo.tmp"/]
[/#if]

[#if useVL53L7CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l7cx_simple_ranging_demo.tmp"/]
[/#if]

[#if useVL53L8CX_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/vl53l8cx_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L1A2_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l1a2_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L1A2_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l1a2_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L3A2_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l3a2_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L3A2_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l3a2_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L4A2_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a2_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L4A2_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a2_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L4A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a1_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L4A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a1_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L4A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l4a1_threshold_detection_demo.tmp"/]
[/#if]

[#if use53L5A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l5a1_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L5A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l5a1_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L5A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l5a1_threshold_detection_demo.tmp"/]
[/#if]

[#if use53L7A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l7a1_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L7A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l7a1_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L7A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l7a1_threshold_detection_demo.tmp"/]
[/#if]

[#if use53L8A1_SIMPLE_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l8a1_simple_ranging_demo.tmp"/]
[/#if]

[#if use53L8A1_MULTI_SENSOR_RANGING_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l8a1_multi_sensor_demo.tmp"/]
[/#if]

[#if use53L8A1_THRESHOLD_DETECTION_DEMO]
[@common.optinclude name=mxTmpFolder + "/53l8a1_threshold_detection_demo.tmp"/]
[/#if]

#ifdef __cplusplus
}
#endif
