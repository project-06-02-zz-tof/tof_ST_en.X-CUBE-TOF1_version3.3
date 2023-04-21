[#ftl]
/**
 ******************************************************************************
 * @file    app_tof_pin_conf.c
 * @author  IMG SW Application Team
 * @brief   This file contains functions for TOF pins
 ******************************************************************************
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

[#assign IpInstance = ""]
[#assign IpName = ""]
[#assign tof_int_exti_pin = ""]
[#assign tof_int_exti_port = ""]

[#list BspIpDatas as SWIP]
  [#if SWIP.variables??]
    [#list SWIP.variables as variables]
      [#if variables.name?contains("IpName")]
        [#assign IpName = variables.value]
      [/#if]
      [#if variables.name?contains("IpInstance")]
        [#assign IpInstance = variables.value]
      [/#if]
      [#if variables.value?contains("TOF_INT_PIN")]
        [#assign tof_int_exti_pin = IpInstance]
        [#assign tof_int_exti_port = IpName]
      [/#if]
    [/#list]
  [/#if]
[/#list]

/* Includes ------------------------------------------------------------------*/
#include "app_tof_pin_conf.h"

[#if tof_int_exti_pin != ""]
extern volatile uint8_t ToF_EventDetected;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TOF_INT_EXTI_PIN)
  {
    ToF_EventDetected = 1;
  }
}
[/#if]
