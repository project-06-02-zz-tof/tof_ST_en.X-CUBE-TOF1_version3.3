[#ftl]

[#assign IpInstance = ""]
[#assign IpName = ""]
[#assign VL53L4A1I2CInstance = ""]

[#list BspIpDatas as SWIP]
  [#if SWIP.variables??]
    [#list SWIP.variables as variables]
      [#if variables.name?contains("IpInstance")]
        [#assign IpInstance = variables.value]
      [/#if]
      [#if variables.name?contains("IpName")]
        [#assign IpName = variables.value]
      [/#if]
      [#if variables.value?contains("53L4A1 BUS IO driver")]
        [#assign VL53L4A1I2CInstance = IpInstance]
      [/#if]
    [/#list]
  [/#if]
[/#list]

/**
 ******************************************************************************
 * @file    53l4a1_conf.h
 * @author  IMG SW Application Team
 * @brief   This file contains definitions for the ToF components bus interfaces
 *          when using the X-NUCLEO-53L4A1 expansion board
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VL53L4A1_CONF_H
#define VL53L4A1_CONF_H

#include "${FamilyName?lower_case}xx_hal.h"
#include "${BoardName}_bus.h"
#include "${BoardName}_errno.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* 
 * the 53L4A1 BSP driver uses this symbol to allocate a structure for each device
 * if you are only using the on-board sensor without break-out boards
 * change its to (1U) in order to save space in RAM memory
 */
#define RANGING_SENSOR_INSTANCES_NBR    (3U)

[#if VL53L4A1I2CInstance != ""]
#define VL53L4A1_hi2c                   (h${VL53L4A1I2CInstance?lower_case})

#define VL53L4A1_I2C_SCL_GPIO_PORT      BUS_${VL53L4A1I2CInstance}_SCL_GPIO_PORT
#define VL53L4A1_I2C_SCL_GPIO_PIN       BUS_${VL53L4A1I2CInstance}_SCL_GPIO_PIN
#define VL53L4A1_I2C_SDA_GPIO_PORT      BUS_${VL53L4A1I2CInstance}_SDA_GPIO_PORT
#define VL53L4A1_I2C_SDA_GPIO_PIN       BUS_${VL53L4A1I2CInstance}_SDA_GPIO_PIN

#define VL53L4A1_I2C_Init               BSP_${VL53L4A1I2CInstance}_Init
#define VL53L4A1_I2C_DeInit             BSP_${VL53L4A1I2CInstance}_DeInit
#define VL53L4A1_I2C_WriteReg           BSP_${VL53L4A1I2CInstance}_Send
#define VL53L4A1_I2C_ReadReg            BSP_${VL53L4A1I2CInstance}_Recv
#define VL53L4A1_GetTick                BSP_GetTick
[/#if]

#ifdef __cplusplus
}
#endif

#endif /* VL53L4A1_CONF_H */
