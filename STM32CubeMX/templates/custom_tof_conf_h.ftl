[#ftl]
/**
 ******************************************************************************
 * @file    custom_tof_conf.h
 * @author  IMG SW Application Team
 * @brief   This file contains definitions of the TOF components bus interfaces
 *          for custom boards
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

[#assign vl53l1cb_i2c_instance = ""]
[#assign vl53l1cb_xshut_port = ""]
[#assign vl53l1cb_xshut_pin = ""]

[#assign vl53l3cx_i2c_instance = ""]
[#assign vl53l3cx_xshut_port = ""]
[#assign vl53l3cx_xshut_pin = ""]

[#assign vl53l4cx_i2c_instance = ""]
[#assign vl53l4cx_xshut_port = ""]
[#assign vl53l4cx_xshut_pin = ""]

[#assign vl53l4cd_i2c_instance = ""]
[#assign vl53l4cd_xshut_port = ""]
[#assign vl53l4cd_xshut_pin = ""]

[#assign vl53l5cx_i2c_instance = ""]
[#assign vl53l5cx_pwr_en_port = ""]
[#assign vl53l5cx_pwr_en_pin = ""]
[#assign vl53l5cx_i2c_rst_port = ""]
[#assign vl53l5cx_i2c_rst_pin = ""]
[#assign vl53l5cx_lpn_port = ""]
[#assign vl53l5cx_lpn_pin = ""]

[#assign vl53l7cx_i2c_instance = ""]
[#assign vl53l7cx_pwr_en_port = ""]
[#assign vl53l7cx_pwr_en_pin = ""]
[#assign vl53l7cx_i2c_rst_port = ""]
[#assign vl53l7cx_i2c_rst_pin = ""]
[#assign vl53l7cx_lpn_port = ""]
[#assign vl53l7cx_lpn_pin = ""]

[#assign vl53l8cx_i2c_instance = ""]
[#assign vl53l8cx_pwr_en_port = ""]
[#assign vl53l8cx_pwr_en_pin = ""]
[#assign vl53l8cx_i2c_rst_port = ""]
[#assign vl53l8cx_i2c_rst_pin = ""]
[#assign vl53l8cx_lpn_port = ""]
[#assign vl53l8cx_lpn_pin = ""]

[#if BspIpDatas??]
  [#list BspIpDatas as SWIP]
    [#if SWIP.defines??]
      [#if SWIP.variables??]
        [#list SWIP.variables as variables]
          [#if variables.name?contains("IpInstance")]
            [#assign IpInstance = variables.value]
          [/#if]
          [#if variables.name?contains("IpName")]
            [#assign IpName = variables.value]
          [/#if]
          [#if variables.value?contains("VL53L1CB BUS IO driver")]
            [#assign vl53l1cb_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L3CX BUS IO driver")]
            [#assign vl53l3cx_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L4CX BUS IO driver")]
            [#assign vl53l4cx_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L4CD BUS IO driver")]
            [#assign vl53l4cd_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L5CX BUS IO driver")]
            [#assign vl53l5cx_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L7CX BUS IO driver")]
            [#assign vl53l7cx_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L8CX BUS IO driver")]
            [#assign vl53l8cx_i2c_instance = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L1CB_XSHUT")]
            [#assign vl53l1cb_xshut_port = IpName]
            [#assign vl53l1cb_xshut_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L3CX_XSHUT")]
            [#assign vl53l3cx_xshut_port = IpName]
            [#assign vl53l3cx_xshut_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L4CX_XSHUT")]
            [#assign vl53l4cx_xshut_port = IpName]
            [#assign vl53l4cx_xshut_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L4CD_XSHUT")]
            [#assign vl53l4cd_xshut_port = IpName]
            [#assign vl53l4cd_xshut_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L5CX_PWR_EN")]
            [#assign vl53l5cx_pwr_en_port = IpName]
            [#assign vl53l5cx_pwr_en_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L5CX_I2C_RST")]
            [#assign vl53l5cx_i2c_rst_port = IpName]
            [#assign vl53l5cx_i2c_rst_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L5CX_LPn")]
            [#assign vl53l5cx_lpn_port = IpName]
            [#assign vl53l5cx_lpn_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L7CX_PWR_EN")]
            [#assign vl53l7cx_pwr_en_port = IpName]
            [#assign vl53l7cx_pwr_en_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L7CX_I2C_RST")]
            [#assign vl53l7cx_i2c_rst_port = IpName]
            [#assign vl53l7cx_i2c_rst_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L7CX_LPn")]
            [#assign vl53l7cx_lpn_port = IpName]
            [#assign vl53l7cx_lpn_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L8CX_PWR_EN")]
            [#assign vl53l8cx_pwr_en_port = IpName]
            [#assign vl53l8cx_pwr_en_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L8CX_I2C_RST")]
            [#assign vl53l8cx_i2c_rst_port = IpName]
            [#assign vl53l8cx_i2c_rst_pin = IpInstance]
          [/#if]
          [#if variables.value?contains("VL53L8CX_LPn")]
            [#assign vl53l8cx_lpn_port = IpName]
            [#assign vl53l8cx_lpn_pin = IpInstance]
          [/#if]
        [/#list]
      [/#if]
    [/#if]
  [/#list]
[/#if]

[#assign useVL53L1CB = false]
[#assign useVL53L3CX = false]
[#assign useVL53L4CX = false]
[#assign useVL53L4CD = false]
[#assign useVL53L5CX = false]
[#assign useVL53L7CX = false]
[#assign useVL53L8CX = false]

[#if RTEdatas??]
[#list RTEdatas as define]

[#if define?contains("VL53L1CB")]
[#assign useVL53L1CB = true]
[/#if]
[#if define?contains("VL53L3CX")]
[#assign useVL53L3CX = true]
[/#if]
[#if define?contains("VL53L4CX")]
[#assign useVL53L4CX = true]
[/#if]
[#if define?contains("VL53L4CD")]
[#assign useVL53L4CD = true]
[/#if]
[#if define?contains("VL53L5CX")]
[#assign useVL53L5CX = true]
[/#if]
[#if define?contains("VL53L7CX")]
[#assign useVL53L7CX = true]
[/#if]
[#if define?contains("VL53L8CX")]
[#assign useVL53L8CX = true]
[/#if]

[/#list]
[/#if]

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_TOF_CONF_H__
#define __CUSTOM_TOF_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "${FamilyName?lower_case}xx_hal.h"
#include "${BoardName}_bus.h"
#include "${BoardName}_errno.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

[#if useVL53L1CB]
#define USE_CUSTOM_RANGING_VL53L1CB (1U)
[/#if]
[#if useVL53L3CX]
#define USE_CUSTOM_RANGING_VL53L3CX (1U)
[/#if]
[#if useVL53L4CX]
#define USE_CUSTOM_RANGING_VL53L4CX (1U)
[/#if]
[#if useVL53L4CD]
#define USE_CUSTOM_RANGING_VL53L4CD (1U)
[/#if]
[#if useVL53L5CX]
#define USE_CUSTOM_RANGING_VL53L5CX (1U)
[/#if]
[#if useVL53L7CX]
#define USE_CUSTOM_RANGING_VL53L7CX (1U)
[/#if]
[#if useVL53L8CX]
#define USE_CUSTOM_RANGING_VL53L8CX (1U)
[/#if]

[#if vl53l1cb_xshut_port != "" && vl53l1cb_xshut_pin != ""]
#define CUSTOM_VL53L1CB_XSHUT_PORT    ${vl53l1cb_xshut_port}
#define CUSTOM_VL53L1CB_XSHUT_PIN     ${vl53l1cb_xshut_pin}
[/#if]

[#if vl53l1cb_i2c_instance != ""]
#define CUSTOM_VL53L1CB_I2C_Init      BSP_${vl53l1cb_i2c_instance}_Init
#define CUSTOM_VL53L1CB_I2C_DeInit    BSP_${vl53l1cb_i2c_instance}_DeInit
#define CUSTOM_VL53L1CB_I2C_WriteReg  BSP_${vl53l1cb_i2c_instance}_Send
#define CUSTOM_VL53L1CB_I2C_ReadReg   BSP_${vl53l1cb_i2c_instance}_Recv
[/#if]

[#if vl53l3cx_xshut_port != "" && vl53l3cx_xshut_pin != ""]
#define CUSTOM_VL53L3CX_XSHUT_PORT    ${vl53l3cx_xshut_port}
#define CUSTOM_VL53L3CX_XSHUT_PIN     ${vl53l3cx_xshut_pin}
[/#if]

[#if vl53l3cx_i2c_instance != ""]
#define CUSTOM_VL53L3CX_I2C_Init      BSP_${vl53l3cx_i2c_instance}_Init
#define CUSTOM_VL53L3CX_I2C_DeInit    BSP_${vl53l3cx_i2c_instance}_DeInit
#define CUSTOM_VL53L3CX_I2C_WriteReg  BSP_${vl53l3cx_i2c_instance}_Send
#define CUSTOM_VL53L3CX_I2C_ReadReg   BSP_${vl53l3cx_i2c_instance}_Recv
[/#if]

[#if vl53l4cx_xshut_port != "" && vl53l4cx_xshut_pin != ""]
#define CUSTOM_VL53L4CX_XSHUT_PORT    ${vl53l4cx_xshut_port}
#define CUSTOM_VL53L4CX_XSHUT_PIN     ${vl53l4cx_xshut_pin}
[/#if]

[#if vl53l4cx_i2c_instance != ""]
#define CUSTOM_VL53L4CX_I2C_Init      BSP_${vl53l4cx_i2c_instance}_Init
#define CUSTOM_VL53L4CX_I2C_DeInit    BSP_${vl53l4cx_i2c_instance}_DeInit
#define CUSTOM_VL53L4CX_I2C_WriteReg  BSP_${vl53l4cx_i2c_instance}_Send
#define CUSTOM_VL53L4CX_I2C_ReadReg   BSP_${vl53l4cx_i2c_instance}_Recv
[/#if]

[#if vl53l4cd_xshut_port != "" && vl53l4cd_xshut_pin != ""]
#define CUSTOM_VL53L4CD_XSHUT_PORT    ${vl53l4cd_xshut_port}
#define CUSTOM_VL53L4CD_XSHUT_PIN     ${vl53l4cd_xshut_pin}
[/#if]

[#if vl53l4cd_i2c_instance != ""]
#define CUSTOM_VL53L4CD_I2C_Init      BSP_${vl53l4cd_i2c_instance}_Init
#define CUSTOM_VL53L4CD_I2C_DeInit    BSP_${vl53l4cd_i2c_instance}_DeInit
#define CUSTOM_VL53L4CD_I2C_WriteReg  BSP_${vl53l4cd_i2c_instance}_Send
#define CUSTOM_VL53L4CD_I2C_ReadReg   BSP_${vl53l4cd_i2c_instance}_Recv
[/#if]

[#if vl53l5cx_pwr_en_port != "" && vl53l5cx_pwr_en_pin != ""]
#define CUSTOM_VL53L5CX_PWR_EN_PORT    ${vl53l5cx_pwr_en_port}
#define CUSTOM_VL53L5CX_PWR_EN_PIN     ${vl53l5cx_pwr_en_pin}
[/#if]

[#if vl53l5cx_i2c_rst_port != "" && vl53l5cx_i2c_rst_pin != ""]
#define CUSTOM_VL53L5CX_I2C_RST_PORT    ${vl53l5cx_i2c_rst_port}
#define CUSTOM_VL53L5CX_I2C_RST_PIN     ${vl53l5cx_i2c_rst_pin}
[/#if]

[#if vl53l5cx_lpn_port != "" && vl53l5cx_lpn_pin != ""]
#define CUSTOM_VL53L5CX_LPn_PORT    ${vl53l5cx_lpn_port}
#define CUSTOM_VL53L5CX_LPn_PIN     ${vl53l5cx_lpn_pin}
[/#if]

[#if vl53l5cx_i2c_instance != ""]
#define CUSTOM_VL53L5CX_I2C_Init      BSP_${vl53l5cx_i2c_instance}_Init
#define CUSTOM_VL53L5CX_I2C_DeInit    BSP_${vl53l5cx_i2c_instance}_DeInit
#define CUSTOM_VL53L5CX_I2C_WriteReg  BSP_${vl53l5cx_i2c_instance}_WriteReg16
#define CUSTOM_VL53L5CX_I2C_ReadReg   BSP_${vl53l5cx_i2c_instance}_ReadReg16
[/#if]

[#if vl53l7cx_pwr_en_port != "" && vl53l7cx_pwr_en_pin != ""]
#define CUSTOM_VL53L7CX_PWR_EN_PORT    ${vl53l7cx_pwr_en_port}
#define CUSTOM_VL53L7CX_PWR_EN_PIN     ${vl53l7cx_pwr_en_pin}
[/#if]

[#if vl53l7cx_i2c_rst_port != "" && vl53l7cx_i2c_rst_pin != ""]
#define CUSTOM_VL53L7CX_I2C_RST_PORT    ${vl53l7cx_i2c_rst_port}
#define CUSTOM_VL53L7CX_I2C_RST_PIN     ${vl53l7cx_i2c_rst_pin}
[/#if]

[#if vl53l7cx_lpn_port != "" && vl53l7cx_lpn_pin != ""]
#define CUSTOM_VL53L7CX_LPn_PORT    ${vl53l7cx_lpn_port}
#define CUSTOM_VL53L7CX_LPn_PIN     ${vl53l7cx_lpn_pin}
[/#if]

[#if vl53l7cx_i2c_instance != ""]
#define CUSTOM_VL53L7CX_I2C_Init      BSP_${vl53l7cx_i2c_instance}_Init
#define CUSTOM_VL53L7CX_I2C_DeInit    BSP_${vl53l7cx_i2c_instance}_DeInit
#define CUSTOM_VL53L7CX_I2C_WriteReg  BSP_${vl53l7cx_i2c_instance}_WriteReg16
#define CUSTOM_VL53L7CX_I2C_ReadReg   BSP_${vl53l7cx_i2c_instance}_ReadReg16
[/#if]

[#if vl53l8cx_pwr_en_port != "" && vl53l8cx_pwr_en_pin != ""]
#define CUSTOM_VL53L8CX_PWR_EN_PORT    ${vl53l8cx_pwr_en_port}
#define CUSTOM_VL53L8CX_PWR_EN_PIN     ${vl53l8cx_pwr_en_pin}
[/#if]

[#if vl53l8cx_i2c_rst_port != "" && vl53l8cx_i2c_rst_pin != ""]
#define CUSTOM_VL53L8CX_I2C_RST_PORT    ${vl53l8cx_i2c_rst_port}
#define CUSTOM_VL53L8CX_I2C_RST_PIN     ${vl53l8cx_i2c_rst_pin}
[/#if]

[#if vl53l8cx_lpn_port != "" && vl53l8cx_lpn_pin != ""]
#define CUSTOM_VL53L8CX_LPn_PORT    ${vl53l8cx_lpn_port}
#define CUSTOM_VL53L8CX_LPn_PIN     ${vl53l8cx_lpn_pin}
[/#if]

[#if vl53l8cx_i2c_instance != ""]
#define CUSTOM_VL53L8CX_I2C_Init      BSP_${vl53l8cx_i2c_instance}_Init
#define CUSTOM_VL53L8CX_I2C_DeInit    BSP_${vl53l8cx_i2c_instance}_DeInit
#define CUSTOM_VL53L8CX_I2C_WriteReg  BSP_${vl53l8cx_i2c_instance}_WriteReg16
#define CUSTOM_VL53L8CX_I2C_ReadReg   BSP_${vl53l8cx_i2c_instance}_ReadReg16
[/#if]

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_TOF_CONF_H__*/
