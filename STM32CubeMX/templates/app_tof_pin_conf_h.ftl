[#ftl]
/**
 ******************************************************************************
 * @file    app_tof_pin_conf.h
 * @author  IMG SW Application Team
 * @brief   This file contains definitions for TOF pins
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
#ifndef __APP_TOF_PIN_CONF_H__
#define __APP_TOF_PIN_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

[#assign IpInstance = ""]
[#assign IpName = ""]
[#assign tof_int_exti_pin = ""]
[#assign tof_int_exti_port = ""]
[#assign vl53l5a1_pwr_en_l_pin = ""]
[#assign vl53l5a1_pwr_en_l_port = ""]
[#assign vl53l5a1_pwr_en_r_pin = ""]
[#assign vl53l5a1_pwr_en_r_port = ""]
[#assign vl53l5a1_pwr_en_c_pin = ""]
[#assign vl53l5a1_pwr_en_c_port = ""]
[#assign vl53l5a1_lpn_l_pin = ""]
[#assign vl53l5a1_lpn_l_port = ""]
[#assign vl53l5a1_lpn_r_pin = ""]
[#assign vl53l5a1_lpn_r_port = ""]
[#assign vl53l5a1_lpn_c_pin = ""]
[#assign vl53l5a1_lpn_c_port = ""]
[#assign vl53l5a1_i2c_rst_l_pin = ""]
[#assign vl53l5a1_i2c_rst_l_port = ""]
[#assign vl53l5a1_i2c_rst_r_pin = ""]
[#assign vl53l5a1_i2c_rst_r_port = ""]
[#assign vl53l5a1_i2c_rst_c_pin = ""]
[#assign vl53l5a1_i2c_rst_c_port = ""]
[#assign vl53l4a2_xshut_l_pin = ""]
[#assign vl53l4a2_xshut_l_port = ""]
[#assign vl53l4a2_xshut_r_pin = ""]
[#assign vl53l4a2_xshut_r_port = ""]
[#assign vl53l4a2_xshut_c_pin = ""]
[#assign vl53l4a2_xshut_c_port = ""]
[#assign vl53l4a1_xshut_l_pin = ""]
[#assign vl53l4a1_xshut_l_port = ""]
[#assign vl53l4a1_xshut_r_pin = ""]
[#assign vl53l4a1_xshut_r_port = ""]
[#assign vl53l4a1_xshut_c_pin = ""]
[#assign vl53l4a1_xshut_c_port = ""]
[#assign vl53l7a1_pwr_en_l_pin = ""]
[#assign vl53l7a1_pwr_en_l_port = ""]
[#assign vl53l7a1_pwr_en_r_pin = ""]
[#assign vl53l7a1_pwr_en_r_port = ""]
[#assign vl53l7a1_pwr_en_c_pin = ""]
[#assign vl53l7a1_pwr_en_c_port = ""]
[#assign vl53l7a1_lpn_l_pin = ""]
[#assign vl53l7a1_lpn_l_port = ""]
[#assign vl53l7a1_lpn_r_pin = ""]
[#assign vl53l7a1_lpn_r_port = ""]
[#assign vl53l7a1_lpn_c_pin = ""]
[#assign vl53l7a1_lpn_c_port = ""]
[#assign vl53l7a1_i2c_rst_l_pin = ""]
[#assign vl53l7a1_i2c_rst_l_port = ""]
[#assign vl53l7a1_i2c_rst_r_pin = ""]
[#assign vl53l7a1_i2c_rst_r_port = ""]
[#assign vl53l7a1_i2c_rst_c_pin = ""]
[#assign vl53l7a1_i2c_rst_c_port = ""]
[#assign vl53l8a1_pwr_en_c_pin = ""]
[#assign vl53l8a1_pwr_en_c_port = ""]
[#assign vl53l8a1_lpn_c_pin = ""]
[#assign vl53l8a1_lpn_c_port = ""]
[#assign vl53l8a1_i2c_rst_c_pin = ""]
[#assign vl53l8a1_i2c_rst_c_port = ""]

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
      [#if variables.value?contains("53L5A1_PWR_EN_L")]
        [#assign vl53l5a1_pwr_en_l_pin = IpInstance]
        [#assign vl53l5a1_pwr_en_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_PWR_EN_R")]
        [#assign vl53l5a1_pwr_en_r_pin = IpInstance]
        [#assign vl53l5a1_pwr_en_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_PWR_EN_C")]
        [#assign vl53l5a1_pwr_en_c_pin = IpInstance]
        [#assign vl53l5a1_pwr_en_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_LPn_L")]
        [#assign vl53l5a1_lpn_l_pin = IpInstance]
        [#assign vl53l5a1_lpn_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_LPn_R")]
        [#assign vl53l5a1_lpn_r_pin = IpInstance]
        [#assign vl53l5a1_lpn_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_LPn_C")]
        [#assign vl53l5a1_lpn_c_pin = IpInstance]
        [#assign vl53l5a1_lpn_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_I2C_RST_L")]
        [#assign vl53l5a1_i2c_rst_l_pin = IpInstance]
        [#assign vl53l5a1_i2c_rst_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_I2C_RST_R")]
        [#assign vl53l5a1_i2c_rst_r_pin = IpInstance]
        [#assign vl53l5a1_i2c_rst_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L5A1_I2C_RST_C")]
        [#assign vl53l5a1_i2c_rst_c_pin = IpInstance]
        [#assign vl53l5a1_i2c_rst_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L4A2_XSHUT_C")]
        [#assign vl53l4a2_xshut_c_pin = IpInstance]
        [#assign vl53l4a2_xshut_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L4A2_XSHUT_L")]
        [#assign vl53l4a2_xshut_l_pin = IpInstance]
        [#assign vl53l4a2_xshut_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L4A2_XSHUT_R")]
        [#assign vl53l4a2_xshut_r_pin = IpInstance]
        [#assign vl53l4a2_xshut_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L4A1_XSHUT_C")]
        [#assign vl53l4a1_xshut_c_pin = IpInstance]
        [#assign vl53l4a1_xshut_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L4A1_XSHUT_L")]
        [#assign vl53l4a1_xshut_l_pin = IpInstance]
        [#assign vl53l4a1_xshut_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L4A1_XSHUT_R")]
        [#assign vl53l4a1_xshut_r_pin = IpInstance]
        [#assign vl53l4a1_xshut_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_PWR_EN_L")]
        [#assign vl53l7a1_pwr_en_l_pin = IpInstance]
        [#assign vl53l7a1_pwr_en_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_PWR_EN_R")]
        [#assign vl53l7a1_pwr_en_r_pin = IpInstance]
        [#assign vl53l7a1_pwr_en_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_PWR_EN_C")]
        [#assign vl53l7a1_pwr_en_c_pin = IpInstance]
        [#assign vl53l7a1_pwr_en_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_LPn_L")]
        [#assign vl53l7a1_lpn_l_pin = IpInstance]
        [#assign vl53l7a1_lpn_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_LPn_R")]
        [#assign vl53l7a1_lpn_r_pin = IpInstance]
        [#assign vl53l7a1_lpn_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_LPn_C")]
        [#assign vl53l7a1_lpn_c_pin = IpInstance]
        [#assign vl53l7a1_lpn_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_I2C_RST_L")]
        [#assign vl53l7a1_i2c_rst_l_pin = IpInstance]
        [#assign vl53l7a1_i2c_rst_l_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_I2C_RST_R")]
        [#assign vl53l7a1_i2c_rst_r_pin = IpInstance]
        [#assign vl53l7a1_i2c_rst_r_port = IpName]
      [/#if]
      [#if variables.value?contains("53L7A1_I2C_RST_C")]
        [#assign vl53l7a1_i2c_rst_c_pin = IpInstance]
        [#assign vl53l7a1_i2c_rst_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L8A1_PWR_EN_C")]
        [#assign vl53l8a1_pwr_en_c_pin = IpInstance]
        [#assign vl53l8a1_pwr_en_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L8A1_LPn_C")]
        [#assign vl53l8a1_lpn_c_pin = IpInstance]
        [#assign vl53l8a1_lpn_c_port = IpName]
      [/#if]
      [#if variables.value?contains("53L8A1_I2C_RST_C")]
        [#assign vl53l8a1_i2c_rst_c_pin = IpInstance]
        [#assign vl53l8a1_i2c_rst_c_port = IpName]
      [/#if]
    [/#list]
  [/#if]
[/#list]

/* Includes ------------------------------------------------------------------*/
#include "${FamilyName?lower_case}xx_hal.h"

/* Exported symbols ----------------------------------------------------------*/
[#if tof_int_exti_pin != "" && tof_int_exti_port != ""]
#define TOF_INT_EXTI_PIN    (${tof_int_exti_pin})
#define TOF_INT_EXTI_PORT   (${tof_int_exti_port})
[/#if]

[#if vl53l5a1_pwr_en_l_pin != "" && vl53l5a1_pwr_en_l_port != ""]
#define VL53L5A1_PWR_EN_L_PIN   (${vl53l5a1_pwr_en_l_pin})
#define VL53L5A1_PWR_EN_L_PORT  (${vl53l5a1_pwr_en_l_port})
[/#if]
[#if vl53l5a1_pwr_en_r_pin != "" && vl53l5a1_pwr_en_r_port != ""]
#define VL53L5A1_PWR_EN_R_PIN   (${vl53l5a1_pwr_en_r_pin})
#define VL53L5A1_PWR_EN_R_PORT  (${vl53l5a1_pwr_en_r_port})
[/#if]
[#if vl53l5a1_pwr_en_c_pin != "" && vl53l5a1_pwr_en_c_port != ""]
#define VL53L5A1_PWR_EN_C_PIN   (${vl53l5a1_pwr_en_c_pin})
#define VL53L5A1_PWR_EN_C_PORT  (${vl53l5a1_pwr_en_c_port})
[/#if]

[#if vl53l5a1_lpn_l_pin != "" && vl53l5a1_lpn_l_port != ""]
#define VL53L5A1_LPn_L_PIN   (${vl53l5a1_lpn_l_pin})
#define VL53L5A1_LPn_L_PORT  (${vl53l5a1_lpn_l_port})
[/#if]
[#if vl53l5a1_lpn_r_pin != "" && vl53l5a1_lpn_r_port != ""]
#define VL53L5A1_LPn_R_PIN   (${vl53l5a1_lpn_r_pin})
#define VL53L5A1_LPn_R_PORT  (${vl53l5a1_lpn_r_port})
[/#if]
[#if vl53l5a1_lpn_c_pin != "" && vl53l5a1_lpn_c_port != ""]
#define VL53L5A1_LPn_C_PIN   (${vl53l5a1_lpn_c_pin})
#define VL53L5A1_LPn_C_PORT  (${vl53l5a1_lpn_c_port})
[/#if]

[#if vl53l5a1_i2c_rst_l_pin != "" && vl53l5a1_i2c_rst_l_port != ""]
#define VL53L5A1_I2C_RST_L_PIN   (${vl53l5a1_i2c_rst_l_pin})
#define VL53L5A1_I2C_RST_L_PORT  (${vl53l5a1_i2c_rst_l_port})
[/#if]
[#if vl53l5a1_i2c_rst_r_pin != "" && vl53l5a1_i2c_rst_r_port != ""]
#define VL53L5A1_I2C_RST_R_PIN   (${vl53l5a1_i2c_rst_r_pin})
#define VL53L5A1_I2C_RST_R_PORT  (${vl53l5a1_i2c_rst_r_port})
[/#if]
[#if vl53l5a1_i2c_rst_c_pin != "" && vl53l5a1_i2c_rst_c_port != ""]
#define VL53L5A1_I2C_RST_C_PIN   (${vl53l5a1_i2c_rst_c_pin})
#define VL53L5A1_I2C_RST_C_PORT  (${vl53l5a1_i2c_rst_c_port})
[/#if]

[#if vl53l4a2_xshut_c_pin != "" && vl53l4a2_xshut_c_port != ""]
#define VL53L4A2_XSHUT_C_PIN   (${vl53l4a2_xshut_c_pin})
#define VL53L4A2_XSHUT_C_PORT  (${vl53l4a2_xshut_c_port})
[/#if]
[#if vl53l4a2_xshut_l_pin != "" && vl53l4a2_xshut_l_port != ""]
#define VL53L4A2_XSHUT_L_PIN   (${vl53l4a2_xshut_l_pin})
#define VL53L4A2_XSHUT_L_PORT  (${vl53l4a2_xshut_l_port})
[/#if]
[#if vl53l4a2_xshut_r_pin != "" && vl53l4a2_xshut_r_port != ""]
#define VL53L4A2_XSHUT_R_PIN   (${vl53l4a2_xshut_r_pin})
#define VL53L4A2_XSHUT_R_PORT  (${vl53l4a2_xshut_r_port})
[/#if]

[#if vl53l4a1_xshut_c_pin != "" && vl53l4a1_xshut_c_port != ""]
#define VL53L4A1_XSHUT_C_PIN   (${vl53l4a1_xshut_c_pin})
#define VL53L4A1_XSHUT_C_PORT  (${vl53l4a1_xshut_c_port})
[/#if]
[#if vl53l4a1_xshut_l_pin != "" && vl53l4a1_xshut_l_port != ""]
#define VL53L4A1_XSHUT_L_PIN   (${vl53l4a1_xshut_l_pin})
#define VL53L4A1_XSHUT_L_PORT  (${vl53l4a1_xshut_l_port})
[/#if]
[#if vl53l4a1_xshut_r_pin != "" && vl53l4a1_xshut_r_port != ""]
#define VL53L4A1_XSHUT_R_PIN   (${vl53l4a1_xshut_r_pin})
#define VL53L4A1_XSHUT_R_PORT  (${vl53l4a1_xshut_r_port})
[/#if]

[#if vl53l7a1_pwr_en_l_pin != "" && vl53l7a1_pwr_en_l_port != ""]
#define VL53L7A1_PWR_EN_L_PIN   (${vl53l7a1_pwr_en_l_pin})
#define VL53L7A1_PWR_EN_L_PORT  (${vl53l7a1_pwr_en_l_port})
[/#if]
[#if vl53l7a1_pwr_en_r_pin != "" && vl53l7a1_pwr_en_r_port != ""]
#define VL53L7A1_PWR_EN_R_PIN   (${vl53l7a1_pwr_en_r_pin})
#define VL53L7A1_PWR_EN_R_PORT  (${vl53l7a1_pwr_en_r_port})
[/#if]
[#if vl53l7a1_pwr_en_c_pin != "" && vl53l7a1_pwr_en_c_port != ""]
#define VL53L7A1_PWR_EN_C_PIN   (${vl53l7a1_pwr_en_c_pin})
#define VL53L7A1_PWR_EN_C_PORT  (${vl53l7a1_pwr_en_c_port})
[/#if]

[#if vl53l7a1_lpn_l_pin != "" && vl53l7a1_lpn_l_port != ""]
#define VL53L7A1_LPn_L_PIN   (${vl53l7a1_lpn_l_pin})
#define VL53L7A1_LPn_L_PORT  (${vl53l7a1_lpn_l_port})
[/#if]
[#if vl53l7a1_lpn_r_pin != "" && vl53l7a1_lpn_r_port != ""]
#define VL53L7A1_LPn_R_PIN   (${vl53l7a1_lpn_r_pin})
#define VL53L7A1_LPn_R_PORT  (${vl53l7a1_lpn_r_port})
[/#if]
[#if vl53l7a1_lpn_c_pin != "" && vl53l7a1_lpn_c_port != ""]
#define VL53L7A1_LPn_C_PIN   (${vl53l7a1_lpn_c_pin})
#define VL53L7A1_LPn_C_PORT  (${vl53l7a1_lpn_c_port})
[/#if]

[#if vl53l7a1_i2c_rst_l_pin != "" && vl53l7a1_i2c_rst_l_port != ""]
#define VL53L7A1_I2C_RST_L_PIN   (${vl53l7a1_i2c_rst_l_pin})
#define VL53L7A1_I2C_RST_L_PORT  (${vl53l7a1_i2c_rst_l_port})
[/#if]
[#if vl53l7a1_i2c_rst_r_pin != "" && vl53l7a1_i2c_rst_r_port != ""]
#define VL53L7A1_I2C_RST_R_PIN   (${vl53l7a1_i2c_rst_r_pin})
#define VL53L7A1_I2C_RST_R_PORT  (${vl53l7a1_i2c_rst_r_port})
[/#if]
[#if vl53l7a1_i2c_rst_c_pin != "" && vl53l7a1_i2c_rst_c_port != ""]
#define VL53L7A1_I2C_RST_C_PIN   (${vl53l7a1_i2c_rst_c_pin})
#define VL53L7A1_I2C_RST_C_PORT  (${vl53l7a1_i2c_rst_c_port})
[/#if]

[#if vl53l8a1_pwr_en_c_pin != "" && vl53l8a1_pwr_en_c_port != ""]
#define VL53L8A1_PWR_EN_C_PIN   (${vl53l8a1_pwr_en_c_pin})
#define VL53L8A1_PWR_EN_C_PORT  (${vl53l8a1_pwr_en_c_port})
[/#if]

[#if vl53l8a1_lpn_c_pin != "" && vl53l8a1_lpn_c_port != ""]
#define VL53L8A1_LPn_C_PIN   (${vl53l8a1_lpn_c_pin})
#define VL53L8A1_LPn_C_PORT  (${vl53l8a1_lpn_c_port})
[/#if]

[#if vl53l8a1_i2c_rst_c_pin != "" && vl53l8a1_i2c_rst_c_port != ""]
#define VL53L8A1_I2C_RST_C_PIN   (${vl53l8a1_i2c_rst_c_pin})
#define VL53L8A1_I2C_RST_C_PORT  (${vl53l8a1_i2c_rst_c_port})
[/#if]

#ifdef __cplusplus
}
#endif

#endif /* __APP_TOF_PIN_CONF_H__ */
