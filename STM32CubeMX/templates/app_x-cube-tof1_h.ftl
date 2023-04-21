[#ftl]
[#assign moduleName = "x-cube-tof1"]
[#if ModuleName??]
    [#assign moduleName = ModuleName]
[/#if]
/**
  ******************************************************************************
  * @file          : app_${moduleName?lower_case}.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_${moduleName?upper_case?replace("-","_")}_H
#define __APP_${moduleName?upper_case?replace("-","_")}_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void ${fctName}(void);
void ${fctProcessName}(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_${moduleName?upper_case?replace("-","_")}_H */
