/**
 * @file    Lil_config.h
 * @author  Homey team
 * @version V1.0.0
 * @date    November 21, 2017
 * @brief   This file provides stuff
 * @details
 *
 *
 *
 *
 * <h2><center>&copy; COPYRIGHT 2017 MyLilHomey</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIL_CONFIG_H
#define __LIL_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief  Enumeration of Lil_MotionDetector
 */


/** @defgroup Lil_MotionDetector_Exported_Macros           SDK EVAL Button Exported Macros
 * @{
 */
#define MOTION_DETECTOR_PIN			GPIO_Pin_14
#define MOTOR_POS_PIN				GPIO_Pin_2
#define MOTOR_NEG_PIN				GPIO_Pin_0
#define MOTOR_CHA_PIN				GPIO_Pin_3 // Unused
#define MOTOR_CHB_PIN				GPIO_Pin_1
/**
 * @}
 */


/** @defgroup Lil_MotionDetector_Exported_Functions        SDK EVAL Button Exported Functions
 * @{
 */

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
