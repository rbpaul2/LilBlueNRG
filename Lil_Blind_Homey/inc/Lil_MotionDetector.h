/**
 * @file    Lil_MotionDetector.h
 * @author  Homey team
 * @version V1.0.0
 * @date    November 21, 2017
 * @brief   This file provides a Motion Detector interface for the OpenPIR
 * @details
 *
 *
 *
 *
 * <h2><center>&copy; COPYRIGHT 2017 MyLilHomey</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIL_MOTIONDETECTOR_H
#define __LIL_MOTIONDETECTOR_H

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
/**
 * @}
 */


/** @defgroup Lil_MotionDetector_Exported_Functions        SDK EVAL Button Exported Functions
 * @{
 */

void Lil_MotionDetectorInit(uint32_t pin);
void Lil_MotionDetectorIrq(uint32_t pin, SdkEvalButtonIrq xIrq);
FlagStatus Lil_MotionDetectorGetState(uint32_t pin);
FlagStatus Lil_MotionDetectorGetITPendingBit(uint32_t pin);
void Lil_MotionDetectorClearITPendingBit(uint32_t pin);

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
