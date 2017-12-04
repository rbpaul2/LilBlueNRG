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
#ifndef __LIL_MOTOR_H
#define __LIL_MOTOR_H

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
	/**
	* @}
	*/

	/*state for encoder
	* moving in clockwise direction from top down,
	* moving in counter-clock direction otherwise
	*/

	enum states {
		state_00,
		state_10,
		state_11,
		state_01
	};



	/*for motor*/
	/*--------variable declaration------------*/



	/*------------function declaration-----------------*/
	/**
	* @brief  Delay function
	* @param  Delay in ms
	* @retval None
	*/

	void Lil_GPIOInit(uint32_t pin, uint8_t mode);

	void Lil_ButtonInit(uint32_t pin);

	void Lil_MotorEncoderIrq_CHA_Init();

	void Lil_MotorEncoderIrq_CHB_Init();


	void MotorInit(uint32_t A, uint32_t B, uint32_t CH_A_IN, uint32_t CH_B_IN);

	void MotorControl(unsigned percentage);

	/*Motor encoder interrupt handler,
	* the handler will determine the current position and direction of the motor
	* basing on the reading from Channel A and Channel B
	*/
	void Encoder_Handler();

	/*initialize the state machine for encoder*/
	void MotorEncoder_init();

	/*Calibrate the motor to determine number of revolution for 100% open */
	void MotorCalibration();

	/*button Debouncer*/
	FlagStatus De_Button();

	/*get the state of one GPIO PIN*/
	FlagStatus Lil_GetState(uint32_t pin);

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
