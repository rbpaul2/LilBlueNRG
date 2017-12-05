/**
  ******************************************************************************
  * @file    BlueNRG1_it.c 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    September-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "gp_timer.h"
#include "ble_const.h" 
#include "app_state.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Com.h"
#include "Lil_MotionDetector.h"
#include "clock.h"
#include "osal.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Examples
  * @{
  */ 

/** @addtogroup GPIO_IOToggle
  * @{
  */ 

extern uint16_t BlindServHandle,
				MotionServHandle,
				TemperatureServHandle,
				HumidityServHandle,
				ThermostatServHandle,
				TemperatureCharHandle,
				HumidityCharHandle,
				MotionDetectedCharHandle,
				BlindCurrPosCharHandle,
				BlindTargPosCharHandle,
				BlindPosStateCharHandle,
				CurrentTempCharHandle,
				TargetTempCharHandle,
				CurrentHCStateCharHandle,
				TargetHCStateCharHandle,
				TempUnitsCharHandle,
				CoolingThresholdCharHandle,
				HeatingThresholdCharHandle;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private macro -------------------------------------------------------------*/
#define MOTION_ON	1
#define MOTION_OFF	0
/* Private variables ---------------------------------------------------------*/
uint8_t MotionDetectedVal;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles SVCall exception.
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void)
{
  SysCount_Handler();
}

void GPIO_Handler(void)
{
	NVIC_DisableIRQ(UART_IRQn);

	struct timer t;
	/* If Motion Detector Interrupt set LED1 */
	if( Lil_MotionDetectorGetITPendingBit((uint32_t)0x00004000) == SET ) {
		Lil_MotionDetectorClearITPendingBit((uint32_t)0x00004000);

		if( Lil_MotionDetectorGetState((uint32_t)0x00004000) == RESET ) {
			//Motion Detector Output went low
			MotionDetectedVal = MOTION_OFF;
			PRINTF("MOTION_OFF %02X\r\n", MotionDetectedVal);
			Timer_Set(&t, CLOCK_SECOND*10);
			while(aci_gatt_update_char_value(MotionServHandle,MotionDetectedCharHandle,0,1,&MotionDetectedVal)==BLE_STATUS_INSUFFICIENT_RESOURCES) {
				APP_FLAG_SET(TX_BUFFER_FULL);
				while(APP_FLAG(TX_BUFFER_FULL)) {
					BTLE_StackTick();
					// Radio is busy (buffer full).
					if(Timer_Expired(&t))
						break;
				}
			}
		}
		else {
			//Motion Detected
			MotionDetectedVal = MOTION_ON;
			PRINTF("MOTION_ON %02X\r\n", MotionDetectedVal);
			Timer_Set(&t, CLOCK_SECOND*10);
			while(aci_gatt_update_char_value(MotionServHandle,MotionDetectedCharHandle,0,1,&MotionDetectedVal)==BLE_STATUS_INSUFFICIENT_RESOURCES) {
				APP_FLAG_SET(TX_BUFFER_FULL);
				while(APP_FLAG(TX_BUFFER_FULL)) {
					BTLE_StackTick();
					// Radio is busy (buffer full).
					if(Timer_Expired(&t))
						break;
				}
			}
		}

	}

	/* If GPIO Interrupt Pin 13 */
	else if ( GPIO_GetITPendingBit(GPIO_Pin_13) == SET ) {
		GPIO_ClearITPendingBit(GPIO_Pin_13);

		if( GPIO_ReadBit(GPIO_Pin_13) == RESET ) {
			SdkEvalLedOff(LED2);
		}
		else {
			SdkEvalLedOn(LED2);
		}
	}

	NVIC_EnableIRQ(UART_IRQn);
}
/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void UART_Handler(void)
{  
  SdkEvalComIOUartIrqHandler();
}

void Blue_Handler(void)
{
   // Call RAL_Isr
   RAL_Isr();
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
