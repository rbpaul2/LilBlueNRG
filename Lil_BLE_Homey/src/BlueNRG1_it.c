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
#include "SDK_EVAL_Config.h"
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

extern volatile uint32_t lSystickCounter;
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
volatile int rtc_cnt = 0;
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

/*
 * This function handle RTC timer interrupt
 * The interrupt handler will set GPIO_Pin_7 to low for 60 seconds
 * and then set GPIO_Pin_7 to high for 90 seconds
 */
void RTC_Handler(void){
	if(SET == RTC_IT_Status(RTC_IT_TIMER))
	{
		/*clear pending interrupt flag*/
		RTC_IT_Clear(RTC_IT_TIMER);
		++rtc_cnt;
		rtc_cnt = rtc_cnt % 5;
		if(rtc_cnt == 2)
		{
			GPIO_WriteBit(GPIO_Pin_6, SET);
		}
		else if(rtc_cnt == 0)
		{
			APP_FLAG_CLEAR(POLL_CO);
			GPIO_WriteBit(GPIO_Pin_6, RESET);
		}
		else if(rtc_cnt == 3)
		{
			APP_FLAG_CLEAR(CO_STABLE);
			APP_FLAG_SET(POLL_CO);
		}
	}
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
  lSystickCounter++;
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
					NVIC_DisableIRQ(UART_IRQn);
				    NVIC_DisableIRQ(GPIO_IRQn);
					BTLE_StackTick();
					NVIC_EnableIRQ(UART_IRQn);
					NVIC_EnableIRQ(GPIO_IRQn);
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
					NVIC_DisableIRQ(UART_IRQn);
					NVIC_DisableIRQ(GPIO_IRQn);
					BTLE_StackTick();
					NVIC_EnableIRQ(UART_IRQn);
					NVIC_EnableIRQ(GPIO_IRQn);
					// Radio is busy (buffer full).
					if(Timer_Expired(&t))
						break;
				}
			}
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
