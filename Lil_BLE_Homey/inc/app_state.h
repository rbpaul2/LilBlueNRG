/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : app_state.h
* Author             : AMS - VMA, RF Application Team
* Version            : V1.0.0
* Date               : 21-Sept-2015
* Description        : Header file wich contains variable used for application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Exported variables ------------------------------------------------------- */  

#define VENT_OPEN	1
#define VENT_CLOSE	0
/** 
  * @brief  Device Role
  */      
#define MASTER_ROLE 0x00
#define SLAVE_ROLE 0x01
 
/** 
  * @brief  Discovery States
 */  
#define INIT                      0x00
#define START_DISCOVERY_PROC      0x01
#define WAIT_EVENT                0x02
#define WAIT_TIMER_EXPIRED        0x04
#define DO_DIRECT_CONNECTION_PROC 0x08
#define ENTER_DISCOVERY_MODE      0x10
#define DO_TERMINATE_GAP_PROC     0x20
#define DO_NON_DISCOVERABLE_MODE  0x40
#define DISCOVERY_ERROR           0x80
 
/** 
  * @brief  Variable which contains some flags useful for application
  */ 
extern volatile long int app_flags;
extern volatile long int char_flags;
/** 
  * @brief  Flags for application
  */ 
#define SET_CONNECTABLE           0x0100
#define CONNECTED_TO_SLAVE        0x0200
#define CONNECTED_TO_HUB		  0x0400
#define NOTIFICATIONS_ENABLED     0x0800

/* Added flags for handling TX, RX characteristics discovery */
#define START_READ_MD_CHAR_HANDLE 0x1000
#define END_READ_MD_CHAR_HANDLE   0x2000
#define START_READ_TM_CHAR_HANDLE 0x4000
#define END_READ_TM_CHAR_HANDLE   0x8000
#define START_READ_HM_CHAR_HANDLE 0x10000
#define END_READ_HM_CHAR_HANDLE   0x20000
#define START_READ_BC_CHAR_HANDLE 0x40000
#define END_READ_BC_CHAR_HANDLE   0x80000
#define START_READ_BT_CHAR_HANDLE 0x100000
#define END_READ_BT_CHAR_HANDLE   0x200000

/* GATT EVT_BLUE_GATT_TX_POOL_AVAILABLE event */
#define TX_BUFFER_FULL            0x01000000
#define POLLING					  0x02000000
#define CAL_START				  0x04000000
#define POLL_CO					  0x08000000
#define CO_STABLE				  0x10000000

/* Flags for registering CHAR_FLAG characteristic notifications */
#define START_TM_CCD		  	  0x00000001
#define END_TM_CCD		  	  	  0x00000002
#define START_MD_CCD		  	  0x00000004
#define END_MD_CCD		  	  	  0x00000008
#define START_HM_CCD		  	  0x00000010
#define END_HM_CCD		  	  	  0x00000020
#define START_BC_CCD		  	  0x00000040
#define END_BC_CCD		  	  	  0x00000080
#define RELAY_BC_CHAR			  0x00000100
#define RELAY_TM_CHAR			  0x00000200
#define RELAY_HM_CHAR			  0x00000400
#define RELAY_MD_CHAR			  0x00000800
#define UPDATE_CURR_TEMP		  0x00001000
#define TARG_TEMP_SET			  0x00002000


/* Exported macros -----------------------------------------------------------*/
#define APP_FLAG(flag) (app_flags & flag)
#define APP_FLAG_SET(flag) (app_flags |= flag)
#define APP_FLAG_CLEAR(flag) (app_flags &= ~flag)

#define CHAR_FLAG(flag) (char_flags & flag)
#define CHAR_FLAG_SET(flag) (char_flags |= flag)
#define CHAR_FLAG_CLEAR(flag) (char_flags &= ~flag)

#define CRITICAL_BLE_TICK() {\
				  	  	  	  NVIC_DisableIRQ(UART_IRQn);\
				  	  	  	  NVIC_DisableIRQ(GPIO_IRQn);\
				  	  	  	  NVIC_DisableIRQ(ADC_IRQn);\
				  	  	  	  NVIC_DisableIRQ(RTC_IRQn);\
				  	  	  	  BTLE_StackTick();\
				  	  	  	  NVIC_EnableIRQ(UART_IRQn);\
				  	  	  	  NVIC_EnableIRQ(GPIO_IRQn);\
				  	  	  	  NVIC_EnableIRQ(ADC_IRQn);\
				  	  	  	  NVIC_EnableIRQ(RTC_IRQn);\
							}



#ifdef __cplusplus
}
#endif

#endif /*__APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
