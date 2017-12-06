/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : Lil_BLE_Homey.c
* Author             : AMS - VMA RF Application team
* Version            : V1.0.0
* Date               : 1-December-2015
* Description        : BlueNRG-1 main file for Chat Master & Slave demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 Chat Master and Slave demo \see BLE_Chat_Master_Slave_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "chat.h"
#include "clock.h"
#include "SDK_EVAL_Config.h"
#include "Lil_MotionDetector.h"
#include "Chat_config.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define LIL_BLE_HOMEY_VERSION_STRING "0.0.1"

#ifndef DEBUG
#define DEBUG 1
#endif

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
volatile uint32_t lSystickCounter=0;
/* Private function prototypes -----------------------------------------------*/
void SdkDelayMs(volatile uint32_t lTimeMs);
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  uint8_t ret;

  /* System Init */
  SystemInit();

  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* Reset GPIOs to default */
  GPIO_DeInit();

  /* Init Clock */
  Clock_Init();

  /* LEDS initialization */
  //SdkEvalLedInit(LED1);

  /* MotionDetector initialization */
  Lil_MotionDetectorInit((uint32_t)0x00004000);
  Lil_MotionDetectorIrq((uint32_t)0x00004000, IRQ_ON_BOTH_EDGE);

  /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
  //SdkEvalComIOConfig(Process_InputData);
  SdkEvalComUartInit(UART_BAUDRATE);

  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BlueNRG-2 Lil Homey Application (version: %s)\r\n", LIL_BLE_HOMEY_VERSION_STRING);
  PRINTF("[DEBUG] \n");

  /* Init Chat Device */
  ret = Lil_BLE_HomeyInit();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Lil_BLE_HomeyInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  PRINTF("BLE Stack Initialized & Device Configured\r\n");

  while(1) {
    /* Disable UART IRQ to avoid calling BLE stack functions while BTLE_StackTick() is running. */
    NVIC_DisableIRQ(UART_IRQn);
    NVIC_DisableIRQ(GPIO_IRQn);
    /* BlueNRG-1 stack tick */
    BTLE_StackTick();
    NVIC_EnableIRQ(UART_IRQn);
    NVIC_EnableIRQ(GPIO_IRQn);
    /* Application tick */
    APP_Tick();
  }
}

/**
* @brief  Delay function
* @param  Delay in ms
* @retval None
*/
void SdkDelayMs(volatile uint32_t lTimeMs)
{
  uint32_t nWaitPeriod = ~lSystickCounter;

  if(nWaitPeriod<lTimeMs)
  {
    while( lSystickCounter != 0xFFFFFFFF);
    nWaitPeriod = lTimeMs-nWaitPeriod;
  }
  else
    nWaitPeriod = lTimeMs+ ~nWaitPeriod;

  while( lSystickCounter != nWaitPeriod ) ;

}


#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
