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
#include "gp_timer.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "chat.h"
#include "clock.h"
#include "SDK_EVAL_Config.h"
#include "Lil_MotionDetector.h"
#include "Lil_Motor.h"
#include "Chat_config.h"
#include "BlueNRG_x_device.h"
#include "app_state.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define LIL_BLE_HOMEY_VERSION_STRING "0.0.1"
#define RTC_PERIOD_30s		(983010) //30 seconds



#define MOTOR_POS_PIN				GPIO_Pin_2
#define MOTOR_NEG_PIN				GPIO_Pin_0

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
ADC_InitType xADC_InitType;
uint8_t ventState;

/* Private function prototypes -----------------------------------------------*/
void SdkDelayMs(volatile uint32_t lTimeMs);
void RTC_Configuration(void);
void ADC_Configuration(void);
void VentControl(uint8_t action);
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

  MotorInit(MOTOR_POS_PIN, MOTOR_NEG_PIN);

  /*initialize the GPIO_Pin_7 to test the RTC timer*/
  Lil_GPIOInit(GPIO_Pin_6, GPIO_Output);

  /*initialize the RTC*/
  RTC_Configuration();

  /* ADC Initialization */
  ADC_Configuration();
  ADC_Cmd(ENABLE);

  /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
  SdkEvalComIOConfig(Process_InputData);
  //SdkEvalComUartInit(UART_BAUDRATE);
  SdkEvalI2CInit(10000);

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

  VentControl(VENT_CLOSE);
  SdkDelayMs(500);
  VentControl(VENT_OPEN);
  ventState = VENT_OPEN;
  while(1) {
    /* Disable UART IRQ to avoid calling BLE stack functions while BTLE_StackTick() is running. */
	CRITICAL_BLE_TICK();
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

void VentControl(uint8_t action)
{
	NVIC_DisableIRQ(UART_IRQn);
    NVIC_DisableIRQ(GPIO_IRQn);
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_DisableIRQ(RTC_IRQn);
	if (action == VENT_OPEN)
	{
		if (ventState != VENT_OPEN)
		{
			struct timer t;
			Timer_Set(&t, 1400);
			lil_vent_open();
			while(!Timer_Expired(&t));
			lil_vent_stop();
			ventState = VENT_OPEN;
		}
	}
	else if (action == VENT_CLOSE)
	{
		if (ventState != VENT_CLOSE)
		{
			struct timer t;
			Timer_Set(&t, 1400);
			lil_vent_close();
			while(!Timer_Expired(&t));
			lil_vent_stop();
			ventState = VENT_CLOSE;
		}

	}
	NVIC_EnableIRQ(UART_IRQn);
    NVIC_EnableIRQ(GPIO_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);
}

void RTC_Configuration(void)
{
	RTC_InitType RTC_Init_struct;
	NVIC_InitType NVIC_InitStructure;

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_RTC, ENABLE);

	/* RTC configuration */
	RTC_Init_struct.RTC_operatingMode = RTC_TIMER_PERIODIC;
	RTC_Init_struct.RTC_PATTERN_SIZE = 1 - 1;
	RTC_Init_struct.RTC_TLR1 = RTC_PERIOD_30s;
	RTC_Init_struct.RTC_TLR2 = 0;
	RTC_Init_struct.RTC_PATTERN1 = 0x00;
	RTC_Init_struct.RTC_PATTERN2 = 0x00;
	RTC_Init_struct.RTC_PATTERN3 = 0x00;
	RTC_Init_struct.RTC_PATTERN4 = 0x00;
	RTC_Init(&RTC_Init_struct);

	/* Enable RTC Timer Interrupt */
	RTC_IT_Config(RTC_IT_TIMER, ENABLE);
	RTC_IT_Clear(RTC_IT_TIMER);

	//delay from datasheet
	for (volatile uint32_t i=0; i<600; i++) {
		__asm("NOP");
	}

	/* Set interrupt priority and enable it */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable RTC */
	RTC_Cmd(ENABLE);
}

void ADC_Configuration(void)
{
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);

	xADC_InitType.ADC_OSR = ADC_OSR_200;
	xADC_InitType.ADC_Input = ADC_Input_AdcPin1;
	xADC_InitType.ADC_ConversionMode = ADC_ConversionMode_Single;
	xADC_InitType.ADC_ReferenceVoltage = ADC_ReferenceVoltage_1V2;
	xADC_InitType.ADC_Attenuation = ADC_Attenuation_9dB54;

	ADC_Init(&xADC_InitType);

	//enable some kinda offset stuff
	ADC_Calibration(ENABLE);
	ADC_AutoOffsetUpdate(ENABLE);
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
