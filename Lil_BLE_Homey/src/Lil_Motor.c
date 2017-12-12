/**
* @file    Lil_MotionDetector.c
* @author  Homey Team
* @version V1.0.0
* @date    September 29, 2015
* @brief   This file provides a Motion Detector interface for the OpenPIR
* @details
*
*
* <h2><center>&copy; COPYRIGHT 2017 MyLilHomey</center></h2>
*/


/* Includes ------------------------------------------------------------------*/
#include "Lil_Motor.h"
#include "app_state.h"
#include <stdio.h>
/** @defgroup SDK_EVAL_Button_Public_Functions                                 SDK EVAL Button Public Functions
* @{
*/
extern volatile int CAL_FLAG;
volatile int cur_pos = 0;
volatile int full_pos = 3000;
volatile int open_flag = 1;
static uint32_t POS, NEG, CH_A, CH_B;
enum states state = state_00;
/*for motor*/

/*initialize given GPIO pins*/
void Lil_GPIOInit(uint32_t pin, uint8_t mode)
{
	GPIO_InitType GPIO_InitStructure;

	//SdkEvalLedOn(LED1);
	//SdkDelayMs(250);
	printf("GPIO INIT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	/* Enable the GPIO Clock */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/* Configure the GPIO pin */
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	//SdkEvalLedOff(LED1);
	//SdkDelayMs(250);
	/* Put the GPIO off */
	GPIO_WriteBit(pin, Bit_RESET);
}

/*initialize the button*/
void Lil_ButtonInit(uint32_t pin)
{
	GPIO_InitType GPIO_InitStructure;

	//SdkEvalLedOn(LED1);
	//SdkDelayMs(250);
	/* Enable the GPIO Clock */
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/* Configure the GPIO pin */
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Input;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);
}



/* Initialize the interrupt for the Channel A of the motor,
* the interrupt will be fired when there's an edge change on Channel A
*/

void MotorInit(uint32_t A, uint32_t B)
{
	//initialize GPIO PINS
	Lil_GPIOInit(A, GPIO_Output);
	Lil_GPIOInit(B, GPIO_Output);

	//Assign Motor Variables
	POS = A;
	NEG = B;
}
void lil_vent_stop()
{
	GPIO_WriteBit(NEG, 0);
	GPIO_WriteBit(POS, 0);
}

void lil_vent_close()
{
	open_flag = 1;
	GPIO_WriteBit(POS, Bit_SET);
	GPIO_WriteBit(NEG, Bit_RESET);
}

void lil_vent_open()
{
	open_flag = 0;
	GPIO_WriteBit(NEG, Bit_SET);
	GPIO_WriteBit(POS, Bit_RESET);
}


/*button Debouncer*/
FlagStatus De_Button() {
	if (Lil_GetState(GPIO_Pin_12) == SET)
	{
		SdkDelayMs(5);
		if (Lil_GetState(GPIO_Pin_12) == SET)
			return SET;
		else
			return RESET;
	}
	else
	{
		return RESET;
	}
}
/*get the state of one GPIO PIN*/
FlagStatus Lil_GetState(uint32_t pin)
{
	if (GPIO_ReadBit(pin))
		return SET;
	else
		return RESET;
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
