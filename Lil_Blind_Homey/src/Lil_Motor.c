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
extern volatile int Cal_FLAG;
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


void Lil_MotorEncoderIrq_CHA_Init()
{
	GPIO_EXTIConfigType GPIO_EXTIStructure;
	NVIC_InitType NVIC_InitStructure;

	printf("CH_A INIT !!!!!!!!!!!!!!!!!\n");
	/* Set the GPIO interrupt priority and enable it */
	NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MED_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configures EXTI line */
	GPIO_EXTIStructure.GPIO_Pin = CH_A;
	GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
	GPIO_EXTIStructure.GPIO_Event = IRQ_ON_BOTH_EDGE;
	GPIO_EXTIConfig(&GPIO_EXTIStructure);

	/* Clear pending interrupt */
	GPIO_ClearITPendingBit(CH_A);

	/* Enable the interrupt */
	GPIO_EXTICmd(CH_A, ENABLE);

}
/* Initialize the interrupt for the Channel B of the motor,
* the interrupt will be fired when there's an edge change on Channel B
*/
void Lil_MotorEncoderIrq_CHB_Init()
{
	printf("CH_B INIT !!!!!!!!!!!!!!!!!!!!!\n");
	GPIO_EXTIConfigType GPIO_EXTIStructure;
	NVIC_InitType NVIC_InitStructure;
	//SdkEvalLedOn(LED3);
	//SdkDelayMs(250);
	/* Set the GPIO interrupt priority and enable it */
	NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configures EXTI line */
	GPIO_EXTIStructure.GPIO_Pin = CH_B;
	GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
	GPIO_EXTIStructure.GPIO_Event = IRQ_ON_BOTH_EDGE;
	GPIO_EXTIConfig(&GPIO_EXTIStructure);

	/* Clear pending interrupt */
	GPIO_ClearITPendingBit(CH_B);

	/* Enable the interrupt */
	GPIO_EXTICmd(CH_B, ENABLE);
	//SdkEvalLedOff(LED3);
	//SdkDelayMs(250);
}


void MotorInit(uint32_t A, uint32_t B, uint32_t CH_A_IN, uint32_t CH_B_IN)
{
	//initialize GPIO PINS
	Lil_GPIOInit(A, GPIO_Output);
	Lil_GPIOInit(B, GPIO_Output);
	Lil_GPIOInit(CH_A_IN, GPIO_Input);
	Lil_GPIOInit(CH_B_IN, GPIO_Input);

	//Assign Motor Variables
	POS = A;
	NEG = B;
	CH_A = CH_A_IN;
	CH_B = CH_B_IN;
	//printf("int motor init cur_pos = %d", cur_pos);
	//Lil_MotorEncoderIrq_CHA_Init();
	Lil_MotorEncoderIrq_CHB_Init();
	//Lil_MotionDetectorIrq(CH_B, IRQ_ON_BOTH_EDGE);
	//MotorEncoder_init();
}

void blind_close()
{
	open_flag = 1;
	GPIO_WriteBit(POS, Bit_SET);
	GPIO_WriteBit(NEG, Bit_RESET);
}

void blind_open()
{
	open_flag = 0;
	GPIO_WriteBit(NEG, Bit_SET);
	GPIO_WriteBit(POS, Bit_RESET);
}
/*Control the motor basing on the given action, The action can be Motor_Close and Motor_open */
void MotorControl(unsigned percentage)
{
	int target_pos = full_pos * percentage / 100;
	if (target_pos > cur_pos)
	{
		blind_close();
		while (target_pos > cur_pos)
		{
			printf("increment cur_pos = %d\n", cur_pos);
		}
		printf("exit increment loop !\n");
	}
	else if((target_pos < cur_pos))
	{
		blind_open();
		while (target_pos < cur_pos) {
			printf("decrement cur_pos = %d\n", cur_pos);
		}
		printf("exit decrement loop !\n");
	}
	else
	{

	}

	GPIO_WriteBit(NEG, 0);
	GPIO_WriteBit(POS, 0);
	printf("after stop bits set 3333333333333333333\n");
}

/*Motor encoder interrupt handler,
* the handler will determine the current position and direction of the motor
* basing on the reading from Channel A and Channel B
*/
void Encoder_Handler() {
	FlagStatus B_VAL = Lil_GetState(CH_B);
	switch (state) {
	case state_00:
		//printf("in state_00 \n");
		if (B_VAL == SET)
		{
			state = state_11;
		}
		break;
	case state_10:
		//printf("in state_10 \n");
		state = state_00;
		break;
	case state_11:
		//printf("in state_11 \n");
		if (B_VAL == RESET)
		{
			state = state_00;
			if (open_flag == 1)
				cur_pos = cur_pos + 1;
			else
				cur_pos = cur_pos - 1;
		}
		break;
	case state_01:
		state = state_11;
		break;
	}//switch state
}

/*initialize the state machine for encoder*/
void MotorEncoder_init() {
	FlagStatus A_VAL = Lil_GetState(CH_A);
	FlagStatus B_VAL = Lil_GetState(CH_B);

	if (A_VAL == RESET && B_VAL == RESET)
	{
		state = state_00;
	}
	else if ((A_VAL == SET && B_VAL == RESET))
	{
		state = state_10;
	}
	else if ((A_VAL == SET && B_VAL == SET))
	{
		state = state_11;
	}
	else
	{
		state = state_01;
	}
}

/* Calibrate the motor to determine number of revolution for 100% open, generated by a button interrupt  */
void MotorCalibration() {
		printf("Enter Motor Calibration, CAL_FLAG = %d \n", APP_FLAG(CAL_START));

		//Fully open the blinds to it's 0% position
		MotorControl(0);
		if (Lil_GetState(GPIO_Pin_12) == SET) {
			printf("enter calibration 555555555555555555 \n");
			blind_close();
			while (De_Button() == SET)
			{
			}
			GPIO_WriteBit(NEG, Bit_RESET);
			GPIO_WriteBit(POS, Bit_RESET);
			full_pos = cur_pos;
			printf("after calibration full_pos = %d \n", full_pos);
		}
		APP_FLAG_CLEAR(CAL_START);

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
