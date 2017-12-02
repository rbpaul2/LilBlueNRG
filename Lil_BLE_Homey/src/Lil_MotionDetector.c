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
#include "Lil_MotionDetector.h"

/** @defgroup SDK_EVAL_Button_Public_Functions                                 SDK EVAL Button Public Functions
 * @{
 */


/**
 * @brief  Configures Motion Detector for sampling from the specified pin
 * @param  pin Specifies the io pin to be configured, @ref GPIO_Pins_Definition
 *         This parameter can be a macro of the form GPIO_Pin_x
 * @retval None.
 */
void Lil_MotionDetectorInit(uint32_t pin)
{
  GPIO_InitType GPIO_InitStructure;

  /* Enables the BUTTON Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

  /* Configures Button pin as input */
  GPIO_InitStructure.GPIO_Pin = pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);

}


/**
 * @brief  Configures Motion Detector to trigger interrupts on the specified pin
 * @param  pin Specifies the io pin to be configured, @ref GPIO_Pins_Definition
 *         This parameter can be a macro of the form GPIO_Pin_x
 * @param  xIrq Specifies the IRQ mode to be configured, @ref SdkEvalButtonIrq
 *         This parameter can be one of following parameters:
 *         @arg IRQ_ON_RISING_EDGE: IRQ on rising edge
 *         @arg IRQ_ON_FALLING_EDGE: IRQ on falling edge
 *         @arg IRQ_ON_BOTH_EDGE: IRQ on both edges
 * @retval None.
 */
void Lil_MotionDetectorIrq(uint32_t pin, SdkEvalButtonIrq xIrq)
{  
  GPIO_EXTIConfigType GPIO_EXTIStructure;
  NVIC_InitType NVIC_InitStructure;

  /* Set the GPIO interrupt priority and enable it */
  NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configures EXTI line */
  GPIO_EXTIStructure.GPIO_Pin = pin;
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
  GPIO_EXTIStructure.GPIO_Event = xIrq;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);

  /* Clear pending interrupt */
  GPIO_ClearITPendingBit(pin);
  
  /* Enable the interrupt */
  GPIO_EXTICmd(pin, ENABLE);
}


/**
 * @brief  Returns the selected Button state.
 * @param  xButton Specifies the Button to be configured, @ref SdkEvalButton
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_1: Push1 button
 *         @arg BUTTON_2: Push2 button
 * @retval FlagStatus: error status of the button @ref FlagStatus
 *         This parameter can be: SET or RESET.
 */
FlagStatus Lil_MotionDetectorGetState(uint32_t pin)
{
  if(GPIO_ReadBit(pin))
    return SET;
  else
    return RESET;
}


/**
 * @brief  Get the pending bit state.
 * @param  xButton Specifies the Button to be configured, @ref SdkEvalButton
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_1: Push1 button
 *         @arg BUTTON_2: Push2 button
 * @retval FlagStatus: error status of the button @ref FlagStatus
 *         This parameter can be: SET or RESET.
 */
FlagStatus Lil_MotionDetectorGetITPendingBit(uint32_t pin)
{
  return GPIO_GetITPendingBit(pin);
}


/**
 * @brief  Clear the pending bit state.
 * @param  xButton Specifies the Button to be configured, @ref SdkEvalButton
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_1: Push1 button
 *         @arg BUTTON_2: Push2 button
 * @retval None
 */
void Lil_MotionDetectorClearITPendingBit(uint32_t pin)
{
  GPIO_ClearITPendingBit(pin);
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
