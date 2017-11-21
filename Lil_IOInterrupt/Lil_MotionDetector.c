/**
 * @file    Lil_MotionDetector.c
 * @author  Homey Team
 * @version V1.0.0
 * @date    September 29, 2015
 * @brief   This file provides all the low level API to manage buttons for eval board.
 * @details
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "Lil_MotionDetector.h"

/**
 * @}
 */




/** @defgroup SDK_EVAL_Button_Public_Functions                                 SDK EVAL Button Public Functions
 * @{
 */


/**
 * @brief  Configures Buttons.
 * @param  xButton Specifies the Button to be configured, @ref SdkEvalButton
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_1: Push1 button
 *         @arg BUTTON_2: Push2 button
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
 * @brief  Configures buttons IRQ mode.
 * @param  xButton Specifies the Button to be configured, @ref SdkEvalButton
 *         This parameter can be one of following parameters:
 *         @arg BUTTON_1: Push1 button
 *         @arg BUTTON_2: Push2 button
 * @param  xButton Specifies the IRQ mode to be configured, @ref SdkEvalButtonIrq
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
