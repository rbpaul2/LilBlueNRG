
/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : GPIO/InputInterrupt/main.c 
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : September-2015
* Description        : Code demostrating the GPIO interrupt functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "Lil_MotionDetector.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_Examples GPIO Examples
  * @{
  */

/** @addtogroup GPIO_InputInterrupt GPIO Input Interrupt Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program code.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System initialization function */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* Reset GPIOs to default */
  GPIO_DeInit();

  /* LEDS initialization */
  SdkEvalLedInit(LED1);
  SdkEvalLedInit(LED2);
    
  /* MotionDetector initialization */
  Lil_MotionDetectorInit(MOTION_DETECTOR_PIN);
  Lil_MotionDetectorIrq(MOTION_DETECTOR_PIN, IRQ_ON_BOTH_EDGE);
  Lil_MotionDetectorInit(GPIO_Pin_13);
  Lil_MotionDetectorIrq(GPIO_Pin_13, IRQ_ON_BOTH_EDGE);
  
  /* Infinite loop */
  while(1);
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

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
