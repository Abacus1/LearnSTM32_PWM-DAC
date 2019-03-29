/**
  ******************************************************************************
   * @file    TIM/PWM_Input/main.c
   * @author  MCD Application Team
   * @version V3.5.0
   * @date    08-April-2011
   * @brief   Main program body
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
   * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
   ******************************************************************************
   */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

 /** @addtogroup STM32F10x_StdPeriph_Examples
   * @{
   */

 /** @addtogroup TIM_PWM_Input
   * @{
   */

 /* Private typedef -----------------------------------------------------------*/
 /* Private define ------------------------------------------------------------*/
 /* Private macro -------------------------------------------------------------*/
 /* Private variables ---------------------------------------------------------*/
TIM_ICInitTypeDef  TIM_ICInitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

/* Private functions ---------------------------------------------------------*/

 /**
   * @brief   Main program
   * @param  None
   * @retval None
   */
int main(void)
{
   /*!< At this stage the microcontroller's clock setting is already configured,
        this is done through SystemInit() function which is called from startup
        file (startup_stm32f10x_xx.s) before to branch to application main.
        To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f10x.c file
      */

   /* System Clocks Configuration */
   RCC_Configuration();

   /* NVIC configuration */
   NVIC_Configuration();

   /* Configure the GPIO ports */
   GPIO_Configuration();

   /* TIM3 configuration: PWM Input mode ------------------------
      The external signal is connected to TIM3 CH2 pin (PA.01),
      The Rising edge is used as active edge,
      The TIM3 CCR2 is used to compute the frequency value
      The TIM3 CCR1 is used to compute the duty cycle value
   ------------------------------------------------------------ */

   TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
   TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
   TIM_ICInitStructure.TIM_ICFilter = 0x0;

   TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

   /* Select the TIM3 Input Trigger: TI2FP2 */
   TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

   /* Select the slave Mode: Reset Mode */
   TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

   /* Enable the Master/Slave Mode */
   TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);

   /* TIM enable counter */
   TIM_Cmd(TIM3, ENABLE);

   /* Enable the CC2 Interrupt Request */
   TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

   while (1);
}

 /**
   * @brief  Configures the different system clocks.
   * @param  None
   * @retval None
   */
void RCC_Configuration(void)
{
   /* TIM3 clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   /* GPIOA clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

 /**
   * @brief  Configure the GPIO Pins.
   * @param  None
   * @retval None
   */
void GPIO_Configuration(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

   /* TIM3 channel 2 pin (PA.07) configuration */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
}

 /**
   * @brief  Configure the nested vectored interrupt controller.
   * @param  None
   * @retval None
   */
void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;

   /* Enable the TIM3 global Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

#ifdef  USE_FULL_ASSERT

 /**
   * @brief  Reports the name of the source file and the source line number
   *         where the assert_param error has occurred.
   * @param  file: pointer to the source file name
   * @param  line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
   /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

   while (1)
   {}
}

#endif

 /**
   * @}
   */

 /**
   * @}
   */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
// Cut from the stm32f10x_tim.c

void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
{
   uint16_t icoppositepolarity = TIM_ICPolarity_Rising;
   uint16_t icoppositeselection = TIM_ICSelection_DirectTI;

   /* Check the parameters */
   assert_param(IS_TIM_LIST6_PERIPH(TIMx));

   /* Select the Opposite Input Polarity */
   if (TIM_ICInitStruct->TIM_ICPolarity == TIM_ICPolarity_Rising)
   {
	   icoppositepolarity = TIM_ICPolarity_Falling;
   }
   else
   {
	   icoppositepolarity = TIM_ICPolarity_Rising;
   }

   /* Select the Opposite Input */
   if (TIM_ICInitStruct->TIM_ICSelection == TIM_ICSelection_DirectTI)
   {
	   icoppositeselection = TIM_ICSelection_IndirectTI;
   }
   else
   {
	   icoppositeselection = TIM_ICSelection_DirectTI;
   }

   if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_1)
   {
	   /* TI1 Configuration */
	   TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
                TIM_ICInitStruct->TIM_ICFilter);
	   /* Set the Input Capture Prescaler value */
	   TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);

	   /* TI2 Configuration */
	   TI2_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);
	   /* Set the Input Capture Prescaler value */
	   TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
   }
   else
   {
	   /* TI2 Configuration */
	   TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
                TIM_ICInitStruct->TIM_ICFilter);
	   /* Set the Input Capture Prescaler value */
	   TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);

	   /* TI1 Configuration */
	   TI1_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);
	   /* Set the Input Capture Prescaler value */
	   TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
   }
}

/**
   * @brief  Configure the TI1 as Input.
   * @param  TIMx: where x can be 1 to 17 except 6 and 7 to select the TIM peripheral.
   * @param  TIM_ICPolarity : The Input Polarity.
   *   This parameter can be one of the following values:
   *     @arg TIM_ICPolarity_Rising
   *     @arg TIM_ICPolarity_Falling
   * @param  TIM_ICSelection: specifies the input to be used.
   *   This parameter can be one of the following values:
   *     @arg TIM_ICSelection_DirectTI: TIM Input 1 is selected to be connected to IC1.
   *     @arg TIM_ICSelection_IndirectTI: TIM Input 1 is selected to be connected to IC2.
   *     @arg TIM_ICSelection_TRC: TIM Input 1 is selected to be connected to TRC.
   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
   *   This parameter must be a value between 0x00 and 0x0F.
   * @retval None
   */
static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                        uint16_t TIM_ICFilter)
{
   uint16_t tmpccmr1 = 0, tmpccer = 0;
   /* Disable the Channel 1: Reset the CC1E Bit */
   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC1E);
   tmpccmr1 = TIMx->CCMR1;
   tmpccer = TIMx->CCER;
   /* Select the Input and set the filter */
   tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC1S)) & ((uint16_t)~((uint16_t)TIM_CCMR1_IC1F)));
   tmpccmr1 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));

   if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM2) || (TIMx == TIM3) ||
      (TIMx == TIM4) ||(TIMx == TIM5))
   {
     /* Select the Polarity and set the CC1E Bit */
     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC1P));
     tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC1E);
   }
   else
   {
     /* Select the Polarity and set the CC1E Bit */
     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC1P | TIM_CCER_CC1NP));
     tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC1E);
   }

   /* Write to TIMx CCMR1 and CCER registers */
   TIMx->CCMR1 = tmpccmr1;
   TIMx->CCER = tmpccer;
}

 /**
   * @brief  Configure the TI2 as Input.
   * @param  TIMx: where x can be 1, 2, 3, 4, 5, 8, 9, 12 or 15 to select the TIM peripheral.
   * @param  TIM_ICPolarity : The Input Polarity.
   *   This parameter can be one of the following values:
   *     @arg TIM_ICPolarity_Rising
   *     @arg TIM_ICPolarity_Falling
   * @param  TIM_ICSelection: specifies the input to be used.
   *   This parameter can be one of the following values:
   *     @arg TIM_ICSelection_DirectTI: TIM Input 2 is selected to be connected to IC2.
   *     @arg TIM_ICSelection_IndirectTI: TIM Input 2 is selected to be connected to IC1.
   *     @arg TIM_ICSelection_TRC: TIM Input 2 is selected to be connected to TRC.
   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
   *   This parameter must be a value between 0x00 and 0x0F.
   * @retval None
   */
static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                        uint16_t TIM_ICFilter)
{
   uint16_t tmpccmr1 = 0, tmpccer = 0, tmp = 0;
   /* Disable the Channel 2: Reset the CC2E Bit */
   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC2E);
   tmpccmr1 = TIMx->CCMR1;
   tmpccer = TIMx->CCER;
   tmp = (uint16_t)(TIM_ICPolarity << 4);
   /* Select the Input and set the filter */
   tmpccmr1 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR1_CC2S)) & ((uint16_t)~((uint16_t)TIM_CCMR1_IC2F)));
   tmpccmr1 |= (uint16_t)(TIM_ICFilter << 12);
   tmpccmr1 |= (uint16_t)(TIM_ICSelection << 8);

   if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM2) || (TIMx == TIM3) ||
      (TIMx == TIM4) ||(TIMx == TIM5))
   {
     /* Select the Polarity and set the CC2E Bit */
     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC2P));
     tmpccer |=  (uint16_t)(tmp | (uint16_t)TIM_CCER_CC2E);
   }
   else
   {
     /* Select the Polarity and set the CC2E Bit */
     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC2P | TIM_CCER_CC2NP));
     tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC2E);
   }

   /* Write to TIMx CCMR1 and CCER registers */
   TIMx->CCMR1 = tmpccmr1 ;
   TIMx->CCER = tmpccer;
}

 /**
   * @brief  Configure the TI3 as Input.
   * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
   * @param  TIM_ICPolarity : The Input Polarity.
   *   This parameter can be one of the following values:
   *     @arg TIM_ICPolarity_Rising
   *     @arg TIM_ICPolarity_Falling
   * @param  TIM_ICSelection: specifies the input to be used.
   *   This parameter can be one of the following values:
   *     @arg TIM_ICSelection_DirectTI: TIM Input 3 is selected to be connected to IC3.
   *     @arg TIM_ICSelection_IndirectTI: TIM Input 3 is selected to be connected to IC4.
   *     @arg TIM_ICSelection_TRC: TIM Input 3 is selected to be connected to TRC.
   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
   *   This parameter must be a value between 0x00 and 0x0F.
   * @retval None
   */
//static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
//                        uint16_t TIM_ICFilter)
//{
//   uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;
//   /* Disable the Channel 3: Reset the CC3E Bit */
//   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC3E);
//   tmpccmr2 = TIMx->CCMR2;
//   tmpccer = TIMx->CCER;
//   tmp = (uint16_t)(TIM_ICPolarity << 8);
//   /* Select the Input and set the filter */
//   tmpccmr2 &= (uint16_t)(((uint16_t)~((uint16_t)TIM_CCMR2_CC3S)) & ((uint16_t)~((uint16_t)TIM_CCMR2_IC3F)));
//   tmpccmr2 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));
//
//   if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM2) || (TIMx == TIM3) ||
//      (TIMx == TIM4) ||(TIMx == TIM5))
//   {
//     /* Select the Polarity and set the CC3E Bit */
//     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC3P));
//     tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC3E);
//   }
//   else
//   {
//     /* Select the Polarity and set the CC3E Bit */
//     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC3P | TIM_CCER_CC3NP));
//     tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC3E);
//   }
//
//   /* Write to TIMx CCMR2 and CCER registers */
//   TIMx->CCMR2 = tmpccmr2;
//   TIMx->CCER = tmpccer;
//}
//
// /**
//   * @brief  Configure the TI4 as Input.
//   * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
//   * @param  TIM_ICPolarity : The Input Polarity.
//   *   This parameter can be one of the following values:
//   *     @arg TIM_ICPolarity_Rising
//   *     @arg TIM_ICPolarity_Falling
//   * @param  TIM_ICSelection: specifies the input to be used.
//   *   This parameter can be one of the following values:
//   *     @arg TIM_ICSelection_DirectTI: TIM Input 4 is selected to be connected to IC4.
//   *     @arg TIM_ICSelection_IndirectTI: TIM Input 4 is selected to be connected to IC3.
//   *     @arg TIM_ICSelection_TRC: TIM Input 4 is selected to be connected to TRC.
//   * @param  TIM_ICFilter: Specifies the Input Capture Filter.
//   *   This parameter must be a value between 0x00 and 0x0F.
//   * @retval None
//   */
//static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
//                        uint16_t TIM_ICFilter)
//{
//   uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;
//
//    /* Disable the Channel 4: Reset the CC4E Bit */
//   TIMx->CCER &= (uint16_t)~((uint16_t)TIM_CCER_CC4E);
//   tmpccmr2 = TIMx->CCMR2;
//   tmpccer = TIMx->CCER;
//   tmp = (uint16_t)(TIM_ICPolarity << 12);
//   /* Select the Input and set the filter */
//   tmpccmr2 &= (uint16_t)((uint16_t)(~(uint16_t)TIM_CCMR2_CC4S) & ((uint16_t)~((uint16_t)TIM_CCMR2_IC4F)));
//   tmpccmr2 |= (uint16_t)(TIM_ICSelection << 8);
//   tmpccmr2 |= (uint16_t)(TIM_ICFilter << 12);
//
//   if((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM2) || (TIMx == TIM3) ||
//      (TIMx == TIM4) ||(TIMx == TIM5))
//   {
//     /* Select the Polarity and set the CC4E Bit */
//     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC4P));
//     tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC4E);
//   }
//   else
//   {
//     /* Select the Polarity and set the CC4E Bit */
//     tmpccer &= (uint16_t)~((uint16_t)(TIM_CCER_CC3P | TIM_CCER_CC4NP));
//     tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC4E);
//   }
//   /* Write to TIMx CCMR2 and CCER registers */
//   TIMx->CCMR2 = tmpccmr2;
//   TIMx->CCER = tmpccer;
//}

 /**
   * @}
   */

 /**
   * @}
   */

 /**
   * @}
   */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

