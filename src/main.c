/**
  ******************************************************************************
  * @file    main.c
  * @author  Anton Kabanov
  * @version V1.0
  * @date    25-February-2019
  * @version V1.1
  * @date    10-March-2019
  * @brief   Default main function.
  ******************************************************************************
*/
//PWM-DAC_1_250219

#include "stm32f10x.h"

/* Private variables */
__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;

int main(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef gpio_init;
	gpio_init.GPIO_Pin = GPIO_Pin_7;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_init);

	//  настраиваем LED как выход (push/pull out speed 2MHz) RM p.160 pulled up
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef led_init;
		led_init.GPIO_Pin = GPIO_Pin_13;
		led_init.GPIO_Speed = GPIO_Speed_2MHz;
		led_init.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &led_init);
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET); //Set13 и выключи

	TIM_TimeBaseInitTypeDef tim_timebase;
	tim_timebase.TIM_CounterMode = TIM_CounterMode_Up;
	tim_timebase.TIM_Prescaler = 720-1; // slowing to 50kHz
	tim_timebase.TIM_ClockDivision = TIM_CKD_DIV1; //This parameter can be a value of @ref TIM_Clock_Division_CKD
	tim_timebase.TIM_Period = 50000; // counting to 1Hz
	TIM_TimeBaseInit(TIM3, &tim_timebase);

	TIM_ICInitTypeDef tim_icinit; //Timer Input Capture Initialization
	tim_icinit.TIM_Channel = TIM_Channel_2;
	tim_icinit.TIM_ICFilter = 0x0;  // This parameter can be a number between 0x0 and 0xF
	tim_icinit.TIM_ICPolarity = TIM_ICPolarity_Rising;
	tim_icinit.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // Capture performed each time an edge is detected on the capture input
	tim_icinit.TIM_ICSelection = TIM_ICSelection_DirectTI; //TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively
	TIM_PWMIConfig(TIM3, &tim_icinit);

	NVIC_InitTypeDef nvicInit; // Nested Vector Interrupt Controller
	nvicInit.NVIC_IRQChannel = TIM3_IRQn; // TIM3 global Interrupt
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInit.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvicInit);

	//TIM_TypeDef timTypeInit;
	//timTypeInit.
	//TIM_(TIM3, &timTypeInit);

	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2); /* Select the TIM3 Input Trigger: TI2FP2 */
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset); /* Select the slave Mode: Reset Mode */
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);    /* Enable the Master/Slave Mode */
	TIM_Cmd(TIM3, ENABLE);    /* TIM enable counter */
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);   /* Enable the CC2 Interrupt Request */

	for(;;)
	{
		if ((DutyCycle >5.9) & (DutyCycle <= 6))   //(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))
		{
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		}
		else
		{
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET); //Set13 и выключи
		}
	}
} // the end of 'main' procedure

//та же функция из примера PWM Input
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2)) //проверяем, что прерывание произошло от TIM3 Capture compare 2 Interrupt Source
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); /* Clear TIM3 Capture compare 2 interrupt pending bit */
		IC2Value = TIM_GetCapture2(TIM3); /* Get the Input Capture value */

		if (IC2Value != 0)
		{
			DutyCycle = (TIM_GetCapture1(TIM3) * 100) / IC2Value; /* Duty cycle computation */
			Frequency = (SystemCoreClock/720) / IC2Value; /* Frequency computation */
		}
		else
		{
			DutyCycle = 0;
			Frequency = 0;
		}
	}
}
////Функция обработчика прерывания от таймера 3 из 2-ой лабы
//void TIM3_IRQHandler(void)
//{
////	if (TIM3->SR | TIM_SR_UIF)
////	if (TIM_GetITStatus(TIM3, TIM_IT_Update)== SET)
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update))
//	{
//		// Сбрасываем флаг переполнения таймера
////		TIM3->SR &= ~TIM_SR_UIF; //Clean UIF Flag
//		TIM_ClearFlag(TIM3, TIM_IT_Update);
//
//		// Считываем логическое состояние вывода светодиода и инвертируем состояние
////		if ( GPIOC->ODR & GPIO_ODR_ODR13 )
//		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
//		{
////			GPIOC->BSRR = GPIO_BSRR_BR13;
//			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
//		}
//		else
//		{
////			GPIOC->BSRR = GPIO_BSRR_BS13;
//			GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
//		}
//	}
//}
