#pragma once
#include <inttypes.h>
#include <stm32h7xx_hal_tim.h>
#include <stm32h7xx_hal_dma.h>

TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

namespace WS2812WH
{
	volatile uint8_t PWM_HI;			// PWM Code HI Log.1 period
	volatile uint8_t PWM_LO;			// PWM Code LO Log.1 period
	
	static void My_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
	{
		if(htim_pwm->Instance==TIM4)
		{
			__HAL_RCC_TIM4_CLK_ENABLE();
			
			hdma_tim4_ch1.Instance = DMA1_Stream0;
			hdma_tim4_ch1.Init.Request = DMA_REQUEST_TIM4_UP;
			hdma_tim4_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
			hdma_tim4_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
			hdma_tim4_ch1.Init.MemInc = DMA_MINC_ENABLE;
			hdma_tim4_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
			hdma_tim4_ch1.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
			hdma_tim4_ch1.Init.Mode = DMA_CIRCULAR;
			hdma_tim4_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
			hdma_tim4_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
			if(HAL_DMA_Init(&hdma_tim4_ch1) != HAL_OK)
			{
				Error_Handler();
			}
			__HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_UPDATE],hdma_tim4_ch1);		//TIM_DMA_ID_CC1
			
			hdma_tim4_ch2.Instance = DMA1_Stream1;
			hdma_tim4_ch2.Init.Request = DMA_REQUEST_TIM4_UP;
			hdma_tim4_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
			hdma_tim4_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
			hdma_tim4_ch2.Init.MemInc = DMA_MINC_ENABLE;
			hdma_tim4_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
			hdma_tim4_ch2.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
			hdma_tim4_ch2.Init.Mode = DMA_CIRCULAR;
			hdma_tim4_ch2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
			hdma_tim4_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
			if(HAL_DMA_Init(&hdma_tim4_ch2) != HAL_OK)
			{
				Error_Handler();
			}
			__HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_UPDATE],hdma_tim4_ch2);		//TIM_DMA_ID_CC2
		}
	}
	
	static void My_TIM_MspPostInit(TIM_HandleTypeDef* htim)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		
		if(htim->Instance==TIM4)
		{
			__HAL_RCC_GPIOD_CLK_ENABLE();
			
			GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
			HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}
	}
	
	static void My_TIM_Init(void)
	{
		TIM_MasterConfigTypeDef sMasterConfig = {0};
		TIM_OC_InitTypeDef sConfigOC = {0};
		
		htim4.Instance = TIM4;
		htim4.Init.Prescaler = 0;
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = 249;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

		My_TIM_PWM_MspInit(&htim4);

		if(HAL_TIM_PWM_Init(&htim4) != HAL_OK)
		{
			Error_Handler();
		}
		
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if(HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
		{
			Error_Handler();
		}
		
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			Error_Handler();
		}
		if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			Error_Handler();
		}

		//__HAL_TIM_ENABLE(&htim4);
		//TIM4->EGR = TIM_EGR_UG;
		
		My_TIM_MspPostInit(&htim4);
	}
	
	static void My_DMA_Init(void)
	{
		__HAL_RCC_DMA1_CLK_ENABLE();
		
		HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	}
	
	void Setup()
	{
		My_DMA_Init();
		My_TIM_Init();
		//DMAInit();

		//__HAL_TIM_ENABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
		//TIM4->CCMR1 |= TIM_CCMR1_OC1PE;

		//TIM4->CR1 |= TIM_CR1_URS;

		PWM_HI = 140;
		PWM_LO = 70;


		
		return;
	}


};
