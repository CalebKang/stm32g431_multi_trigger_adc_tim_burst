/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
#ifdef BURST_EVERY_UPDATE
  extern uint32_t tim_burst[TIM_BURST_COUNT];
  extern uint32_t tim_burst_up[TIM_BURST_COUNT];
  extern uint32_t tim_burst_dn[TIM_BURST_COUNT];
#else
  extern uint32_t tim_burst[TIM_BURST_COUNT];
  extern uint32_t tim_burst_up[TIM_BURST_COUNT];
#endif
extern __IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];             /* ADC group regular conversion data */
extern __IO uint16_t aADCxConvertedDataB[ADC_CONVERTED_DATA_BUFFER_SIZE];             /* ADC group regular conversion data */

static uint32_t dma_channel1_counter = 0;
static uint32_t dma_channel2_counter = 0;
static uint32_t tim8_up_counter = 0;


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	dma_channel1_counter++;
	
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC1(DMA1);
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_10);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_10);

#ifdef BURST_EVERY_UPDATE
		if(LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_UP)
		{
			tim_burst[0] = tim_burst_up[0];
			tim_burst[1] = tim_burst_up[1];
			tim_burst[2] = tim_burst_up[2];
		}
		else
		{
			tim_burst[0] = tim_burst_dn[0];
			tim_burst[1] = tim_burst_dn[1];
			tim_burst[2] = tim_burst_dn[2];
		}
#else
		tim_burst[0] = tim_burst_up[0];
    tim_burst[1] = tim_burst_up[1];
    tim_burst[2] = tim_burst_up[2];
    tim_burst[3] = tim_burst_up[3];
    tim_burst[4] = tim_burst_up[4];
    tim_burst[5] = tim_burst_up[5];
#endif		
#if 0
		if(LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_UP)
		{
			tim_burst[0] = tim_burst_up[0];
			tim_burst[1] = tim_burst_up[1];
			tim_burst[2] = tim_burst_up[2];
		}
		else
		{
			tim_burst[0] = tim_burst_dn[0];
			tim_burst[1] = tim_burst_dn[1];
			tim_burst[2] = tim_burst_dn[2];
		}
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  	LL_TIM_ConfigDMABurst(TIM8, LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_LENGTH_3TRANSFERS);
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, TIM_BURST_COUNT);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
#endif
  }
  
	if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
  {
  }

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	dma_channel2_counter++;
	
  if(LL_DMA_IsActiveFlag_TC2(DMA1) == 1)
  {
    /* Clear flag DMA transfer complete */
    LL_DMA_ClearFlag_TC2(DMA1);
		LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);
		LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
		
		aADCxConvertedDataB[0] = aADCxConvertedData[0];
		aADCxConvertedDataB[1] = aADCxConvertedData[1];
		aADCxConvertedDataB[2] = aADCxConvertedData[2];
	}
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */
	tim8_up_counter++;
	
  if(LL_TIM_IsActiveFlag_UPDATE(TIM8) == 1)
  {
    /* Clear the update interrupt flag */
    LL_TIM_ClearFlag_UPDATE(TIM8);
		
		if(LL_TIM_GetDirection(TIM8) == LL_TIM_COUNTERDIRECTION_UP)
		{
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
			
//			LL_mDelay(1);
		}
		else
		{
//			LL_mDelay(1);
		}
  }

  /* USER CODE END TIM8_UP_IRQn 0 */
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

  /* USER CODE END TIM8_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
