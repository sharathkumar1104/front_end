/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "variables.h"
#include "math.h"
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
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
/* USER CODE BEGIN EV */

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
   while (1)
  {
  }
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
  * @brief This function handles Pre-fetch fault, memory access fault.
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

//  PLL();
//  abc_to_dq();
//  if(check_if_PLL_locked()){
//    voltageController();
//    currentControlTriggered();
//    dq_to_pwm();
//  }

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

//  if(measured_values.Ia == 100 && measured_values.Ib == 100 && measured_values.Ic == 100 &&
//		  measured_values.Va == 400 && measured_values.Vb == 400 &&  measured_values.Vc == 400  )
//  {
//	  measured_values.Ia = 0;
//	  measured_values.Ib = -10;
//	  measured_values.Ic = -100;
//	  measured_values.Va = -400;
//	  measured_values.Vb = -400;
//	  measured_values.Vc = -400;
//
//  }else if(measured_values.Ib == -10){
//	 measured_values.Ia = 0;
//	 measured_values.Ib = 50;
//     measured_values.Ic = 100;
//	 measured_values.Va = 400;
//	 measured_values.Vb = 400;
//	 measured_values.Vc = 400;
//  }else{
//	  measured_values.Ia = 100;
//	 	 measured_values.Ib = 100;
//	      measured_values.Ic = 100;
//	 	 measured_values.Va = 400;
//	 	 measured_values.Vb = 400;
//	 	 measured_values.Vc = 400;
//  }




  grid.Va = va_array[timer3_variables.sample_index];
  grid.Vb = vb_array[timer3_variables.sample_index];
  grid.Vc = vc_array[timer3_variables.sample_index];

  grid.Ia = ia_array[timer3_variables.sample_index];
  grid.Ib = ib_array[timer3_variables.sample_index];
  grid.Ic = ic_array[timer3_variables.sample_index];

  // for instantaneous values
  instVal.Va_inst = va_array[timer3_variables.sample_index];
  instVal.Vb_inst = vb_array[timer3_variables.sample_index];
  instVal.Vc_inst = vc_array[timer3_variables.sample_index];

  instVal.Ia_inst = ia_array[timer3_variables.sample_index];
  instVal.Ib_inst = ib_array[timer3_variables.sample_index];
  instVal.Ic_inst = ic_array[timer3_variables.sample_index];

  instVal.Vdc_inst = 400;
  instVal.Idc_inst = 100;

  // for average calculation
  avg_calculations.Vdc = 300;
  avg_calculations.Idc = 100;

  // for rms calculation
  rms_calculations.Va = va_array[timer3_variables.sample_index];
  rms_calculations.Vb = vb_array[timer3_variables.sample_index];
  rms_calculations.Vc = vc_array[timer3_variables.sample_index];

  rms_calculations.Ia = ia_array[timer3_variables.sample_index];
  rms_calculations.Ib = ib_array[timer3_variables.sample_index];
  rms_calculations.Ic = ic_array[timer3_variables.sample_index];


//
  timer3_variables.sample_index = (timer3_variables.sample_index + 1);

  if(timer3_variables.sample_index > 359){
  	timer3_variables.sample_index = 0;
  }
//
//  uint32_t dac_val = (uint32_t)((pll.Theta / (2.0f * M_PI)) * 4095.0f);
//
//  HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
//  HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  adcReadings();
  Counters();
  PLL();
  abc_to_dq();
  check_if_PLL_locked();
  rms_calculation_to_be_placed_in_interrupt();
  average_calculation_to_be_placed_in_interrupt();

  instantaneousDiagnostics();
  rmsDiagnostics();
  averageDiagnostics();
  stateControl();

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */

  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
