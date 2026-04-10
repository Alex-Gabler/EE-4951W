/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines
  *
  * Change from original:
  *   Added EXTI15_10_IRQHandler to service ADS1292R DRDY on PA10.
  *   HAL_GPIO_EXTI_IRQHandler calls HAL_GPIO_EXTI_Callback which is
  *   defined in main.c and routes to ADS1292R_DRDY_Callback().
  ******************************************************************************
  */

#include "main.h"
#include "stm32l4xx_it.h"

/* ---- Cortex-M4 exception handlers (unchanged) -------------------------- */

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void)      {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)   {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ---- Peripheral IRQ handlers ------------------------------------------- */

/**
  * @brief  EXTI15_10 — handles PA10 (DRDY) falling edge from ADS1292R.
  *         HAL_GPIO_EXTI_IRQHandler clears the pending bit and calls
  *         HAL_GPIO_EXTI_Callback(DRDY_Pin), which is defined in main.c.
  */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(DRDY_Pin);
}
