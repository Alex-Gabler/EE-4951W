/*
 * SPI.c
 *
 *  Created on: Feb 17, 2026
 *      Author: HRV group, Alec Duvall
 *
 */

#include "main.h"

SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;          // full-duplex
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;            // CPOL = 0
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;                // CPHA = 1  (Mode 1)
  hspi2.Init.NSS = SPI_NSS_SOFT;                        // software NSS
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // start conservative
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}



