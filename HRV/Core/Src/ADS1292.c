/*
 * ADS1292.c
 *
 *  Created on: Feb 17, 2026
 *      Author: Alec Duvall
 */
#include "main.h"


void ADS1292_Init(void)
{
  // Ensure CS high before anything
  ADS_CS_HIGH();

  // If you have PWDN/RESET pins, drive them appropriately here:
  // - RESET is active low
  // - PWDN is active low
  // Datasheet has timing; start with generous delays.

  HAL_Delay(10);

  // Optional: send a simple command to confirm SPI works.
  // Example: SDATAC (stop read continuous) is commonly sent early.
  uint8_t cmd = 0x11; // SDATAC for ADS129x family (common command)
  ADS_CS_LOW();
  HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
  ADS_CS_HIGH();

  HAL_Delay(1);
}

