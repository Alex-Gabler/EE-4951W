/*
 * SPI.h
 *
 *  Created on: Feb 17, 2026
 *      Author: werew
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#define ADS_CS_GPIO_Port   GPIOB      // need to change
#define ADS_CS_Pin         GPIO_PIN_12 // change

static inline void ADS_CS_LOW(void)
{
  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET);
}

static inline void ADS_CS_HIGH(void)
{
  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);
}



#endif /* INC_SPI_H_ */
