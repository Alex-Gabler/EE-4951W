/*
 * ADS1292.h
 *
 *  Created on: Mar 17, 2026
 *      Author: werew
 */

#ifndef INC_ADS1292_H_
#define INC_ADS1292_H_


#include "main.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;

    GPIO_TypeDef *start_port;
    uint16_t start_pin;
} ADS1292_HandleTypeDef;

/* Command opcodes */
#define ADS1292_CMD_WAKEUP   0x02
#define ADS1292_CMD_STANDBY  0x04
#define ADS1292_CMD_RESET    0x06
#define ADS1292_CMD_START    0x08
#define ADS1292_CMD_STOP     0x0A
#define ADS1292_CMD_RDATAC   0x10
#define ADS1292_CMD_SDATAC   0x11
#define ADS1292_CMD_RDATA    0x12

#define ADS1292_CMD_RREG     0x20
#define ADS1292_CMD_WREG     0x40

/* Registers */
#define ADS1292_REG_ID       0x00
#define ADS1292_REG_CONFIG1  0x01
#define ADS1292_REG_CONFIG2  0x02
#define ADS1292_REG_LOFF     0x03
#define ADS1292_REG_CH1SET   0x04
#define ADS1292_REG_CH2SET   0x05

HAL_StatusTypeDef ADS1292_Init(ADS1292_HandleTypeDef *dev);
HAL_StatusTypeDef ADS1292_SendCommand(ADS1292_HandleTypeDef *dev, uint8_t cmd);
HAL_StatusTypeDef ADS1292_ReadRegister(ADS1292_HandleTypeDef *dev, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ADS1292_WriteRegister(ADS1292_HandleTypeDef *dev, uint8_t reg, uint8_t value);
HAL_StatusTypeDef ADS1292_ReadID(ADS1292_HandleTypeDef *dev, uint8_t *id);
void ADS1292_HardReset(ADS1292_HandleTypeDef *dev);



#endif /* INC_ADS1292_H_ */
