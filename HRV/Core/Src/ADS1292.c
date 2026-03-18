/*
 * ADS1292.c
 *
 *  Created on: Feb 17, 2026
 *      Author: Alec Duvall
 */

#include "ADS1292.h"

static void ADS1292_CS_Low(ADS1292_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void ADS1292_CS_High(ADS1292_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

void ADS1292_HardReset(ADS1292_HandleTypeDef *dev)
{
    /* PWDN/RESET is active low */
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);

    /* Give device time after reset */
    HAL_Delay(10);
}

HAL_StatusTypeDef ADS1292_SendCommand(ADS1292_HandleTypeDef *dev, uint8_t cmd)
{
    HAL_StatusTypeDef status;

    ADS1292_CS_Low(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    ADS1292_CS_High(dev);

    /* Small delay helps command decode timing */
    HAL_Delay(1);

    return status;
}

HAL_StatusTypeDef ADS1292_ReadRegister(ADS1292_HandleTypeDef *dev, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t tx[3];
    uint8_t rx[3] = {0};

    tx[0] = ADS1292_CMD_RREG | (reg & 0x1F);
    tx[1] = 0x00;   /* read 1 register => N-1 = 0 */
    tx[2] = 0x00;   /* dummy byte to clock out data */

    ADS1292_CS_Low(dev);
    status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 3, HAL_MAX_DELAY);
    ADS1292_CS_High(dev);

    if (status == HAL_OK)
    {
        *value = rx[2];
    }

    return status;
}

HAL_StatusTypeDef ADS1292_WriteRegister(ADS1292_HandleTypeDef *dev, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t tx[3];

    tx[0] = ADS1292_CMD_WREG | (reg & 0x1F);
    tx[1] = 0x00;   /* write 1 register => N-1 = 0 */
    tx[2] = value;

    ADS1292_CS_Low(dev);
    status = HAL_SPI_Transmit(dev->hspi, tx, 3, HAL_MAX_DELAY);
    ADS1292_CS_High(dev);

    HAL_Delay(1);

    return status;
}

HAL_StatusTypeDef ADS1292_ReadID(ADS1292_HandleTypeDef *dev, uint8_t *id)
{
    HAL_StatusTypeDef status;

    /* Device wakes in RDATAC, so stop that first */
    status = ADS1292_SendCommand(dev, ADS1292_CMD_SDATAC);
    if (status != HAL_OK)
    {
        return status;
    }

    return ADS1292_ReadRegister(dev, ADS1292_REG_ID, id);
}

HAL_StatusTypeDef ADS1292_Init(ADS1292_HandleTypeDef *dev)
{
    HAL_StatusTypeDef status;

    /* Idle pin states */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);

    if (dev->start_port != NULL)
    {
        HAL_GPIO_WritePin(dev->start_port, dev->start_pin, GPIO_PIN_RESET);
    }

    ADS1292_HardReset(dev);

    status = ADS1292_SendCommand(dev, ADS1292_CMD_SDATAC);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

