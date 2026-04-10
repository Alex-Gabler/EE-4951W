/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* Exported functions --------------------------------------------------------*/
void Error_Handler(void);
void I2C_Scanner(void);

/* ---- Pin definitions ------------------------------------------------------
 *
 *  Pin          | Port | Bit | Function
 *  -------------|------|-----|-----------------------------
 *  B1           | PC13 | 13  | User push-button
 *  USART_TX     | PA2  |  2  | UART2 TX → USB-Serial
 *  USART_RX     | PA3  |  3  | UART2 RX ← USB-Serial
 *  LD2          | PA5  |  5  | Green LED on Nucleo board
 *  ADS_START    | PA6  |  6  | ADS1292R START (active high)
 *  ADS_RESET    | PA7  |  7  | ADS1292R RESET (active low)
 *  TMS          | PA13 | 13  | SWD
 *  TCK          | PA14 | 14  | SWD
 *  DRDY         | PA10 | 10  | ADS1292R DRDY (falling edge)
 *  SPI2_SCK     | PB10 | 10  | ADS1292R SCLK
 *  SPI2_CS      | PB12 | 12  | ADS1292R CS  (software)
 *  SPI2_MISO    | PC2  |  2  | ADS1292R DOUT
 *  SPI2_MOSI    | PC3  |  3  | ADS1292R DIN
 *  MPU6050_INT  | PB5  |  5  | MPU6050 interrupt (polled)
 *  I2C1_SCL     | PB8  |  8  | MPU6050 SCL
 *  I2C1_SDA     | PB9  |  9  | MPU6050 SDA
 *  Heartbeat    | PC6  |  6  | Optional heartbeat LED
 * -------------------------------------------------------------------------- */

/* Nucleo on-board */
#define B1_Pin              GPIO_PIN_13
#define B1_GPIO_Port        GPIOC

#define USART_TX_Pin        GPIO_PIN_2
#define USART_TX_GPIO_Port  GPIOA

#define USART_RX_Pin        GPIO_PIN_3
#define USART_RX_GPIO_Port  GPIOA

#define LD2_Pin             GPIO_PIN_5
#define LD2_GPIO_Port       GPIOA

/* ADS1292R control lines */
#define ADS_START_Pin       GPIO_PIN_6
#define ADS_START_GPIO_Port GPIOA

#define ADS_RESET_Pin       GPIO_PIN_7
#define ADS_RESET_GPIO_Port GPIOA

#define DRDY_Pin            GPIO_PIN_10
#define DRDY_GPIO_Port      GPIOA

/* SWD */
#define TMS_Pin             GPIO_PIN_13
#define TMS_GPIO_Port       GPIOA

#define TCK_Pin             GPIO_PIN_14
#define TCK_GPIO_Port       GPIOA

/* Optional heartbeat LED (PC6 — same as original GPIO_PIN_6 on GPIOC) */
#define Heartbeat_Led_Pin       GPIO_PIN_6
#define Heartbeat_Led_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
