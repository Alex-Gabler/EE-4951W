/**
 * @file    ads1292r.c
 * @brief   ADS1292R ECG front-end driver for STM32L4 (HAL, SPI2, polling DRDY flag)
 *
 * SPI mode : CPOL=0, CPHA=1  (Mode 1)
 * Max SCLK  : 4 MHz  (BaudRatePrescaler_8 @ 32 MHz)
 * CS        : PB12, software-controlled
 * DRDY      : PA10, EXTI falling-edge → sets drdy_flag
 * START     : PA6,  held HIGH during continuous conversion
 * RESET     : PA7,  active-low pulse during init
 */

#include "ads1292r.h"
#include "main.h"
#include <stdio.h>

/* -------------------------------------------------------------------------
 * Private state
 * ------------------------------------------------------------------------- */
static volatile uint8_t drdy_flag = 0;
static float   latest_mv          = 0.0f;

/* -------------------------------------------------------------------------
 * Low-level SPI helpers
 * ------------------------------------------------------------------------- */
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;

static inline void CS_Low(void)
{
    HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_RESET);
}

static inline void CS_High(void)
{
    HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_SET);
}

static uint8_t SPI_Byte(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* -------------------------------------------------------------------------
 * Register read / write
 * ------------------------------------------------------------------------- */
static void WriteReg(uint8_t reg, uint8_t val)
{
    CS_Low();
    SPI_Byte(ADS1292R_CMD_WREG | (reg & 0x1F));
    SPI_Byte(0x00);   /* number of registers to write minus 1 */
    SPI_Byte(val);
    CS_High();
    HAL_Delay(1);
}

static uint8_t ReadReg(uint8_t reg)
{
    uint8_t val;
    CS_Low();
    SPI_Byte(ADS1292R_CMD_RREG | (reg & 0x1F));
    SPI_Byte(0x00);
    val = SPI_Byte(0x00);
    CS_High();
    return val;
}

static void SendCmd(uint8_t cmd)
{
    CS_Low();
    SPI_Byte(cmd);
    CS_High();
    HAL_Delay(1);
}

/* -------------------------------------------------------------------------
 * ADS1292R_Init
 * ------------------------------------------------------------------------- */
void ADS1292R_Init(void)
{
    /* Configure PB12 as push-pull output for CS (SPI_NSS_SOFT in HAL) */
    GPIO_InitTypeDef g = {0};
    g.Pin   = ADS_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS_CS_PORT, &g);
    CS_High();

    /* Hardware reset: pulse RESET low for ≥ 2*t_CLK then release */
    HAL_GPIO_WritePin(ADS_RESET_PORT, ADS_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ADS_RESET_PORT, ADS_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(50);   /* wait for POR sequence to complete */

    /* Stop any ongoing continuous read before configuring registers */
    SendCmd(ADS1292R_CMD_SDATAC);
    HAL_Delay(2);

    /* ---- Register configuration ----------------------------------------
     *
     * CONFIG1 (0x01)
     *   [7]   SINGLE_SHOT = 0  → continuous conversion
     *   [2:0] DR           = 001 → 500 SPS (good balance for ECG)
     *         000=125, 001=250, 010=500, 011=1k, 100=2k, 101=4k, 110=8k
     *
     * CONFIG2 (0x02)
     *   [5]   INT_TEST = 0   → no test signal
     *   [4]   TEST_FREQ = 0
     *   [3]   VREF_4V  = 0  → 2.42 V internal reference
     *   [1]   CLK_EN   = 0  → CLK output disabled (save power)
     *   [0]   INT_REFP = 1  → internal reference enabled
     *   Default 0xA0 with INT_REFP set = 0xA3 (per datasheet p.53)
     *
     * CH1SET (0x04)
     *   [7]   PD1    = 0    → channel 1 powered on
     *   [6:4] GAIN1  = 011  → PGA gain = 6
     *   [3:0] MUX1   = 0000 → normal electrode input
     *   Value: 0x30
     *
     * CH2SET (0x05)
     *   [7]   PD2    = 0    → channel 2 powered on (used for respiration)
     *   [6:4] GAIN2  = 000  → gain = 6 (same encoding)
     *   [3:0] MUX2   = 0000 → normal electrode input
     *   Value: 0x00
     *
     * RESP1 (0x09)  — disable on-chip respiration since we use MPU6050
     *   0x02 (default, respiration circuitry off)
     *
     * RESP2 (0x0A)
     *   0x03 (default)
     * -------------------------------------------------------------------- */

    WriteReg(ADS1292R_REG_CONFIG1, 0x02);   /* 500 SPS, continuous */
    WriteReg(ADS1292R_REG_CONFIG2, 0xA3);   /* int ref ON           */
    WriteReg(ADS1292R_REG_CH1SET,  0x30);   /* gain=6, normal input */
    WriteReg(ADS1292R_REG_CH2SET,  0x00);   /* gain=6, normal input */
    WriteReg(ADS1292R_REG_RESP1,   0x02);   /* resp circuit off     */
    WriteReg(ADS1292R_REG_RESP2,   0x03);

    /* Verify ID register — ADS1292R returns 0x73 */
    uint8_t id = ReadReg(ADS1292R_REG_ID);
    if (id == 0x73)
        printf("ADS1292R ID OK (0x%02X)\r\n", id);
    else
        printf("ADS1292R ID WARN: got 0x%02X, expected 0x73\r\n", id);

    /* Assert START high to begin continuous conversions, then enter RDATAC */
    HAL_GPIO_WritePin(ADS_START_PORT, ADS_START_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    SendCmd(ADS1292R_CMD_RDATAC);
    HAL_Delay(10);

    drdy_flag  = 0;
    latest_mv  = 0.0f;

    printf("ADS1292R init done\r\n");
}

/* -------------------------------------------------------------------------
 * ADS1292R_DRDY_Callback  (call from HAL_GPIO_EXTI_Callback)
 * ------------------------------------------------------------------------- */
void ADS1292R_DRDY_Callback(void)
{
    drdy_flag = 1;
}

/* -------------------------------------------------------------------------
 * ADS1292R_GetLatestMv
 * ------------------------------------------------------------------------- */
float ADS1292R_GetLatestMv(void)
{
    return latest_mv;
}

/* -------------------------------------------------------------------------
 * ADS1292R_Process
 *
 * In RDATAC mode the ADS1292R sends 9 bytes on every DRDY falling edge:
 *   Bytes 0-2  : 24-bit status word
 *   Bytes 3-5  : CH1  (24-bit, two's complement)
 *   Bytes 6-8  : CH2  (24-bit, two's complement)
 *
 * Output over UART2:
 *   t_us,ecg_mv,resp_mm,rmax,rmin\r\n
 * ------------------------------------------------------------------------- */
void ADS1292R_Process(float resp_mm, float resp_max, float resp_min)
{
    if (!drdy_flag)
        return;

    drdy_flag = 0;

    uint8_t rx[9] = {0};
    uint8_t tx[9] = {0};

    CS_Low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 9, HAL_MAX_DELAY);
    CS_High();

    /* ---- Decode CH1 (bytes 3-5) ---- */
    int32_t raw_ch1 = ((int32_t)rx[3] << 16) |
                      ((int32_t)rx[4] <<  8) |
                       (int32_t)rx[5];

    /* Sign-extend 24-bit → 32-bit */
    if (raw_ch1 & 0x800000)
        raw_ch1 |= 0xFF000000;

    latest_mv = (float)raw_ch1 * ADS1292R_LSB_MV;

    /* ---- Stream unified CSV ---- */
    uint32_t t_us = HAL_GetTick() * 1000UL;
    printf("%lu,%.4f,%.3f,%.3f,%.3f\r\n",
           t_us,
           latest_mv,
           resp_mm,
           resp_max,
           resp_min);
}
