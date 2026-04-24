#ifndef ADS1292R_H
#define ADS1292R_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Pin mapping — matches the GPIO init in main.c
 *   CS    : PB12   (manual NSS, SPI_NSS_SOFT)
 *   DRDY  : PA10   (EXTI falling edge)
 *   START : PA6    (ADS_START_Pin)
 *   RESET : PA7    (ADS_RESET_Pin)
 * ----------------------------------------------------------------------- */
#define ADS_CS_PORT         GPIOB
#define ADS_CS_PIN          GPIO_PIN_12

#define ADS_DRDY_PORT       DRDY_GPIO_Port   /* PA10, defined in main.h */
#define ADS_DRDY_PIN        DRDY_Pin

#define ADS_START_PORT      GPIOA
#define ADS_START_PIN       ADS_START_Pin    /* PA6, defined in main.h */

#define ADS_RESET_PORT      GPIOA
#define ADS_RESET_PIN       ADS_RESET_Pin    /* PA7, defined in main.h */

/* -----------------------------------------------------------------------
 * ADS1292R Register addresses
 * ----------------------------------------------------------------------- */
#define ADS1292R_REG_ID         0x00
#define ADS1292R_REG_CONFIG1    0x01
#define ADS1292R_REG_CONFIG2    0x02
#define ADS1292R_REG_LOFF       0x03
#define ADS1292R_REG_CH1SET     0x04
#define ADS1292R_REG_CH2SET     0x05
#define ADS1292R_REG_RLDSENS    0x06
#define ADS1292R_REG_LOFFSENS   0x07
#define ADS1292R_REG_LOFFSTAT   0x08
#define ADS1292R_REG_RESP1      0x09
#define ADS1292R_REG_RESP2      0x0A
#define ADS1292R_REG_GPIO       0x0B

/* -----------------------------------------------------------------------
 * SPI opcodes
 * ----------------------------------------------------------------------- */
#define ADS1292R_CMD_WAKEUP     0x02
#define ADS1292R_CMD_STANDBY    0x04
#define ADS1292R_CMD_RESET      0x06
#define ADS1292R_CMD_START      0x08
#define ADS1292R_CMD_STOP       0x0A
#define ADS1292R_CMD_RDATAC     0x10   /* Read Data Continuously */
#define ADS1292R_CMD_SDATAC     0x11   /* Stop Read Data Continuously */
#define ADS1292R_CMD_RDATA      0x12
#define ADS1292R_CMD_RREG       0x20   /* | reg addr */
#define ADS1292R_CMD_WREG       0x40   /* | reg addr */

/* -----------------------------------------------------------------------
 * Gain setting for CH1SET (PGA_GAIN field, bits [6:4])
 * Default: gain = 6  →  full-scale = ±(VREF/6) = ±403 mV
 * ----------------------------------------------------------------------- */
#define ADS1292R_GAIN           6
#define ADS1292R_VREF_MV        2420.0f   /* internal reference (mV) */
/* LSB size in mV:  2 * VREF / GAIN / 2^23 */
#define ADS1292R_LSB_MV  ( (2.0f * ADS1292R_VREF_MV / ADS1292R_GAIN) / 8388608.0f )

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise the ADS1292R.
 *         Call once after MX_SPI2_Init() and MX_GPIO_Init().
 */
void ADS1292R_Init(void);
typedef struct {
    float   sdnn;
    float   rmssd;
    float   pnn50;
    float   rr_ms;
    uint8_t beats;
    uint8_t valid;
} HRV_Results;

void ADS1292R_Process(float resp_mm, float resp_max, float resp_min);
HRV_Results ADS1292R_GetHRV(void);
/**
 * @brief  Poll for a new sample and, when one is ready, stream it over UART.
 *         Call repeatedly from the main while(1) loop.
 *         Output format:  t_us,ecg_mv,resp_mm,rmax,rmin\r\n
 *         resp_* values are passed in from the respiration path.
 */
void ADS1292R_Process(float resp_mm, float resp_max, float resp_min);

/**
 * @brief  Returns the most recent CH1 reading in millivolts.
 *         Valid after the first DRDY interrupt fires.
 */
float ADS1292R_GetLatestMv(void);

/**
 * @brief  EXTI callback — call from HAL_GPIO_EXTI_Callback when pin == DRDY_Pin.
 *         Sets an internal flag; actual SPI read happens in ADS1292R_Process().
 */
void ADS1292R_DRDY_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS1292R_H */
