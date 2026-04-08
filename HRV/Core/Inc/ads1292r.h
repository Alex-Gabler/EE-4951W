#ifndef ADS1292R_H
#define ADS1292R_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===================== PIN MAPPING =====================
 *  CS        PB12  ADS_CS_Pin / ADS_CS_GPIO_Port
 *  START     PA6   ADS_START_Pin / ADS_START_GPIO_Port
 *  RESET     PA7   ADS_RESET_Pin / ADS_RESET_GPIO_Port
 *  DRDY      PA10  DRDY_Pin / DRDY_GPIO_Port
 *  MISO      PC2   SPI2
 *  MOSI      PC3   SPI2
 *  SCK       PB10  SPI2
 */
#define ADS_CS_Pin       GPIO_PIN_12
#define ADS_CS_GPIO_Port GPIOB

#define ADS_CMD_RESET   0x06
#define ADS_CMD_START   0x08
#define ADS_CMD_RDATAC  0x10
#define ADS_CMD_SDATAC  0x11

/* ===================== HRV RESULTS =====================
 * Populated after every new beat once at least 8 RR intervals
 * have been collected. Recalculated over a rolling 64-beat window.
 *
 *  sdnn   — std deviation of RR intervals (ms)         overall variability
 *  rmssd  — root mean square of successive diffs (ms)  beat-to-beat variability
 *  pnn50  — % of successive pairs differing > 50 ms    parasympathetic tone
 *  rr_ms  — most recent RR interval (ms)
 *  beats  — number of beats in the current window
 */
typedef struct {
    float   sdnn;
    float   rmssd;
    float   pnn50;
    float   rr_ms;
    uint8_t beats;
    uint8_t valid;   /* 1 once enough beats have been collected */
} HRV_Results;

/* ===================== PUBLIC API ===================== */
void ADS1292R_Init(void);
void ADS1292R_Process(void);

/* Read the latest HRV results (safe to call from main loop) */
HRV_Results ADS1292R_GetHRV(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS1292R_H */
