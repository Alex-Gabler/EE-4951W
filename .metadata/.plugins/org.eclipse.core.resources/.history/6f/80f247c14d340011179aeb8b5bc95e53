/**
 * @file    ads1292r.c
 * @brief   ADS1292R ECG driver with BPM + HRV (SDNN, RMSSD, pNN50).
 *
 *  HRV is calculated over a rolling window of the last 64 RR intervals.
 *  Metrics are printed over UART after every new beat (once ≥8 beats collected):
 *
 *    # HRV SDNN=42.3 RMSSD=38.1 pNN50=31.2 RR=812 BEATS=32
 *
 *  The ecg.html viewer already passes lines starting with '#' to the log,
 *  so these will appear there automatically.
 */

#include "ads1292r.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

extern SPI_HandleTypeDef hspi2;

/* ===== CONSTANTS ===== */
#define VREF            2.42f
#define GAIN            6.0f
#define FS              500.0f
#define DECIMATE        5
#define FS_DEC          100.0f
#define BPM_REFRACTORY  40      /* samples at 100 SPS = 400 ms min between beats */

/* HRV window: keep last N RR intervals.
 * 64 beats ~= 1 minute at 60 BPM — good balance of stability vs responsiveness. */
#define HRV_WINDOW      64
#define HRV_MIN_BEATS   8       /* minimum beats before reporting HRV */

/* ===== BIQUAD ===== */
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} Biquad;

static Biquad s_bqHP, s_bqN1, s_bqN2, s_bqLP;

/* ===== ISR STATE ===== */
static volatile uint8_t s_sampleReady = 0;
static volatile uint8_t s_raw[3];

/* ===== BPM STATE ===== */
static float   s_bpm_threshold      = 0.4f;
static float   s_bpm_peak           = 0.0f;
static float   s_bpm_baseline       = 0.0f;
static uint8_t s_bpm_inPeak         = 0;
static int32_t s_bpm_lastPeakSample = 0;
static int32_t s_bpm_sampleCount    = 0;
static float   s_bpm_value          = 0.0f;
static float   s_bpm_rr_avg         = 0.0f;
static uint8_t s_decimator          = 0;

/* ===== HRV STATE ===== */
/* Circular buffer of RR intervals in milliseconds */
static float    s_rr_buf[HRV_WINDOW];
static uint8_t  s_rr_head  = 0;    /* next write position */
static uint8_t  s_rr_count = 0;    /* number of valid entries (up to HRV_WINDOW) */
static HRV_Results s_hrv   = {0};

/* Debug: print first N raw frames */
static uint8_t s_debug_frames = 20;

/* ===== SPI ===== */
static inline uint8_t spi_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * 8u;
    while (n--) { __NOP(); }
}

static inline void cs_low(void)  { HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);   }

/* ===== ADS TRANSACTIONS ===== */
static void ads_command(uint8_t cmd)
{
    cs_low();  delay_us(5);
    spi_xfer(cmd);
    delay_us(5);  cs_high();
    delay_us(20);
}

static uint8_t ads_read_reg(uint8_t reg)
{
    cs_low();  delay_us(5);
    spi_xfer(0x20 | reg);
    spi_xfer(0x00);
    uint8_t val = spi_xfer(0x00);
    delay_us(5);  cs_high();
    delay_us(20);
    return val;
}

static void ads_write_reg(uint8_t reg, uint8_t val)
{
    cs_low();  delay_us(5);
    spi_xfer(0x40 | reg);
    spi_xfer(0x00);
    spi_xfer(val);
    delay_us(5);  cs_high();
    delay_us(20);
}

/* ===== FILTERS ===== */
static void designHighPass(Biquad *bq, float fc)
{
    float w = tanf((float)M_PI * fc / FS);
    float k = 1.0f / (1.0f + w);
    bq->b0 =  k; bq->b1 = -k; bq->b2 = 0.0f;
    bq->a1 = k * (w - 1.0f); bq->a2 = 0.0f;
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

static void designNotch(Biquad *bq, float f0, float r)
{
    float w0 = 2.0f * (float)M_PI * f0 / FS;
    float c  = cosf(w0);
    bq->b0 = 1.0f; bq->b1 = -2.0f * c; bq->b2 = 1.0f;
    bq->a1 = -2.0f * r * c; bq->a2 = r * r;
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

static void designLowPass(Biquad *bq, float fc)
{
    float w  = tanf((float)M_PI * fc / FS);
    float w2 = w * w;
    float n  = 1.0f / (1.0f + sqrtf(2.0f) * w + w2);
    bq->b0 = w2 * n; bq->b1 = 2.0f * bq->b0; bq->b2 = bq->b0;
    bq->a1 = 2.0f * (w2 - 1.0f) * n;
    bq->a2 = (1.0f - sqrtf(2.0f) * w + w2) * n;
    bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

static float applyBiquad(Biquad *bq, float x0)
{
    float y0 = bq->b0*x0 + bq->b1*bq->x1 + bq->b2*bq->x2
                         - bq->a1*bq->y1  - bq->a2*bq->y2;
    bq->x2 = bq->x1; bq->x1 = x0;
    bq->y2 = bq->y1; bq->y1 = y0;
    return y0;
}

/* ===== CONVERSION ===== */
static int32_t to24Signed(uint8_t b1, uint8_t b2, uint8_t b3)
{
    int32_t v = ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | b3;
    if (v & 0x800000L) v |= (int32_t)0xFF000000L;
    return v;
}

static float toMillivolts(int32_t code)
{
    return (float)code * (2.0f * VREF) / (GAIN * 16777216.0f) * 1000.0f;
}

/* ===== HRV CALCULATION =====
 *
 *  Called after every valid R-peak with the new RR interval in ms.
 *  Stores it in the circular buffer then recomputes all three metrics
 *  over however many beats are currently in the window.
 *
 *  SDNN  = sqrt( (1/N) * sum( (RR_i - RR_mean)^2 ) )
 *  RMSSD = sqrt( (1/(N-1)) * sum( (RR_i+1 - RR_i)^2 ) )
 *  pNN50 = 100 * count(|RR_i+1 - RR_i| > 50ms) / (N-1)
 */
static void computeHRV(float rr_ms)
{
    /* Store new RR interval in circular buffer */
    s_rr_buf[s_rr_head] = rr_ms;
    s_rr_head = (s_rr_head + 1) % HRV_WINDOW;
    if (s_rr_count < HRV_WINDOW) s_rr_count++;

    s_hrv.rr_ms  = rr_ms;
    s_hrv.beats  = s_rr_count;
    s_hrv.valid  = (s_rr_count >= HRV_MIN_BEATS) ? 1 : 0;

    if (!s_hrv.valid)
        return;

    uint8_t n = s_rr_count;

    /* --- Rebuild a linear array from the circular buffer --- */
    float rr[HRV_WINDOW];
    for (uint8_t i = 0; i < n; i++) {
        /* oldest entry is at s_rr_head when buffer is full,
           or at index 0 when still filling */
        uint8_t idx = (s_rr_count < HRV_WINDOW)
                    ? i
                    : (uint8_t)((s_rr_head + i) % HRV_WINDOW);
        rr[i] = s_rr_buf[idx];
    }

    /* --- SDNN --- */
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; i++) mean += rr[i];
    mean /= n;

    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = rr[i] - mean;
        var += d * d;
    }
    s_hrv.sdnn = sqrtf(var / n);

    /* --- RMSSD and pNN50 (need at least 2 intervals) --- */
    if (n < 2) {
        s_hrv.rmssd = 0.0f;
        s_hrv.pnn50 = 0.0f;
        return;
    }

    float sum_sq_diff = 0.0f;
    uint8_t nn50_count = 0;

    for (uint8_t i = 0; i < n - 1; i++) {
        float diff = rr[i + 1] - rr[i];
        sum_sq_diff += diff * diff;
        if (fabsf(diff) > 50.0f) nn50_count++;
    }

    s_hrv.rmssd = sqrtf(sum_sq_diff / (n - 1));
    s_hrv.pnn50 = 100.0f * nn50_count / (n - 1);
}

/* ===== BPM + HRV DETECTOR ===== */
static uint8_t detectBPM(float mV)
{
    s_bpm_sampleCount++;
    s_bpm_baseline = 0.999f * s_bpm_baseline + 0.001f * mV;
    float centered = mV - s_bpm_baseline;

    if (!s_bpm_inPeak && centered > s_bpm_threshold) {
        s_bpm_inPeak = 1;
        s_bpm_peak   = centered;
    }

    if (s_bpm_inPeak) {
        if (centered > s_bpm_peak) s_bpm_peak = centered;

        if (centered < s_bpm_threshold * 0.5f) {
            s_bpm_inPeak = 0;
            int32_t rr_samples = s_bpm_sampleCount - s_bpm_lastPeakSample;

            if (rr_samples > BPM_REFRACTORY) {
                s_bpm_lastPeakSample = s_bpm_sampleCount;

                /* RR interval in ms (at 100 SPS: 1 sample = 10 ms) */
                float rr_ms = rr_samples * (1000.0f / FS_DEC);

                /* Smooth BPM */
                s_bpm_rr_avg = (s_bpm_rr_avg == 0.0f) ? rr_ms
                             : 0.75f * s_bpm_rr_avg + 0.25f * rr_ms;
                s_bpm_value = 60000.0f / s_bpm_rr_avg;

                /* Adaptive threshold */
                s_bpm_threshold = 0.5f * s_bpm_peak;
                if (s_bpm_threshold < 0.05f) s_bpm_threshold = 0.05f;
                if (s_bpm_threshold > 2.0f)  s_bpm_threshold = 2.0f;

                /* Update HRV window with this RR interval */
                computeHRV(rr_ms);

                return 1;
            }
            s_bpm_peak = 0.0f;
        }
    }
    return 0;
}

/* ===== REGISTER VERIFY ===== */
static void verifyRegisters(void)
{
    static const struct { uint8_t reg; const char *name; uint8_t expected; } chk[] = {
        {0x01,"CONFIG1 ",0x02},{0x02,"CONFIG2 ",0xA0},{0x03,"LOFF    ",0x10},
        {0x04,"CH1SET  ",0x00},{0x05,"CH2SET  ",0x81},{0x06,"RLD_SENS",0x23},
        {0x07,"LOFF_SN ",0x00},{0x09,"RESP1   ",0x02},{0x0A,"RESP2   ",0x07},
    };
    printf("=== REGISTER VERIFICATION ===\r\n");
    uint8_t allPass = 1;
    for (uint8_t i = 0; i < sizeof(chk)/sizeof(chk[0]); i++) {
        uint8_t got = ads_read_reg(chk[i].reg);
        uint8_t pass = (got == chk[i].expected);
        if (!pass) allPass = 0;
        printf("%s  exp 0x%02X  got 0x%02X  %s\r\n",
               chk[i].name, chk[i].expected, got, pass ? "PASS" : "FAIL");
    }
    printf("%s\r\n=============================\r\n\r\n",
           allPass ? "ALL PASS" : "MISMATCH — check wiring/SPI mode");
}

/* ===== DRDY ISR ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != DRDY_Pin)
        return;

    cs_low();
    __NOP(); __NOP(); __NOP(); __NOP();

    /* 3 status bytes — discard */
    spi_xfer(0x00); spi_xfer(0x00); spi_xfer(0x00);

    /* CH1 — 3 bytes (ECG) */
    s_raw[0] = spi_xfer(0x00);
    s_raw[1] = spi_xfer(0x00);
    s_raw[2] = spi_xfer(0x00);

    /* CH2 — 3 bytes (powered down, clock out to complete frame) */
    spi_xfer(0x00); spi_xfer(0x00); spi_xfer(0x00);

    cs_high();
    s_sampleReady = 1;
}

/* ===== PUBLIC: GET HRV ===== */
HRV_Results ADS1292R_GetHRV(void)
{
    return s_hrv;
}

/* ===== PUBLIC: INIT ===== */
void ADS1292R_Init(void)
{
    /* Reconfigure PB12 from SPI2_NSS AF to plain GPIO output */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = ADS_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ADS_CS_GPIO_Port, &GPIO_InitStruct);

    cs_high();
    HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_SET);

    /* Hardware reset */
    HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    ads_command(ADS_CMD_RESET);  HAL_Delay(20);
    ads_command(ADS_CMD_SDATAC); HAL_Delay(10);

    uint8_t id = ads_read_reg(0x00);
    printf("=== CHIP ID ===\r\nID = 0x%02X  %s\r\n\r\n", id,
           (id == 0x53 || id == 0x73) ? "PASS" : "FAIL: unexpected ID");

    ads_write_reg(0x01, 0x02); HAL_Delay(5);
    ads_write_reg(0x02, 0xA0); HAL_Delay(150);
    ads_write_reg(0x03, 0x10); HAL_Delay(5);
    ads_write_reg(0x04, 0x00); HAL_Delay(5);
    ads_write_reg(0x05, 0x81); HAL_Delay(5);
    ads_write_reg(0x06, 0x23); HAL_Delay(5);
    ads_write_reg(0x07, 0x00); HAL_Delay(5);
    ads_write_reg(0x09, 0x02); HAL_Delay(5);
    ads_write_reg(0x0A, 0x07); HAL_Delay(5);

    verifyRegisters();

    /* Clear HRV buffer */
    memset(s_rr_buf, 0, sizeof(s_rr_buf));
    s_rr_head  = 0;
    s_rr_count = 0;
    memset(&s_hrv, 0, sizeof(s_hrv));

    designHighPass(&s_bqHP, 0.67f);
    designNotch   (&s_bqN1, 60.0f, 0.88f);
    designNotch   (&s_bqN2, 60.0f, 0.96f);
    designLowPass (&s_bqLP, 40.0f);

    HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    ads_command(ADS_CMD_START);  HAL_Delay(10);
    ads_command(ADS_CMD_RDATAC); HAL_Delay(10);

    /* Poll DRDY to confirm data flow */
    printf("Waiting for DRDY pulses...\r\n");
    uint32_t drdy_count = 0;
    uint32_t timeout = HAL_GetTick() + 2000;
    while (drdy_count < 5 && HAL_GetTick() < timeout) {
        if (HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == GPIO_PIN_RESET) {
            drdy_count++;
            cs_low();
            for (int i = 0; i < 9; i++) spi_xfer(0x00);
            cs_high();
            HAL_Delay(2);
        }
    }

    if (drdy_count >= 5)
        printf("DRDY OK — %lu pulses detected\r\n\r\n", drdy_count);
    else
        printf("WARNING: DRDY not toggling\r\n\r\n");

    printf("Ready — ECG_mV | # BPM | # HRV on UART\r\nECG_mV\r\n");
}

/* ===== PUBLIC: PROCESS ===== */
void ADS1292R_Process(void)
{
    if (!s_sampleReady)
        return;

    __disable_irq();
    uint8_t b1 = s_raw[0];
    uint8_t b2 = s_raw[1];
    uint8_t b3 = s_raw[2];
    s_sampleReady = 0;
    __enable_irq();

    int32_t code = to24Signed(b1, b2, b3);
    float   mV   = toMillivolts(code);

    /* Debug hex dump for first 20 frames */
    if (s_debug_frames > 0) {
        s_debug_frames--;
        printf("# RAW %02X %02X %02X  code=%ld  mV=%.4f\r\n",
               b1, b2, b3, (long)code, mV);
    }

    /* Filter chain */
    mV = applyBiquad(&s_bqHP, mV);
    mV = applyBiquad(&s_bqN1, mV);
    mV = applyBiquad(&s_bqN2, mV);
    mV = applyBiquad(&s_bqLP, mV);

    /* Decimate 500 → 100 SPS */
    if (++s_decimator < DECIMATE)
        return;
    s_decimator = 0;

    /* Send ECG sample — ecg.html reads this as the waveform */
    printf("%.4f\r\n", mV);

    /* Beat detection */
    if (detectBPM(mV)) {
        /* BPM line */
        printf("# BPM = %.1f\r\n", s_bpm_value);
        HAL_GPIO_TogglePin(Heartbeat_Led_GPIO_Port, Heartbeat_Led_Pin);

        /* HRV line — printed every beat once enough data is collected.
         * Format: # HRV SDNN=xx.x RMSSD=xx.x pNN50=xx.x RR=xxx BEATS=xx
         * ecg.html logs all lines starting with '#' automatically. */
        if (s_hrv.valid) {
            printf("# HRV SDNN=%.1f RMSSD=%.1f pNN50=%.1f RR=%.0f BEATS=%d\r\n",
                   s_hrv.sdnn,
                   s_hrv.rmssd,
                   s_hrv.pnn50,
                   s_hrv.rr_ms,
                   s_hrv.beats);
        } else {
            /* Tell the user how many more beats are needed */
            printf("# HRV collecting... %d/%d beats\r\n",
                   s_hrv.beats, HRV_MIN_BEATS);
        }
    }
}
