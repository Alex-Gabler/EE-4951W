/**
 * @file    ads1292r.c
 * @brief   ADS1292R ECG driver — STM32L4 HAL, SPI2, DRDY EXTI.
 *
 * Signal chain (all at 500 SPS):
 *   Raw 24-bit → mV conversion → HP 0.67 Hz → Notch 60 Hz (×2) → LP 40 Hz
 *   → decimate ÷5 → 100 SPS output
 *
 * UART output format (matches dashboard):
 *   t_us,ecg_mv,resp_mm,rmax,rmin\r\n      ← waveform sample
 *   # BPM = xx.x\r\n                        ← on every detected beat
 *   # HRV SDNN=x RMSSD=x pNN50=x RR=x BEATS=x\r\n  ← HRV after 8+ beats
 *
 * HRV metrics (rolling window of last 64 RR intervals):
 *   SDNN  = std-dev of RR intervals (ms)
 *   RMSSD = RMS of successive RR differences (ms)
 *   pNN50 = % of successive differences > 50 ms
 */

#include "ads1292r.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ── Name aliases ──────────────────────────────────────────────────────────
 * ads1292r.h uses UPPER_CASE defines (ADS_CS_PORT / ADS_CS_PIN).
 * The driver code was written with CubeMX-style names.  Bridge them here
 * so neither the header nor main.h needs to change.
 * ────────────────────────────────────────────────────────────────────────── */
#define ADS_CS_GPIO_Port    ADS_CS_PORT
#define ADS_CS_Pin          ADS_CS_PIN
#define ADS_CMD_RESET       ADS1292R_CMD_RESET
#define ADS_CMD_SDATAC      ADS1292R_CMD_SDATAC
#define ADS_CMD_START       ADS1292R_CMD_START
#define ADS_CMD_RDATAC      ADS1292R_CMD_RDATAC

extern SPI_HandleTypeDef  hspi2;

/* =========================================================================
 * Constants
 * ========================================================================= */
#define VREF             2.42f        /* internal reference voltage (V)          */
#define GAIN             6.0f         /* PGA gain — CH1SET bits [6:4] = 000      */
#define FS               500.0f       /* ADS1292R output data rate (SPS)         */
#define DECIMATE         5            /* 500 / 5 = 100 SPS to dashboard          */
#define FS_DEC           100.0f       /* decimated sample rate                    */
#define BPM_REFRACTORY   40           /* min samples between beats @ 100 SPS     */
                                      /* = 400 ms → max detectable ~150 BPM      */

#define HRV_WINDOW       64           /* rolling RR interval buffer size          */
#define HRV_MIN_BEATS    8            /* beats needed before reporting HRV        */

/* =========================================================================
 * Biquad IIR filter
 * ========================================================================= */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2, y1, y2;
} Biquad;

static Biquad s_hp, s_n1, s_n2, s_lp;

/* =========================================================================
 * Module state
 * ========================================================================= */

/* ISR → main handoff */
static volatile uint8_t s_ready  = 0;

/* Decimation */
static uint8_t s_dec = 0;

/* BPM detection */
static float   s_thr      = 0.4f;    /* adaptive peak threshold (mV)            */
static float   s_peak     = 0.0f;    /* current peak amplitude                   */
static float   s_baseline = 0.0f;    /* slow IIR baseline tracker                */
static uint8_t s_inPeak   = 0;
static int32_t s_lastPeak = 0;       /* sample index of last confirmed R-peak    */
static int32_t s_cnt      = 0;       /* decimated sample counter                 */
static float   s_rrAvg    = 0.0f;    /* smoothed RR interval (samples)           */
static float   s_bpm      = 0.0f;

/* HRV */
static float   s_rrBuf[HRV_WINDOW];  /* circular buffer of RR intervals (ms)    */
static uint8_t s_rrHead  = 0;
static uint8_t s_rrCount = 0;
static HRV_Results s_hrv = {0};

static float    s_latest_mv = 0.0f;
static uint32_t s_latest_t_us = 0;

/* =========================================================================
 * SPI helpers
 * ========================================================================= */
static inline uint8_t spi_byte(uint8_t tx)
{
    uint8_t rx = 0;
    (void)HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 1U);
    return rx;
}

static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * 8u;
    while (n--) { __NOP(); }
}

static void delay_ms_tim2(uint32_t ms)
{
    uint32_t t0 = TIM2->CNT;
    while ((TIM2->CNT - t0) < (ms * 1000U)) {}
}

static inline void cs_lo(void) { HAL_GPIO_WritePin(ADS_CS_GPIO_Port,    ADS_CS_Pin,    GPIO_PIN_RESET); }
static inline void cs_hi(void) { HAL_GPIO_WritePin(ADS_CS_GPIO_Port,    ADS_CS_Pin,    GPIO_PIN_SET);   }

/* =========================================================================
 * ADS1292R register access
 * ========================================================================= */
static void ads_cmd(uint8_t cmd)
{
    cs_lo(); delay_us(5);
    spi_byte(cmd);
    delay_us(5); cs_hi();
    delay_us(20);
}

static uint8_t ads_rreg(uint8_t reg)
{
    cs_lo(); delay_us(5);
    spi_byte(0x20 | (reg & 0x1F));
    spi_byte(0x00);
    uint8_t v = spi_byte(0x00);
    delay_us(5); cs_hi();
    delay_us(20);
    return v;
}

static void ads_wreg(uint8_t reg, uint8_t val)
{
    cs_lo(); delay_us(5);
    spi_byte(0x40 | (reg & 0x1F));
    spi_byte(0x00);
    spi_byte(val);
    delay_us(5); cs_hi();
    delay_us(20);
}

/* =========================================================================
 * Filter design
 * ========================================================================= */

/* First-order high-pass Butterworth */
static void design_hp(Biquad *q, float fc)
{
    float w = tanf((float)M_PI * fc / FS);
    float k = 1.0f / (1.0f + w);
    q->b0 =  k;  q->b1 = -k;  q->b2 = 0.0f;
    q->a1 = k * (w - 1.0f);   q->a2 = 0.0f;
    q->x1 = q->x2 = q->y1 = q->y2 = 0.0f;
}

/* Second-order IIR notch */
static void design_notch(Biquad *q, float f0, float r)
{
    float w0 = 2.0f * (float)M_PI * f0 / FS;
    float c  = cosf(w0);
    q->b0 = 1.0f;  q->b1 = -2.0f * c;  q->b2 = 1.0f;
    q->a1 = -2.0f * r * c;              q->a2 = r * r;
    q->x1 = q->x2 = q->y1 = q->y2 = 0.0f;
}

/* Second-order low-pass Butterworth */
static void design_lp(Biquad *q, float fc)
{
    float w  = tanf((float)M_PI * fc / FS);
    float w2 = w * w;
    float n  = 1.0f / (1.0f + sqrtf(2.0f) * w + w2);
    q->b0 = w2 * n;  q->b1 = 2.0f * q->b0;  q->b2 = q->b0;
    q->a1 = 2.0f * (w2 - 1.0f) * n;
    q->a2 = (1.0f - sqrtf(2.0f) * w + w2) * n;
    q->x1 = q->x2 = q->y1 = q->y2 = 0.0f;
}

static float biquad(Biquad *q, float x0)
{
    float y0 = q->b0*x0 + q->b1*q->x1 + q->b2*q->x2
                        - q->a1*q->y1  - q->a2*q->y2;
    q->x2 = q->x1;  q->x1 = x0;
    q->y2 = q->y1;  q->y1 = y0;
    return y0;
}

/* =========================================================================
 * ADC conversion
 * ========================================================================= */
static int32_t sign_extend24(uint8_t b0, uint8_t b1, uint8_t b2)
{
    int32_t v = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    if (v & 0x800000L) v |= (int32_t)0xFF000000L;
    return v;
}

static float to_mv(int32_t code)
{
    /* mV = code × (2 × VREF) / (GAIN × 2^23) × 1000 */
    return (float)code * (2.0f * VREF * 1000.0f) / (GAIN * 8388608.0f);
}

/* =========================================================================
 * HRV computation
 *
 * Called on every confirmed R-peak with the RR interval in ms.
 * Maintains a circular buffer and recomputes SDNN, RMSSD, pNN50.
 * ========================================================================= */
static void update_hrv(float rr_ms)
{
    /* Push new interval into circular buffer */
    s_rrBuf[s_rrHead] = rr_ms;
    s_rrHead = (s_rrHead + 1) % HRV_WINDOW;
    if (s_rrCount < HRV_WINDOW) s_rrCount++;

    s_hrv.rr_ms = rr_ms;
    s_hrv.beats = s_rrCount;
    s_hrv.valid = (s_rrCount >= HRV_MIN_BEATS) ? 1 : 0;

    if (!s_hrv.valid)
        return;

    uint8_t n = s_rrCount;

    /* Linearise circular buffer: rr[0] = oldest, rr[n-1] = newest */
    float rr[HRV_WINDOW];
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (n < HRV_WINDOW)
                    ? i
                    : (uint8_t)((s_rrHead + i) % HRV_WINDOW);
        rr[i] = s_rrBuf[idx];
    }

    /* SDNN = sqrt( mean( (RRi - mean_RR)^2 ) ) */
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; i++) mean += rr[i];
    mean /= (float)n;

    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = rr[i] - mean;
        var += d * d;
    }
    s_hrv.sdnn = sqrtf(var / (float)n);

    if (n < 2) {
        s_hrv.rmssd = 0.0f;
        s_hrv.pnn50 = 0.0f;
        return;
    }

    /* RMSSD = sqrt( mean( (RRi+1 - RRi)^2 ) )
     * pNN50 = 100 × count(|diff| > 50ms) / (n-1)           */
    float   sum_sq = 0.0f;
    uint8_t nn50   = 0;
    for (uint8_t i = 0; i < n - 1; i++) {
        float d = rr[i + 1] - rr[i];
        sum_sq += d * d;
        if (fabsf(d) > 50.0f) nn50++;
    }
    s_hrv.rmssd = sqrtf(sum_sq / (float)(n - 1));
    s_hrv.pnn50 = 100.0f * (float)nn50 / (float)(n - 1);
}

/* =========================================================================
 * R-peak / BPM detector
 *
 * Runs at 100 SPS (post-decimation).
 * Returns 1 on every confirmed beat, 0 otherwise.
 * ========================================================================= */
static uint8_t detect_beat(float mv)
{
    s_cnt++;

    /* Very slow IIR baseline tracker (~0.1 Hz) */
    s_baseline = 0.999f * s_baseline + 0.001f * mv;
    float c = mv - s_baseline;

    /* Rising edge: enter peak */
    if (!s_inPeak && c > s_thr) {
        s_inPeak = 1;
        s_peak   = c;
    }

    if (s_inPeak) {
        if (c > s_peak) s_peak = c;

        /* Falling edge: confirm beat */
        if (c < s_thr * 0.5f) {
            s_inPeak = 0;
            int32_t rr_samples = s_cnt - s_lastPeak;

            if (rr_samples > BPM_REFRACTORY) {
                s_lastPeak = s_cnt;

                float rr_ms = rr_samples * (1000.0f / FS_DEC);

                /* Smoothed RR average for BPM display */
                s_rrAvg = (s_rrAvg == 0.0f) ? rr_ms
                        : 0.75f * s_rrAvg + 0.25f * rr_ms;
                s_bpm   = 60000.0f / s_rrAvg;

                /* Adaptive threshold: half the last peak height */
                s_thr = 0.5f * s_peak;
                if (s_thr < 0.05f) s_thr = 0.05f;
                if (s_thr > 2.0f)  s_thr = 2.0f;

                update_hrv(rr_ms);
                return 1;
            }
            s_peak = 0.0f;
        }
    }
    return 0;
}

/* =========================================================================
 * Register verification — called once after init
 * ========================================================================= */
static void verify_regs(void)
{
    /* Expected values after our write sequence */
    static const struct { uint8_t reg; const char *name; uint8_t exp; } tbl[] = {
        {0x01, "CONFIG1 ", 0x02},
        {0x02, "CONFIG2 ", 0xA3},   /* internal reference ON */
        {0x03, "LOFF    ", 0x10},
        {0x04, "CH1SET  ", 0x00},   /* gain=6, normal input  */
        {0x05, "CH2SET  ", 0x81},   /* powered down          */
        {0x06, "RLD_SENS", 0x23},
        {0x07, "LOFF_SN ", 0x00},
        {0x09, "RESP1   ", 0x02},
        {0x0A, "RESP2   ", 0x07},
    };
    printf("=== REGISTER VERIFY ===\r\n");
    uint8_t ok = 1;
    for (uint8_t i = 0; i < sizeof(tbl)/sizeof(tbl[0]); i++) {
        uint8_t got  = ads_rreg(tbl[i].reg);
        uint8_t pass = (got == tbl[i].exp);
        if (!pass) ok = 0;
        printf("%s exp=0x%02X got=0x%02X %s\r\n",
               tbl[i].name, tbl[i].exp, got, pass ? "PASS" : "FAIL");
    }
    printf("%s\r\n=======================\r\n\r\n",
           ok ? "ALL PASS" : "MISMATCH — check SPI mode/wiring");
}

/* =========================================================================
 * DRDY interrupt — only signal the main loop.
 *
 * Keep SPI out of EXTI context so USB and SysTick are not blocked by a
 * 500 Hz sensor interrupt.
 * ========================================================================= */
void ADS1292R_DRDY_Callback(void)
{
    s_ready = 1;
}

static void read_ads_frame(void)
{
    cs_lo();
    __NOP(); __NOP(); __NOP(); __NOP();

    /* 3 status bytes — read and discard */
    spi_byte(0x00); spi_byte(0x00); spi_byte(0x00);

    /* CH1 — ECG */
    uint8_t b0 = spi_byte(0x00);
    uint8_t b1 = spi_byte(0x00);
    uint8_t b2 = spi_byte(0x00);

    /* CH2 — powered down, clock out to complete the 9-byte frame */
    spi_byte(0x00); spi_byte(0x00); spi_byte(0x00);

    cs_hi();

    float mv = to_mv(sign_extend24(b0, b1, b2));
    mv = biquad(&s_hp, mv);
    mv = biquad(&s_n1, mv);
    mv = biquad(&s_n2, mv);
    mv = biquad(&s_lp, mv);

    s_latest_mv = mv;
    s_latest_t_us = TIM2->CNT;

    if (++s_dec < DECIMATE)
        return;
    s_dec = 0;

    if (detect_beat(mv)) {
        HAL_GPIO_TogglePin(Heartbeat_Led_GPIO_Port, Heartbeat_Led_Pin);
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

HRV_Results ADS1292R_GetHRV(void)
{
    return s_hrv;
}

float ADS1292R_GetLatestMv(void)
{
    return s_latest_mv;
}

void ADS1292R_Init(void)
{
    /* ---- CS pin: override SPI AF → plain GPIO output ---- */
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin   = ADS_CS_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ADS_CS_GPIO_Port, &g);

    /* Idle state */
    cs_hi();
    HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_SET);

    /* ---- Hardware reset ---- */
    HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_RESET);
    delay_ms_tim2(10);
    HAL_GPIO_WritePin(ADS_RESET_GPIO_Port, ADS_RESET_Pin, GPIO_PIN_SET);
    delay_ms_tim2(100);

    ads_cmd(ADS_CMD_RESET);  delay_ms_tim2(20);
    ads_cmd(ADS_CMD_SDATAC); delay_ms_tim2(10);

    /* ---- Chip ID check ---- */
    uint8_t id = ads_rreg(0x00);
    printf("=== CHIP ID ===\r\nID = 0x%02X  %s\r\n\r\n",
           id, (id == 0x53 || id == 0x73) ? "PASS" : "FAIL: unexpected");

    /* ---- Register configuration ----
     *
     * CONFIG1  0x02 : 500 SPS, continuous conversion
     * CONFIG2  0xA3 : internal 2.42 V reference ON  ← critical
     * LOFF     0x10 : lead-off comparator threshold default
     * CH1SET   0x00 : gain=6, normal electrode input
     * CH2SET   0x81 : CH2 powered down (we use MPU6050 for respiration)
     * RLD_SENS 0x23 : IN1P + IN1N drive RLD (reduces common-mode noise)
     * LOFF_SN  0x00 : lead-off sense disabled
     * RESP1    0x02 : on-chip respiration off
     * RESP2    0x07 : ADS1292R-specific default
     */
    ads_wreg(0x01, 0x02); delay_ms_tim2(5);
    ads_wreg(0x02, 0xA3); delay_ms_tim2(150);  /* wait for reference to settle */
    ads_wreg(0x03, 0x10); delay_ms_tim2(5);
    ads_wreg(0x04, 0x00); delay_ms_tim2(5);
    ads_wreg(0x05, 0x81); delay_ms_tim2(5);
    ads_wreg(0x06, 0x23); delay_ms_tim2(5);
    ads_wreg(0x07, 0x00); delay_ms_tim2(5);
    ads_wreg(0x09, 0x02); delay_ms_tim2(5);
    ads_wreg(0x0A, 0x07); delay_ms_tim2(5);

    verify_regs();

    /* ---- Clear HRV state ---- */
    memset(s_rrBuf, 0, sizeof(s_rrBuf));
    s_rrHead  = 0;
    s_rrCount = 0;
    memset(&s_hrv, 0, sizeof(s_hrv));

    /* ---- Design filter chain ----
     * HP  0.67 Hz  : removes DC + baseline wander
     * Notch 60 Hz  : mains interference (×2 for steeper null)
     * LP  40  Hz   : anti-alias before decimation, removes EMG
     */
    design_hp   (&s_hp, 0.67f);
    design_notch(&s_n1, 60.0f, 0.88f);
    design_notch(&s_n2, 60.0f, 0.96f);
    design_lp   (&s_lp, 40.0f);

    /* ---- Start conversions ---- */
    HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_SET);
    delay_ms_tim2(10);
    ads_cmd(ADS_CMD_START);  delay_ms_tim2(10);
    ads_cmd(ADS_CMD_RDATAC); delay_ms_tim2(10);

    /* ---- Start µs timer (bare register — no HAL TIM header needed) ---- */
    TIM2->CNT = 0;          /* reset counter                               */
    TIM2->CR1 |= TIM_CR1_CEN; /* start — MX_TIM2_Init already configured it */

    printf("ADS1292R ready\r\n");
    printf("Format: t_us,ecg_mv,resp_mm,rmax,rmin\r\n");
}

void ADS1292R_Process(float resp_mm, float resp_max, float resp_min)
{
    (void)resp_mm;
    (void)resp_max;
    (void)resp_min;

    read_ads_frame();
}
