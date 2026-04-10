/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Dual vital-signs monitor — ADS1292R ECG + MPU6050 respiration
  *
  * Serial output (115200 baud, USART2):
  *   t_us,ecg_mv,resp_mm,resp_max,resp_min\r\n
  *
  * UART commands (from dual-scope.html):
  *   'r'  reset respiration peak envelope
  *   'b'  recalibrate respiration baseline
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "ads1292r.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHEST_LENGTH_CM      12.0f   /* distance from sensor to sternum (tune to patient) */
#define SAMPLE_RATE_MS       20U     /* 50 Hz respiration sample tick                     */
#define COMP_ALPHA           0.98f   /* complementary filter weight toward gyro            */
#define LP_ALPHA             0.20f   /* low-pass smoothing on angle                        */
#define BASELINE_SAMPLES     100U    /* samples averaged for resting baseline              */
#define BIAS_SAMPLES         200U    /* samples averaged for gyro bias                     */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
extern Struct_MPU6050 MPU6050;

/* Gyro bias — measured once at startup with the sensor at rest */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

/* Respiration state */
static float baselineAngle = 0.0f;
static float fusedAngle    = 0.0f;
static float lpFiltered    = 0.0f;

static float peakMax = -9999.0f;
static float peakMin =  9999.0f;

static uint32_t prevTime       = 0;
static uint32_t lastSampleTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */
static void MeasureBaseline(void);
static void ResetPeaks(void);
static void RecalibrateBaseline(void);
static void HandleUARTCommands(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ---- Respiration helpers ------------------------------------------------ */

static void MeasureBaseline(void)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < BASELINE_SAMPLES; i++)
    {
        MPU6050_ProcessData(&MPU6050);
        sum += MPU6050_GetPitchDeg(&MPU6050);
        HAL_Delay(5);
    }
    baselineAngle = sum / (float)BASELINE_SAMPLES;
    fusedAngle    = baselineAngle;
    lpFiltered    = 0.0f;
}

static void ResetPeaks(void)
{
    peakMax = -9999.0f;
    peakMin =  9999.0f;
}

static void RecalibrateBaseline(void)
{
    MeasureBaseline();
    ResetPeaks();
    prevTime       = HAL_GetTick();
    lastSampleTime = HAL_GetTick();
}

/* ---- UART command handler ----------------------------------------------- */

static void HandleUARTCommands(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
        if      (ch == 'r') ResetPeaks();
        else if (ch == 'b') RecalibrateBaseline();
    }
}

/* ---- EXTI callback — routes DRDY to ADS1292R driver -------------------- */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DRDY_Pin)
    {
        ADS1292R_DRDY_Callback();
    }
}

/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    MX_USB_OTG_FS_PCD_Init();

    /* USER CODE BEGIN 2 */

    /* ---- MPU6050 init ---- */
    MPU6050_Initialization();
    MPU6050_CalibrateGyro(6);
    MPU6050_CalibrateAccel(6);
    MPU6050_PrintActiveOffsets();
    HAL_Delay(1000);

    printf("Measuring gyro bias...\r\n");
    MPU6050_MeasureGyroBiasXYZ(BIAS_SAMPLES, 10,
                                &gyro_bias_x,
                                &gyro_bias_y,
                                &gyro_bias_z);
    printf("Gyro bias: bx=%.6f by=%.6f bz=%.6f deg/s\r\n",
           gyro_bias_x, gyro_bias_y, gyro_bias_z);

    printf("Measuring respiration baseline...\r\n");
    MeasureBaseline();

    prevTime       = HAL_GetTick();
    lastSampleTime = HAL_GetTick();

    /* ---- ADS1292R init (must come after MX_SPI2_Init + MX_GPIO_Init) ---- */
    ADS1292R_Init();

    /* USER CODE END 2 */

    /* ---- Main loop ------------------------------------------------------- */
    /* USER CODE BEGIN WHILE */

    float displacement_mm = 0.0f;

    while (1)
    {
        /* Handle serial commands from the browser app */
        HandleUARTCommands();

        /* ---- Respiration tick (50 Hz) ------------------------------------ */
        uint32_t now = HAL_GetTick();
        if ((now - lastSampleTime) >= SAMPLE_RATE_MS)
        {
            lastSampleTime = now;

            MPU6050_ProcessData(&MPU6050);

            float dt = (now - prevTime) / 1000.0f;
            prevTime = now;

            /* Complementary filter: fuse gyro integration with accel pitch */
            float accelAngle = MPU6050_GetPitchDeg(&MPU6050);
            float gyroRate   = MPU6050.gyro_y - gyro_bias_y;

            fusedAngle = COMP_ALPHA * (fusedAngle + gyroRate * dt)
                         + (1.0f - COMP_ALPHA) * accelAngle;

            float relativeAngle = fusedAngle - baselineAngle;

            lpFiltered = LP_ALPHA * relativeAngle
                         + (1.0f - LP_ALPHA) * lpFiltered;

            displacement_mm = 10.0f * CHEST_LENGTH_CM
                              * sinf(lpFiltered * (float)M_PI / 180.0f);

            if (displacement_mm > peakMax) peakMax = displacement_mm;
            if (displacement_mm < peakMin) peakMin = displacement_mm;
        }

        /* ---- ECG tick (500 SPS via DRDY interrupt) ----------------------- */
        /*
         * ADS1292R_Process checks the drdy_flag set by HAL_GPIO_EXTI_Callback.
         * When a new sample is ready it reads 9 bytes over SPI and prints:
         *   t_us,ecg_mv,resp_mm,resp_max,resp_min
         *
         * Passing the latest respiration values means every ECG sample carries
         * a consistent snapshot of both signals — the browser overlays them.
         */
        ADS1292R_Process(displacement_mm, peakMax, peakMin);

        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* --------------------------------------------------------------------------
 * Clock, peripheral init — unchanged from your original except SPI2
 * -------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState       = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange  = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM       = 1;
    RCC_OscInitStruct.PLL.PLLN       = 16;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();

    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
    HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00B07CB4;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)                                           Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)  Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)                       Error_Handler();
}

/**
  * @brief SPI2 for ADS1292R
  *   Mode 1 (CPOL=0, CPHA=1), 8-bit, software CS, ~4 MHz
  */
static void MX_SPI2_Init(void)
{
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;       /* was 4BIT  */
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;        /* CPOL=0    */
    hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;         /* CPHA=1    */
    hspi2.Init.NSS               = SPI_NSS_SOFT;            /* manual CS */
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 32MHz/8=4MHz */
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 7;
    hspi2.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;   /* was ENABLE */
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
        Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

static void MX_USB_OTG_FS_PCD_Init(void)
{
    hpcd_USB_OTG_FS.Instance                = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints      = 6;
    hpcd_USB_OTG_FS.Init.speed              = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.phy_itface        = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable        = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable  = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable        = DISABLE;
    hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Output levels */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin | ADS_START_Pin | ADS_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Heartbeat_Led_GPIO_Port, Heartbeat_Led_Pin, GPIO_PIN_RESET);

    /* B1 user button — falling edge */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* LD2 + ADS START + ADS RESET — push-pull outputs */
    GPIO_InitStruct.Pin   = LD2_Pin | ADS_START_Pin | ADS_RESET_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Heartbeat LED */
    GPIO_InitStruct.Pin   = Heartbeat_Led_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Heartbeat_Led_GPIO_Port, &GPIO_InitStruct);

    /* PA8 — MCO */
    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DRDY (PA10) — falling-edge EXTI for ADS1292R */
    GPIO_InitStruct.Pin  = DRDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

    /* PB5 — MPU6050 INT (input, no pull) */
    GPIO_InitStruct.Pin  = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* Enable NVIC for DRDY (PA10 → EXTI15_10 line) */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
