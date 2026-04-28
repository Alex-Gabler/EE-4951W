/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Dual vital-signs monitor — ADS1292R ECG + MPU6050 respiration
  *
  * Serial output (115200 baud, USART2):
  *   t_us,ecg_mv,resp_mm,resp_max,resp_min\r\n   ← every ECG sample (100 SPS)
  *   # BPM=xx.x\r\n                               ← on every detected beat
  *   # HRV SDNN=x RMSSD=x pNN50=x RR=x BEATS=x  ← HRV after 8+ beats
  *
  * UART commands (from dashboard):
  *   'r'  reset respiration peak envelope
  *   'b'  recalibrate respiration baseline
  *
  * TIM2 is configured as a free-running 1 MHz (1 µs/tick) counter for
  * accurate RR-interval timestamps used by the HRV calculation.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "MPU6050.h"
#include "ads1292r.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t t_us;
    float    mv;
    float    resp_mm;
} CombinedPacket_t;

_Static_assert(sizeof(CombinedPacket_t) == 16, "CombinedPacket_t size mismatch");
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHEST_LENGTH_CM   12.0f   /* sensor-to-sternum distance — tune per patient */
#define SAMPLE_RATE_US    20000U  /* 50 Hz respiration tick                         */
#define COMP_ALPHA        0.98f   /* complementary filter gyro weight               */
#define LP_ALPHA          0.20f   /* low-pass on angle                              */
#define BASELINE_SAMPLES  100U
#define BIAS_SAMPLES      10U
#define USB_ONLY_DIAGNOSTIC 0U
#define FAST_SENSOR_STARTUP 1U
#define ADS_PROCESS_US    10000U
#define SAMPLE_RATE_MS    20U
#define USB_BATCH_PACKETS 8U
#define USB_BATCH_MS      (SAMPLE_RATE_MS * USB_BATCH_PACKETS)
#define ADS_PROCESS_MS    160U
#define STREAM_CAL_SAMPLES 100U
#define USB_PACKET_MAGIC  0x31565248U /* "HRV1", little-endian */
#define ENABLE_ADS1292R   1U
#define ENABLE_MPU6050    1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern Struct_MPU6050 MPU6050;
extern USBD_HandleTypeDef hUsbDeviceFS;

#if !FAST_SENSOR_STARTUP
static float gyro_bias_x = 0.0f;
#endif
static float gyro_bias_y = 0.0f;
#if !FAST_SENSOR_STARTUP
static float gyro_bias_z = 0.0f;
#endif

static float baselineAngle = 0.0f;
static float fusedAngle    = 0.0f;
static float lpFiltered    = 0.0f;

static float peakMax = -9999.0f;
static float peakMin =  9999.0f;
static float displacement_mm = 0.0f;

static uint32_t prevTime       = 0;
static uint32_t lastSampleTime = 0;
static uint32_t lastAdsProcessTime = 0;
static uint32_t prevSampleMs = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastAdsProcessMs = 0;
static uint32_t streamCalCount = 0;
static float streamCalPitchSum = 0.0f;
static float streamCalGyroYSum = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
#if !FAST_SENSOR_STARTUP
static void MeasureBaseline(void);
#endif
static void ResetPeaks(void);
static void HandleUARTCommands(void);
static uint8_t SendCombinedUSBBatch(uint32_t newest_ms);
static void UpdateSample(uint32_t sample_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void delay_us_tim2(uint32_t us)
{
    uint32_t t0 = TIM2->CNT;
    while ((TIM2->CNT - t0) < us) {}
}

#if !FAST_SENSOR_STARTUP
static void MeasureBaseline(void)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < BASELINE_SAMPLES; i++) {
        MPU6050_ProcessData(&MPU6050);
        sum += MPU6050_GetPitchDeg(&MPU6050);
        delay_us_tim2(5000U);
    }
    baselineAngle = sum / (float)BASELINE_SAMPLES;
    fusedAngle    = baselineAngle;
    lpFiltered    = 0.0f;
}
#endif

static void ResetPeaks(void)
{
    peakMax = -9999.0f;
    peakMin =  9999.0f;
}

static void HandleUARTCommands(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
        if (ch == 'r') ResetPeaks();
    }
}

static void UpdateSample(uint32_t sample_ms)
{
#if ENABLE_MPU6050
    MPU6050_ProcessData(&MPU6050);

    float accelAngle = MPU6050_GetPitchDeg(&MPU6050);
    float gyroRate = MPU6050.gyro_y - gyro_bias_y;

    if (streamCalCount < STREAM_CAL_SAMPLES) {
        streamCalPitchSum += accelAngle;
        streamCalGyroYSum += MPU6050.gyro_y;
        streamCalCount++;

        if (streamCalCount == STREAM_CAL_SAMPLES) {
            baselineAngle = streamCalPitchSum / (float)STREAM_CAL_SAMPLES;
            gyro_bias_y = streamCalGyroYSum / (float)STREAM_CAL_SAMPLES;
            fusedAngle = baselineAngle;
            lpFiltered = 0.0f;
            prevSampleMs = sample_ms;
            ResetPeaks();
        }

        displacement_mm = 0.0f;
        return;
    }

    float dt = (sample_ms - prevSampleMs) / 1000.0f;
    prevSampleMs = sample_ms;

    fusedAngle = COMP_ALPHA * (fusedAngle + gyroRate * dt)
               + (1.0f - COMP_ALPHA) * accelAngle;

    float relativeAngle = fusedAngle - baselineAngle;
    lpFiltered = LP_ALPHA * relativeAngle
               + (1.0f - LP_ALPHA) * lpFiltered;

    displacement_mm = 10.0f * CHEST_LENGTH_CM
                    * sinf(lpFiltered * (float)M_PI / 180.0f);

    if (displacement_mm > peakMax) peakMax = displacement_mm;
    if (displacement_mm < peakMin) peakMin = displacement_mm;
#else
    (void)sample_ms;
    displacement_mm = 0.0f;
    peakMax = 0.0f;
    peakMin = 0.0f;
#endif
}

static uint8_t SendCombinedUSBBatch(uint32_t newest_ms)
{
    static CombinedPacket_t batch[USB_BATCH_PACKETS];

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return USBD_FAIL;
    }
    if (!CDC_IsHostConnected_FS()) {
        return USBD_FAIL;
    }

    for (uint32_t i = 0; i < USB_BATCH_PACKETS; i++) {
        uint32_t sample_ms = newest_ms - ((USB_BATCH_PACKETS - 1U - i) * SAMPLE_RATE_MS);
        UpdateSample(sample_ms);
#if ENABLE_ADS1292R
        if ((sample_ms - lastAdsProcessMs) >= ADS_PROCESS_MS) {
            lastAdsProcessMs = sample_ms;
            ADS1292R_Process(displacement_mm, peakMax, peakMin);
        }
#endif
        batch[i] = (CombinedPacket_t) {
            .magic = USB_PACKET_MAGIC,
            .t_us = sample_ms * 1000U,
#if ENABLE_ADS1292R
            .mv = ADS1292R_GetLatestMv(),
#else
            .mv = 0.0f,
#endif
            .resp_mm = displacement_mm,
        };
    }

    return CDC_Transmit_FS((uint8_t *)batch, sizeof(batch));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if !USB_ONLY_DIAGNOSTIC
  MX_USART2_UART_Init();
#if ENABLE_MPU6050
  MX_I2C1_Init();
#endif
#if ENABLE_ADS1292R
  MX_SPI2_Init();
#endif
#endif
  /* USER CODE BEGIN 2 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = (SystemCoreClock / 1000000U) - 1U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;

    MX_USB_DEVICE_Init();

#if USB_ONLY_DIAGNOSTIC
    lastSampleMs = HAL_GetTick();
#else
#if ENABLE_MPU6050
    /* ---- MPU6050 init ---- */
    MPU6050_Initialization();
#if !FAST_SENSOR_STARTUP
    MPU6050_CalibrateGyro(6);
    MPU6050_CalibrateAccel(6);
    MPU6050_PrintActiveOffsets();
    delay_us_tim2(1000000U);

    printf("Measuring gyro bias...\r\n");
    MPU6050_MeasureGyroBiasXYZ(BIAS_SAMPLES, 10,
                                &gyro_bias_x, &gyro_bias_y, &gyro_bias_z);
    printf("Gyro bias: bx=%.6f by=%.6f bz=%.6f deg/s\r\n",
           gyro_bias_x, gyro_bias_y, gyro_bias_z);

    printf("Measuring respiration baseline...\r\n");
    MeasureBaseline();
#else
    MPU6050_ProcessData(&MPU6050);
    baselineAngle = MPU6050_GetPitchDeg(&MPU6050);
    fusedAngle    = baselineAngle;
    lpFiltered    = 0.0f;
#endif
#endif

    prevTime       = TIM2->CNT;
    lastSampleTime = TIM2->CNT;
    lastAdsProcessTime = TIM2->CNT;
    prevSampleMs = HAL_GetTick();
    lastSampleMs = prevSampleMs;
    lastAdsProcessMs = prevSampleMs;

#if ENABLE_ADS1292R
    /* ---- ADS1292R init (after SPI2 + GPIO + TIM2) ---- */
    ADS1292R_Init();
#endif
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
#if USB_ONLY_DIAGNOSTIC
        uint32_t nowMs = HAL_GetTick();
        if ((nowMs - lastSampleMs) >= USB_BATCH_MS) {
            lastSampleMs = nowMs;
            (void)SendCombinedUSBBatch(nowMs);
        }
#else
        HandleUARTCommands();

        /* ---- Respiration tick @ 50 Hz ---- */
        uint32_t nowMs = HAL_GetTick();
        if ((nowMs - lastSampleMs) >= USB_BATCH_MS) {
            lastSampleMs = nowMs;
            (void)SendCombinedUSBBatch(nowMs);
        }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|ADS_START_Pin|ADS_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Heartbeat_Led_GPIO_Port, Heartbeat_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin ADS_START_Pin ADS_RESET_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|ADS_START_Pin|ADS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Heartbeat_Led_Pin */
  GPIO_InitStruct.Pin = Heartbeat_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Heartbeat_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
