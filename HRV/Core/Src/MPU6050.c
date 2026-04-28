#include "MPU6050.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

Struct_MPU6050 MPU6050;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_I2C_TIMEOUT_MS 2U

static void MPU_WriteWord(uint8_t reg, int16_t value);
static void MPU_ReadWord(uint8_t reg, int16_t *value);
static void MPU6050_PID(uint8_t readAddr, float kP, float kI, uint8_t loops);

static void delay_ms_tim2(uint32_t ms)
{
    uint32_t t0 = TIM2->CNT;
    while ((TIM2->CNT - t0) < (ms * 1000U)) {}
}

static void MPU_WriteWord(uint8_t reg, int16_t value)
{
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, MPU6050_I2C_TIMEOUT_MS);
}

static void MPU_ReadWord(uint8_t reg, int16_t *value)
{
    uint8_t buf[2];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, MPU6050_I2C_TIMEOUT_MS);
    *value = (int16_t)((buf[0] << 8) | buf[1]);
}

void MPU6050_GetActiveOffsets(int16_t *offsets)
{
    MPU_ReadWord(MPU6050_RA_XA_OFFS_H,     &offsets[0]);
    MPU_ReadWord(MPU6050_RA_XA_OFFS_H + 2, &offsets[1]);
    MPU_ReadWord(MPU6050_RA_XA_OFFS_H + 4, &offsets[2]);
    MPU_ReadWord(MPU6050_RA_XG_OFFS_H,     &offsets[3]);
    MPU_ReadWord(MPU6050_RA_XG_OFFS_H + 2, &offsets[4]);
    MPU_ReadWord(MPU6050_RA_XG_OFFS_H + 4, &offsets[5]);
}

static void MPU6050_PID(uint8_t readAddr, float kP, float kI, uint8_t loops)
{
    uint8_t saveAddr = (readAddr == MPU6050_RA_ACCEL_XOUT_H) ?
                       MPU6050_RA_XA_OFFS_H : MPU6050_RA_XG_OFFS_H;

    int16_t data;
    float reading;
    float error, pTerm;
    float iTerm[3] = {0};
    int16_t bitZero[3] = {0};
    uint16_t gravity = 16384; // +/-2g mode

    for (int i = 0; i < 3; i++)
    {
        MPU_ReadWord(saveAddr + i * 2, &data);
        reading = (float)data;

        if (saveAddr == MPU6050_RA_XA_OFFS_H)
        {
            bitZero[i] = data & 1;
            iTerm[i] = reading * 8.0f;
        }
        else
        {
            iTerm[i] = reading * 4.0f;
        }
    }

    for (int L = 0; L < loops; L++)
    {
        for (int c = 0; c < 100; c++)
        {
            uint32_t eSum = 0;

            for (int i = 0; i < 3; i++)
            {
                MPU_ReadWord(readAddr + i * 2, &data);
                reading = (float)data;

                if (readAddr == MPU6050_RA_ACCEL_XOUT_H && i == 2)
                    reading -= gravity;

                error = -reading;
                eSum += (uint32_t)fabsf(reading);

                pTerm = kP * error;
                iTerm[i] += (error * 0.001f) * kI;

                if (saveAddr == MPU6050_RA_XA_OFFS_H)
                {
                    data = (int16_t)roundf((pTerm + iTerm[i]) / 8.0f);
                    data = (data & 0xFFFE) | bitZero[i];
                }
                else
                {
                    data = (int16_t)roundf((pTerm + iTerm[i]) / 4.0f);
                }

                MPU_WriteWord(saveAddr + i * 2, data);
            }

            if (eSum < 100) break;
            delay_ms_tim2(1);
        }

        kP *= 0.75f;
        kI *= 0.75f;
        printf(".");
    }

    printf("\r\nCalibration done\r\n");
}

void MPU6050_CalibrateGyro(uint8_t loops)
{
    float kP = 0.3f;
    float kI = 90.0f;
    float scale = (100 - ((loops - 1) * 20)) * 0.01f;

    kP *= scale;
    kI *= scale;

    printf("Calibrating Gyro...\r\n");
    MPU6050_PID(MPU6050_RA_GYRO_XOUT_H, kP, kI, loops);
}

void MPU6050_CalibrateAccel(uint8_t loops)
{
    float kP = 0.3f;
    float kI = 20.0f;
    float scale = (100 - ((loops - 1) * 20)) * 0.01f;

    kP *= scale;
    kI *= scale;

    printf("Calibrating Accel...\r\n");
    MPU6050_PID(MPU6050_RA_ACCEL_XOUT_H, kP, kI, loops);
}

void MPU6050_WriteOffset(uint8_t reg, int16_t value)
{
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;

    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      data,
                      2,
	                  MPU6050_I2C_TIMEOUT_MS);
}

void MPU6050_ReadRaw3(uint8_t startReg, int16_t *data)
{
    uint8_t buffer[6];

    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR,
                     startReg,
                     I2C_MEMADD_SIZE_8BIT,
                     buffer,
                     6,
	                 MPU6050_I2C_TIMEOUT_MS);

    data[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    data[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    data[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
}



void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, MPU6050_I2C_TIMEOUT_MS);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, MPU6050_I2C_TIMEOUT_MS);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, MPU6050_I2C_TIMEOUT_MS);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, MPU6050_I2C_TIMEOUT_MS);
}

void MPU6050_Initialization(void)
{
    delay_ms_tim2(50);

    uint8_t who_am_i = 0;
    MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);

    if (who_am_i == 0x68)
    {
        printf("MPU6050 who_am_i = 0x%02X...OK\r\n", who_am_i);
    }
    else
    {
        printf("ERROR! MPU6050 who_am_i = 0x%02X - skipping IMU init\r\n", who_am_i);
        return;
    }

    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x80);
    delay_ms_tim2(100);

    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
    delay_ms_tim2(50);

    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39); // 200 Hz
    delay_ms_tim2(50);

    MPU6050_Writebyte(MPU6050_CONFIG, 0x02); // DLPF
    delay_ms_tim2(50);

    uint8_t FS_SCALE_GYRO = 0x0; // +/-250 deg/s
    MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO << 3);
    delay_ms_tim2(50);

    uint8_t FS_SCALE_ACC = 0x0; // +/-2g
    MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC << 3);
    delay_ms_tim2(50);

    MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);

    uint8_t INT_LEVEL = 0x0;
    uint8_t LATCH_INT_EN = 0x0;
    uint8_t INT_RD_CLEAR = 0x1;
    MPU6050_Writebyte(MPU6050_INT_PIN_CFG,
                      (INT_LEVEL << 7) | (LATCH_INT_EN << 5) | (INT_RD_CLEAR << 4));
    delay_ms_tim2(50);

    uint8_t DATA_RDY_EN = 0x1;
    MPU6050_Writebyte(MPU6050_INT_ENABLE, DATA_RDY_EN);
    delay_ms_tim2(50);

    printf("MPU6050 setting is finished\r\n");
}

void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050)
{
    uint8_t data[14];
    MPU6050_Readbytes(MPU6050_RA_ACCEL_XOUT_H, 14, data);

    mpu6050->acc_x_raw = (int16_t)((data[0] << 8) | data[1]);
    mpu6050->acc_y_raw = (int16_t)((data[2] << 8) | data[3]);
    mpu6050->acc_z_raw = (int16_t)((data[4] << 8) | data[5]);

    mpu6050->temperature_raw = (int16_t)((data[6] << 8) | data[7]);

    mpu6050->gyro_x_raw = (int16_t)((data[8] << 8) | data[9]);
    mpu6050->gyro_y_raw = (int16_t)((data[10] << 8) | data[11]);
    mpu6050->gyro_z_raw = (int16_t)((data[12] << 8) | data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
    switch (FS_SCALE_GYRO)
    {
        case 0: LSB_Sensitivity_GYRO = 131.0f; break;
        case 1: LSB_Sensitivity_GYRO = 65.5f;  break;
        case 2: LSB_Sensitivity_GYRO = 32.8f;  break;
        case 3: LSB_Sensitivity_GYRO = 16.4f;  break;
        default: LSB_Sensitivity_GYRO = 131.0f; break;
    }

    switch (FS_SCALE_ACC)
    {
        case 0: LSB_Sensitivity_ACC = 16384.0f; break;
        case 1: LSB_Sensitivity_ACC = 8192.0f;  break;
        case 2: LSB_Sensitivity_ACC = 4096.0f;  break;
        case 3: LSB_Sensitivity_ACC = 2048.0f;  break;
        default: LSB_Sensitivity_ACC = 16384.0f; break;
    }
}

void MPU6050_DataConvert(Struct_MPU6050* mpu6050)
{
    mpu6050->acc_x = mpu6050->acc_x_raw / LSB_Sensitivity_ACC;
    mpu6050->acc_y = mpu6050->acc_y_raw / LSB_Sensitivity_ACC;
    mpu6050->acc_z = mpu6050->acc_z_raw / LSB_Sensitivity_ACC;

    mpu6050->temperature = ((float)mpu6050->temperature_raw / 340.0f) + 36.53f;

    mpu6050->gyro_x = mpu6050->gyro_x_raw / LSB_Sensitivity_GYRO;
    mpu6050->gyro_y = mpu6050->gyro_y_raw / LSB_Sensitivity_GYRO;
    mpu6050->gyro_z = mpu6050->gyro_z_raw / LSB_Sensitivity_GYRO;
}

void MPU6050_PrintActiveOffsets(void)
{
    int16_t offsets[6];
    MPU6050_GetActiveOffsets(offsets);

    printf("Active Offsets:\r\n");
    printf("  Accel X: %d\r\n", offsets[0]);
    printf("  Accel Y: %d\r\n", offsets[1]);
    printf("  Accel Z: %d\r\n", offsets[2]);
    printf("  Gyro  X: %d\r\n", offsets[3]);
    printf("  Gyro  Y: %d\r\n", offsets[4]);
    printf("  Gyro  Z: %d\r\n", offsets[5]);
}

float MPU6050_GetPitchDeg(Struct_MPU6050* mpu6050)
{
    return atan2f(mpu6050->acc_x,
                  sqrtf(mpu6050->acc_y * mpu6050->acc_y +
                        mpu6050->acc_z * mpu6050->acc_z)) * 180.0f / (float)M_PI;
}

void MPU6050_MeasureGyroBiasXYZ(uint16_t samples, uint32_t delay_ms,
                                float *bx, float *by, float *bz)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        MPU6050_ProcessData(&MPU6050);
        sx += MPU6050.gyro_x;
        sy += MPU6050.gyro_y;
        sz += MPU6050.gyro_z;
        delay_ms_tim2(delay_ms);
    }

    *bx = sx / samples;
    *by = sy / samples;
    *bz = sz / samples;
}

int MPU6050_DataReady(void)
{
    return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}

void MPU6050_ProcessData(Struct_MPU6050* mpu6050)
{
    MPU6050_Get6AxisRawData(mpu6050);
    MPU6050_DataConvert(mpu6050);
}
