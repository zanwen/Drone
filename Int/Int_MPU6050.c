#include "Int_MPU6050.h"

void Int_MPU6050_SetSampleRate(uint16_t sampleRate);
static void Int_MPU6050_SetDLPF(uint16_t maxBandwidth);

void Int_MPU6050_Init(void) {
    /* MPU复位 */
    // reset=1
    Int_MPU6050_SendByte(MPU_PWR_MGMT1_REG, 1 << 7);
    HAL_Delay(100);
    // 退出低功耗模式进入正常工作模式, sleep=0
    Int_MPU6050_SendByte(MPU_PWR_MGMT1_REG, 0x00);

    /* 采样率和低通滤波器带宽 */
    Int_MPU6050_SetSampleRate(500); // 2ms采样一次

    /* 配置加速度和角速度满量程范围 */
    // 角度 +=2000°/s
    Int_MPU6050_SendByte(MPU_GYRO_CFG_REG, 3 << 3);
    // 加速度 +-2g
    Int_MPU6050_SendByte(MPU_ACCEL_CFG_REG, 0);

    /* 停用不需要的功能：FIFO、备用的I2C总线、中断 */
    Int_MPU6050_SendByte(MPU_FIFO_EN_REG, 0);
    Int_MPU6050_SendByte(MPU_INT_EN_REG, 0);
    Int_MPU6050_SendByte(MPU_USER_CTRL_REG, 0);

    /* 测试MPU工作是否正常 */
    uint8_t devAddr;
    Int_MPU6050_ReadByte(MPU_DEVICE_ID_REG, &devAddr);
    if (devAddr != MPU_IIC_ADDR) {
        LOG_ERROR("unmatched MPU6050 device ID: 0x%02X", devAddr)
        return;
    }

    /* 时钟源配置、温度传感器停用 */
    Int_MPU6050_SendByte(MPU_PWR_MGMT1_REG, (1 << 3) | 1);
    LOG_DEBUG("Int_MPU6050_Init done")
}

void Int_MPU6050_SetSampleRate(uint16_t sampleRate) {
    // sampleRateDivider => [0, 255]
    if (sampleRate < 4) {
        sampleRate = 4;
    } else if (sampleRate > 1000) {
        sampleRate = 1000;
    }

    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // 角速度输出速率选1kHz（和加速度1kHz的固定输出速率保持一致）
    uint8_t sampleRateDivider = 1000 / sampleRate - 1;
    Int_MPU6050_SendByte(MPU_SAMPLE_RATE_REG, sampleRateDivider);
    // 采样率必须至少是信号最高频率分量（带宽上限）的两倍，才能无失真地还原信号
    Int_MPU6050_SetDLPF(sampleRate / 2);
}

// digital low pass filter configuration
static void Int_MPU6050_SetDLPF(uint16_t maxBandwidth) {
    uint8_t DLPF_CFG;
    if (maxBandwidth >= 188) {
        DLPF_CFG = 1;
    } else if (maxBandwidth >= 98) {
        DLPF_CFG = 2;
    } else if (maxBandwidth >= 42) {
        DLPF_CFG = 3;
    } else if (maxBandwidth >= 20) {
        DLPF_CFG = 4;
    } else if (maxBandwidth >= 10) {
        DLPF_CFG = 5;
    } else if (maxBandwidth >= 5) {
        DLPF_CFG = 6;
    }
    Int_MPU6050_SendByte(MPU_CFG_REG, DLPF_CFG << 0);
}

void Int_MPU6050_SendByte(uint8_t regAddr, uint8_t byte) {
    HAL_I2C_Mem_Write(&hi2c1, MPU_IIC_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &byte, 1, HAL_MAX_DELAY);
    LOG_DEBUG("Int_MPU6050_SendByte reg = %#x, byte = %#x", regAddr, byte);
}

void Int_MPU6050_SendBytes(uint8_t regAddr, uint8_t *bytes, uint16_t size) {
    HAL_I2C_Mem_Write(&hi2c1, MPU_IIC_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, bytes, size, HAL_MAX_DELAY);
}

void Int_MPU6050_ReadByte(uint8_t regAddr, uint8_t *byteBuf) {
    HAL_I2C_Mem_Read(&hi2c1, MPU_IIC_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, byteBuf, 1, HAL_MAX_DELAY);
}

void Int_MPU6050_ReadBytes(uint8_t regAddr, uint8_t *byteBuf, uint16_t size) {
    HAL_I2C_Mem_Read(&hi2c1, MPU_IIC_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, byteBuf, size, HAL_MAX_DELAY);
}

/**
 * @brief 读取加速度
 *
 * @param accelX
 * @param accelY
 * @param accelZ
 */
void Int_MPU6050_ReadAccel(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
    uint8_t buf[6];
    Int_MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG, buf, 6);
    *accelX = (buf[0] << 8) | buf[1];
    *accelY = (buf[2] << 8) | buf[3];
    *accelZ = (buf[4] << 8) | buf[5];
}

/**
 * @brief 读取角速度
 *
 * @param gyroX
 * @param gyroY
 * @param gyroZ
 */
void Int_MPU6050_ReadGyro(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
    uint8_t buf[6];
    Int_MPU6050_ReadBytes(MPU_GYRO_XOUTH_REG, buf, 6);
    *gyroX = (buf[0] << 8) | buf[1];
    *gyroY = (buf[2] << 8) | buf[3];
    *gyroZ = (buf[4] << 8) | buf[5];
}



