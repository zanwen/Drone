#include "App_Flight.h"
#include "Com_Kalman.h"
#include "Com_Logger.h"
#include "Int_LED.h"
#include "main.h"
#include "Com_IMU.h"
#include "Int_MPU6050.h"

#define BAT_VOLTAGE_THRESHOLD 4000


PilotLEDState ledState = {1000, AlwaysOn};
MPU6050Data mpu6050 = {0};
FlightAngle flightAngle = {0};

static float batteryVoltage = BAT_VOLTAGE_THRESHOLD;
static MPU6050Data mpuCalibrationBias = {0}; // MPU校准偏移量

void App_Flight_DetectBatVoltage(void) {
    // Low pass filter for battery voltage calculation
    batteryVoltage += 0.2 * (batteryAdcValue * 3300 / 4096 * 2 - batteryVoltage);
    LOG_DEBUG("batteryVoltage = %.2fV", batteryVoltage / 1000);
}

/**
 * @brief 计算MPU零偏校准偏移量mpuCalibrationOffset，在MPU初始化后执行一次
 *
 */
void App_Flight_MPUCalibrate(void) {
    uint8_t motionLessCheckCnt = 30;
    int16_t gyroBias[3] = {0};               // 记录三轴角度偏差
    int16_t lastGyro[3] = {0};               // 记录上一次三轴角度
    const int16_t biasMax = 5, biasMin = -5; // 角度偏差允许的最大最小值
    /* 在检测到姿态静止cnt次后才开始计算校准偏移量 */
    while (motionLessCheckCnt--) {
        // 在检测到mpu角速度变化量较小时则认为是一次姿态静止，否则轮询检测
        do {
            App_Flight_FetchMPUData();
            gyroBias[0] = lastGyro[0] - mpu6050.gyroX;
            gyroBias[1] = lastGyro[1] - mpu6050.gyroY;
            gyroBias[2] = lastGyro[2] - mpu6050.gyroZ;
            lastGyro[0] = mpu6050.gyroX;
            lastGyro[1] = mpu6050.gyroY;
            lastGyro[2] = mpu6050.gyroZ;
        } while (gyroBias[0] < biasMin || gyroBias[1] < biasMin || gyroBias[2] < biasMin ||
                 gyroBias[0] > biasMax || gyroBias[1] > biasMax || gyroBias[2] > biasMax);
    }

    /* 计算零偏校准偏移量 */
    // 忽略前100次（掐头去尾取平均值），取后256次的数据的平均值
    int32_t biasSum[6] = {0};
    for (uint16_t i = 0; i < 356; i++) {
        App_Flight_FetchMPUData();
        if (i >= 100) {
            biasSum[0] += mpu6050.accelX;
            biasSum[1] += mpu6050.accelY;
            biasSum[2] += mpu6050.accelZ - 16383; // 重力加速度 1g=>16383
            biasSum[3] += mpu6050.gyroX;
            biasSum[4] += mpu6050.gyroY;
            biasSum[5] += mpu6050.gyroZ;
        }
    }
    // 取平均值 >>8就是除以256
    mpuCalibrationBias.accelX = biasSum[0] >> 8;
    mpuCalibrationBias.accelY = biasSum[1] >> 8;
    mpuCalibrationBias.accelZ = biasSum[2] >> 8;
    mpuCalibrationBias.gyroX = biasSum[3] >> 8;
    mpuCalibrationBias.gyroY = biasSum[4] >> 8;
    mpuCalibrationBias.gyroZ = biasSum[5] >> 8;
}

/**
 * @brief 获取MPU6050数据
 *
 */
void App_Flight_FetchMPUData(void) {
    Int_MPU6050_ReadAccel(&mpu6050.accelX, &mpu6050.accelY, &mpu6050.accelZ);
    Int_MPU6050_ReadGyro(&mpu6050.gyroX, &mpu6050.gyroY, &mpu6050.gyroZ);
    // LOG_DEBUG("Before Filter => Accel X: %d, Y: %d, Z: %d, Gyro X: %d, Y:%d, Z: %d", mpu6050.accelX,
    //           mpu6050.accelY, mpu6050.accelZ, mpu6050.gyroX, mpu6050.gyroY, mpu6050.gyroZ);
    // 零偏校准
    mpu6050.accelX = mpu6050.accelX - mpuCalibrationBias.accelX;
    mpu6050.accelY = mpu6050.accelY - mpuCalibrationBias.accelY;
    mpu6050.accelZ = mpu6050.accelZ - mpuCalibrationBias.accelZ;
    mpu6050.gyroX = mpu6050.gyroX - mpuCalibrationBias.gyroX;
    mpu6050.gyroY = mpu6050.gyroY - mpuCalibrationBias.gyroY;
    mpu6050.gyroZ = mpu6050.gyroZ - mpuCalibrationBias.gyroZ;

    /* 对加速度计进行一阶卡尔曼滤波 */
    // 加速度数据通常包含多种噪声源，包括环境振动、电气噪声和传感器本身的误差
    // 加速度数据常用于计算姿态角，需要较高的精度
    Com_Kalman_1(&ekf[0], mpu6050.accelX);
    mpu6050.accelX = ekf[0].out;
    Com_Kalman_1(&ekf[1], mpu6050.accelY);
    mpu6050.accelY = ekf[1].out;
    Com_Kalman_1(&ekf[2], mpu6050.accelZ);
    mpu6050.accelZ = ekf[2].out;

    /* 对陀螺仪进行简单的低通滤波 */
    // 陀螺仪的噪声特性相对简单，主要是高频噪声
    // 角速度数据的变化相对平缓，低频分量才是有用信号
    static int16_t lastGyro[3] = {0};
    mpu6050.gyroX = lastGyro[0] * 0.85 + mpu6050.gyroX * 0.15;
    lastGyro[0] = mpu6050.gyroX;
    mpu6050.gyroY = lastGyro[1] * 0.85 + mpu6050.gyroY * 0.15;
    lastGyro[1] = mpu6050.gyroY;
    mpu6050.gyroZ = lastGyro[2] * 0.85 + mpu6050.gyroZ * 0.15;
    lastGyro[2] = mpu6050.gyroZ;

    // LOG_DEBUG("After Filter => Accel X: %d, Y: %d, Z: %d, Gyro X: %d, Y:%d, Z: %d", mpu6050.accelX,
    //           mpu6050.accelY, mpu6050.accelZ, mpu6050.gyroX, mpu6050.gyroY, mpu6050.gyroZ);
}

void App_Flight_PilotLED(void) {
    static uint32_t lastTick = 0;
    uint32_t curTick = HAL_GetTick();
    if (curTick - lastTick >= ledState.durationTime) {
        ledState.state = AlwaysOn;
        lastTick = curTick;
    }
    switch (ledState.state) {
        case AlwaysOn:
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED2_GPIO_Port, LED2_Pin);
            Int_LED_On(LED3_GPIO_Port, LED3_Pin);
            Int_LED_On(LED4_GPIO_Port, LED4_Pin);
            break;
        case AlwaysOff:
            Int_LED_Off(LED1_GPIO_Port, LED1_Pin);
            Int_LED_Off(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            break;
        case AllFlashLight:
            Int_LED_Toggle(LED1_GPIO_Port, LED1_Pin);
            Int_LED_Toggle(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Toggle(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Toggle(LED4_GPIO_Port, LED4_Pin);
            break;
        case AlternateFlash:
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            ledState.state = AllFlashLight;
            break;
        case WARNING:
            Int_LED_On(LED1_GPIO_Port, LED1_Pin);
            Int_LED_On(LED3_GPIO_Port, LED3_Pin);
            Int_LED_Off(LED2_GPIO_Port, LED2_Pin);
            Int_LED_Off(LED4_GPIO_Port, LED4_Pin);
            ledState.state = AllFlashLight;
            break;
        default:
            ledState.state = AlwaysOff;
            break;
    }
    HAL_Delay(100);
}
