#include "App_Task.h"
#include "App_Flight.h"
#include "Com_IMU.h"
#include "FreeRTOS.h"
#include "Int_Si24R1.h"
#include "task.h"
#include "Int_IP5305T.h"

#define START_STACK_DEPTH   256
#define START_TASK_PRIORITY 1
TaskHandle_t start_task_handle;
void Start_Task(void *pvParameters);

/* 2ms周期任务 */
#define TASK_2MS_STACK_DEPTH 256
#define TASK_2MS_PRIORITY    4
TaskHandle_t task_2ms_handle;
void App_Task_2MS(void *pvParameters);

/* 4ms周期任务 */
#define TASK_4MS_STACK_DEPTH 256
#define TASK_4MS_PRIORITY    3
TaskHandle_t task_4ms_handle;
void App_Task_4MS(void *pvParameters);

/* 50ms周期任务 */
#define TASK_50MS_STACK_DEPTH 256
#define TASK_50MS_PRIORITY    2
TaskHandle_t task_50ms_handle;
void App_Task_50MS(void *pvParameters);

/* 10000ms周期任务 */
#define TASK_10000MS_STACK_DEPTH 128
#define TASK_10000MS_PRIORITY    2
TaskHandle_t task_10000ms_handle;
void App_Task_10000MS(void *pvParameters);

/**
 * @description: 入口函数：创建启动任务、启动调度器
 * @return {*}
 */
void FreeRTOS_Start(void) {
    /* 1. 创建启动任务 */
    xTaskCreate((TaskFunction_t)Start_Task, (char *)"Start_Task",
                (configSTACK_DEPTH_TYPE)START_STACK_DEPTH, (void *)NULL,
                (UBaseType_t)START_TASK_PRIORITY, (TaskHandle_t *)&start_task_handle);

    /* 2. 启动调度器 */
    vTaskStartScheduler();
}

/**
 * @description: 启动任务，创建其他任务
 * @param {void *} pvParameters
 * @return {*}
 */
void Start_Task(void *pvParameters) {
    taskENTER_CRITICAL();
    /* 创建显示任务 */
    xTaskCreate((TaskFunction_t)App_Task_2MS, (char *)"App_Task_2MS",
                (configSTACK_DEPTH_TYPE)TASK_2MS_STACK_DEPTH, (void *)NULL,
                (UBaseType_t)TASK_2MS_PRIORITY, (TaskHandle_t *)&task_2ms_handle);

    xTaskCreate((TaskFunction_t)App_Task_4MS, (char *)"App_Task_4MS",
                (configSTACK_DEPTH_TYPE)TASK_4MS_STACK_DEPTH, (void *)NULL,
                (UBaseType_t)TASK_4MS_PRIORITY, (TaskHandle_t *)&task_4ms_handle);

    xTaskCreate((TaskFunction_t)App_Task_50MS, (char *)"App_Task_50MS",
                (configSTACK_DEPTH_TYPE)TASK_50MS_STACK_DEPTH, (void *)NULL,
                (UBaseType_t)TASK_50MS_PRIORITY, (TaskHandle_t *)&task_50ms_handle);

    xTaskCreate((TaskFunction_t)App_Task_10000MS, (char *)"App_Task_10000MS",
                (configSTACK_DEPTH_TYPE)TASK_10000MS_STACK_DEPTH, (void *)NULL,
                (UBaseType_t)TASK_10000MS_PRIORITY, (TaskHandle_t *)&task_10000ms_handle);
    vTaskDelete(NULL);
    taskEXIT_CRITICAL();
}

/**
 * @description: 2ms周期任务
 * @param {void *} pvParameters
 * @return {*}
 */
void App_Task_2MS(void *pvParameters) {

    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();

    while (1) {
        /* 获取MPU 6轴数据 */
        App_Flight_FetchMPUData();
        /* 计算欧拉角，注意第二个参数=调用周期  1ms=0.001 */
        GetAngle(&mpu6050, &flightAngle, 0.006);
        /* 三个串级PID计算，注意参数=调用周期 1ms=0.001 */
        App_Flight_PIDControl(0.006);
        /* 电机控制 */
        App_Flight_MotorControl();
        vTaskDelayUntil(&pxPreviousWakeTime, 6);
    }
}

/**
 * @description: 4ms周期任务
 * @param {void *} pvParameters
 * @return {*}
 */
void App_Task_4MS(void *pvParameters) {

    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();

    while (1) {
        /* 2.4G接收数据，注意调用周期 <= 发送周期 */
        App_Flight_RxRCDATA();
        vTaskDelayUntil(&pxPreviousWakeTime, 4);
    }
}

/**
 * @description: 50ms周期任务
 * @param {void} *pvParameters
 * @return {*}
 */
void App_Task_50MS(void *pvParameters) {
    TickType_t pxPreviousWakeTime;
    pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        /* 无关紧要的灯控系统任务，周期随意 */
        App_Flight_PilotLED();
        vTaskDelayUntil(&pxPreviousWakeTime, 200);
    }
}

void App_Task_10000MS(void *pvParameters) {
    TickType_t pxPreviousWakeTime;
    vTaskDelay(10000); // 按键开机10s后才开始周期性唤醒，避免和开机按键组合成连续短按
    pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        Int_IP5305T_Wakeup();
        vTaskDelayUntil(&pxPreviousWakeTime, 10000);
    }
}
