#include "Int_IP5305T.h"
#include "FreeRTOS.h"
#include "task.h"

void Int_IP5305T_ShortPress(void);

void Int_IP5305T_Wakeup(void) {
    Int_IP5305T_ShortPress();
}

void Int_IP5305T_PowerDown(void) {
    Int_IP5305T_ShortPress();
    vTaskDelay(200);
    Int_IP5305T_ShortPress();
}

void Int_IP5305T_ShortPress(void) {
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
}
