#ifndef __INT_LED_H__
#define __INT_LED_H__

#include "stm32f1xx_hal.h"

void Int_LED_On(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Int_LED_Off(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Int_LED_Toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* __INT_LED_H__ */
