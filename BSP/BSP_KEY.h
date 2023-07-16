#ifndef __BSP_KEY_H
#define __BSP_KEY_H
#include "stm32f1xx_hal.h"
#define KEY0  HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_3)
#define KEY1  HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_4)
#define KEY2  HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_5)
#define KEY3 HAL_GPIO_ReadPin (GPIOB,GPIO_PIN_6)

#define KEY0_PRES 1 //KEYO
#define KEY1_PRES 2 //KEY1
#define KEY2_PRES 3 //KEY2
#define KEY3_PRES 4 //WK_UP
uint8_t KEY_Scan (uint8_t mode) ;
#endif
