#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

void Error_Handler(void);

#define hDebugUart huart1
#define hHub75Bus hspi1

#ifdef __cplusplus
}
#endif

#endif
