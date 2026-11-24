#pragma once

#include "./stm32_mcu.h"

#if defined(ARDUINO_ARCH_STM32)


int stm32_findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]);


#endif
