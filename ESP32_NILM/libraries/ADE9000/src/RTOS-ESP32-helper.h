/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */



#ifndef RTOS_ESP32_H_
#define RTOS_ESP32_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>

void attachInterruptPinCoreTask(void * pvParameters);

// This function takes exactly the same arguments of the attachInterrupt(...)
// and an additional argument at the end to specify the core where the Interrupt
// should run.
// FIXME: After the first exec the core always stays the same!
void attachInterruptPinnedToCore (uint8_t pin, void (*)(void), int mode,
    const BaseType_t xCoreID);


#ifdef __cplusplus
}
#endif

#endif  // RTOS_ESP32_H_
