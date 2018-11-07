/*
 * Copyright:     Gregor Richter
 * Author:        Gregor Richter
 * Remarks:       all rights reserved, this Copyright must be included
 */

#include <Arduino.h>
#include <esp32-hal.h>
#include <freertos/FreeRTOS.h>

#include "RTOS-ESP32-helper.h"


typedef void (*voidFuncPtr)(void);
static voidFuncPtr __pinInterruptHandlers[GPIO_PIN_COUNT] = { 0, };


typedef struct {
  uint8_t pin;
  voidFuncPtr handler;
  int mode;
  TaskHandle_t parent_process;
} attachInterrup_pvParameters;

attachInterrup_pvParameters pvParameters;

void attachInterruptPinCoreTask(void * pvParameters) {
  attachInterrup_pvParameters *_pvParameters;

  _pvParameters = (attachInterrup_pvParameters *) pvParameters;

  attachInterrupt(_pvParameters->pin, _pvParameters->handler,
      _pvParameters->mode);


  // Serial.println("Task Done!");
  xTaskNotifyGive(_pvParameters->parent_process);
  vTaskDelete(NULL);
}


void attachInterruptPinnedToCore(uint8_t pin, voidFuncPtr handler, int mode,
    const BaseType_t xCoreID) {

  pvParameters.pin = pin;
  pvParameters.handler = handler;
  pvParameters.mode = mode;
  pvParameters.parent_process = xTaskGetCurrentTaskHandle();

  TaskHandle_t xHandle = NULL;

  // Serial.print("Create Interrupt on Core ");
  // Serial.println(xCoreID);

  xTaskCreatePinnedToCore(
                    attachInterruptPinCoreTask,   // Function namev
                    "",         // RTOS Name
                    500,        // Stack size in words
                    (void*) &pvParameters,  // Task input parameter
                    0,          // Priority of the task
                    &xHandle,   // Task handle
                    xCoreID);   // Core where the ISR should run

  // Waiting attachInterruptPinCoreTask max 100ms
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

  // delay(1000);
  // Serial.println("DONE!");
}
