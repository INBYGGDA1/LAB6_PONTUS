/*
 * ================================================================
 * File: main.c
 * Author: Pontus Svensson
 * Email: psn19003@student.mdu.se
 * Date: 2023-10-09
 * Description: Dining phhilosophers
 *
 * License: This code is distributed under the MIT License. visit
 * https://opensource.org/licenses/MIT for more information.
 * ================================================================
 */

//=============================================================================
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "tm4c129_functions.h"
#include "tm4c129_rtos.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "utils/uartstdio.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "inc/hw_memmap.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "portable.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define NUMBER_OF_PHILOSOPHERS 5
#define NUMBER_OF_CHOPSTICKS 5

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SemaphoreHandle_t chopstick_semaphore[NUMBER_OF_CHOPSTICKS];
SemaphoreHandle_t butler_semaphore;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void vDiningPhilosopher(void *pvParameters) {
  uint32_t philosopher_id = *(uint32_t *)pvParameters;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Left chopstick
  uint32_t left_chopstick = philosopher_id;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // right chopstick
  uint32_t right_chopstick = (philosopher_id + 1) % NUMBER_OF_CHOPSTICKS;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create some delay
  const TickType_t thinking_time = pdMS_TO_TICKS(1000 * (philosopher_id + 1));
  const TickType_t eating_time = pdMS_TO_TICKS(1000 * (philosopher_id + 2));
  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Think
    UARTprintf("\n(%d): Thinking time\n", philosopher_id);
    vTaskDelay(thinking_time);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Pickup chopsticks in odd and even order
    // Even numbered philosophers will always pick up the left chopstick first.
    // Odd numbered philosophers will always pick up the right chopstick first.
    // This solution will always end up with atleast one philosopher dining.
    if (philosopher_id % 2 == 0) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Pick up left chopstick
      xSemaphoreTake(chopstick_semaphore[left_chopstick], portMAX_DELAY);
      vWorker(10); // Workers are here to make the output somewhat readable
      UARTprintf("\n(%d): Left chopstick picked up\n", philosopher_id);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // pickup right chopstick
      xSemaphoreTake(chopstick_semaphore[right_chopstick], portMAX_DELAY);
      vWorker(10);
      UARTprintf("\n(%d): Right chopstick picked up\n", philosopher_id);
    } else {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // pickup right chopstick
      xSemaphoreTake(chopstick_semaphore[right_chopstick], portMAX_DELAY);
      vWorker(10);
      UARTprintf("\n(%d): Left chopstick picked up\n", philosopher_id);

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // pickup left chopstick
      xSemaphoreTake(chopstick_semaphore[left_chopstick], portMAX_DELAY);
      vWorker(10);
      UARTprintf("\n(%d): Right chopstick picked up\n", philosopher_id);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Eat
    UARTprintf("\n(%d): Eating time\n", philosopher_id);
    vTaskDelay(eating_time);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Put down the chopsticks
    if (philosopher_id % 2 == 0) {

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTprintf("\n(%d): Put down left chopstick\n", philosopher_id);
      xSemaphoreGive(chopstick_semaphore[left_chopstick]);
      vWorker(10);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTprintf("\n(%d): Put down right chopstick\n", philosopher_id);
      xSemaphoreGive(chopstick_semaphore[right_chopstick]);
      vWorker(10);
    } else {

      UARTprintf("\n(%d): Put down right chopstick\n", philosopher_id);
      xSemaphoreGive(chopstick_semaphore[right_chopstick]);
      vWorker(10);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTprintf("\n(%d): Put down left chopstick\n", philosopher_id);
      xSemaphoreGive(chopstick_semaphore[left_chopstick]);
      vWorker(10);
    }
    vWorker(2000);
  }
  vPortFree(&philosopher_id);
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void) {
  TaskHandle_t xTaskPhilosopherHandle[NUMBER_OF_PHILOSOPHERS];
  BaseType_t xTaskPhilosopherReturn[NUMBER_OF_PHILOSOPHERS];
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t systemClock;
  uint32_t *philosopher_id;
  uint32_t i;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ConfigureSystemClock(120000000, &systemClock);
  ConfigureUART();
  UARTClearScreen();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (i = 0; i < NUMBER_OF_PHILOSOPHERS; i++) {
    philosopher_id = pvPortMalloc(sizeof(uint32_t));
    *philosopher_id = i;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    xTaskPhilosopherReturn[i] =
        xTaskCreate(vDiningPhilosopher, "Philosopher", 128,
                    (void *)philosopher_id, 1, &xTaskPhilosopherHandle[i]);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (xTaskPhilosopherReturn[i] != pdPASS) {
      UARTprintf("Unable to create task\n");
    }
  }
  for (i = 0; i < NUMBER_OF_PHILOSOPHERS; i++) {

    chopstick_semaphore[i] = xSemaphoreCreateMutex();
    if (chopstick_semaphore[i] == NULL) {
      UARTprintf("Unable to create semahore\n");
    }
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  vTaskStartScheduler();
}
