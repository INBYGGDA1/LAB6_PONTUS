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

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t thinking_time = pdMS_TO_TICKS(2000);
  const TickType_t eating_time = pdMS_TO_TICKS(2000);
  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Think
    vTaskDelayUntil(&xLastWakeTime, thinking_time);
    UARTprintf("P(%d): Thinking time\n", philosopher_id);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Pickup chopsticks
    // Check for chopstick, is the philosophers own chopstick and the next one
    // available the philosopher can eat
    // Make sure we can take both forks at once to avoid any deadlocks.
    xSemaphoreTake(chopstick_semaphore[philosopher_id], portMAX_DELAY);

    xSemaphoreTake(
        chopstick_semaphore[(philosopher_id + 1) % NUMBER_OF_CHOPSTICKS],
        portMAX_DELAY);
    UARTprintf("P(%d): Chopstick time\n", philosopher_id);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Eat
    UARTprintf("P(%d): Eating time\n", philosopher_id);
    vTaskDelayUntil(&xLastWakeTime, eating_time);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    xSemaphoreGive(chopstick_semaphore[philosopher_id]);
    xSemaphoreGive(
        chopstick_semaphore[(philosopher_id + 1) % NUMBER_OF_CHOPSTICKS]);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    UARTprintf("P(%d): Put down the chopsticks\n", philosopher_id);
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
