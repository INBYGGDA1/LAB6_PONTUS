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
#include <stdlib.h>
#include "time.h"
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

#define BUFFER_SIZE 2

volatile char byteCount[BUFFER_SIZE];
volatile int bufferIndex = 0;
SemaphoreHandle_t xSemaphoreCountingFULL, xSemaphoreCountingEmpty,
    xSemaphoreBinary;
/*================================================================*/
void produceByte() {
  char byte = 'c'; // Place 1 as the byte
  if (bufferIndex < BUFFER_SIZE) {
    byteCount[bufferIndex] = byte;
    bufferIndex++;
  }
}

/*================================================================*/
void removeByteFromBuffer() {
  if (bufferIndex > 0) {
    byteCount[bufferIndex] = '\0';
    bufferIndex--;
  }
}
/*================================================================*/
void producer(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(200); // 200 ms delay
  while (1) {

    vTaskDelay(xDelay); // Delay for 200 ms

    UARTprintf("+ PRODUCER --> TAKE SemEmpty: Empty-Count (%d)\n",
               uxSemaphoreGetCount(xSemaphoreCountingEmpty));
    // Take the empty semaphore as soon as there is an available slot, otherwise
    // wait here
    if (xSemaphoreTake(xSemaphoreCountingEmpty, (TickType_t)portMAX_DELAY) ==
        pdTRUE) {

      // CRITICAL SECTION
      UARTprintf("+ PRODUCER --> TAKE BINARY, VALUE (%d)\n",
                 uxSemaphoreGetCount(xSemaphoreBinary));
      if (xSemaphoreTake(xSemaphoreBinary, (TickType_t)portMAX_DELAY) ==
          pdFALSE) { // Take the binary semaphore to ensure mutual exclusion
                     // when writing to the buffer.
        UARTprintf("+ PRODUCER --> Error taking xSemaphoreBinary\n");
      } else {

        UARTprintf("+ PRODUCER --> Producing byte, Position (%d)\n",
                   bufferIndex);
        produceByte();
        while (xSemaphoreGive(xSemaphoreBinary) != pdTRUE)
          ;
        // End CRITICAL SECTION

        UARTprintf("+ PRODUCER --> GIVE BINARY, Updated VALUE (%d)\n",
                   uxSemaphoreGetCount(xSemaphoreBinary));
        while (xSemaphoreGive(xSemaphoreCountingFULL) != pdTRUE)
          ; // Decrease available spots

        UARTprintf("+ PRODUCER --> GIVE SemFull, Updated Full-Count (%d)\n",
                   uxSemaphoreGetCount(xSemaphoreCountingFULL));
      }
    }
  }
}
void consumer(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(400); // 400 msecond delay
  while (1) {

    vTaskDelay(xDelay);

    UARTprintf("- CONSUMER --> TAKE SemFull, Full-Count (%d)\n",
               uxSemaphoreGetCount(xSemaphoreCountingFULL));
    if (xSemaphoreTake(xSemaphoreCountingFULL, (TickType_t)portMAX_DELAY) ==
        pdTRUE) { // If the xSemaphoreCountingFULL counter is > 0, we have bytes
                  // to consume. Otherwise wait here until we have bytes to
                  // consume
      UARTprintf("- CONSUMER --> TAKE BINARY, VALUE (%d)\n",
                 uxSemaphoreGetCount(xSemaphoreBinary));
      if (xSemaphoreTake(xSemaphoreBinary, (TickType_t)portMAX_DELAY) ==
          pdFALSE) { // Take the binary semaphore to ensure mutual exclusion
                     // when writing to the buffer.
        UARTprintf("- CONSUMER --> Error taking xSemaphoreBinary\n");
      } else {

        // CRITICAL SECTION
        UARTprintf("- CONSUMER --> Removing byte, Position (%d)\n",
                   bufferIndex);
        removeByteFromBuffer();
        while (xSemaphoreGive(xSemaphoreBinary) != pdTRUE)
          ;
        UARTprintf("- CONSUMER --> GIVE BINARY, Updated VALUE (%d)\n",
                   uxSemaphoreGetCount(xSemaphoreBinary));
        // END CRITICAL SECTION

        while (xSemaphoreGive(xSemaphoreCountingEmpty) != pdTRUE)
          ;
        UARTprintf("- CONSUMER --> GIVE SemEmpty, Updated Empty-Count (%d)\n",
                   uxSemaphoreGetCount(xSemaphoreCountingEmpty));
      }
    }
  }
}

int main(void) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int randomConsumer, randomProducer;
  int i;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  srand(time(NULL));
  // no more than 20 tasks
  randomConsumer = rand() % 10 + 1;
  randomProducer = rand() % 10 + 1;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TaskHandle_t xProducerHandle[randomProducer];
  TaskHandle_t xConsumerHandle[randomConsumer];
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t xProducerReturn[randomProducer];
  BaseType_t xConsumerReturn[randomConsumer];
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ConfigureUART();
  UARTClearScreen();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a random amount of producers and consumers
  for (i = 0; i < randomProducer; i++) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    xProducerReturn[i] =
        xTaskCreate(producer, "producer", 64, NULL, 1, &xProducerHandle[i]);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (xProducerReturn[i] != pdPASS) {
      UARTprintf("Unable to create produces task\n");
    }
  }
  for (i = 0; i < randomConsumer; i++) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    xConsumerReturn[i] =
        xTaskCreate(consumer, "consumer", 64, NULL, 1, &xConsumerHandle[i]);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (xConsumerReturn[i] != pdPASS) {
      UARTprintf("Unable to create consumer task\n");
    }
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Increase this semaphore whenever producer produces a byte. The
  // Buffer is full if xSemaphoreCountingFULL is equal to BUFFER_SIZE.
  // By xSemaphoreGive we increase the counter since we start at 0.
  xSemaphoreCountingFULL = xSemaphoreCreateCounting(BUFFER_SIZE, 0);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Whenever this semaphore is equal to the buffer size,
  // the BUFFER_SIZE is empty. By xSemaphoreTake we decrease
  // the semaphore counter since we start at BUFFER_SIZE
  // available slots.
  xSemaphoreCountingEmpty = xSemaphoreCreateCounting(BUFFER_SIZE, BUFFER_SIZE);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xSemaphoreBinary = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphoreBinary);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (xSemaphoreCountingEmpty == NULL || xSemaphoreCountingFULL == NULL ||
      xSemaphoreBinary == NULL) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    UARTprintf("Unable to create semaphores\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  vTaskStartScheduler();
}
