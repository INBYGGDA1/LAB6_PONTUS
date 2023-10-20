/*
 * ================================================================
 * File: main.c
 * Author: Pontus Svensson
 * Email: psn19003@student.mdu.se
 * Date: 2023-10-09
 * Description: Random number of producers and consumer.
 *
 * License: This code is distributed under the MIT License. visit
 * https://opensource.org/licenses/MIT for more information.
 * ================================================================
 */

//=============================================================================
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "time.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "tm4c129_functions.h"
#include "tm4c129_rtos.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "utils/uartstdio.h"
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
#define BUFFER_SIZE 2

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
volatile char byteCount[BUFFER_SIZE];
volatile int bufferIndex = 0;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SemaphoreHandle_t xSemaphoreCountingFULL, xSemaphoreCountingEmpty,
    xSemaphoreBinary;
/*================================================================*/
void produceByte() {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  char byte = 'c'; // Place c as the byte
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (bufferIndex < BUFFER_SIZE) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    byteCount[bufferIndex] = byte;
    bufferIndex++;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
}

/*================================================================*/
void removeByteFromBuffer() {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (bufferIndex > 0) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    byteCount[bufferIndex] = '\0';
    bufferIndex--;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
}
/*================================================================*/
void producer(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int i = *(int *)pvParameters;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(200);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  while (1) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vTaskDelay(xDelay); // Delay for 200 ms
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Take the empty semaphore as soon as there is an available slot, otherwise
    // wait here
    if (xSemaphoreTake(xSemaphoreCountingEmpty, (TickType_t)portMAX_DELAY) ==
        pdTRUE) {

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTprintf("\n+ P(%d)--> TAKE SemEmpty: Empty-Count (%d)\n", i,
                 uxSemaphoreGetCount(xSemaphoreCountingEmpty));
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // CRITICAL SECTION
      // Take the binary semaphore to ensure mutual exclusion
      // when writing to the buffer.
      if (xSemaphoreTake(xSemaphoreBinary, (TickType_t)portMAX_DELAY) ==
          pdFALSE) { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n+ P(%d)--> Error taking xSemaphoreBinary\n", i);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      } else {
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n+ P(%d)--> TAKE BINARY, VALUE (%d)\n", i,
                   uxSemaphoreGetCount(xSemaphoreBinary));
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n+ P(%d)--> Producing byte, Position (%d)\n", i,
                   bufferIndex);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        produceByte();
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Give the binary semaphore
        while (xSemaphoreGive(xSemaphoreBinary) != pdTRUE)
          ;
        // End CRITICAL SECTION

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n+ P(%d)--> GIVE BINARY, Updated VALUE (%d)\n", i,
                   uxSemaphoreGetCount(xSemaphoreBinary));
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Decrease available spots
        while (xSemaphoreGive(xSemaphoreCountingFULL) != pdTRUE)
          ;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n+ P(%d)--> GIVE SemFull, Updated Full-Count (%d)\n", i,
                   uxSemaphoreGetCount(xSemaphoreCountingFULL));
      }
    }
  }
}
/*================================================================*/
void consumer(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int i = *(int *)pvParameters;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(200);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  while (1) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vTaskDelay(xDelay);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Increase the counting semaphore that keeps track of the current items in
    // the buffer
    if (xSemaphoreTake(xSemaphoreCountingFULL, (TickType_t)portMAX_DELAY) ==
        pdTRUE) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTprintf("\n- C(%d)--> TAKE SemFull, Full-Count (%d)\n", i,
                 uxSemaphoreGetCount(xSemaphoreCountingFULL));
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Take the binary semaphore to ensure mutual exclusion before writing to
      // the buffer
      if (xSemaphoreTake(xSemaphoreBinary, (TickType_t)portMAX_DELAY) ==
          pdFALSE) {
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n- C(%d)--> Error taking xSemaphoreBinary\n", i);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      } else {
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n- C(%d)--> TAKE BINARY, VALUE (%d)\n", i,
                   uxSemaphoreGetCount(xSemaphoreBinary));
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // CRITICAL SECTION
        UARTprintf("\n- C(%d)--> Removing byte, Position (%d)\n", i,
                   bufferIndex);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        removeByteFromBuffer();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Release the binary semaphore to allow other tasks to produce or
        // consume bytes
        while (xSemaphoreGive(xSemaphoreBinary) != pdTRUE)
          ;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n- C(%d)--> GIVE BINARY, Updated VALUE (%d)\n", i,
                   uxSemaphoreGetCount(xSemaphoreBinary));
        // END CRITICAL SECTION

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Increase one slot in the semaphore that keeps track of the available
        // slots in the buffer since we just consumed one byte
        while (xSemaphoreGive(xSemaphoreCountingEmpty) != pdTRUE)
          ;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTprintf("\n- C(%d)--> GIVE SemEmpty, Updated Empty-Count (%d)\n", i,
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
  randomConsumer = rand() % 9 + 1;
  randomProducer = rand() % 9 + 1;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TaskHandle_t xProducerHandle;
  TaskHandle_t xConsumerHandle;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t xProducerReturn;
  BaseType_t xConsumerReturn;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ConfigureUART();
  UARTClearScreen();
  UARTprintf("Consumers: %d\nProducers: %d\n", randomConsumer, randomProducer);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a random amount of producers and consumers
  for (i = 0; i < randomProducer; i++) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // To allow the ids of the tasks to be printed
    int *k = pvPortMalloc(sizeof(int));
    *k = i;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    xProducerReturn =
        xTaskCreate(producer, "producer", 64, k, 1, &xProducerHandle);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (xProducerReturn != pdPASS) {
      UARTprintf("Unable to create produces task\n");
    }
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (i = 0; i < randomConsumer; i++) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    int *k = pvPortMalloc(sizeof(int));
    *k = i;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    xConsumerReturn =
        xTaskCreate(consumer, "consumer", 64, k, 1, &xConsumerHandle);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (xConsumerReturn != pdPASS) {
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
