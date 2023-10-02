/*================================================================*/
#include <stdint.h>
#include <stdbool.h>

/*================================================================*/
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

/*================================================================*/
#include "inc/hw_memmap.h"

/*================================================================*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"

/*================================================================*/
#define BUFFER_SIZE 2

/*================================================================*/
volatile char byteCount[BUFFER_SIZE];
volatile int bufferIndex = 0;
SemaphoreHandle_t xSemaphoreCountingFULL, xSemaphoreCountingEmpty,
    xSemaphoreBinary;
/*================================================================*/
/* The error routine that is called if the driver library         */
/* encounters an error.                                           */
/*================================================================*/
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif

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
/*            Helper function configure uart                      */
/*================================================================*/
void ConfigureUART() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
}

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
        xSemaphoreGive(xSemaphoreBinary);
        UARTprintf("- CONSUMER --> GIVE BINARY, Updated VALUE (%d)\n",
                   uxSemaphoreGetCount(xSemaphoreBinary));
        // END CRITICAL SECTION

        xSemaphoreGive(xSemaphoreCountingEmpty);
        UARTprintf("- CONSUMER --> GIVE SemEmpty, Updated Empty-Count (%d)\n",
                   uxSemaphoreGetCount(xSemaphoreCountingEmpty));
      }
    }
  }
}

int main(void) {
  TaskHandle_t xProducerHandle, xConsumerHandle;
  BaseType_t xProducerReturn;
  BaseType_t xConsumerReturn;
  ConfigureUART();
  UARTprintf("\033[2J");

  xProducerReturn =
      xTaskCreate(producer, "producer", 128, NULL, 1, &xProducerHandle);
  xConsumerReturn =
      xTaskCreate(consumer, "consumer", 128, NULL, 1, &xConsumerHandle);
  if (xProducerReturn == pdPASS && xConsumerReturn == pdPASS) {
    UARTprintf("Producer & Consumer Tasks created\n");
  }
  xSemaphoreCountingFULL = xSemaphoreCreateCounting(
      BUFFER_SIZE,
      0); // Increase this semaphore whenever producer produces a byte. The
          // Buffer is full if xSemaphoreCountingFULL is equal to BUFFER_SIZE.
          // By xSemaphoreGive we increase the counter since we start at 0.
  xSemaphoreCountingEmpty = xSemaphoreCreateCounting(
      BUFFER_SIZE,
      BUFFER_SIZE); // Whenever this semaphore is equal to the buffer size,
                    // the BUFFER_SIZE is empty. By xSemaphoreTake we decrease
                    // the semaphore counter since we start at BUFFER_SIZE
                    // available slots.
  xSemaphoreBinary = xSemaphoreCreateBinary();
  if (xSemaphoreCountingEmpty == NULL || xSemaphoreCountingFULL == NULL ||
      xSemaphoreBinary == NULL) {
    UARTprintf("Unable to create semaphores\n");
  }
  xSemaphoreGive(xSemaphoreBinary);

  vTaskStartScheduler();
}
