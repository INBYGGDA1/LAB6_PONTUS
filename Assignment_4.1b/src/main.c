#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "inc/hw_memmap.h"

#include "FreeRTOS.h"
#include "../inc/FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"

#define BUFFER_SIZE 3

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

char produceByte() {
  char byte = 'c'; // Place 1 as the byte
  return byte;
}

void removeByteFromBuffer() {
  byteCount[bufferIndex] = '\0';
  bufferIndex--;
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
  TaskHandle_t *xConsumerHandle = (TaskHandle_t *)pvParameters;
  while (1) {

    char byte = produceByte();
    // semphr
    if (bufferIndex == BUFFER_SIZE) {
      // semphr unlock
      UARTprintf("P: BUFFER FULL: SLEEPING\n");
      vTaskSuspend(NULL);
    }
    UARTprintf("P: PLACING Byte in Buffer: IDX[%d]\n", bufferIndex);
    // semaphore here
    byteCount[bufferIndex] = byte;
    bufferIndex++;
    if (bufferIndex == 1) {
      UARTprintf("P: BUFFER is not EMPTY, WAKING CONSUMER\n");
      // Wake consumer
      vTaskResume(*xConsumerHandle);
    }
    // semphr unlock
  }
}
void consumer(void *pvParameters) {

  TaskHandle_t *xProducerHandle = (TaskHandle_t *)pvParameters;
  while (1) {

    if (xSemaphoreTake(xSemaphoreCountingEmpty, (TickType_t)1) ==
        pdFAIL) { // If the buffer is empty, we cant consume, go to sleep, I.E.
                  // the count is equal to 0
      UARTprintf("C: Buffer empty, going to sleep\n");
      vTaskSuspend(NULL);
    } else {
      UARTprintf("C: Taking xSemaphoreCountingEmpty\n");
    }

    if (xSemaphoreTake(xSemaphoreBinary, (TickType_t)portMAX_DELAY) ==
        pdFAIL) { // Take the binary semaphore to ensure mutual exclusion when
                  // writing to the buffer. We Cannot end up here if the buffer
                  // is non empty
      UARTprintf("C: Error taking xSemaphoreBinary\n");
    } else {
      UARTprintf("C: Taking Binary SEM\n");
      UARTprintf("C: Removing byte from buffer: IDX[%d]\n", bufferIndex);
      removeByteFromBuffer();
      xSemaphoreGive(xSemaphoreCountingEmpty);
      xSemaphoreGive(xSemaphoreBinary);
    }

    uxSemaphoreGetCount();
    if (xSemaphoreTake(xSemaphoreCountingFULL, (TickType_t)1) ==
        pdFAIL) { // If the count in the semaphore is equal to BUFFER_SIZE, the
                  // buffer is full

      // Wake producer
      UARTprintf("C: Waking up producer\n");
      vTaskResume(*xProducerHandle);
    }
    // semaphore unlock
  }
}

int main(void) {
  TaskHandle_t xProducerHandle, xConsumerHandle;
  BaseType_t xProducerReturn;
  BaseType_t xConsumerReturn;
  ConfigureUART();
  UARTprintf("\033[2J");

  xProducerReturn = xTaskCreate(producer, "producer", 128,
                                (void *)&xConsumerHandle, 1, &xProducerHandle);
  xConsumerReturn = xTaskCreate(consumer, "consumer", 128,
                                (void *)&xProducerHandle, 1, &xConsumerHandle);
  if (xProducerReturn == pdPASS && xConsumerReturn == pdPASS) {
    UARTprintf("Producer & Consumer Tasks created\n");
  }
  xSemaphoreCountingFULL = xSemaphoreCreateCounting(
      BUFFER_SIZE,
      0); // Increase this semaphore whenever producer produces a byte. The
          // Buffer is full if xSemaphoreCountingFULL is equal to BUFFER_SIZE
  xSemaphoreCountingEmpty = xSemaphoreCreateCounting(
      BUFFER_SIZE,
      BUFFER_SIZE); // Whenever this semaphore is equal to the buffer size, the
                    // BUFFER_SIZE is empty
  xSemaphoreBinary = xSemaphoreCreateBinary();
  if (xSemaphoreCountingEmpty == NULL || xSemaphoreCountingFULL == NULL ||
      xSemaphoreBinary == NULL) {
    UARTprintf("Unable to create semaphores\n");
  }

  vTaskStartScheduler();
}
