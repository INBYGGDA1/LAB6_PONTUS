/*
 * ================================================================
 * File: main.c
 * Author: Pontus Svensson, Carl Larsson
 * Email: psn19003@student.mdu.se, cln20001@student.mdu.se
 * Date: 2023-10-03
 * Description: The output is 15 characters at a time. Consider the following
 * text consisting of 26 characters as an example: “Embedded systems is so fun”.
 * The visible text on line 1 on the UART should only be the last 15 characters
 * of the above text, i.e., “stems is so fun” If any key, at any point is
 * pressed, the corresponding character is appended to the text and only last 15
 * characters of the text are shown. For example, if you press “i”, the output
 * on the serial console program should immediately change from “stems is so
 * fun” to: “tems is so funi”.
 *
 * License: This code is distributed under the MIT License. visit
 * https://opensource.org/licenses/MIT for more information.
 * ================================================================
 */

/*================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
#include "queue.h"

#define uxQueueLength 10
#define STRING_LENGTH 16
struct xUartVariable {
  char toPrint;
};
/*================================================================*/
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif

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
void uartPrinter(void *pvParameters) {
  char input[128];
  char strHolder[128];
  char strToPrint[STRING_LENGTH] = {0};
  int bytesRead;
  int i;
  int total_number_of_bytes = 0, current_str_length = 0;
  int strShift = 0;

  for (;;) {
    UARTprintf("Input: ");
    bytesRead = UARTgets(input, sizeof(input)); // bytesRead does not include \0
    for (i = 0; i < 128; i++) {
      strHolder[i] = 0;
    }
    total_number_of_bytes = bytesRead + strlen(strToPrint); // excluding \0
    strShift = total_number_of_bytes - STRING_LENGTH;
    for (i = 0; i < strlen(strToPrint); i++) {
      strHolder[i] = strToPrint[i];
    }
    strHolder[i] = '\0';
    current_str_length = strlen(strHolder);
    for (i = 0; i < bytesRead; i++) {
      strHolder[current_str_length] = input[i];
      current_str_length++;
    }
    strHolder[current_str_length + i] = '\0';

    if (total_number_of_bytes >= STRING_LENGTH) {

      while (strShift >= 0) {

        for (i = 0; i < strlen(strHolder);
             i++) { // Shift all characters to the left
          strHolder[i] = strHolder[i + 1];
        }
        strHolder[i] = '\0';
        strShift--;
      }
      for (i = 0; i < strlen(strHolder); i++) {
        strToPrint[i] = strHolder[i];
      }
      strToPrint[i] = '\0';
    } else {
      for (i = 0; i < strlen(strHolder); i++) {
        strToPrint[i] = strHolder[i];
      }
      strToPrint[i] = '\0';

    }
    UARTprintf("%s\n", strToPrint);
  }
}

void uartInput(void *pvParameters) {
  QueueHandle_t xQueue1 = *(QueueHandle_t *)pvParameters;
  char inputBuffer[STRING_LENGTH];
  char *pinputBuffer = inputBuffer;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(10); // 200 ms delay
  for (;;) {
    // vTaskDelayUntil(&xLastWakeTime, xDelay); // Delay for 200 ms
    // Maybe some check to if user inputs more letters than 1
    UARTprintf("Input: ");
    UARTgets(inputBuffer, sizeof(inputBuffer));
    UARTprintf("ECHO: %s\n", inputBuffer);
    while (*pinputBuffer) {
      if (xQueueSend(xQueue1, pinputBuffer, (TickType_t)portMAX_DELAY) !=
          pdPASS) {
        UARTprintf("Failed sending to queue\n");
      } // Send one character at a time to the queue
      pinputBuffer++;
    }
  }
}
int main(void) {
  // QueueHandle_t xQueue1 = NULL;
  TaskHandle_t xUartPrinterHandle, xUartInputHandle;
  BaseType_t xUartPrinterReturn, xUartInputReturn;
  // char buffer[STRING_LENGTH];
  ConfigureUART();
  UARTprintf("\033[2J");
  // xQueue1 = xQueueCreate(10, sizeof(char));
  // if (xQueue1 != NULL) {
  //   UARTprintf("Queue successfully created\n");
  // }
  xUartPrinterReturn = xTaskCreate(uartPrinter, "Print serial", 128, NULL, 1,
                                   &xUartPrinterHandle);
  // xUartInputReturn =
  //     xTaskCreate(uartInput, "Input serial", 256, NULL, 1,
  //     &xUartInputHandle);
  if (xUartPrinterReturn != pdFALSE) {
    UARTprintf("UARTPrinter task successfully started\n");
  }
  // if (xUartInputReturn != pdFALSE) {
  //   UARTprintf("UARTInput task successfully started\n");
  // }
  // UARTgets(buffer, sizeof(buffer));
  // UARTprintf("ECHO: %s\n", buffer);
  vTaskStartScheduler();
}
