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
#include "drivers/buttons.h"
#include "drivers/pinout.h"
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

#define STRING_LENGTH 16
volatile int display_bytes_read = 0;
struct xParam {
  char str_to_print[STRING_LENGTH];
  int total_number_of_bytes_read;
  QueueHandle_t message_queue;
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
  struct xParam param = *(struct xParam *)pvParameters;
  char input[128];
  char strHolder[128];
  int bytesRead, i;
  int total_number_of_bytes = 0, current_str_length = 0, strShift = 0;

  for (;;) {
    UARTprintf("Input: ");
    bytesRead = UARTgets(input, sizeof(*input)); // bytesRead does not include \0
    for (i = 0; i < 128; i++) {                 // Reset the array
      strHolder[i] = 0;
    }

    total_number_of_bytes =
        bytesRead + strlen(param.str_to_print);    // excluding \0
    param.total_number_of_bytes_read += bytesRead; // Update the bytes read
    strShift =
        total_number_of_bytes -
        STRING_LENGTH; // If the value is positive we need to shift the string
    for (i = 0; i < strlen(param.str_to_print);
         i++) { // Copy previous string to a temp array
      strHolder[i] = param.str_to_print[i];
    }
    strHolder[i] = '\0';
    current_str_length =
        strlen(strHolder); // Append the input string to the previous string
    for (i = 0; i < bytesRead; i++) {
      strHolder[current_str_length] = input[i];
      current_str_length++;
    }
    strHolder[current_str_length + i] = '\0';

    if (total_number_of_bytes >=
        STRING_LENGTH) { // If the string is greater than STRING_LENGTH we need
                         // to shift it

      while (strShift >= 0) {

        for (i = 0; i < strlen(strHolder);
             i++) { // Shift all characters to the left
          strHolder[i] = strHolder[i + 1];
        }
        strHolder[i] = '\0';
        strShift--;
      }
      for (i = 0; i < strlen(strHolder); i++) { // Copy to the str to be printed
        param.str_to_print[i] = strHolder[i];
      }
      param.str_to_print[i] = '\0';
    } else {
      for (i = 0; i < strlen(strHolder); i++) {
        param.str_to_print[i] = strHolder[i];
      }
      param.str_to_print[i] = '\0';
    }
    xQueueSend(param.message_queue, (const void *)&param,
               (TickType_t)portMAX_DELAY);
  }
}

void ConfigureBUTTON() {
  PinoutSet(false, false);
  ButtonsInit();
  GPIOPinTypeGPIOOutput(CLP_D1_PORT, CLP_D1_PIN);
}
void buttonPoller(void *pvParameters) {
  // struct xParam param = *(struct xParam *)pvParameters;
  unsigned char ucDelta, ucState;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xDelay = pdMS_TO_TICKS(10000); // 200 ms delay
  for (;;) {

    ucState = ButtonsPoll(&ucDelta, 0);

    if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta)) {

      display_bytes_read = 1;
      LEDWrite(CLP_D1, CLP_D1);

      vTaskDelayUntil(&xLastWakeTime, xDelay);
      LEDWrite(CLP_D1, 0);
      display_bytes_read = 0;
    }
  }
}

void gatekeeper(void *pvParameters) {
  struct xParam param = *(struct xParam *)pvParameters;
  for (;;) {
    xQueueReceive(param.message_queue, &param, (TickType_t)1);
    if (display_bytes_read == 1) {
      UARTprintf("\033[2J");
      UARTprintf("%s\n", param.str_to_print);
      UARTprintf("%d\n", param.total_number_of_bytes_read);
    } else {
      UARTprintf("\033[2J");
      UARTprintf("%s\n", param.str_to_print);
    }
  }
}
int main(void) {
  QueueHandle_t xQueue1 = NULL;
  TaskHandle_t xUartPrinterHandle, xGatekeeper, xButtonPoll;
  BaseType_t xUartPrinterReturn, xbGatekeeper, xbButtonPoll;
  struct xParam param;

  param.message_queue = xQueue1;
  param.total_number_of_bytes_read = 0;

  ConfigureUART();
  ConfigureBUTTON();
  UARTprintf("\033[2J");
  xQueue1 = xQueueCreate(10, sizeof(struct xParam));
  if (xQueue1 != NULL) {
    UARTprintf("Queue successfully created\n");
  }
  xUartPrinterReturn = xTaskCreate(uartPrinter, "Print serial", 128,
                                   (void *)&param, 1, &xUartPrinterHandle);
  xbGatekeeper = xTaskCreate(gatekeeper, "gatekeeper", 128, (void *)&param, 1,
                             &xGatekeeper);
  xbButtonPoll =
      xTaskCreate(buttonPoller, "ButtonPoll", 128, NULL, 1, &xButtonPoll);
  if (xbButtonPoll != pdFALSE) {
    UARTprintf("xButtonPoll task successfully started\n");
  }
  if (xUartPrinterReturn != pdFALSE) {
    UARTprintf("UARTPrinter task successfully started\n");
  }
  if (xbGatekeeper != pdFALSE) {
    UARTprintf("Gatekeeper task successfully started\n");
  }
  vTaskStartScheduler();
}
