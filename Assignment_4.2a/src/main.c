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

#define STRING_LENGTH 16 // 15 characters + \0
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
/*================================================================*/
void uartPrinter(void *pvParameters) {
  char input[128] = {0};
  char strHolder[128] = {0};
  char strToPrint[STRING_LENGTH] = {0};
  int bytesRead = 0, i = 0;
  int total_number_of_bytes = 0, current_str_length = 0, strShift = 0;

  for (;;) {
    UARTprintf("Input: ");
    bytesRead = UARTgets(input, sizeof(input)); // bytesRead does not include \0
    for (i = 0; i < 128; i++) {                 // Reset the array
      strHolder[i] = 0;
    }

    total_number_of_bytes = bytesRead + strlen(strToPrint); // excluding \0
    strShift =
        total_number_of_bytes -
        STRING_LENGTH; // If the value is positive we need to shift the string
    for (i = 0; i < strlen(strToPrint);
         i++) { // Copy previous string to a temp array
      strHolder[i] = strToPrint[i];
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

      while (strShift >= 0) { // strShift == 0 is one shift since the \0 is
                              // included in the STRING_LENGTH

        for (i = 0; i < strlen(strHolder);
             i++) { // Shift all characters to the left
          strHolder[i] = strHolder[i + 1];
        }
        strHolder[i] = '\0';
        strShift--;
      }
      for (i = 0; i < strlen(strHolder); i++) { // Copy to the str to be printed
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

/*================================================================*/
int main(void) {
  TaskHandle_t xUartPrinterHandle;
  BaseType_t xUartPrinterReturn;
  ConfigureUART();
  UARTprintf("\033[2J");
  xUartPrinterReturn = xTaskCreate(uartPrinter, "Print serial", 128, NULL, 1,
                                   &xUartPrinterHandle);
  if (xUartPrinterReturn != pdFALSE) {
    UARTprintf("UARTPrinter task successfully started\n");
  }
  vTaskStartScheduler();
}
