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
#include "tm4c129_functions.h"

/*================================================================*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#define STRING_LENGTH 15 // 15 characters + \0
/*================================================================*/
void uartPrinter(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  char strHolder[128] = {0};
  char strToPrint[STRING_LENGTH] = {0};
  char c;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int i = 0;
  int total_number_of_bytes = 0, current_str_length = 0;

  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ResetString(strHolder, 128);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // UARTprintf("Input: ");
    while (UARTCharsAvail(UART0_BASE)) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTClearScreen();
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      c = UARTCharGet(UART0_BASE);
      total_number_of_bytes++; // After receiving a char we increase the bytes

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // If the total_number_of_bytes is greater than STRING_LENGTH we need to
      // shift the string
      if (total_number_of_bytes > STRING_LENGTH) {
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Shift by one character
        for (i = 0; i < strlen(strToPrint);
             i++) { // Copy previous string to a temp array
          strHolder[i] = strToPrint[i];
        }
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Append the latest character.
        strHolder[i] = c;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Reset the printed string to avoid printing any characters which
        // should not be present
        ResetString(strToPrint, STRING_LENGTH);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Shift the string, We only need to shift once since it is the first
        // character
        for (i = 0; i < STRING_LENGTH; i++) {
          strToPrint[i] = strHolder[i + 1];
        }
      } else {
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // We just append the character
        strToPrint[total_number_of_bytes - 1] = c;
      }
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Specify the null terminator to be the end of the string
      strToPrint[total_number_of_bytes] = '\0';
      UARTprintf("%s", strToPrint);
    }
  }
}

/*================================================================*/
int main(void) {
  TaskHandle_t xUartPrinterHandle;
  BaseType_t xUartPrinterReturn;
  ConfigureUART();
  UARTClearScreen();
  xUartPrinterReturn = xTaskCreate(uartPrinter, "Print serial", 128, NULL, 1,
                                   &xUartPrinterHandle);
  if (xUartPrinterReturn != pdFALSE) {
    UARTprintf("UARTPrinter task successfully started\n");
  }
  vTaskStartScheduler();
}
