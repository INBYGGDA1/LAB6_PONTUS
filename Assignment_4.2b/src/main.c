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
/*================================================================*/
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
#include "timers.h"
#include "queue.h"
#include "portable.h"

/*================================================================*/
#define STRING_LENGTH 15

/*================================================================*/
volatile int display_bytes_read = 0;
QueueHandle_t message_queue;

/*================================================================*/
struct xParam {
  char str_to_print[STRING_LENGTH];
  int total_number_of_bytes_read;
};

/*================================================================*/
void uartPrinter(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct xParam *param = pvPortMalloc(sizeof(struct xParam));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  char strHolder[128] = {0};
  char c;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int i = 0;
  int current_number_of_bytes = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    ResetString(strHolder, 128);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // UARTprintf("Input: ");
    // while (UARTCharsAvail(UART0_BASE)) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    c = UARTCharGet(UART0_BASE);
    current_number_of_bytes++; // After receiving a char we increase the
                               // bytes
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // If the total_number_of_bytes is greater than STRING_LENGTH we need to
    // shift the string
    if (current_number_of_bytes > STRING_LENGTH) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Shift by one character
      for (i = 0; i < strlen(param->str_to_print);
           i++) { // Copy previous string to a temp array
        strHolder[i] = param->str_to_print[i];
      }
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Append the latest character.
      strHolder[i] = c;
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Reset the printed string to avoid printing any characters which
      // should not be present
      ResetString(param->str_to_print, STRING_LENGTH);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Shift the string and copy it to the string to be printed, We only need
      // to shift once since it is one first character
      for (i = 0; i < STRING_LENGTH; i++) {
        param->str_to_print[i] = strHolder[i + 1];
      }
      // Decrement the current number of bytes to not make the variable go on
      // forever, We are not allowed to print more characters than
      // STRING_LENGTH so it is unecessary to keep incrementing it.
      current_number_of_bytes--;
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    } else {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // We just append the character if the current_number_of_bytes is <
      // STRING_LENGTH
      param->str_to_print[current_number_of_bytes - 1] = c;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Specify the null terminator to be the end of the string
    param->str_to_print[current_number_of_bytes] = '\0';
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Increase the total amount of bytes read.
    param->total_number_of_bytes_read++;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Send the string to the gatekeeper for printing
    if (xQueueSend(message_queue, param, (TickType_t)portMAX_DELAY) != pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // We should never reach this statement
      UARTprintf("Unable to send message to queue\n");
    }
  }
}

/*================================================================*/
void ConfigureBUTTON() {
  PinoutSet(false, false);
  ButtonsInit();
  GPIOPinTypeGPIOOutput(CLP_D1_PORT, CLP_D1_PIN);
}

/*================================================================*/
static void timer_callback(TimerHandle_t xTimer) {
  display_bytes_read = 0;
  LEDWrite(CLP_D1, 0);
}

/*================================================================*/
// Task to poll the button
void buttonPoller(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Button changed flag

  unsigned char ucDelta, ucState;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // 10s delay
  const TickType_t xDelay = pdMS_TO_TICKS(10000);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a 10s timer for the button timeout
  TimerHandle_t timer_handle =
      xTimerCreate("timer", xDelay, pdFALSE, NULL, timer_callback);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // poll for a button press
    ucState = ButtonsPoll(&ucDelta, 0);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta)) {

      display_bytes_read = 1;
      LEDWrite(CLP_D1, CLP_D1);
      xTimerStart(timer_handle, 0);
    }
  }
}

/*================================================================*/
// Function to continuously print
void gatekeeper(void *pvParameters) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int buttonTracker = -1;
  int toPrintOrNotToPrint = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct xParam *param = pvPortMalloc(sizeof(struct xParam));

  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    toPrintOrNotToPrint = 0;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // check for new input to the mq
    if (xQueueReceive(message_queue, param, (TickType_t)0) == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // indicate that we should update the terminal if
      toPrintOrNotToPrint = 1;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Has the button been pressed
    if (buttonTracker != display_bytes_read) {
      toPrintOrNotToPrint = 1;
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Update value to last known state
      buttonTracker = display_bytes_read;
    }
    if (toPrintOrNotToPrint) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      UARTClearScreen();
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Print the string
      UARTprintf("%s", param->str_to_print);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // If button also is pressed
      // print the bytes read value
      if (display_bytes_read == 1) {
        UARTprintf("\n%d", param->total_number_of_bytes_read);
      }
    }
  }
}

/*================================================================*/
int main(void) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TaskHandle_t xUartPrinterHandle, xGatekeeper, xButtonPoll;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t xUartPrinterReturn, xbGatekeeper, xbButtonPoll;

  uint32_t systemClock;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ConfigureSystemClock(120000000, &systemClock); // initialize the systemClock
  ConfigureUART();                               // Start uart
  ConfigureBUTTON();                             // init the buttons and leds
  UARTClearScreen();                             // Clear the serial terminal
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a message_queue
  message_queue = xQueueCreate(2, sizeof(struct xParam));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (message_queue != NULL) {
    UARTprintf("Queue successfully created\n");
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create the tasks
  xUartPrinterReturn = xTaskCreate(uartPrinter, "Print serial", 128, NULL, 1,
                                   &xUartPrinterHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xbGatekeeper =
      xTaskCreate(gatekeeper, "gatekeeper", 128, NULL, 1, &xGatekeeper);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xbButtonPoll =
      xTaskCreate(buttonPoller, "ButtonPoll", 128, NULL, 1, &xButtonPoll);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (xbButtonPoll != pdFALSE) {
    UARTprintf("xButtonPoll task successfully started\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (xUartPrinterReturn != pdFALSE) {
    UARTprintf("UARTPrinter task successfully started\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (xbGatekeeper != pdFALSE) {
    UARTprintf("Gatekeeper task successfully started\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  vTaskStartScheduler();
}
