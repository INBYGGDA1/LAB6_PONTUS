/*================================================================*/
#include <math.h>
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
#include "timers.h"
#include "queue.h"
#include "portable.h"

#define WAIT_TIME 20
// If below 50 then joystick-x and microphone becomes correlated
#define BLOCK_TIME 50

QueueHandle_t mic_queue, joy_queue, acc_queue;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct microphone_values {
  uint32_t mic_val[8];
};
struct joystick_values {
  uint32_t joy_x[4];
  uint32_t joy_y[4];
};
struct accelerometer_values {
  uint32_t acc_x[2];
  uint32_t acc_y[2];
  uint32_t acc_z[2];
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// The error routine that is called if the driver library
// encounters an error.
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif
//=============================================================================
// Configure the UART.
void ConfigureUART(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
}
//=============================================================================
// Initialize ADC0 for accelerometer (gyroscope)
void initialize_adc0(void) {
  // Enable the ADC0 module.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
  }
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)) {
  }
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {
  }
}
//=============================================================================

void task_mic(void *pvParameters) {

  int T = 1;
  int i = 0;
  int samplesReturned = 0;

  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T); // 10s delay
  TickType_t xLasWakeTime = xTaskGetTickCount();
  struct microphone_values *mic_val =
      pvPortMalloc(sizeof(struct microphone_values)); // 8 samples

  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay); // Periodicity with 1

    int toSend = 0;
    for (i = 0; i < 8; i++) {
      mic_val->mic_val[i] = 0;
    }
    // Sample some data
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR,
                         3); // A sequence of 0 collects 8 samples
    ADCSequenceStepConfigure(ADC1_BASE, 0, 7,
                             ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH9);
    ADCSequenceEnable(ADC1_BASE, 0); // Enable the ADC sequence
    // ADCIntEnable(ADC1_BASE, 0);
    ADCProcessorTrigger(
        ADC1_BASE,
        0); // // Wait until the sample sequence has completed. //
    while (!ADCIntStatus(ADC1_BASE, 0, false)) {
    } // // Read the
      // value from the ADC.
      // for (i = 0; i < 8; i++) {

    samplesReturned = ADCSequenceDataGet(
        ADC1_BASE, 0,
        mic_val->mic_val); // Providing the full array should place all the
                           // value in FIFO in the array (Up to 8 values)
    // }
    for (i = 0; i < 8; i++) {
      if (mic_val->mic_val[i] != 0) {
        toSend++;
      }
    }
    xQueueSend(mic_queue, &mic_val, 50);
    if (toSend == 8) {
    }
  }
}
void task_joy(void *pvParameters) {
  int T = 2;
  int i;
  int samplesReturned = 0;
  const TickType_t xDelay =
      pdMS_TO_TICKS(WAIT_TIME * T); // 10s delay
  TickType_t xLasWakeTime = xTaskGetTickCount();
  struct joystick_values *joy = pvPortMalloc(sizeof(struct joystick_values));
  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay); // Periodicity with 1
    int toSend = 0;
    for (i = 0; i < 4; i++) {
      joy->joy_x[i] = 5000;
      joy->joy_y[i] = 5000;
    }

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
    // Sample some data
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR,
                         2); // A sequence of 2, captures up to 4 samples
    ADCSequenceStepConfigure(ADC1_BASE, 0, 7,
                             ADC_CTL_IE | ADC_CTL_END |
                                 ADC_CTL_CH0); // Since the buffer is of size 0,
                                               // 0 samples will get discarded
    ADCSequenceEnable(ADC1_BASE, 0);
    // ADCIntEnable(ADC1_BASE, 0);
    ADCProcessorTrigger(ADC1_BASE, 0);
    while (!ADCIntStatus(ADC1_BASE, 0, false)) {
    }
    samplesReturned = ADCSequenceDataGet(ADC1_BASE, 0, joy->joy_x);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
    // ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR,
    //                      0); // A sequence of 0, captures up to 4 samples
    ADCSequenceStepConfigure(ADC1_BASE, 0, 7,
                             ADC_CTL_IE | ADC_CTL_END |
                                 ADC_CTL_CH8); // Channel 8
    ADCSequenceEnable(ADC1_BASE,
                      0); // Enable the sequence for channel 8
    // ADCIntEnable(ADC1_BASE, 0);
    ADCProcessorTrigger(ADC1_BASE, 0); // Trigger the sampling
    while (!ADCIntStatus(ADC1_BASE, 0, false)) {
    }
    ADCSequenceDataGet(ADC1_BASE, 0,
                       joy->joy_y); // Place the data in the y buffer
    for (i = 0; i < 4; i++) {       // Check so that all arrays have a value
      if (joy->joy_x[i] != 5000 && joy->joy_y[i] != 5000) {
        toSend++;
      }
    }
    xQueueSend(joy_queue, &joy, 50);
    if (toSend == 4) {
      // Dont send
    }
  }
  vPortFree(joy);
}

void task_acc(void *pvParameters) {
  int T = 4;
  int i;
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T); // 10s delay
  TickType_t xLasWakeTime = xTaskGetTickCount();
  struct accelerometer_values *acc =
      pvPortMalloc(sizeof(struct accelerometer_values));
  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay); // Periodicity with 1
    int toSend = 0;
    for (i = 0; i < 2; i++) {
      acc->acc_x[i] = 0;
      acc->acc_y[i] = 0;
      acc->acc_z[i] = 0;
    }

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR,
                         0); // A sequence of 2, captures up to 4 samples
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1,
                             ADC_CTL_IE | ADC_CTL_END |
                                 ADC_CTL_CH2); // Channel 8
    ADCSequenceEnable(ADC0_BASE,
                      2); // Enable the sequence for channel 8
    // ADCIntEnable(ADC1_BASE, 2);
    ADCProcessorTrigger(ADC0_BASE, 2); // Trigger the sampling
    while (!ADCIntStatus(ADC0_BASE, 2, false)) {
    }
    ADCSequenceDataGet(ADC0_BASE, 2,
                       acc->acc_x); // Place the data in the x buffer

    // Sample some data

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    // ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR,
    //                      0); // A sequence of 2, captures up to 4 samples
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1,
                             ADC_CTL_IE | ADC_CTL_END |
                                 ADC_CTL_CH1); // Channel 8
    ADCSequenceEnable(ADC0_BASE,
                      2); // Enable the sequence for channel 8
    // ADCIntEnable(ADC1_BASE, 2);
    ADCProcessorTrigger(ADC0_BASE, 2); // Trigger the sampling
    while (!ADCIntStatus(ADC0_BASE, 2, false)) {
    }
    ADCSequenceDataGet(ADC0_BASE, 2,
                       acc->acc_y); // Place the data in the x buffer

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    // ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR,
    //                      0); // A sequence of 2, captures up to 4 samples
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1,
                             ADC_CTL_IE | ADC_CTL_END |
                                 ADC_CTL_CH1); // Channel 8
    ADCSequenceEnable(ADC0_BASE,
                      2); // Enable the sequence for channel 8
    // ADCIntEnable(ADC1_BASE, 2);
    ADCProcessorTrigger(ADC0_BASE, 2); // Trigger the sampling
    while (!ADCIntStatus(ADC0_BASE, 2, false)) {
    }
    ADCSequenceDataGet(ADC0_BASE, 2,
                       acc->acc_z); // Place the data in the x buffer

    for (i = 0; i < 2; i++) {
      if (acc->acc_x[i] != 0 && acc->acc_y[i] != 0 && acc->acc_z[i] != 0) {
        toSend++;
      }
    }
    xQueueSend(acc_queue, &acc, 50);
    if (toSend == 2) {
      // Dont send
      // Unable to send
    }
  }
  vPortFree(acc);
}
void xGatekeeper(void *pvParameters) {
  int T = 8;
  int i;
  int micAverageTemp = 0;
  int micAverage = 0;
  int micAverageTodB = 0;
  int joyAverageXtemp = 0, joyAverageYtemp = 0;
  int joyAverageX = 0, joyAverageY = 0;
  int accAverageXtemp = 0, accAverageYtemp = 0, accAverageZtemp = 0;
  int accAverageX = 0, accAverageY = 0, accAverageZ = 0;
  BaseType_t micQueueReturned, joyQueueReturned, accQueueReturned;
  struct joystick_values *joy = pvPortMalloc(sizeof(struct joystick_values));
  struct accelerometer_values *acc =
      pvPortMalloc(sizeof(struct accelerometer_values));
  struct microphone_values *mic_val =
      pvPortMalloc(sizeof(struct microphone_values));
  const TickType_t xDelay = pdMS_TO_TICKS(5 * T); // 10s delay
  TickType_t xLasWakeTime = xTaskGetTickCount();

  for (;;) {

    vTaskDelayUntil(&xLasWakeTime, xDelay);

    micAverageTemp = 0;
    joyAverageXtemp = 0;
    joyAverageYtemp = 0;
    accAverageXtemp = 0;
    accAverageYtemp = 0;
    accAverageZtemp = 0;

    int toPrintOrNotToPrint = 0;
    micQueueReturned = xQueueReceive(mic_queue, &mic_val, portMAX_DELAY);
    if (micQueueReturned == pdPASS) {
      // UARTprintf("MIC: ");
      // for (i = 0; i < 8; i++) {
      //
      //   UARTprintf("%d ", mic_val->mic_val[i]);
      // }
      // UARTprintf("\n");
      for (i = 0; i < 8; i++) {
        micAverageTemp += mic_val->mic_val[i];
      }
      micAverage = micAverageTemp / 8;
      micAverageTodB = round(20.0 * log10(micAverage));
      toPrintOrNotToPrint = 1;
    }
    joyQueueReturned = xQueueReceive(joy_queue, &joy, portMAX_DELAY);
    if (joyQueueReturned == pdPASS) {
      // UARTprintf("JOY-X: ");
      // for (i = 0; i < 4; i++) {
      //
      //   UARTprintf("%d ", joy->joy_x[i]);
      // }
      //
      // UARTprintf("\n");
      // UARTprintf("JOY-Y: ");
      // for (i = 0; i < 4; i++) {
      //
      //   UARTprintf("%d ", joy->joy_y[i]);
      // }
      // UARTprintf("\n");
      for (i = 0; i < 4; i++) {
        joyAverageXtemp += joy->joy_x[i];
        joyAverageYtemp += joy->joy_y[i];
      }
      joyAverageX = joyAverageXtemp / 4;
      joyAverageY = joyAverageYtemp / 4;
      toPrintOrNotToPrint = 1;
    }
    accQueueReturned = xQueueReceive(acc_queue, &acc, portMAX_DELAY);
    if (accQueueReturned == pdPASS) {
      for (i = 0; i < 2; i++) {
        accAverageXtemp += acc->acc_x[i];
        accAverageYtemp += acc->acc_y[i];
        accAverageZtemp += acc->acc_z[i];
      }
      accAverageX = accAverageXtemp / 2;
      accAverageY = accAverageYtemp / 2;
      accAverageZ = accAverageZtemp / 2;
      toPrintOrNotToPrint = 1;
    }

    if (toPrintOrNotToPrint) {
      UARTprintf("\033[2J");
      // UARTprintf("MIC: %d, dB\n", micAverageTodB);
      // UARTprintf("ACC: %d x, %d y, %d z\n", accAverageX, accAverageY,
      //            accAverageZ);
      // UARTprintf("JOY: %d x, %d y\n", joyAverageX, joyAverageY);
      UARTprintf("MIC: %d, dB\n", mic_val->mic_val[0]);
      UARTprintf("ACC: %d x, %d y, %d z\n", acc->acc_x[0], acc->acc_y[0],
                 acc->acc_z[0]);
      UARTprintf("JOY: %d x, %d y\n", joy->joy_x[0], joy->joy_y[0]);
    }
  }
  vPortFree(mic_val);
  vPortFree(acc);
  vPortFree(joy);
}
int main(void) {
  int queueSize = 3;
  TaskHandle_t xTaskMicHandle, xTaskAccHandle, xTaskJoyHandle,
      xGatekeerperHandle;
  BaseType_t xTaskMicHandleReturn, xTaskAccHandleReturn, xTaskJoyHandleReturn,
      xGatekeerperHandleReturn;
  struct microphone_values mic;
  struct accelerometer_values acc;
  struct joystick_values joy;
  ConfigureUART();
  UARTprintf("\033[2J");
  initialize_adc0();

  mic_queue = xQueueCreate(queueSize, sizeof(&mic));
  joy_queue = xQueueCreate(queueSize, sizeof(&joy));
  acc_queue = xQueueCreate(queueSize, sizeof(&acc));

  if (mic_queue != NULL && joy_queue != NULL && acc_queue != NULL) {
    UARTprintf("Queues succesfully created\n");
  }
  xTaskMicHandleReturn =
      xTaskCreate(task_mic, "Mic", 512, NULL, 3, &xTaskMicHandle);
  xTaskAccHandleReturn =
      xTaskCreate(task_acc, "Acc", 512, NULL, 3, &xTaskAccHandle);
  xTaskJoyHandleReturn =
      xTaskCreate(task_joy, "Joy", 512, NULL, 3, &xTaskJoyHandle);
  xGatekeerperHandleReturn =
      xTaskCreate(xGatekeeper, "GP", 512, NULL, 2, &xGatekeerperHandle);
  if (xTaskMicHandleReturn != pdFALSE && xTaskJoyHandleReturn != pdFALSE &&
      xTaskAccHandleReturn != pdFALSE && xGatekeerperHandleReturn != pdFALSE) {
    UARTprintf("Tasks started succesfully\n");
  }

  vTaskStartScheduler();
}
