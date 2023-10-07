//=============================================================================
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "tm4c129_functions.h"
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

//=============================================================================
#define WAIT_TIME 5
#define MIC_SAMPLES 8
#define JOY_SAMPLES 4
#define ACC_SAMPLES 2
#define QUEUE_SIZE 10

//=============================================================================
QueueHandle_t mic_queue;
QueueHandle_t joy_queue;
QueueHandle_t acc_queue;

//=============================================================================
struct microphone_values {
  uint32_t mic_val[MIC_SAMPLES];
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct joystick_values {
  uint32_t joy_x[JOY_SAMPLES];
  uint32_t joy_y[JOY_SAMPLES];
};
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
struct accelerometer_values {
  uint32_t acc_x[ACC_SAMPLES];
  uint32_t acc_y[ACC_SAMPLES];
  uint32_t acc_z[ACC_SAMPLES];
};

//=============================================================================
// Microphone task which takes 8 samples from the microhphone and sends the
// values to the xGatekeeper.
//=============================================================================
void task_mic(void *pvParameters) {
  const int T = 1;
  uint32_t samplesReturned = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct microphone_values mic_val;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a periodic delay
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Since the microphone is the only task using the "0" sequence it only has to
  // be initialize once
  ADC_newSequence(ADC0_BASE, 0, ADC_CTL_CH8, MIC_SAMPLES);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    vTaskDelayUntil(&xLasWakeTime, xDelay);
    samplesReturned = 0;
    sampleData(ADC0_BASE, 0, ADC_CTL_CH8, MIC_SAMPLES, &samplesReturned,
               mic_val.mic_val, 0);
    if (samplesReturned == MIC_SAMPLES) {
      while (xQueueSend(mic_queue, &mic_val, portMAX_DELAY) != pdTRUE) {
      }
    }
  }
}

//=============================================================================
// Joystick task which samples the x and y pin for the joystick
//=============================================================================
void task_joy(void *pvParameters) {
  const int T = 2;
  uint32_t samplesReturned = 0;
  int i = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct joystick_values joy;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a periodic delay
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay);
    samplesReturned = 0;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Since the joystick needs 4 samples per axis, we need to reinitialize the
    // sequence for each axis. Sequence 1 has maximum of 4 samples.
    sampleData(ADC0_BASE, 1, ADC_CTL_CH9, JOY_SAMPLES, &samplesReturned,
               joy.joy_x, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH0, JOY_SAMPLES, &samplesReturned,
               joy.joy_y, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (samplesReturned == 8) {
      while (xQueueSend(joy_queue, &joy, portMAX_DELAY) != pdTRUE) {
      }
    }
  }
}

//=============================================================================
// accelerometer task. Samples the values for the x,y & z axis and then send the
// samples to the xGatekeeper
//=============================================================================
void task_acc(void *pvParameters) {
  const int T = 4;
  int i = 0;
  uint32_t samplesReturned = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct accelerometer_values acc;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Create a periodic delay
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay);
    samplesReturned = 0;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH3, ACC_SAMPLES, &samplesReturned,
               acc.acc_x, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH2, ACC_SAMPLES, &samplesReturned,
               acc.acc_y, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH1, ACC_SAMPLES, &samplesReturned,
               acc.acc_z, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (samplesReturned == ACC_SAMPLES * 3) {
      while (xQueueSend(acc_queue, &acc, portMAX_DELAY) != pdTRUE) {
      }
    }
  }
}

//=============================================================================
// xGatekeeper, receive all values in the queeus, calculate the average and
// print values to UART
//=============================================================================
void xGatekeeper(void *pvParameters) {
  const int T = 8;
  int i = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t micAverage = 0;
  uint32_t micAverageTodB = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t joyAverageX = 0;
  uint32_t joyAverageY = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t accAverageX = 0;
  uint32_t accAverageY = 0;
  uint32_t accAverageZ = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t toPrintOrNotToPrint = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct joystick_values joy;
  struct accelerometer_values acc;
  struct microphone_values mic_val;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const TickType_t xDelay = pdMS_TO_TICKS(5 * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {

    vTaskDelayUntil(&xLasWakeTime, xDelay);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check for a message in the mic queue. Dont block!
    if (xQueueReceive(mic_queue, &mic_val, 0) == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      calculate_average(MIC_SAMPLES, mic_val.mic_val, &micAverage);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Convert the average to dB
      micAverageTodB = round(20.0 * log10(micAverage));
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // new samples has been retrieved, print them
      toPrintOrNotToPrint = 1;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check for a message in the joy_queue. Dont block!
    if (xQueueReceive(joy_queue, &joy, 0) == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Sum the samples
      calculate_average(JOY_SAMPLES, joy.joy_x, &joyAverageX);
      calculate_average(JOY_SAMPLES, joy.joy_y, &joyAverageY);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // New values has been retrieved, Print the values
      toPrintOrNotToPrint = 1;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check for message in the acc_queue. Dont block
    if (xQueueReceive(acc_queue, &acc, 0) == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      calculate_average(ACC_SAMPLES, acc.acc_x, &accAverageX);
      calculate_average(ACC_SAMPLES, acc.acc_y, &accAverageY);
      calculate_average(ACC_SAMPLES, acc.acc_z, &accAverageZ);
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      toPrintOrNotToPrint = 1;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // If new values has been retrieved we print them
    if (toPrintOrNotToPrint) {
      UARTprintf("\033[2J");
      UARTprintf("MIC: %d, dB\n", micAverageTodB);
      UARTprintf("ACC: %d x, %d y, %d z\n", accAverageX, accAverageY,
                 accAverageZ);
      UARTprintf("JOY: %d x, %d y\n", joyAverageX, joyAverageY);
      toPrintOrNotToPrint = 0;
    }
  }
}
int main(void) {
  uint32_t systemClock;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // There exist some errata with using the main oscillator and adc, hence the
  // internal oscillator is specified instead
  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_INT |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   120000000);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TaskHandle_t xTaskMicHandle;
  TaskHandle_t xTaskAccHandle;
  TaskHandle_t xTaskJoyHandle;
  TaskHandle_t xGatekeerperHandle;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t xTaskMicHandleReturn;
  BaseType_t xTaskAccHandleReturn;
  BaseType_t xTaskJoyHandleReturn;
  BaseType_t xGatekeerperHandleReturn;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ConfigureUART();
  UARTprintf("\033[2J");
  ADC_init(SYSCTL_PERIPH_ADC0);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable GPIO_PORTE Since this is where the
  // BoosterPack EDUMKII, joystick, accelerometer & microphone connects to.
  // See the table below.
  GPIOPort_init(SYSCTL_PERIPH_GPIOE);
  SENSOR_enable(MICROPHONE | JOYSTICK | ACCELEROMETER);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  mic_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct microphone_values));
  joy_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct joystick_values));
  acc_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct accelerometer_values));

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (mic_queue != NULL && joy_queue != NULL && acc_queue != NULL) {
    UARTprintf("Queues succesfully created\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xTaskMicHandleReturn =
      xTaskCreate(task_mic, "Mic", 128, NULL, 3, &xTaskMicHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xTaskAccHandleReturn =
      xTaskCreate(task_acc, "Acc", 128, NULL, 3, &xTaskAccHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xTaskJoyHandleReturn =
      xTaskCreate(task_joy, "Joy", 128, NULL, 3, &xTaskJoyHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xGatekeerperHandleReturn =
      xTaskCreate(xGatekeeper, "GP", 128, NULL, 2, &xGatekeerperHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (xTaskMicHandleReturn != pdFALSE && xTaskJoyHandleReturn != pdFALSE &&
      xTaskAccHandleReturn != pdFALSE && xGatekeerperHandleReturn != pdFALSE) {
    UARTprintf("Tasks started succesfully\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  vTaskStartScheduler();
}
