/*================================================================*/
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

/*================================================================*/
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "inc/hw_ints.h"
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
//=============================================================================
#define WAIT_TIME 5
#define MIC_SAMPLES 8
#define JOY_SAMPLES 4
#define ACC_SAMPLES 2

//=============================================================================
QueueHandle_t mic_queue, joy_queue, acc_queue;

//=============================================================================
struct microphone_values {
  uint32_t mic_val[MIC_SAMPLES];
};

struct joystick_values {
  uint32_t joy_x[JOY_SAMPLES];
  uint32_t joy_y[JOY_SAMPLES];
};

struct accelerometer_values {
  uint32_t acc_x[ACC_SAMPLES];
  uint32_t acc_y[ACC_SAMPLES];
  uint32_t acc_z[ACC_SAMPLES];
};
//=============================================================================

//=============================================================================
// The error routine that is called if the driver library
// encounters an error.
//=============================================================================
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif
//=============================================================================
// Configure the UART.
//=============================================================================
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
// Helper function to create a new sequence for the
// ADC. The sequence needs to be redinitialized for each
// channel since the maximum value one sequence can
// do is 8 which only works for sequence number 0.
// And since all structs need 8 + 6 + 4 > 8 we cant
// take the samples using one sequence.
//=============================================================================
void ADC_newSequence(uint32_t ui32base, uint32_t ui32SequenceNum,
                     uint32_t ui32ADC_Channel, uint32_t ui32Samples) {
  int i;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable interrupts for the specified ADC_BASE
  // and sequence number
  ADCIntEnable(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Clear any interrupts that might occur
  ADCIntClear(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Disable the sequencer before making any changes
  ADCSequenceDisable(ui32base, ui32SequenceNum);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Set the trigger for the sampling sequence to be manual
  ADCSequenceConfigure(ui32base, ui32SequenceNum, ADC_TRIGGER_PROCESSOR, 0);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Configure each step in the sampling process. Here we could add set another
  // channel to sample from in another step.
  for (i = 0; i < ui32Samples; i++) {
    ADCSequenceStepConfigure(ui32base, ui32SequenceNum, i, ui32ADC_Channel);
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // At the last step of the sequence, indicate that
  // it is the last and create an interrupt
  ADCSequenceStepConfigure(ui32base, ui32SequenceNum, ui32Samples - 1,
                           ADC_CTL_END | ADC_CTL_IE | ui32ADC_Channel);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Lastly we need to enable the sequence
  ADCSequenceEnable(ui32base, ui32SequenceNum);
}

//=============================================================================
// Helper function to sample the data
//=============================================================================
void sampleData(uint32_t ui32base, uint32_t ui32SequenceNum,
                uint32_t ui32ADC_Channel, uint32_t ui32Samples,
                uint32_t *ui32SamplesRead, void *buffer, uint32_t newSequence) {

  if (newSequence == 1) {
    ADC_newSequence(ui32base, ui32SequenceNum, ui32ADC_Channel, ui32Samples);
  }
  ADCProcessorTrigger(ui32base, ui32SequenceNum);
  while (!ADCIntStatus(ui32base, ui32SequenceNum, false)) {
  }
  *ui32SamplesRead += ADCSequenceDataGet(ui32base, ui32SequenceNum, buffer);
  ADCIntClear(ui32base, ui32SequenceNum);
}
//=============================================================================
void initialize_adc0(void) {
  int i;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable the ADC0 module
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlDelay(10);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable GPIO_PORTE Since this is where the
  // TM4C129EXL samples the BoosterPack
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlDelay(10);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Microphone (MIC): Pin PE5, GPIO_PIN_5, ADC_CTL_CH8, AIN8

  // Joystick (JOY):
  // - X-Axis: Pin PE4, GPIO_PIN_4, ADC_CTL_CH9, AIN9
  // - Y-Axis: Pin PE3, GPIO_PIN_3, ADC_CTL_CH0, AIN0

  // Gyroscope (ACC):
  // - X-Axis: Pin PE0, GPIO_PIN_0, ADC_CTL_CH3, AIN3
  // - Y-Axis: Pin PE1, GPIO_PIN_1, ADC_CTL_CH2, AIN2
  // - Z-Axis: Pin PE2, GPIO_PIN_2, ADC_CTL_CH1, AIN1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Enable all pins that should be set as ADC pins
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                                      GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
}

//=============================================================================
void task_mic(void *pvParameters) {
  const int T = 1;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct microphone_values *mic_val =
      pvPortMalloc(sizeof(struct microphone_values));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  uint32_t samplesReturned = 0;
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
               mic_val->mic_val, 0);
    if (samplesReturned == MIC_SAMPLES) {
      while (xQueueSend(mic_queue, mic_val, portMAX_DELAY) != pdTRUE) {
      }
    }
  }
}

//=============================================================================
void task_joy(void *pvParameters) {
  const int T = 2;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct joystick_values *joy = pvPortMalloc(sizeof(struct joystick_values));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int i = 0;
  uint32_t samplesReturned = 0;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay);
    samplesReturned = 0;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Since the joystick needs 4 samples per axis, we need to reinitialize the
    // sequence for each axis. Sequence 1 has maximum of 4 samples.
    sampleData(ADC0_BASE, 1, ADC_CTL_CH9, JOY_SAMPLES, &samplesReturned,
               joy->joy_x, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH0, JOY_SAMPLES, &samplesReturned,
               joy->joy_y, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (samplesReturned == 8) {
      while (xQueueSend(joy_queue, joy, portMAX_DELAY) != pdTRUE) {
      }
    }
  }
  vPortFree(joy);
}

void task_acc(void *pvParameters) {
  const int T = 4;
  int i = 0;
  uint32_t samplesReturned = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  const TickType_t xDelay = pdMS_TO_TICKS(WAIT_TIME * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct accelerometer_values *acc =
      pvPortMalloc(sizeof(struct accelerometer_values));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {
    vTaskDelayUntil(&xLasWakeTime, xDelay);
    samplesReturned = 0;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH3, ACC_SAMPLES, &samplesReturned,
               acc->acc_x, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH2, ACC_SAMPLES, &samplesReturned,
               acc->acc_y, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    sampleData(ADC0_BASE, 1, ADC_CTL_CH1, ACC_SAMPLES, &samplesReturned,
               acc->acc_z, 1);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (samplesReturned == ACC_SAMPLES * 3) {
      while (xQueueSend(acc_queue, acc, portMAX_DELAY) != pdTRUE) {
      }
    }
  }
  vPortFree(acc);
}
void xGatekeeper(void *pvParameters) {
  const int T = 8;
  int i = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int micAverageTemp = 0;
  int micAverage = 0;
  int micAverageTodB = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int joyAverageXtemp = 0;
  int joyAverageYtemp = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int joyAverageX = 0;
  int joyAverageY = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int accAverageXtemp = 0;
  int accAverageYtemp = 0;
  int accAverageZtemp = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int accAverageX = 0;
  int accAverageY = 0;
  int accAverageZ = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int toPrintOrNotToPrint = 0;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t micQueueReturned;
  BaseType_t joyQueueReturned;
  BaseType_t accQueueReturned;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct joystick_values *joy = pvPortMalloc(sizeof(struct joystick_values));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct accelerometer_values *acc =
      pvPortMalloc(sizeof(struct accelerometer_values));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct microphone_values *mic_val =
      pvPortMalloc(sizeof(struct microphone_values));
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  const TickType_t xDelay = pdMS_TO_TICKS(5 * T);
  TickType_t xLasWakeTime = xTaskGetTickCount();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  for (;;) {

    vTaskDelayUntil(&xLasWakeTime, xDelay); // Make the task periodic

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Reset all the calculations
    toPrintOrNotToPrint = 0;
    micAverageTemp = 0;
    joyAverageXtemp = 0;
    joyAverageYtemp = 0;
    accAverageXtemp = 0;
    accAverageYtemp = 0;
    accAverageZtemp = 0;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check for a message in the mic queue. Dont block!
    micQueueReturned = xQueueReceive(mic_queue, mic_val, 0);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (micQueueReturned == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Sum all the values collected
      for (i = 0; i < MIC_SAMPLES; i++) {
        micAverageTemp += mic_val->mic_val[i];
      }
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Calculate the average of the samples
      micAverage = micAverageTemp / MIC_SAMPLES;
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Convert the average to dB
      micAverageTodB = round(20.0 * log10(micAverage));
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // new samples has been retrieved, print them
      toPrintOrNotToPrint = 1;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check for a message in the joy_queue. Dont block!
    joyQueueReturned = xQueueReceive(joy_queue, joy, 0);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (joyQueueReturned == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Sum the samples
      for (i = 0; i < JOY_SAMPLES; i++) {
        joyAverageXtemp += joy->joy_x[i];
        joyAverageYtemp += joy->joy_y[i];
      }
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Calculate the average for the joystick
      joyAverageX = joyAverageXtemp / JOY_SAMPLES;
      joyAverageY = joyAverageYtemp / JOY_SAMPLES;
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // New values has been retrieved, Print the values
      toPrintOrNotToPrint = 1;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Check for message in the acc_queue. Dont block
    accQueueReturned = xQueueReceive(acc_queue, acc, 0);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if (accQueueReturned == pdPASS) {
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Sum the samples on their respective axis
      for (i = 0; i < ACC_SAMPLES; i++) {
        accAverageXtemp += acc->acc_x[i];
        accAverageYtemp += acc->acc_y[i];
        accAverageZtemp += acc->acc_z[i];
      }
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Calculate the average
      accAverageX = accAverageXtemp / ACC_SAMPLES;
      accAverageY = accAverageYtemp / ACC_SAMPLES;
      accAverageZ = accAverageZtemp / ACC_SAMPLES;
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
    }
  }
  vPortFree(mic_val);
  vPortFree(acc);
  vPortFree(joy);
}
int main(void) {
  uint32_t systemClock;
  systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_INT |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   120000000);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int queueSize = 3;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TaskHandle_t xTaskMicHandle, xTaskAccHandle, xTaskJoyHandle,
      xGatekeerperHandle;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  BaseType_t xTaskMicHandleReturn, xTaskAccHandleReturn, xTaskJoyHandleReturn,
      xGatekeerperHandleReturn;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  struct microphone_values mic;
  struct accelerometer_values acc;
  struct joystick_values joy;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ConfigureUART();
  UARTprintf("\033[2J");
  initialize_adc0();

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  mic_queue = xQueueCreate(queueSize, sizeof(mic));
  joy_queue = xQueueCreate(queueSize, sizeof(joy));
  acc_queue = xQueueCreate(queueSize, sizeof(acc));

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (mic_queue != NULL && joy_queue != NULL && acc_queue != NULL) {
    UARTprintf("Queues succesfully created\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xTaskMicHandleReturn =
      xTaskCreate(task_mic, "Mic", 512, NULL, 3, &xTaskMicHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xTaskAccHandleReturn =
      xTaskCreate(task_acc, "Acc", 512, NULL, 3, &xTaskAccHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xTaskJoyHandleReturn =
      xTaskCreate(task_joy, "Joy", 512, NULL, 3, &xTaskJoyHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  xGatekeerperHandleReturn =
      xTaskCreate(xGatekeeper, "GP", 512, NULL, 2, &xGatekeerperHandle);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (xTaskMicHandleReturn != pdFALSE && xTaskJoyHandleReturn != pdFALSE &&
      xTaskAccHandleReturn != pdFALSE && xGatekeerperHandleReturn != pdFALSE) {
    UARTprintf("Tasks started succesfully\n");
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  vTaskStartScheduler();
}
