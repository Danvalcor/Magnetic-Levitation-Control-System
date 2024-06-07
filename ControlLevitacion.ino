// Define the microcontroller
#define PART_TM4C123GH6PM

// Include required libraries
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"

// Variable Definitions
// PID variables
#define pid_max 255       // Define the maximum value the PID control loop can have. This value is later scaled to match the dutyCycle range.
#define pid_min 1         // Define the minimum value the PID control loop can have. This value is limited to 1 since the dutyCycle register cannot be less than 1.
float distanceBuffer[3];  // Stores the last three previous distances. Used to estimate the centered integral of the distance.
float error = 0;          // Calculated error, set point - distance.
float setPoint = 0;       // Set the current reference point in millimeters.
float proportional = 0;   // Proportional term of the PID control loop.
float integral = 0;       // Integral term of the PID control loop.
float derivative = 0;     // Derivative term of the PID control loop.
float kp = 0;             // Proportional gain of the PID control. This value is received from the computer.
float kd = 0;             // Derivative gain of the PID control. This value is received from the computer.
float ki = 0;             // Integral gain of the PID control. This value is received from the computer.
float dt = 0;             // Time step of the control task in seconds.
float pid = 0;            // Total output of the PID. It is the sum of the proportional, integral, and derivative terms.

// ADC variables
#define SENSOR GPIO_PIN_3      // Sensor input. (PE_3)
uint32_t sensor = 0;           // Stores the ADC value of the sensor.
float distance = 0;            // Stores the calculated distance with it.

// Define LEDs to indicate different states.
#define RED_LED       GPIO_PIN_1      // Red LED on the Tiva C. (PF_1)
#define BLUE_LED      GPIO_PIN_2      // Blue LED on the Tiva C. (PF_2)
#define GREEN_LED     GPIO_PIN_3      // Green LED on the Tiva C. (PF_3)

// RTOS variables
volatile bool doControlFlag = false;  // Flag set by the SysTick timer. Indicates it is time to execute the control task.

// PWM variables
uint32_t pwmPeriod = 0;         // PWM generator period. Sets the PWM frequency. This value is received from the computer.
uint32_t dutyCycle = 0;         // Final dutyCycle variable passed to the PID library.
#define enable GPIO_PIN_0       // Output to the H-bridge enable pin. (PD_0)
#define output GPIO_PIN_1       // Output to the H-bridge direction pin 1. (PD_1)

// UART variables
uint8_t sendBuffer[4];          // Buffer to store the values to be sent to the computer.
uint8_t buffer[20];             // Buffer to store the values received from the computer.
bool startSavingBytes = false;  // Flag indicating that a start byte has been received and all subsequent bytes should be stored in the receive buffer.
bool sendData = false;
uint32_t bufferCount = 0;       // Number of bytes placed in the receive buffer.
uint8_t inByte = 0;             // Currently received byte from the serial.
bool receivedParametersFlag = false; // Flag set to true once control constants have been received from the computer.

void ISRSysTick(void) {
  // Enable the control task.
  doControlFlag = true;
}

void send() {
  // Send the bytes currently stored in the buffer over serial.
  UARTCharPut(UART0_BASE, sendBuffer[3]);
  UARTCharPut(UART0_BASE, sendBuffer[2]);
  UARTCharPut(UART0_BASE, sendBuffer[1]);
  UARTCharPut(UART0_BASE, sendBuffer[0]);
}

void sendFloat(float f) {
  // Send a float via serial. The value of f is copied into the send buffer before sending it over serial.
  memcpy(&sendBuffer, &f, sizeof f);
  send();
}

void setup() {
  // Enable the floating-point unit. Enables lazy stacking in the FPU, so floating-point calculations can be performed within an interrupt.
  // Page 243 of the TivaWare datasheet.
  FPUEnable();
  FPULazyStackingEnable();

  // Set the processor clock to 80 MHz.
  // Page 479 of the TivaWare datasheet.
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // Set the PWM clock to 80 MHz / 32 = 2.5 MHz
  SysCtlPWMClockSet(SYSCTL_PWMDIV_32);
  //SysCtlPWMClockSet(SYSCTL_PWMDIV_64); Check

  // Enable GPIO ports F, A, D, E and wait until they are ready.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);            // LEDs are on GPIOF.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);            // The serial port is on GPIOA.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);            // Control output is on GPIOD.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);            // Analog inputs are on GPIOE.
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

  // Enable UART0, PWM1, and ADC0 peripherals, and wait until they are ready.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

  // Configure the following pins as ADC inputs.
  GPIOPinTypeADC(GPIO_PORTE_BASE, SENSOR);

  // Configure the following pins as digital outputs.
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);

  // Configure the following pins as PWM outputs.
  GPIOPinConfigure(GPIO_PD0_M1PWM0);
  GPIOPinTypePWM(GPIO_PORTD_BASE, enable);

  // Configure the following pins as UART inputs/outputs.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Configure the PWM generator. The port PD_0 is the PWM output. This can be found in Table 23.3 Signal Names on page 1339 of the Tiva datasheet.
  PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  // Set the PWM period in PWM clock ticks.
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 65000);

  // Set the pulse width in PWM clock ticks. This value must be less than the PWM period but greater than 0.
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);

  // Enable the PWM module.
  PWMGenEnable(PWM1_BASE, PWM_GEN_0);
  PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

  // Configure the ADC to trigger on demand. Also enable hardware averaging to compute an average of 16 ADC samples.
  // The hardware sample averaging circuit can be found on page 807 of the Tiva datasheet.
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
  ADCHardwareOversampleConfigure(ADC0_BASE, 16);
  ADCSequenceEnable(ADC0_BASE, 3);            // Enable the ADC to be triggered
  ADCIntClear(ADC0_BASE, 3);

  // Configure the UART peripheral to operate at 1000000 baud.
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 1000000, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  // Enable system interrupts.
  IntMasterEnable();

  // Configure the SysTick timer to trigger an interrupt every 140 us. This interrupt signals
  // when the control task should be executed. Modifying this value changes the system speed.
  // The value is calculated using clock ticks as follows:
  // 1/80 MHz = 12.5 ns
  // 140 us / 12.5 ns = 11200
  SysTickPeriodSet(11200);
  SysTickEnable();
  SysTickIntEnable();

  // Enable the LEDs to indicate system initialization is complete.
  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, GREEN_LED);
}

void readSensor() {
  // Trigger a conversion.
  ADCProcessorTrigger(ADC0_BASE, 3);
  // Wait until the conversion is completed.
  while (!ADCIntStatus(ADC0_BASE, 3, false)) {}
  // Clear the ADC interrupt flag.
  ADCIntClear(ADC0_BASE, 3);
  // Read the value.
  ADCSequenceDataGet(ADC0_BASE, 3, &sensor);

  // Convert the value to voltage.
  // The value of the ADC ranges from 0 to 4096. The sensor generates a 0 to 3.3V signal, so we can calculate the sensor output voltage.
  // sensorVoltage = sensor * (3.3/4096)

  // We now need to convert the sensor output voltage to distance in millimeters.
  // We do this using the sensor datasheet graph that shows the output voltage against distance.
  distance = sensor * (3.3 / 4096);
  // If the sensor voltage is less than 0.4V, the distance is approximately 1500 mm.
  if (distance <= 0.4) {
    distance = 1500;
  }
  // If the sensor voltage is greater than 2.8V, the distance is approximately 80 mm.
  else if (distance >= 2.8) {
    distance = 80;
  }
  // Otherwise, calculate the distance using the equation.
  else {
    distance = 4800.0 / (distance - 0.4);
  }
}

void controlTask() {
  // Define temporary variables.
  float tempIntegral = 0;
  float tempDistance = 0;

  // Calculate the average distance using the last three distance values.
  // We sum all the previous values and divide them by 3.
  for (int i = 0; i < 3; i++) {
    tempDistance += distanceBuffer[i];
  }
  tempDistance /= 3.0;
  distanceBuffer[0] = distanceBuffer[1];
  distanceBuffer[1] = distanceBuffer[2];
  distanceBuffer[2] = distance;

  // Calculate the error.
  error = setPoint - distance;

  // Calculate the integral term of the PID control loop. The integral term is the sum of the previous errors.
  for (int i = 0; i < 3; i++) {
    tempIntegral += distanceBuffer[i];
  }
  integral = ki * tempIntegral * dt;

  // Calculate the proportional term of the PID control loop.
  proportional = kp * error;

  // Calculate the derivative term of the PID control loop.
  derivative = kd * (distanceBuffer[2] - distanceBuffer[0]) / (2.0 * dt);

  // Calculate the total PID output.
  pid = proportional + integral + derivative;

  // Ensure the PID output is within the allowed range.
  if (pid > pid_max) {
    pid = pid_max;
  } else if (pid < pid_min) {
    pid = pid_min;
  }

  // Convert the PID output to the PWM duty cycle.
  dutyCycle = (uint32_t)((pid / pid_max) * pwmPeriod);
}

void loop() {
  setup();
  while (1) {
    // If the doControlFlag is set, execute the control task.
    if (doControlFlag) {
      // Clear the flag.
      doControlFlag = false;

      // Read the sensor value.
      readSensor();

      // Execute the control task.
      controlTask();

      // Set the PWM duty cycle.
      PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, dutyCycle);

      // If the sendData flag is set, send the distance value to the computer.
      if (sendData) {
        sendFloat(distance);
      }
    }
  }
}
