#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "DistanceSensorManager.h"

#include <driverlib/pwm.h>
#include <driverlib/adc.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/timer.h>

#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include "inc/hw_timer.h"

#include "uartstdio.h"
#include "pwmManager.h"
#include "uartManager.h"
#include "DistanceSensorManager.h"

#define ADCFront = ADC0_BASE;
#define ADCRight = ADC1_BASE;



void ADCFront_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

//    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
//    };

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

}

void ADCRight_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

//    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
//    };

    ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE |
                             ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 2);
    ADCIntClear(ADC1_BASE, 2);

}

uint32_t ADC_DataGetFront (void) {
    uint32_t valueF;
    int sum = 0;

    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);

    while (!ADCIntStatus(ADC0_BASE, 3, false)) { };

    int i;
    for(i = 0; i<1; i++){
    	ADCSequenceDataGet(ADC0_BASE, 3, &valueF);
    	sum += valueF;
    }
    valueF = sum/1;
    return valueF;
}

uint32_t ADC_DataGetRight(void){
    uint32_t valueR;
    int sum = 0;

    ADCIntClear(ADC1_BASE,2);
    ADCProcessorTrigger(ADC1_BASE, 2);

    while (!ADCIntStatus(ADC1_BASE, 2, false)) { };

//    ADCSequenceDataGet(ADC1_BASE, 2, &valueR);
    int i = 0;
    for(i = 0; i<1; i++){
    	ADCSequenceDataGet(ADC1_BASE, 2, &valueR);
    	sum += valueR;
    }
    valueR = sum/1;

    return valueR;
}
//Pin E3
float distanceFront(void) {
	float sum = 0;
	float avgDistance;
	int i;
	for(i=0; i<4; i++){
		uint32_t value = ADC_DataGetFront();
		float distance = 118848/(pow(value, 1.273));
		sum += distance;
	}
	avgDistance = sum/4;
	return avgDistance;
}
//Pin E2
float distanceRight() {
	float sum = 0;
	float avgDistance;
	int i = 0;
	for(i = 0; i<4; i++){
		uint32_t value = ADC_DataGetRight();
		float distance = 8458.4/(pow(value,0.916));
		sum += distance;
	}
	avgDistance = sum/4;
	return avgDistance;
}

void distanceFrontPrint() {
	float x = distanceFront();
	text2PC(x,"distance front = ");
	if (x <= 7) {
		red_blink();
		UART_PrintString("front is blocked\r\n");
	}
	else {
		green_blink();
		UART_PrintString("front is clear\r\n");
	}
}

void distanceRightPrint() {
	float x = distanceFront();
	text2PC(x,"distance right = ");
	if (x <= 7) {
		red_blink();
		UART_PrintString("right is blocked\r\n");
	}
	else {
		green_blink();
		UART_PrintString("right is clear\r\n");
	}
}

void ADC_init() {
	ADCFront_Init();
	ADCRight_Init();
}

void RS_init(void){
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);		// Enable Port D Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		// Enable Port F Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);		// Enable Timer0 Peripheral
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);	// Set Timer0 to Periodic mode
 //   SysCtlDelay(1000);
	TimerEnable(TIMER0_BASE, TIMER_A);					// Enable Timer0
}

bool RS_Read(void){
    uint32_t startTime, endTime, pinValue;
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;                       // Set Timer2 to 0 to start timing
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);         // Make PortB pin1 output to charge the sensor
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);      // Start charing the sensor
    SysCtlDelay(1000);                                          // Wait to finish charging

    startTime = TimerValueGet(TIMER0_BASE, TIMER_A);            // Capture startTime
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1);          // Make PortB pin1 input to time the decaying
    pinValue = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1);        // Assign the value when sensor is in fully charged state
    while (pinValue & GPIO_PIN_1) {                             // Loop to compare to the current state to fully-charged state
        pinValue = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1);
    }
    endTime = TimerValueGet(TIMER0_BASE, TIMER_A);              // Capture endTime when fully decayed
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);         // Make Red LED an indicator
	if (endTime-startTime>10000) {
		// 10000 is a vague value to compare. When sensor is not reflective, which is
		//  either the sensor is not on white surface or sensor is too far from white surface,
		// (endTime-startTime) > 10000
		return true;           // Red on if sensor is on black tape
	}
	else {                                                      // Red off if otherwise
		return false;
	}

}
//void RS_init(void){
//	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);		// Enable Port B Peripheral
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		// Enable Port F Peripheral
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);		// Enable Timer2 Peripheral
//	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);	// Set Timer2 to Periodic mode
// //   SysCtlDelay(1000);
//	TimerEnable(TIMER2_BASE, TIMER_B);					// Enable Timer2
//}
//
//bool RS_Read(void){
//    uint32_t startTime, endTime, pinValue;
//    HWREG(TIMER2_BASE + TIMER_O_TBV) = 0;                       // Set Timer2 to 0 to start timing
//    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);         // Make PortB pin1 output to charge the sensor
//    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);      // Start charing the sensor
//    SysCtlDelay(1000);                                          // Wait to finish charging
//
//    startTime = TimerValueGet(TIMER2_BASE, TIMER_B);            // Capture startTime
//    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1);          // Make PortB pin1 input to time the decaying
//    pinValue = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1);        // Assign the value when sensor is in fully charged state
//    while (pinValue & GPIO_PIN_1) {                             // Loop to compare to the current state to fully-charged state
//        pinValue = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1);
//    }
//    endTime = TimerValueGet(TIMER2_BASE, TIMER_B);              // Capture endTime when fully decayed
//	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);         // Make Red LED an indicator
//	if (endTime-startTime>10000) {
//		// 10000 is a vague value to compare. When sensor is not reflective, which is
//		//  either the sensor is not on white surface or sensor is too far from white surface,
//		// (endTime-startTime) > 10000
//		return true;           // Red on if sensor is on black tape
//	}
//	else {                                                      // Red off if otherwise
//		return false;
//	}
//
//}
