#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>

#include "uartManager.h"
#include "pwmManager.h"
#include "uartstdio.h"

#define PWM_BASE_FREQ 55 //Hz

//#define RightWheel_Bit = PWM_OUT_2_BIT
//#define	LeftWheel_Bit = PWM_OUT_3_BIT

//#define RightWheel_Run = PWM_OUT_2
//#define LeftWheel_Run = PWM_OUT_3

volatile uint32_t pwmMax;
volatile uint32_t ui32PWMClock;
volatile uint32_t pwmAdjust;

/* Function to set mode for motors */
void setMode() {
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6, 0xFF);		//Set PB6(mode) high
}
//xPhase: 0 forward; 1 backward
void setxPhase(int a) {
	if(a == 1) {
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0x00);	//Set 2 wheels forward direction
	}
	if(a == 0) {
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0xFF);	//Set 2 wheels reverse direction
	}
}

/* Function set 2 wheel at forward direction */
void forwardPhase (void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0x00);
}

/* Function set 2 wheels at reverse direction */
void reversePhase (void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
}

/* Function set 2 wheels at u-turn mode
 * Right wheel = forward; left wheel = reverse
 */
void u_turnPhase (void) {
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0xFF);	//left wheel
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x00);	//right wheel
}

/* Function initial for PWM module
 * using PWM module 1, generation 1
 */
void PWM_init() {
	pwmAdjust = 350;		//Inital pwm width, means that wheel run at 35% of power at start
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);	//PWM output
	//PB2 and PB3 for xPhase, PB6 for mode
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6);

	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	GPIOPinConfigure(GPIO_PA7_M1PWM3);


	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //set sys clock to 80 MHz
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);    //set pwm clock

	ui32PWMClock = SysCtlClockGet() / 64; //setting our PWM clock to a variable
	pwmMax = (ui32PWMClock / PWM_BASE_FREQ) - 1; //the count to be loaded to the load register

	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, pwmMax);
	setMode();
	setxPhase(1);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pwmAdjust * pwmMax / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pwmAdjust * pwmMax / 1000);
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

/* Function to set wheels running along wall
 * Parameters: pwm_right and pwm_Left
 * Speed of two parameters will be decided by PID controller
 */
void wall_run(int pwm_right, int pwm_left){
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pwm_right);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pwm_left);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

/* Function to set U-turn speed for 2 wheel */
void u_turn(){
	u_turnPhase();
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 6000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 6000);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

/* Function set 2 wheels run forward at max speed */
void run(){
	forwardPhase();
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pwmMax);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pwmMax);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

/* Function to set 2 wheel reverse at max speed */
void reverse() {
	reversePhase();
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, pwmMax);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pwmMax);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

/* Function to stop 2 wheels */
void stop(){
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false);	//PWM_OUT_2_BIT = Right wheel
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);	//PWM_OUT_3_BIT = Left wheel

}



//	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
//
//	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
//
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//
//	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);	//PWM output
//	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6);	//PB2 and PB3 for xPhase, PB6 for mode
//
//	GPIOPinConfigure(GPIO_PA6_M1PWM2);
//	GPIOPinConfigure(GPIO_PA7_M1PWM3);
//
//	//while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
//	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 400);
//	setMode(1);
//	setxPhase(1);
//	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 200);
//	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 200);
//
//	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
//	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
//	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
//
//	//Control motor
//	while(1) {
//		char r = UARTCharGet(UART0_BASE);
//		UARTCharPut(UART0_BASE, r);
//		switch(r) {
//			case 'b':
//				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
//				PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 200);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 200);
//				break;
//			case 's':
//				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false);
//				PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
//				break;
//			case '1':
//				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
//				PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 100);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 100);
//				break;
//			case '3':
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 300);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 300);
//				break;
//			case '4':
//				PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
//				PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 400);
//				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 400);
//				break;
//		}
//	}
