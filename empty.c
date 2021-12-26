#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

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
#include "inc/hw_ints.h"

#include "uartstdio.h"
#include "pwmManager.h"
#include "uartManager.h"
#include "DistanceSensorManager.h"

Semaphore_Handle DataSem;
Task_Handle RobotTask;

float data[2][20];
int row = 0, column = 0;
int rowPrint;
int preRS = 0;
int evenValue = 0;
int count = 0;
bool status;
int pretape = 0, tape = 0, countstop = 0;
int runTime = 0;
int time = 0, preTime = 0;

/* Function check status of robot if it pass the tape, how many tape passed and thin or wide tape.
 */
void status_check(void){
	if (RS_Read()){
		if (pretape == 0 && tape == 0){
			tape = 1;
			countstop = countstop + 1;
		}
		pretape++;
		time++;
	}
	else {
		pretape = 0;
		tape = 0;
		preTime = time;
		time = 0;
	}

	if (countstop == 1){
		blue_on();
		status = true;
	}
	else if (countstop == 2){
		status = false;
		led_off();
	}
}

/* check the width of tape */
bool tapeCheck(void) {
	if (count >= 3){
		float rate = time / preTime;
		if(rate >= 1.7) return true;
		else return false;
	}
	return false;
}

bool autoMode = false;		//variable to indicate mode of robot

/* Function set robot to auto mode*/
void set_Mode(void){
	autoMode = true;
}

/* Function set robot to manual mode*/
void dis_Mode(void){
	stop();
	autoMode = false;
}

/* Function to set data to PC via bluetooth */
void putData(void) {
	while(1){
		Semaphore_pend(DataSem, BIOS_WAIT_FOREVER);		//Waiting for signal from semaphore to run
		led_off();		//off the led
		green_blink();		//blink the green
		int i;
		for(i = 0; i < 20; i++){
			UART_PrintFloat(data[rowPrint][i]);		//Send data to PC via bluetooth using UART1
			UARTCharPut(UART1_BASE,'>');
		}
		UART_PrintString("<-end->");
		blue_on();
	}
}

/* Structure for user command with pointer function*/
struct UserCommand arr_cmd[] = {
		{"AM\r", set_Mode},
		{"MM\r", dis_Mode},
		{"OR\r", red_on},
		{"OB\r", blue_on},
		{"OG\r", green_on},
		{"OW\r", white_on},
		{"FA\r", led_off},
		{"DF\r", distanceFrontPrint},
		{"DR\r", distanceRightPrint},
		{"RF\r", run},
		{"RB\r", reverse},
		{"RU\r", u_turn},
		{"SP\r", stop},
};


float Xstar = 900;
float X;
float Error;
float Kp = 55;
float Ki = 10;
float Kd = 25;
float preError = 0;

/* PID Controller for the wheels*/
float PID_Controller (){
	X = distanceRight() * 100;		//Take the right distance of robot to the wall in mm
	Error = (Xstar - X);			//Calculate the error
//	text2PC_BT(Xstar, "target: ");
//	text2PC_BT(X, "distance right: ");
//	text2PC_BT(Error, "error: ");
	if(Error <= 50 && Error >= -50) Error = 0;		//Acceptable error

	status_check();	//Check when start, when stop collect data
	if(status) {
		blue_on();	//turn on blue LED
		evenValue = 1 - evenValue;	//Toggle value which indicate 100ms
		if(evenValue == 1){
			if(column == 20){
				rowPrint = row;
				row = 1 - row;
				column = 0;
				Semaphore_post(DataSem);		//Signal semaphore to enable putData
//				putData();
//				count = 0;
			}
			data[row][column] = (Error / 100);
//			UART_PrintFloat(count);
			column++;
//			count++;
		}
	}
	count++;
//	if(count == 19){
//	UART_PrintFloat(count);
//	count = 0;
//	}


	int32_t P = Kp * Error;
	int32_t I = Ki * (Error + preError);
	int32_t D = Kd * (Error - preError);

	float pwmAdjust = (P + I + D) / 100;
	preError = Error;

	return pwmAdjust;
}

//Function to operate the bot
const int PWM_MAX = 11362;

int PWM_Last_Right = 6000;
int PWM_Last_Left = 6000;

void Operation() {
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	if(autoMode == false) return;		//Check mode of robot

//	if(RS_Read()){
//				red_on();
//			}
//			else {
//				green_on();
//			}



	if(distanceFront() < 7) {		//Check front distance of robot
		u_turn();					//Perform u turn function
		while(distanceFront() < 15){ };		//Still perform u turn function if robot is still in dead end
		run();
	}

	int rightAdjust = PID_Controller();
//	text2PC_BT(rightAdjust,"right adjust: ");

	int leftAdjust = rightAdjust;

//	if (rightAdjust == 0) {
//		PWM_Last_Left = 6000;
//		PWM_Last_Right = 6000;
//	}
	if (rightAdjust < 0) {	//Robot is far the wall
		PWM_Last_Left = 6000;
		PWM_Last_Right += rightAdjust;	//Right wheel slow down
	}
	else if (rightAdjust > 0) {	//Robot is close the wall

 		PWM_Last_Left -= leftAdjust;	//Left wheel run faster
 		PWM_Last_Right = 6000;
	}

	//Set max and min pwm for left and right wheel
	if (PWM_Last_Right > 6000) PWM_Last_Right = 6000;
	if (PWM_Last_Right < 1200) PWM_Last_Right = 1200;
	if (PWM_Last_Left > 6000) PWM_Last_Left = 6000;
	if (PWM_Last_Left < 1200) PWM_Last_Left = 1200;

//	text2PC_BT(rightAdjust, "right adjust: ");
//	text2PC_BT(PWM_Last_Left, "pwm last left: ");
//	text2PC_BT(PWM_Last_Right, "pwm last right: ");
//	UART_PrintString("----------\r\n");

	if(tapeCheck()){		//True if robot go to double width tape
			Swi_post(StopSwi);			//Stop running
			count = count * 0.05;		//calculate running time
			text2PC_BT(count, "Running time [sec]: ");		//Print out running time
			count = 0;
	}
	else {
		forwardPhase();		//set phase of motor (wheels)
		wall_run(PWM_Last_Right,PWM_Last_Left);		//Perform wall running
	}

}

/* Function to get the command code from user via bluetooth */
void userCMD(void) {
	char* input = UART_GetString();		//Get the command from user
	int i;
	for(i = 0; i < 12; i++){
		if(strcmp(input, arr_cmd[i].Command) == 0) {		//Check the command in command list
			(*arr_cmd[i].CommandFunc)();		//Execute the function respect to command code
			break;
		}
	}
}

void hardware_init(void);

void Timer_ISR(void);

int main(){

	Task_Params taskParams;

	DataSem = Semaphore_create(0, NULL, NULL);

	Task_Params_init(&taskParams);
	taskParams.priority = 2;
	RobotTask = Task_create((Task_FuncPtr)userCMD, &taskParams, NULL);

	PWM_init();
	UART_init();
	LED_init();
	ADC_init();
	RS_init();
	hardware_init();

	BIOS_start();
}

void hardware_init(void) {
	uint32_t ui32Period;

	//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Timer 2 setup code
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);			// enable Timer 2 periph clks
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
	//TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // cfg Timer 2 mode - periodic

	ui32Period = (SysCtlClockGet()/20);	// period = CPU clk div 2 (500ms)
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);	// set Timer 2 period

	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);	// enables Timer 2 to interrupt CPU

	// IntEnable(INT_TIMER2B);
	IntMasterEnable();

	TimerEnable(TIMER2_BASE, TIMER_A);	// enable Timer 2
}
