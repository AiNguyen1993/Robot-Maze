#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>

#include "uartstdio.h"

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <uartManager.h>



void ftoa(float f,char *buf)
{
    int pos=0,ix,dp,num;
    if (f<0)
    {
        buf[pos++]='-';
        f = -f;
    }
    dp=0;
    while (f>=10.0)
    {
        f=f/10.0;
        dp++;
    }
    for (ix=1;ix<8;ix++)
    {
            num = (int)f;
            f=f-num;
            if (num>9)
                buf[pos++]='#';
            else
                buf[pos++]='0'+num;
            if (dp==0) buf[pos++]='.';
            f=f*10.0;
            dp--;
    }
}

void delay(int a){
	int x = 0;
	while(x<a){
		x++;
	}
}

void text2PC(float a, char* b){
	char buff[10];
	ftoa(a, buff);
	UARTprintf(b);
	UARTprintf(buff);
	UARTprintf("\r");
}

void UART_init(void) {
/* Configure UART0 for USB */

	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0,115200,16000000);

/*
 * Configure UART1 for Bluetooth
 */
//	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//	GPIOPinConfigure(GPIO_PB0_U1RX);
//	GPIOPinConfigure(GPIO_PB1_U1TX);
//	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
//	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

}

void LED_init(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void UART_PrintString (char* c){
	long int i = 0;
	while (c[i] != '\0') {
		UARTCharPut(UART1_BASE, c[i]);
		i++;
	}
}

char * UART_GetString(void) {
	char result[100] = "";
	while (1) {
		char r = NULL;
		while (r == NULL) {
			r = UARTCharGet(UART1_BASE);
			UARTCharPut(UART1_BASE, r);
		}
		if (r=='\r') return result;
		strncat(result, &r, 1);
	}
}

void UART_PrintFloat(float data) {
	char str[10];
	ftoa(data, str);
	UART_PrintString(str);
}

void text2PC_BT(float a, char* b){
	char buff[10];
	ftoa(a, buff);
	UART_PrintString(b);
	UART_PrintString(buff);
	UART_PrintString("\r\n");
}

void led_off(void){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
}

void white_on(void){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
}
void red_on(void){
	led_off();
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xF);
}

void blue_on(void){
	led_off();
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0xF);
}

void green_on(void){
	led_off();
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xF);
}

void red_blink() {
	int i = 0;
	while(i<5){
		led_off();
		red_on();
		delay(500000);
		i++;
	}
	led_off();
}

void green_blink() {
	int i = 0;
	while(i<3){
		led_off();
		green_on();
		delay(500000);
		i++;
	}
	led_off();
}

void blue_blink() {
	int i = 0;
	while(i<5){
		led_off();
		blue_on();
		delay(500000);
		i++;
	}
	led_off();
}




