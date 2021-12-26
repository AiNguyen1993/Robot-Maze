
#ifndef MANAGER_H_
#define MANAGER_H_

struct UserCommand {
	char* Command;
	void (*CommandFunc)(void);
};

void delay(int a);

/* Print text to PC using USB*/
void text2PC(float a, char* b);

/* Print text to PC using bluetooth */
void text2PC_BT(float a, char* b);

/* Initial UART0 and UART1 */
void UART_init(void);

/* Initial LED */
void LED_init(void);


void UART_PrintString(char* c);

char* UART_GetString();

void UART_Echo();

/* Convert float to string */
void ftoa (float f, char* buf);

void UART_PrintFloat(float data);

void led_off(void);

void white_on(void);

void red_on(void);

void green_on(void);

void blue_on(void);

void red_blink(void);

void green_blink(void);

void blue_blink(void);

#endif /* MANAGER_H_ */
