
#ifndef PWMMANAGER_H_
#define PWMMANAGER_H_

void setMode();

void setxPhase(int a);

void forwardPhase(void);

void reversePhase(void);

void u_turnPhase(void);

void PWM_init();

//void PWM1A_Duty( int32_t cycle);

void wall_run(int pwm_right, int pwm_left);

void u_turn(void);

void run(void);

void reverse(void);

void stop(void);

//void reverse(void);
//
//void turnRight(void);

#endif /* PWMMANAGER_H_ */
