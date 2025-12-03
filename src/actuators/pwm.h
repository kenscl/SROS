#ifndef __PWM
#define __PWM
void PWM_init();
void PWM_start();
void PWM_set_duty_cycle(float duty_cycle);
void PWM_thread();
#endif
