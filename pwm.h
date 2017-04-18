#ifndef PWM_H_
#define PWM_H_

/*void SetDutyCycle(unsigned int DutyCycle, unsigned int Frequency);*/
void SetMotorDutyCycle(float leftDutyCycle, float rightDutyCycle, unsigned int Frequency, int dir);
void SetServoDutyCycle(float DutyCycle, unsigned int Frequency);
void InitPWM();
void PWM_ISR();

#endif /* PWM_H_ */
