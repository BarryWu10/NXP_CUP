/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 * 
 * Author:  
 * Created:  
 * Modified:  
 */

#include "MK64F12.h"
#include "uart.h"
#include "pwm.h"
#include "camera_FTM_blank.h"

void initialize();
void en_interrupts();
void delay();

int main(void)
{
	// Initialize UART and PWM
	initialize();
  //uart_init();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time

	// Print welcome over serial
	uart_put("Running... \n\r");
	
	for(;;){  //then loop forever
    //You can use fork() to run two for loops at the same time, if you need different delays !!!! 
    //Camera reading code goes here
    maincam();
    
    
    //Driving Car
		SetMotorDutyCycle(25, 10000, 1);
		//SetServoDutyCycle(6 , 50); //turn left
		//delay(100);
		//SetServoDutyCycle(7, 50); //turn right left
		//delay(100);
		//SetServoDutyCycle(10, 50);
	}
	return 0;
}
	
/**
 * Waits for a delay (in milliseconds)
 * 
 * del - The delay in milliseconds
 */
void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}


void initialize()
{
	// Initialize UART
	uart_init();	
	
	// Initialize the FlexTimer
	InitPWM();
	
	//initGPIO();
}