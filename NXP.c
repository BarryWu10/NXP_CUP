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
//#include "camera_FTM_blank.h"
#include "stdio.h"
void findSpecialPoints();
void initialize();
void en_interrupts();
void delay();
void deriveLine();

void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
uint16_t line[128];

// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 0;
int capcnt = 0;
char str[100],str2[100];
int counter;
int sum;
int avg;

uint16_t avg_line[124];

int16_t derive_line[128];

int derivativeMax1_index = 0;
int derivativeMin1_index = 0;
int trigger1_index = 0;
int trigger2_index = 0;

// ADC0VAL holds the current ADC value
unsigned short ADC0VAL;

int16_t derive_line[128];

int main(void)
{
	int i;
	int deltaX1;
	int deltaX2;
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
		
    if (debugcamdata) {

			// Every 2 seconds

			//if (capcnt >= (2/INTEGRATION_TIME)) {

			if (capcnt >= (100)) {

				GPIOB_PCOR |= (1 << 22);

				// send the array over uart

				sprintf(str,"%i\n\r",-1); // start value

				uart_put(str);

				for (i = 0; i < 128; i++) {

					sprintf(str,"%i\n\r", line[i]);

					uart_put(str);

				}

				sprintf(str,"%i\n\r",-2); // end value

				uart_put(str);

				capcnt = 0;

				GPIOB_PSOR |= (1 << 22);

			}

		}
    //SetMotorDutyCycle(25, 10000, 1);
    //deriveLine();
		//findSpecialPoints();
		//deltaX1 = 64 - derivativeMax1_index;
		//deltaX2 = derivativeMin1_index - 64;
    
		deriveLine();
    findSpecialPoints();
		sprintf(str,"Max = %i ",derivativeMax1_index );
		uart_put(str);
		sprintf(str,"Min = %i ",derivativeMin1_index );
		uart_put(str);
		
					//sprintf(str,"Derived array:\n");
					//uart_put(str);
		//for (i = 0; i < 127; i++) {
			//		sprintf(str,"%i ", derive_line[i]);
				//	uart_put(str);
		//}
		deltaX1= 64-derivativeMax1_index;
		deltaX2 = derivativeMin1_index-64;
		if((deltaX1 - deltaX2) > 10 && (deltaX1 - deltaX2) < 20 ){
		/*stear right*/
			SetServoDutyCycle(8.0, 50);
			SetMotorDutyCycle(30, 10000, 1,1);
			delay(25);
			
		}
		else if((deltaX2 - deltaX1) > 10 && (deltaX2 - deltaX1) < 20){
			/*stear left*/
			SetServoDutyCycle(5.5 , 50);
			SetMotorDutyCycle(30, 10000, 1,0);
			delay(25);
		}
		else{
			SetServoDutyCycle(6.6 , 50);
			SetMotorDutyCycle(30, 10000, 1,3);
		}
    
    //Driving Car
		//SetMotorDutyCycle(25, 10000, 1);
		//SetServoDutyCycle(6 , 50); //turn left
		//delay(100);
		//SetServoDutyCycle(7, 50); //turn forward
		//delay(100);
		//SetServoDutyCycle(8, 50); // turn right
    //delay(100);
	}
	return 0;
}
	
void findSpecialPoints(){
	int i;
	int current1;
	int current2;
  
  derivativeMax1_index = 5;
  derivativeMin1_index = 123;
  
  //derive line exist
  for(i=5; i < 64; i++){
    current1 = derive_line[i]/*avg*/;
		current2 = derive_line[124 - i]/*avg*/;
    
  //find max
    if((current1 > derive_line[derivativeMax1_index] && i <64) ){
      //sprintf(str2,"in loop MAX1= %i\n\r", derivativeMax1_index);
      //uart_put(str2);
			derivativeMax1_index = i;
		}
  
   //find min
    if((current2 < derive_line[derivativeMin1_index] && (124 - i) > 64) ){
			derivativeMin1_index = 124-i;
		}
  }
  
  /*
  derivativeMax1_index = 0;
  derivativeMin1_index = 0xFFFF;
	/doing 2 to optimize code/
	for(i = 1; i < 127; i ++){
		current1 = line[i];
		current2 = line[i];
		if(current1 > line[derivativeMax1_index]){
      //sprintf(str2,"in loop MAX1= %i\n\r", derivativeMax1_index);
      //uart_put(str2);
			derivativeMax1_index = i;
		}
		else if(current1 < trigger1_index){
			trigger1_index = i;
		}
		else{
		}
		if(current2 < line[derivativeMin1_index]){
			derivativeMin1_index = i;
		}
		else if(current2 < trigger2_index){
			trigger2_index = 127 - i;
		}
		else{
		}
	}
  */
}

void deriveLine(){
	int i, a, b, c;
	int delta;
	derive_line[0] = 0;
	sum = 0;
	a = line[0];
	b = line[1];
	c = line[2];
	for(i = 3; i < 128; i++){
		sum = a+b+c;
		avg_line[i-3] = sum/3;
		if((i % 3) == 0){
			a = line[i];
		}
		else if((i % 3) == 1){
			b = line[i];
		}
		else{
			c = line[i];
		}
	}
	//for (i = 0; i < 127; i++) {
		//	sprintf(str,"%i\n\r", avg_line[i]);
			//uart_put(str);
				//}
	for(i = 0; i < 128; i++ ){
		line[i] = (line[i-1]+line[i]+line[i+1])/3; 
		//if(avg_line[i] >= avg_line[i-1]){
			delta = avg_line[i] - avg_line[i-1];
		//}else{
			//delta = avg_line[i-1] - avg_line[i];
		//}
		//delta = avg_line[i] - avg_line[i-1];
		sum += delta;
		derive_line[i] = delta;
	}
	sum=0;
	a = derive_line[0];
	b = derive_line[1];
	c = derive_line[2];
	for(i = 3; i < 128; i++){
		sum = a+b+c;
		derive_line[i-3] = sum/3;
		if((i % 3) == 0){
			a = derive_line[i];
		}
		else if((i % 3) == 1){
			b = derive_line[i];
		}
		else{
			c = derive_line[i];
		}
	}
	  //sprintf(str,"Start\n");
		//uart_put(str);
		//for (i = 0; i < 125; i++) {
			//sprintf(str,"%i ", derive_line[i]);
			//uart_put(str);
				//}
	//avg = sum/125;
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



/* ADC0 Conversion Complete ISR  */
void ADC0_IRQHandler(void) {
	// Reading ADC0_RA clears the conversion complete flag
	//INSERT CODE HERE
	ADC0VAL = ADC0_RA;
}

/* 
* FTM2 handles the camera driving logic
*	This ISR gets called once every integration period
*		by the periodic interrupt timer 0 (PIT0)
*	When it is triggered it gives the SI pulse,
*		toggles clk for 128 cycles, and stores the line
*		data from the ADC into the line variable
*/
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
  	//INSERT CODE HERE
		FTM2_SC &= ~(FTM_SC_TOF_MASK);
	
	// Toggle clk
	//CHECK
	//if (clkval ==0){
		//clkval =1;
		//GPIOC_PDOR |= (1 << 7); 
	//}else{
		//clkval =0;
		//GPIOC_PDOR &= ~(1 << 7);
	//}
	//clkval= !clkval;
	GPIOB_PTOR = (1 << 9);
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		//INSERT CODE HERE
		FTM2_SC &= ~(FTM_SC_TOIE_MASK);//?
	}
	return;
}

/* PIT0 determines the integration period
*		When it overflows, it triggers the clock logic from
*		FTM2. Note the requirement to set the MOD register
* 	to reset the FTM counter because the FTM counter is 
*		always counting, I am just enabling/disabling FTM2 
*		interrupts to control when the line capture occurs
*/
void PIT0_IRQHandler(void){
	if (debugcamdata) {
		// Increment capture counter so that we can only 
		//	send line data once every ~2 seconds
		capcnt += 1;
	}
	// Clear interrupt
	//INSERT CODE HERE
	PIT_TFLG0 = 1; //??
	
	// Setting mod resets the FTM counter
	//INSERT CODE HERE
	//PIT_TCTRL0 |=PIT_TCTRL_CHN_MASK;
	//FTM2_MOD = ((10*(10^6))/DEFAULT_SYSTEM_CLOCK);
	FTM2_MOD = (DEFAULT_SYSTEM_CLOCK/100000);//??
	// Enable FTM2 interrupts (camera)
	//INSERT CODE HERE
	FTM2_SC |= FTM_SC_TOIE_MASK;
	return;
}


/* Initialization of FTM2 for camera */
void init_FTM2(){
	// Enable clock
	//INSERT CODE HERE
	//SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;
	// Disable Write Protection
	//INSERT CODE HERE
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	//FTM2_FMS &= ~(FTM_FMS_WPEN_MASK);
	
	// Set output to '1' on init
	//INSERT CODE HERE
    //FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	
	// Initialize the CNT to 0 before writing to MOD
	//INSERT CODE HERE
	FTM2_CNT =0;
	// Set the Counter Initial Value to 0
	//INSERT CODE HERE
	//counter=0; //??
	FTM2_CNTIN=0;
	
	// Set the period (~10us)
	//INSERT CODE HERE
	FTM2_MOD = (DEFAULT_SYSTEM_CLOCK/100000);
	// 50% duty
	//INSERT CODE HERE
	//FTM2_C0V = FTM_CnV_VAL(0x8000);
	
	// Set edge-aligned mode
	//INSERT CODE HERE
	FTM2_C0SC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	//INSERT CODE HERE
	//n=0 or 1??
	FTM2_C0SC &= ~(FTM_CnSC_ELSA_MASK);
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	
	// Enable hardware trigger from FTM2
	//INSERT CODE HERE
	//FTM2_SYNC |= FTM_SYNCONF_HWTRIGMODE_MASK; //??????
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	
	// Don't enable interrupts yet (disable)
	//INSERT CODE HERE
	FTM2_SC &= ~(FTM_SC_TOIE_MASK);
	// No prescalar, system clock
	//INSERT CODE HERE
	FTM2_SC |= FTM_SC_CLKS(1);
	FTM2_SC |= FTM_SC_PS(0);
	
	// Set up interrupt
	//INSERT CODE HERE
	NVIC_EnableIRQ(FTM2_IRQn);
	return;
}

/* Initialization of PIT timer to control 
* 		integration period
*/
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	//INSERT CODE HERE
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable timers to continue in debug mode
	//INSERT CODE HERE // In case you need to debug
	//PIT_MCR |= PIT_MCR_FRZ_MASK;
	PIT_MCR= 0x0;
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	//INSERT CODE HERE
	PIT_LDVAL0 = (uint32_t)(INTEGRATION_TIME * DEFAULT_SYSTEM_CLOCK);
	
	// Enable timer interrupts
	//INSERT CODE HERE
	
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	//INSERT CODE HERE
	
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	//INSERT CODE HERE
	
		PIT_TFLG0 = 1;

	// Enable PIT interrupt in the interrupt controller
	//INSERT CODE HERE
		NVIC_EnableIRQ(PIT0_IRQn);
	return;
}


/* Set up pins for GPIO
* 	PTB9 		- camera clk
*		PTB23		- camera SI
*		PTB22		- red LED
*/
void init_GPIO(void){
	// Enable LED and GPIO so we can see results
	//INSERT CODE HERE
	
	
	//initialize push buttons and LEDs
	
	//initialize clocks for each different port used.
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK |	SIM_SCGC5_PORTE_MASK; //LED
	
	//SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //button sw2
	//SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //button sw3
	
	//Configure Port Control Register for Inputs with pull enable and pull up resistor

	// Configure mux for Outputs
	PORTB_PCR21 |= PORT_PCR_MUX(1); // Blue Led; Port B, Pin 21
	PORTB_PCR22 |= PORT_PCR_MUX(1); // Red Led; Port B, Pin 22
	PORTE_PCR26 |= PORT_PCR_MUX(1); // Green Led; Port E, Pin 26
	PORTB_PCR9 |= PORT_PCR_MUX(1); // output from PTC7
	PORTB_PCR23 |= PORT_PCR_MUX(1);
	
	//PORTC_PCR6 |= PORT_PCR_MUX(1); // Built in Push Button; Port C, Pin 6
	//PORTA_PCR4 = PORT_PCR_MUX(1); //check alternative
	// Switch the GPIO pins to output mode (Red and Blue LEDs)
	
	GPIOB_PDDR |= (1 << 21) | (1 << 22); // Loads data into the direction register
	GPIOE_PDDR |= (1 << 26);
	GPIOB_PDDR |= (1 << 9);
	GPIOB_PDDR |= (1 << 23);
	 
	
	// Turn off the LEDs
	
	GPIOB_PSOR |= (1 << 21) | (1 << 22); // Drives output to the LEDs high, initializing them to "off" state
	GPIOE_PSOR |= (1 << 26);

	// Set the push buttons as an input
	
	//GPIOC_PDDR &= ~(1 << 6); 
	//GPIOA_PDDR &= ~(1 << 4); 
	
	// interrupt configuration for SW3(Rising Edge) and SW2 (Either)
	//PORTC_PCR6 |= PORT_PCR_IRQC(11);//sw2 rising only
	//PORTA_PCR4 |= PORT_PCR_IRQC(9);//sw3 both rising and falling 
	
	return;
}

/* Set up ADC for capturing camera data */
void init_ADC0(void) {
    unsigned int calib;
    // Turn on ADC0
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK ;
	// Single ended 16 bit conversion, no clock divider
	//INSERT CODE HERE
	ADC0_CFG1 |= ADC_CFG1_ADIV(0);
	ADC0_CFG1 |= ADC_CFG1_MODE_MASK;
    
    // Do ADC Calibration for Singled Ended ADC. Do not touch.
    ADC0_SC3 = ADC_SC3_CAL_MASK;
    while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
    calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
    calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
    calib = calib >> 1; calib |= 0x8000;
    ADC0_PG = calib;
    
    // Select hardware trigger.
    //INSERT CODE HERE
    ADC0_SC2 |= ADC_SC2_ADTRG_MASK;
    // Set to single ended mode	
	//INSERT CODE HERE
	//ADC0_SC1A &= ~(ADC_SC1_DIFF_MASK);
	ADC0_SC1A = 0x41;
	
	// Set up FTM2 trigger on ADC0
	//INSERT CODE HERE // FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(10);
	//INSERT CODE HERE // Alternative trigger en.
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;
	//INSERT CODE HERE // Pretrigger A
	SIM_SOPT7 &= ~(SIM_SOPT7_ADC0PRETRGSEL_MASK);
	
	// Enable NVIC interrupt
    //INSERT CODE HERE
	NVIC_EnableIRQ(ADC0_IRQn);
}