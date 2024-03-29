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
#include "stdio.h"

//void Button_Init(void);
void initialize();
void en_interrupts();
void plotCamera(int);
void plotSmooth(int);
void plotDerive(int);
void plotDerive2(int);
void calculateDerivatives();
void turn();
void delay(int del);
void initPDB(void);

//initialize stuff
void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void initFTM1(void);
//interrupts
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
void FTM1_IRQHandler(void);
void PORTC_IRQHandler(void);
void PORTA_IRQHandler(void);

#define cameraPlot 0
#define smoothPlot 0
#define derivePlot 0

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

#define fastSpeed 70 //70
#define fastTurn 65//65

#define medSpeed 60
#define medTurn 55

#define slowSpeed 50
#define slowTurn 45

#define brake_speed 70
#define inside_wheel .35
#define outside_wheel .135
 // Light and dark area cutoff values
#define cutoff_L .75
#define cutoff_D .25
#define brake_servo 1
#define freq 10000
#define brake_servo_r -1
#define noise_cutoff_L -700
#define noise_cutoff 700
#define noise_cutoff_High 4500
#define noise_cutoff_High_L -4500

#define brake_coeff_M .12
#define s_time_M 350
#define min_brake_M 50
#define brake_coeff2_M .12
#define s_time2_M 800

#define brake_coeff_F .14
#define s_time_F 300
#define min_brake_F 45
#define brake_coeff2_F .175
#define s_time2_F 800

/*
#define speedLimit 70// 75
#define turnLimit 62//65
*/
// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;
// clkval toggles with each FTM interrupt
int clkval = 0;
// line stores the current array of camera data
int speedLimit = 0;
int turnLimit;
int brake;
uint16_t line[128];					//lines
uint16_t avg_line[128];			//smooth line
int16_t derive_line[128];		//derive line
int16_t derive2_line[128];		//second derivative line

double brake_coeff, brake_coeff2;
int s_time, s_time2, min_brake;

float kp_L = 3.5/45;
float kp_H = 4.0/45;
float kd;
float ki;
float servoFactor;
int brake_set;
int brake_snapshot;
// These variables are for streaming the camera
//	 data over UART
int debugcamdata = 1;
int capcnt = 0;
char str[100],str2[100];
int counter;
float midpoint, p_midpoint, b_midpoint;
char current_state, previous_state, previous_state2, lt_state;
int s_counter, b_counter, t_counter;
int max_high, min_low;
 // Light and dark area cutoff values

int d_area;  // boolean, true if the car is in a dark area.
int d_counter = 0; //counts how many times the camera thinks it is dark
int line_lost = 0; // boolean, true if line is lost

int state = -1;
int brake_time;
int cross  = 0;
// ADC0VAL holds the current ADC value
unsigned short ADC0VAL;

int timer_counter;




int main(void){
	//initialize all the interrupts
	initialize();
	for(;;){

		//while(GPIOC_PDIR == (0<<6)){
			/*
			GPIOB_PSOR = (1 << 22);      //sets red to on
			GPIOE_PSOR = (1 << 26);      //sets green to on
			GPIOB_PSOR = (1 << 21);      //sets blue to on
			*/
			//speedLimit = 0;
			//state = -1;
			//}
		//if(state == -1){
			//speedLimit = 0;
		//}







		/*
		*		Only 1 plot can be active
		*/

		//sprintf(str,"%f,", midpoint);
		//uart_put(str);
		//SetMotorDutyCycle(0, 0, 10000, 1);


		//servo test to see its limit
	/*
	SetMotorDutyCycle(0, 0, 10000, 1);
	//SetServoDutyCycle(8.0 ,50);
	//delay(100);
	//SetServoDutyCycle(8.5 ,50);
	//delay(100);
		SetServoDutyCycle(9.0 ,50);
	delay(100);
		SetServoDutyCycle(9.5 ,50);
	delay(100);
	SetServoDutyCycle(9.75 ,50);
	delay(100);
		SetServoDutyCycle(10.0 ,50);
	delay(100);
	//SetServoDutyCycle(10.5 ,50);
	//delay(100);
	//SetServoDutyCycle(11.0 ,50);
	//delay(100);
	//SetServoDutyCycle(11.5 ,50);
	//delay(100);
	//SetServoDutyCycle(12.0 ,50);
	//delay(100);
	*/
	}
	return 0;
}

void PORTC_IRQHandler(void){ //For switch 2
	PORTC_ISFR |= PORT_ISFR_ISF_MASK;
	
	//while(GPIOC_PDIR == (0<<6)){
			/*
			GPIOB_PSOR = (1 << 22);      //sets red to on
			GPIOE_PSOR = (1 << 26);      //sets green to on
			GPIOB_PSOR = (1 << 21);      //sets blue to on
			*/
	//		speedLimit = 0;
		//	state = -1;
			//}
		//if(state == -1){
			//speedLimit = 0;
		//}
	return;
}

void PORTA_IRQHandler(void){ //For switch 3


    //clears the interrupt
	PORTA_ISFR |= PORT_ISFR_ISF_MASK;
	state += 1;
	if(state == 4){
		state =0;
	}
	if(state == -1){
			speedLimit = 0;
		GPIOB_PCOR = (1 << 22);      //sets red to on
  GPIOE_PCOR = (1 << 26);      //sets green to on
  GPIOB_PCOR = (1 << 21);      //sets blue to on
	}else if(state == 1){
		speedLimit = fastSpeed;
		turnLimit = fastTurn;
		brake_coeff = brake_coeff_F;
		brake_coeff2 = brake_coeff2_F;
		s_time = s_time_F;
		s_time2 = s_time2_F;
		min_brake = min_brake_F;
		
		
		GPIOB_PCOR = (1 << 22);      //sets red to on
  GPIOE_PSOR = (1 << 26);      //sets green to off
  GPIOB_PSOR = (1 << 21);      //sets blue to off
	}else if(state == 2){
		speedLimit = medSpeed;
		turnLimit = medTurn;
		brake_coeff = brake_coeff_M;
		brake_coeff2 = brake_coeff2_M;
		s_time = s_time_M;
		s_time2 = s_time2_M;
		min_brake = min_brake_M;
		GPIOB_PSOR = (1 << 22);      //sets red to off
  GPIOE_PCOR = (1 << 26);      //sets green to on
  GPIOB_PSOR = (1 << 21);      //sets blue to off
	}
	else{
		speedLimit = slowSpeed;
		turnLimit = slowTurn;
		GPIOB_PSOR = (1 << 22);      //sets red to off
  GPIOE_PSOR = (1 << 26);      //sets green to off
  GPIOB_PCOR = (1 << 21);      //sets blue to on
	}


	return;
}


void plotCamera(int plot)
{		int i;
		//to read in matlab
		if (debugcamdata & plot) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (100)) {
				GPIOB_PCOR |= (1 << 22);
				// send the array over uart
				sprintf(str,"%i\r\n",-1); // start value
				uart_put(str);
				for (i = 0; i < 127; i++) {
					sprintf(str,"%i,", line[i]);
					uart_put(str);
				}
				sprintf(str,"%i\r\n", line[127]);
				uart_put(str);
				sprintf(str,"%i\r\n",-2); // end value
				uart_put(str);
				capcnt = 0;
				GPIOB_PSOR |= (1 << 22);
			}
		}
		//no more matlab
}

void plotSmooth(int plot)
{		int i;
		//to read in matlab
		if (debugcamdata & plot) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (100)) {
				GPIOE_PCOR |= (1 << 26);
				// send the array over uart
				sprintf(str,"%i\r\n",-1); // start value
				uart_put(str);
				for (i = 0; i < 127; i++) {
					sprintf(str,"%i,", avg_line[i]);
					uart_put(str);
				}
				sprintf(str,"%i\r\n", avg_line[127]);
				uart_put(str);
				sprintf(str,"%i\r\n",-2); // end value
				uart_put(str);
				capcnt = 0;
				GPIOE_PSOR |= (1 << 26);
			}
		}
		//no more matlab
}

void plotDerive2(int plot)
{		int i;
		//to read in matlab
		if (debugcamdata & plot) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (100)) {
				GPIOB_PCOR |= (1 << 21);
				GPIOE_PCOR |= (1 << 26);
				// send the array over uart
				sprintf(str,"%i\r\n",-1); // start value
				uart_put(str);
				for (i = 0; i < 127; i++) {
					sprintf(str,"%i,", derive2_line[i]);
					uart_put(str);
				}
				sprintf(str,"%i\r\n", derive2_line[127]);
				uart_put(str);
				sprintf(str,"%i\r\n",-2); // end value
				uart_put(str);
				capcnt = 0;
				GPIOB_PSOR |= (1 << 21);
				GPIOE_PSOR |= (1 << 26);
			}
		}
		//no more matlab
}

void plotDerive(int plot)
{		int i;
		//to read in matlab
		if (debugcamdata & plot) {
			// Every 2 seconds
			//if (capcnt >= (2/INTEGRATION_TIME)) {
			if (capcnt >= (100)) {
				GPIOB_PCOR |= (1 << 21);
				// send the array over uart
				sprintf(str,"%i\r\n",-1); // start value
				uart_put(str);
				for (i = 0; i < 127; i++) {
					sprintf(str,"%i,", derive_line[i]);
					uart_put(str);
				}
				sprintf(str,"%i\r\n", derive_line[127]);
				uart_put(str);
				sprintf(str,"%i\r\n",-2); // end value
				uart_put(str);
				capcnt = 0;
				GPIOB_PSOR |= (1 << 21);
			}
		}
		//no more matlab
}

void initialize()
{
	//initialize all the timers/signals/stuff
	uart_init();
	InitPWM();
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
	initFTM1();
	GPIOB_PCOR = (1 << 22);      //sets red to on
  GPIOE_PCOR = (1 << 26);      //sets green to on
  GPIOB_PCOR = (1 << 21);      //sets blue to on
}

/* ADC0 Conversion Complete ISR  */
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
		calculateDerivatives();


		plotCamera(cameraPlot);
		plotSmooth(smoothPlot);
		plotDerive(derivePlot);
		//plotDerive2(0);


		turn();
		//INSERT CODE HERE
		FTM2_SC &= ~(FTM_SC_TOIE_MASK);//?
	}
	return;
}

void turn(void){
	int i;
	int max_R, max_L, min_R, min_L;
	int L_high, L_low, R_high, R_low;
	int noise, line_lost;
	// initialize ints to 0
	noise = 0;
	line_lost = 0;
	L_high = 0;
	L_low = 0;
	R_high = 0;
	R_low = 0;

  //derive line exist
  for(i=0; i < 50; i++){
		//find max left and index
		if ( (derive_line[64-i] > R_high) && (derive_line[64-i] < noise_cutoff_High)){
			max_R = 64-i;
			R_high = derive_line[64-i];
		}
		//find min left and index
		if ( (derive_line[64-i] < R_low) && (derive_line[64-1] > noise_cutoff_High_L)){
			min_R = 64-i;
			R_low = derive_line[64-i];
		}
		//find max right and index
		if ( (derive_line[64+i] > L_high) && (derive_line[64+i] < noise_cutoff_High)){
			max_L = 64+i;
			L_high = derive_line[64+i];
		}
		//find min left and index
		if ( (derive_line[64+i] < L_low) && (derive_line[64+1] > noise_cutoff_High_L)){
			min_L = 64+i;
			L_low = derive_line[64+i];
		}
	}

	// threshold values for noise, need to be calibrated
	if((R_high < noise_cutoff) && ( L_high < noise_cutoff) && (R_low > (noise_cutoff_L)) && (L_low > (noise_cutoff_L))){
		noise = 1;
		cross = 0;
	}

	//if largest peak @ 64+ or lowest peak @ 64-
	// if the line is lost and there is no noise, continue the turn by setting cross to 1.
	if ((R_high < L_high) || (R_low < L_low)){
		line_lost = 1;
		// no noise, but line is lost
		if (noise == 0){
			midpoint = p_midpoint;
			line_lost = 1;
			cross = 1;
		}
	}
	// if there is no noise, and line is not lost
	if (noise == 0 && line_lost == 0){
		p_midpoint = midpoint;
		midpoint = (float)(max_R+min_L)/2.0;
		cross = 0;
	} else {
		// it found a cross or line was lost
		if (cross == 0){
			if ( previous_state == 'l'){
				midpoint = 60;
				cross = 1;
			} else if (previous_state == 'r'){
				midpoint = 68;
				cross = 1;
			} else {
				midpoint = 64;
				cross = 1;
			}
		} else {
			midpoint = p_midpoint;
		}
	}

	if (brake) {
		midpoint = b_midpoint;
	}

	//calculate the servo factor using error
	servoFactor = (float) (((midpoint - 64.0)*kp_L));


// FOCUS ON HERE!
	if ( midpoint > 66){
		//turn left

		// make sure servo isn't out of range
		if (servoFactor > 1.79){
			servoFactor = 1.79;
		}
		// set states for other code
		previous_state2 = previous_state;
		previous_state = current_state;
		current_state = 'l';
		// if the turn is hard enough
		if (servoFactor > brake_servo){
			// if the
			if ( timer_counter > s_time){
				if (brake == 0){
					if (timer_counter > s_time2){
						brake_time = timer_counter*brake_coeff2;
					}else{
					brake_time = timer_counter *brake_coeff;
					}
				}
				brake = 1;
				FTM1_CNT &= ~(FTM_CNT_COUNT_MASK);
				// not linear
			}else {
				brake = 0;
				timer_counter = 0;
				brake_time = 0;
				//not braking, because it hasn't sped up enough set duty cycles
				SetServoDutyCycle(9.75 - servoFactor, 50);
				SetMotorDutyCycle(turnLimit-(inside_wheel*turnLimit*servoFactor),turnLimit+(outside_wheel*turnLimit*servoFactor), freq, 1);
			}
			// if brake boolean is on brake until the brake timer hits 0 then brake = 0
				if (brake){
					GPIOB_PTOR = (1 << 22);      //sets red to on
				GPIOE_PTOR = (1 << 26);      //sets green to on
				GPIOB_PTOR = (1 << 21);      //sets blue to on
				b_midpoint = midpoint;
				SetMotorDutyCycle(brake_speed, brake_speed, freq, 0);
				SetServoDutyCycle(9.75 - servoFactor, 50);
				GPIOB_PTOR = (1 << 22);      //sets red to on
				GPIOE_PTOR = (1 << 26);      //sets green to on
				GPIOB_PTOR = (1 << 21);      //sets blue to on
				}
			} else{
			// Its not a hard turn just do a normal turn
			SetServoDutyCycle(9.75 - servoFactor, 50);
			SetMotorDutyCycle(turnLimit-(inside_wheel*turnLimit*servoFactor),turnLimit+(outside_wheel*turnLimit*servoFactor), freq, 1);
		}
	} else if ( midpoint < 62){
		// turns right
		// make sure servo isn't out of range
		if (servoFactor < -1.79){
			servoFactor = -1.79;
		}
		// set states for other code
		previous_state2 = previous_state;
		previous_state = current_state;
		current_state = 'r';
		// if the turn is hard enough to check for braking
		if (servoFactor < (brake_servo_r)){
			if ( timer_counter > s_time){
				if (brake == 0){
					if (timer_counter > s_time2){
						brake_time = timer_counter*brake_coeff2;
					}else{
					brake_time = timer_counter *brake_coeff;
					}
				}
				brake = 1;
				FTM1_CNT &= ~(FTM_CNT_COUNT_MASK);
				// not linear needs to be changed
			}else {
				brake = 0;
				timer_counter = 0;
				brake_time = 0;
				//not braking, because it hasn't sped up enough set duty cycles
				SetServoDutyCycle(9.75 - servoFactor, 50);
				// left wheel is outside, so it increases in speed because servoFactor is negative, and inside wheel decreses
				SetMotorDutyCycle(turnLimit-(outside_wheel*turnLimit*servoFactor),turnLimit+(inside_wheel*turnLimit*servoFactor), freq, 1);
			}
			// if brake boolean is on brake until the brake timer hits 0, then brake = 0
			// only executes if brake is set to 1
			if (brake){
				GPIOB_PTOR = (1 << 22);      //sets red to on
				GPIOE_PTOR = (1 << 26);      //sets green to on
				GPIOB_PTOR = (1 << 21);      //sets blue to on
				b_midpoint = midpoint;
				SetMotorDutyCycle(brake_speed, brake_speed, freq, 0);
				SetServoDutyCycle(9.75 - servoFactor, 50);
				GPIOB_PTOR = (1 << 22);      //sets red to on
				GPIOE_PTOR = (1 << 26);      //sets green to on
				GPIOB_PTOR = (1 << 21);      //sets blue to on
				}
			}else{
				SetServoDutyCycle(9.75 - servoFactor, 50);
				SetMotorDutyCycle(turnLimit-(outside_wheel*turnLimit*servoFactor),turnLimit+(inside_wheel*turnLimit*servoFactor), freq, 1);
			}
	} else {
	// Straight
	// set states for other code
	previous_state2 = previous_state;
	previous_state = current_state;
	current_state = 's';
	// turn on timer if its not on already
	if (timer_counter <1){
		FTM1_CNT &= ~(FTM_CNT_COUNT_MASK);
		timer_counter = 1;
		brake = 0;
		brake_time = 0;
	}
	// goes straight
	SetServoDutyCycle(9.75,50);
	SetMotorDutyCycle(speedLimit,speedLimit, freq, 1);
}
}



void calculateDerivatives(void){

	int i;
	int delta, sum;
	avg_line[0] = line[0];
	avg_line[1] = line[1];
	derive_line[0] = avg_line[1] - avg_line[0];
	for(i = 2; i < 127; i++ ){
		avg_line[i] = (((line[i-2])+ (line[i-1]) + (line[i]) + (line[i+1]) + (line[i+2]))/6);
		avg_line[i+1] = (((line[i-1])+ (line[i]) + (line[i+1]) + (line[i+2]) + (line[i+3]))/6);
		derive_line[i-1] = ((avg_line[i+1] - avg_line[i-1])/2);
	}
	derive_line[126] = (avg_line[127] - avg_line[125])/2;
	derive_line[127] = (avg_line[127] - avg_line[126]);
	/*
	derive2_line[0] = derive_line[1] - derive_line[0];
	for(i = 2; i < 127; i++ ){
		derive2_line[i-1] = ((derive_line[i+1] - derive_line[i-1])/2);
	}
	derive2_line[126] = (derive_line[127] - derive_line[125])/2;
	derive2_line[127] = (derive_line[127] - derive_line[126]);
	*/
}

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

void delay(int del){
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
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

	// Enable hardware trigger from FTM2
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
	PIT_LDVAL0 = (uint32_t)((2.0*INTEGRATION_TIME) * DEFAULT_SYSTEM_CLOCK);

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
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
		// Configure mux for Outputs
	PORTB_PCR21 = PORT_PCR_MUX(1); // Blue Led; Port B, Pin 21
	PORTB_PCR22 = PORT_PCR_MUX(1); // Red Led; Port B, Pin 22
	PORTB_PCR9 = PORT_PCR_MUX(1); // output from PTC7
	PORTB_PCR23 = PORT_PCR_MUX(1);

	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //LED
	PORTE_PCR26 = PORT_PCR_MUX(1); // Green Led; Port E, Pin 26

	//Configure Port Control Register for Inputs with pull enable and pull up resistor
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; //button sw2
	PORTC_PCR6 = PORT_PCR_MUX(1); // Built in Push Button; Port C, Pin 6

	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //button sw3
	PORTA_PCR4 = PORT_PCR_MUX(1); //check alternative
	// Switch the GPIO pins to output mode (Red and Blue LEDs)

	GPIOB_PDDR |= (1 << 21) | (1 << 22) | (1 << 9) | (1 << 23); // Loads data into the direction register
	GPIOE_PDDR |= (1 << 26);
	//GPIOB_PDDR |= (1 << 9);
	//GPIOB_PDDR |= (1 << 23);


	// Turn off the LEDs

	GPIOB_PSOR |= (1 << 21) | (1 << 22); // Drives output to the LEDs high, initializing them to "off" state
	GPIOE_PSOR |= (1 << 26);

	// Set the push buttons as an input

	GPIOC_PDDR |= (0 << 6);
	GPIOA_PDDR |= (0 << 4);

	// interrupt configuration for SW3(Rising Edge) and SW2 (Either)
	PORTC_PCR6 = PORT_PCR_IRQC(9);//sw2 rising only
	//PORTA_PCR4 = PORT_PCR_IRQC(9);//sw2 rising only
	PORTA_PCR4 |= PORT_PCR_IRQC(9);//sw3 both rising and falling


	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);

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

void initFTM1(void){
	//Enable clock for FTM module (use FTM0)
	SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK; // X



	//turn off FTM Mode to write protection
	// bit 2 (WPDIS) is 1 for write disable, 0 for write enable // X
	FTM1_MODE |= 0x05; // XY

	//divide the input clock down by 128,
	FTM1_SC |= FTM_SC_PS(7);


	//reset the counter to zero
	FTM1_CNT = 0;

	//Set the overflow rate
	//(Sysclock/128)- clock after prescaler
	//(Sysclock/128)/1000- slow down by a factor of 1000 to go from
	//Mhz to Khz, then 1/KHz = msec
	//Every 1msec, the FTM counter will set the overflow flag (TOF) and
	FTM1->MOD = (DEFAULT_SYSTEM_CLOCK/(1<<7))/1000;

	//Select the System Clock
	FTM1_SC |= FTM_SC_CLKS(1);

	//Enable the interrupt mask. Timer overflow Interrupt enable
	FTM1_SC |= FTM_SC_TOIE_MASK;

	NVIC_EnableIRQ(FTM1_IRQn);

	return;
}

void FTM1_IRQHandler(void){ //For FTM timer
	//clear FTM interrupt
	FTM1_SC &= ~(FTM_SC_TOF_MASK);
    //if timer variable is initiated, increment timer
	//
	if (brake) {
		if (brake_time > min_brake){
			brake_time -=1;
	} else{
		brake_time = 0;
		brake = 0;
		timer_counter = 0;
	}
}else{
	 if (timer_counter>0 && timer_counter < 1000){
		 timer_counter += 1;
		 brake = 0;
		 brake_time = 0;
	}
}

	return;
}