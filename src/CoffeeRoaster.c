/*	CoffeeRoaster.c - 5/14/2016 1:29:19 PM
 *	By Tyson Loveless
 *	I acknowledge all content contained herein, excluding template, notated headers or example 
 *	code, is my own original work.
 */ 


#include <avr/io.h>
#include <stdlib.h>

#include "scheduler.h"
#include "5110.c"            //nokia LCD
#include "31855.c"           //thermoprobes
#include "io.c"
#include "PID_v1.c"          //pid library
#include "pwm.h"             //using phase correct pwm
#include "timer.h"
#include "coffeestart.h"     //intro graphic
#include "coffeefinish.h"    //finish graphic

//set debugging mode: 1, turn off: 0
//saves space on chip if turned off
#define DEBUGGING 0
#if (DEBUGGING)
	#include "ftoa.h"
	#include "math.h"
#endif

#define TIMEDIV 60

//define which channel the thermoprobes are in
//set prior to starting to ensure probes are in correct channel
#define BEANMASS    0         
#define BASE        1


//--------Shared Variables----------------------------------------------------
	//for the temperatures
	int tmpBeanMass;
	int tmpBase;
	int last_temp;
	
	//flags
	char on;
	char finished;
	short profile;
	char reset;
	
	//variables for the roast profiles
	//---------------------------------
			//roast profiles
			#define num_profiles 5
			unsigned short roast[num_profiles][4] = {
				{ 211, 306, 400, 426 },  //city
				{ 210, 340, 400, 436 },  //city+
				{ 200, 270, 394, 444 },  //full city
				{ 100, 110, 210, 230 },   //demo
				{ 100, 500, 300, 100 } //demo 2
			};
	
			char *roast_name[num_profiles] = {
				"  City   ",
				"  City+  ",
				"Full City",
				"  Demo   ",
				" Demo 2  "
			};
			
			char *label[4] = {
				"FirstRamp  ",
				"FirstShelf ",
				"2nd Ramp   ",
				"2nd Shelf  "
			};
			
			char length[4] = {
				2, 2, 4, 4
			};
	
			enum PHASE {Waiting, Heating, Cooling } phase;
	
			// indexes for the profiles:
			#define RAMP1TEMP 0
			#define SHELF1TEMP 1
			#define RAMP2TEMP 2
			#define SHELF2TEMP 3
		
	
	//variables for PID function
	//-------------------------
			const int WindowSize = 255;
			unsigned long windowStartTime;//, currentMillis, previousMillis, TSwitch, TStart;
			//char currentTriacPower=0;
			//int seconds;

			//Define Variables we'll be connecting to
			double Setpoint, Input, Output;

			//Define the aggressive and conservative Tuning Parameters
			double aggKp=4, aggKi=0.2, aggKd=1;
			double consKp=1, consKi=0.05, consKd=0.25;
	
			// Tuning parameters
			float Kp=2; 			//Initial Proportional Gain
			float Ki=5; 			//Initial Integral Gain
			float Kd=1; 			//Initial Differential Gain
	
			//function that calls PID function
			void update(int max_temp, double rampRate, unsigned long startTime, int startTemp, unsigned long endTime);
			
			void updateTime();
			
//--------End Shared Variables------------------------------------------------



//Interrupt Service Routine for INT0 - this is for zero-cross detection via AVR182 specifications
//  I am using this currently to simply output to an LED to remind me there is an active current
//     on my board
ISR(INT0_vect)
{
	delay_ms(100);
	
	PORTA = 0x01;
	delay_ms(10);
	PORTA = 0x00;
}



//--------User defined FSMs---------------------------------------------------
int SMTimer_Tick(int state);			//time of operation
int SMTemp_BM_Tick(int state);			//temp of the bean mass
int SMTemp_Base_Tick(int state);		//temp of the base
int SMHeat_Tick(int state);				//PWM of heater
int SMInput_Tick(int state);			//get input from user
int SMUpdatePID_Tick(int state);		//update the setpoint and PWM input


//--------State Machine Definitions----------------------------------------------------

//--------SMTemp_BM States Begin----------------------------------------------------
enum SMTemp_BM { SMTemp_BM_update };
	
/*---------------------------------Bean Mass Temp-------------------------------
*
* This reads and updates the temperature of the actual mass of beans within
*    the roasting chamber
*
----------------------------------------------------------------------------*/
int SMTemp_BM_Tick(int state) {
	
	
	static char temp[4];
	
	switch(state) {
		case SMTemp_BM_update:			tmpBeanMass = ThermoReadF(BEANMASS);
										itoa(tmpBeanMass, temp, 10);
										if (!finished) {
											lcd_goto_xy(11, 4);
											lcd_string_format(temp);
											lcd_str("  ");
										}
										else state = -1;
										break;
		
		default:						if (!finished) {
											state = SMTemp_BM_update;
										}
										break;
	}
	return state;
}
//--------SMTemp_BM States End----------------------------------------------------

//--------SMTemp_Base States Begin----------------------------------------------------
enum SMTemp_Base { SMTemp_Base_update };
	
/*---------------------------------Base Temp-------------------------------------
 *
 * This reads and updates the environmenal (tempurature entering the roasting
 *    chamber) temperature.
 *
 ----------------------------------------------------------------------------*/
int SMTemp_Base_Tick(int state) {
	
	static char temp[4];
	
	switch(state) {
		case SMTemp_Base_update:		tmpBase = ThermoReadF(BASE);
										itoa(tmpBase, temp, 10);
										if (!finished) {
											lcd_goto_xy(11, 3);
											lcd_string_format(temp);
											lcd_str("  ");
										}
										else state = -1;
										break;
		
		default:						if (!finished) {
											state = SMTemp_Base_update;
										}
										break;
	}
	return state;
}
//--------SMTemp_Base States End----------------------------------------------------


//--------SMTimer States Begin----------------------------------------------------
enum SMTimer { SMTimer_displayTime };
	
	/*---------------------------------timer-------------------------------------
 *
 *  Function for displaying time to the Nokia 5110 LCD
 *
 ----------------------------------------------------------------------------*/
int SMTimer_Tick(int state) {

	
	switch (state) {
		case SMTimer_displayTime:		if (on && !finished) {
											updateTime();  //displays the roasting time on the LCD display
											lcd_goto_xy(2, 1);
											if (phase == Heating || phase == Waiting) {
												lcd_str("  Heating  ");
											}
											else if (phase == Cooling) {
												lcd_str("  Cooling  ");
											}
										}
										else {
											state = -1;
										}
										break;
								
		default:						if (!finished) {
											if (on) {
												state = SMTimer_displayTime;
												minuteOnes = 0;
												minuteTens = 0;
												secondOnes = 0;
												secondTens = 0;
												milliOnes = 0;
												milliTens = 0;
												lcd_goto_xy(4, 2);
												lcd_chr('0' + minuteOnes);
												lcd_chr(':');
												lcd_chr('0' + secondTens);
												lcd_chr('0' + secondOnes);
												lcd_chr(':');
												lcd_chr('0' + milliTens);
												lcd_chr('0' + milliOnes);
												lcd_goto_xy(2, 1);
												if (phase == Heating || phase == Waiting) {
													lcd_str("  Heating  ");
												}
												else if (phase == Cooling) {
													lcd_str("  Cooling  ");
												}
											}
											else {
												lcd_goto_xy(2, 1);
												if (phase == Cooling) {
													lcd_str("  Cooling  ");
												}
												else
												lcd_str("Make Choice");
											}
										}
										break;
	}
	/*
	lcd_goto_xy(2, 1);
	if (!finished) {
		if (on) {
			if (phase == Heating || phase == Waiting) {
				lcd_str("  Heating  ");
			}
			else if (phase == Cooling) {
				lcd_str("  Cooling  ");
			}
		}
		else  {
			if (phase == Cooling) {
				lcd_str("  Cooling  ");
			}
			else
			lcd_str("Make Choice");
		}
	}*/
	
	return state;
}
//--------SMTimer States End----------------------------------------------------


/*---------------------------------Input-------------------------------------
 *  This function gets input from the user.  Special care is taken to ensure 
 *     the roaster does not suffer from a bad user - a cool cycle will always 
 *     engage if the roaster is hotter than it ought to be depending on 
 *     the state of inputs.
 ----------------------------------------------------------------------------*/
enum SMInput { SMInput_Init, SMInput_Wait, SMInput_ButtonPress, SMInput_Start, SMInput_WaitforCool };
int SMInput_Tick(int state) {
	static unsigned char roastBtn, startSwitch, i;
	
	char tempELabel[] = {"E_Temp:"};
	char tempBMLabel[] = {"BM_Temp:"};
	
	
	roastBtn = ~PINC & 0x02;
	startSwitch = PINC & 0x01;
	
	switch(state){
		case SMInput_Init:			if (i-- <= 0) {
										lcd_clear();
										state = SMInput_Wait;
										profile = on = 0;
										lcd_goto_xy(3, 3);
										lcd_string_format((char*)tempELabel);
										lcd_goto_xy(2, 4);
										lcd_string_format((char*)tempBMLabel);
										lcd_goto_xy(3, 5);
										lcd_str(roast_name[profile]);
									}
									break;
		
		case SMInput_Wait:			if (startSwitch) {
										state = SMInput_Start;
										on = 1;
										lcd_goto_xy(3, 5);
										lcd_str("         ");
									}
			
									else if (roastBtn) {
										state = SMInput_ButtonPress;
										profile = (profile+1) % num_profiles;
										lcd_goto_xy(3, 5);
										lcd_str(roast_name[profile]);
									}
									break;
									
		case SMInput_ButtonPress:	state = roastBtn ? state : SMInput_Wait;
									break;
									
		case SMInput_Start:			if (!startSwitch) {
										//state = SMInput_Wait;
										state = SMInput_WaitforCool;
										on = 0;
										lcd_goto_xy(3, 5);
										//lcd_str("EarlyTerm");
									}
									else if (finished) {
										state = -1;
										printPictureOnLCD(finishPic);
									}
									break;
									
		case SMInput_WaitforCool:	if (tmpBeanMass < 100)
										state = -1;
									
									break;
									
		default:					if (finished) {
											if (roastBtn || !startSwitch) {
												finished = 0;
												lcd_clear();
												reset = 1;
											}
										}
									else {
										state = SMInput_Init;
										printPictureOnLCD(initPic);
										i = 20;
									}
									break;
	}
	
	
	return state;
}


/*---------------------------------HEATING-----------------------------------
 *
 *  This function moves through the different heating cycles for a user -
 *    selected roast profile.
 *    
 ----------------------------------------------------------------------------*/
enum SMHeat_PWM { SMHeat_Wait, SMHeat_Cool, SMHeat_Heat }; //SMHeat_FirstRamp, SMHeat_FirstShelf, SMHeat_SecondRamp, SMHeat_SecondShelf,
int SMHeat_Tick(int state) {
 
	static int max_temp; //in degrees F
	static int startTemp; //in degrees F
	static double ramp_rate; //in degrees F/sec
	static unsigned long startTime; //in seconds
	static unsigned long endTime;	//in seconds
	
	static unsigned char i; //index

	switch(state) {
		case SMHeat_Wait:				if (on) {
											state = SMHeat_Heat;//SMHeat_FirstRamp;
											phase = Heating;
											i = 0;
											lcd_goto_xy(1,0);
											lcd_str(label[i]);
//											lcd_str("FirstRamp  ");
											max_temp = roast[profile][i];
//											max_temp = roast[profile][RAMP1TEMP];
											last_temp = startTemp = tmpBeanMass;
											ramp_rate = (double)(max_temp-startTemp)/(double)(length[i]*TIMEDIV);
//											ramp_rate = (double)(max_temp-startTemp)/(double)(2*TIMEDIV);
											startTime = seconds();	
											endTime = startTime + length[i]*TIMEDIV;
//											endTime = startTime + 2*TIMEDIV;
											PWM_on();
										}
										else if (!finished) {
											if (tmpBeanMass > 110)
											{
												state = SMHeat_Cool;
												lcd_goto_xy(1,0);
												lcd_str("   Too Hot!  ");
												phase = Cooling;
												PWM_off();
											}
										}
										break;
										
		case SMHeat_Heat:				if (tmpBeanMass >= max_temp-4) {
											lcd_goto_xy(1,0);
											i++;
											lcd_str(label[i]);
											max_temp = roast[profile][i];
											last_temp = startTemp = tmpBeanMass;
											ramp_rate = (double)(max_temp-startTemp)/(double)(length[i]*TIMEDIV);
											startTime = seconds();
											endTime = startTime + length[i]*TIMEDIV;
										}
										if (!on) {
											PWM_off();
											if (tmpBeanMass > 100)
											{
												state = SMHeat_Cool;
												lcd_goto_xy(1,0);
												lcd_str("Early Term!");
												phase = Cooling;
											}
											else {
												state = -1;
												lcd_goto_xy(1,0);
												lcd_str("           ");
												phase = Waiting;
											}
										}
										update(max_temp, ramp_rate, startTime, startTemp, endTime);
										break;
										
/*		case SMHeat_FirstRamp:			if (tmpBeanMass >= max_temp-4) {
											state = SMHeat_FirstShelf;
											lcd_goto_xy(1,0);
											lcd_str("FirstShelf ");
											max_temp = roast[profile][SHELF1TEMP];
											last_temp = tmpBeanMass;
											startTemp = tmpBeanMass;
											ramp_rate = (double)(max_temp-startTemp)/(double)(2*TIMEDIV);
											startTime = seconds();
											endTime = startTime + 2*TIMEDIV;
										}
										if (!on) {
											PWM_off();
											if (tmpBeanMass > 100)
											{
												state = SMHeat_Cool;
												lcd_goto_xy(1,0);
												lcd_str("Early Term!");
												phase = Cooling;
											}
											else {
												state = -1;
												lcd_goto_xy(1,0);
												lcd_str("           ");
												phase = Waiting;
											}
										}
										update(max_temp, ramp_rate, startTime, startTemp, endTime);
										break;
								
		case SMHeat_FirstShelf:			if (tmpBeanMass >= max_temp-4) {
											state = SMHeat_SecondRamp;
											lcd_goto_xy(1,0);
											lcd_str("2nd Ramp   ");
											max_temp = roast[profile][RAMP2TEMP];
											last_temp = tmpBeanMass;
											startTemp = tmpBeanMass;
											ramp_rate = (double)(max_temp-startTemp)/(double)(4*TIMEDIV);
											startTime = seconds();
											endTime = startTime + 4*TIMEDIV;
										}
										if (!on) {
											PWM_off();
											if (tmpBeanMass > 100)
											{
												state = SMHeat_Cool;
												lcd_goto_xy(1,0);
												lcd_str("Early Term!");
												phase = Cooling;
											}
											else {
												state = -1;
												lcd_goto_xy(1,0);
												lcd_str("            ");
												phase = Waiting;
											}
										}
										update(max_temp, ramp_rate, startTime, startTemp, endTime);
										break;
										
		case SMHeat_SecondRamp:			if (tmpBeanMass >= max_temp-4) {
											state = SMHeat_SecondShelf;
											lcd_goto_xy(1,0);
											lcd_str("2nd Shelf  ");
 											max_temp = roast[profile][SHELF2TEMP];
											last_temp = tmpBeanMass;
											startTemp = tmpBeanMass;
											ramp_rate = (double)(max_temp-startTemp)/(double)(4*TIMEDIV);
 											startTime = seconds();
 											endTime = startTime + 4*TIMEDIV;
										}
										if (!on) {
											PWM_off();
											if (tmpBase > 100)
											{
												state = SMHeat_Cool;
												lcd_goto_xy(1,0);
												lcd_str("Early Term!");
												phase = Cooling;
											}
											else {
												state = -1;
												lcd_goto_xy(1,0);
												lcd_str("          ");
												phase = Waiting;
											}
										}
										update(max_temp, ramp_rate, startTime, startTemp, endTime);
										break;
										
										
		case SMHeat_SecondShelf:		if (tmpBeanMass >= max_temp-4) {
											state = SMHeat_Cool;
											lcd_goto_xy(1,0);
											lcd_str("CoolingStage ");
											phase = Cooling;
											PWM_off();
										}
										if (!on) {
											PWM_off();
											if (tmpBeanMass > 100)
											{
												state = SMHeat_Cool;
												lcd_goto_xy(1,0);
												lcd_str("Early Term!");
												phase = Cooling;
											}
											else {
												state = -1;
												lcd_goto_xy(1,0);
												lcd_str("           ");
												phase = Waiting;
											}
										}
										update(max_temp, ramp_rate, startTime, startTemp, endTime);
										break;
		
		
*/		
		case SMHeat_Cool:				if (tmpBeanMass >= 100) {
											PWM_off();
										}
										else  {
											state = -1;
											lcd_goto_xy(1,0);
											lcd_str("             ");
											if (on) finished = 1;
										}
									
										break;
		
		default:						if (!finished) {
											state = SMHeat_Wait;
											phase = Waiting;
											windowStartTime = millis();
											PID_init(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT, 0, WindowSize);
											PID_SetMode(AUTOMATIC);
											//max_temp = 100;
											//PWM_on();
											DDRD |= 0x80; 
											PORTD &= 0x7F;
										}
										break;
										
	}
	
	return state;
}



/*---------------------------------Update-------------------------------------
 *
 *  Updates the PID tunings to be more aggressive when far away from setpoint
 *    If close to the setpoint, tunings are kept conservative 
 *
 ----------------------------------------------------------------------------*/
void update(int max_temp, double rampRate, unsigned long startTime, int startTemp, unsigned long endTime){	
	
	static double elapsedTime;
	static double goal;
	static char tmp[4];
	if ((endTime - seconds()) < 15 && (tmpBeanMass - max_temp) > 30 )
	{
		set_PWM(255);
		return;
	}
	elapsedTime = (seconds()-startTime);  //in seconds
	if (rampRate != 0){
		goal = startTemp + rampRate*elapsedTime;  // F + F/sec * sec = F
	}
	else
		goal = startTemp + elapsedTime;  // F + F/sec * sec = F
	
	itoa(goal, tmp, 10);
	lcd_goto_xy(3, 5);
	lcd_str("Goal: ");
	lcd_str(tmp);
	lcd_str(" ");
	
	//debugging stuff
	if (DEBUGGING) {
		itoa((int)elapsedTime, tmp, 10);
		lcd_str(tmp);
		lcd_str(" ");
		itoa(startTemp, tmp, 10);
		lcd_str(tmp);
		lcd_str(" ");
		ftoa(rampRate, tmp, 10);
		lcd_str(tmp);
	}
	//end debugging
	
	Setpoint = goal;
	//int diff = (tmpBase-tmpBeanMass);
	if ((tmpBase - 40) < tmpBeanMass) {  //tempBase always needs to be higher than tmpBeanMess
									 // to ensure we do not lose momentum
		Input = tmpBase;  
	}
	else
		Input = tmpBeanMass;
	
	double gap = abs(Setpoint-Input); //distance away from setpoint
	if (gap < 8)
	{  //we're close to setpoint, use conservative tuning parameters
		PID_SetTunings(consKp, consKi, consKd);
	}
	else
	{
		//we're far from setpoint, or decreasing - use aggressive tuning parameters
		PID_SetTunings(aggKp, aggKi, aggKd);
	}
	PID_Compute();          //Run the PID loop 
	if (goal-tmpBeanMass > 5 ) Output = 250;  //brute force error correction
	//PWM_on();
	if (Output == 0) {  //PWM < 7 does not turn on SSR at all.
		Output = 7;
	}
	set_PWM(Output);     //Write out the output from the  PID loop to our PWM pin
}


/*---------------------------------updateTime-------------------------------------
 *
 *  updates the milliseconds passed while the roaster is ON (switch is in on pos)
 *    these variables are set globally and are used in computing PID Settings
 *
 ----------------------------------------------------------------------------*/
void updateTime() {
	milliOnes += 0x01;
	if (milliOnes >= 10){
		milliTens += 1;
		milliOnes = 0;
		lcd_goto_xy(9, 2);
		lcd_chr('0' + milliTens);
		lcd_chr('0' + milliOnes);
		if (milliTens >= 10) {
			secondOnes += 1;
			milliTens = 0;
			lcd_goto_xy(7, 2);
			lcd_chr('0' + secondOnes);
			lcd_goto_xy(9, 2);
			lcd_chr('0' + milliTens);
			if (secondOnes >= 10) {
				secondTens += 1;
				secondOnes = 0;
				lcd_goto_xy(6, 2);
				lcd_chr('0' + secondTens);
				lcd_chr('0' + secondOnes);
				if (secondTens >= 6) {
					minuteOnes += 1;
					secondTens = 0;
					lcd_goto_xy(4, 2);
					lcd_chr('0' + minuteOnes);
					lcd_goto_xy(6, 2);
					lcd_chr('0' + secondTens);
					if (minuteOnes >= 10) {
						minuteTens += 1;
						minuteOnes = 0;
						lcd_goto_xy(3, 2);
						lcd_chr('0' + minuteTens);
						lcd_chr('0' + minuteOnes);
						
					}
				}
			}
		}
	}
	lcd_goto_xy(10, 2);
	lcd_chr('0' + milliOnes);
}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------MAIN--------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
int main(void) {
	
	DDRA = 0xFF; PORTA = 0x00;
	DDRB = 0xFF; PORTB = 0x00;
	DDRC = 0x00; PORTC = 0x02;
	DDRD = 0<<PORTD2;		// Set PD2 as input (Using for interupt INT0)
	PORTD = 0<<PORTD2;		// Enable PD2 pull-up resistor
	
	// period for the tasks
	unsigned long int SMTemp_BM_calc = 100;
	unsigned long int SMTimer_calc = 10;
	unsigned long int SMTemp_Base_calc = 100;
	unsigned long int SMHeat_calc = 1000;
	unsigned long int SMInput_calc = 50;
	
	// Calculating the GCD
	unsigned long int tmpGCD = 1;
	tmpGCD = findGCD(SMTemp_BM_calc, SMTimer_calc);
	tmpGCD = findGCD(tmpGCD, SMTemp_Base_calc);
	tmpGCD = findGCD(tmpGCD, SMHeat_calc);
	tmpGCD = findGCD(tmpGCD, SMInput_calc);
	
	//Greatest common divisor for all tasks or smallest time unit for tasks.
	unsigned long int GCD = tmpGCD;

	//Recalculate GCD periods for scheduler
	unsigned long int SMTemp_BM_period = SMTemp_BM_calc/GCD;
	unsigned long int SMTimer_period = SMTimer_calc/GCD;
	unsigned long int SMTemp_Base_period = SMTemp_Base_calc/GCD;
	unsigned long int SMHeat_period = SMHeat_calc/GCD;
	unsigned long int SMInput_period = SMInput_calc/GCD;
	
	//Declare an array of tasks
	static task task0, task1, task2, task3, task4;
	task *tasks[] = { &task0, &task1, &task2, &task3, &task4 };
	const unsigned short numTasks = sizeof(tasks)/sizeof(task*);
	
	// Task 0
	task0.state = -1;						//Task initial state.
	task0.period = SMTemp_BM_period;		//Task Period.
	task0.elapsedTime = 0;	//Task current elapsed time.
	task0.TickFct = &SMTemp_BM_Tick;		//Function pointer for the tick.

	
	// Task 1
	task1.state = -1;						//Task initial state.
	task1.period = SMTimer_period;			//Task Period.
	task1.elapsedTime = 0;					//Task current elapsed time.
	task1.TickFct = &SMTimer_Tick;			//Function pointer for the tick.
	
	// Task 2
	task2.state = -1;						//Task initial state.
	task2.period = SMTemp_Base_period;			//Task Period.
	task2.elapsedTime = 0;		//Task current elapsed time.
	task2.TickFct = &SMTemp_Base_Tick;			//Function pointer for the tick.
	
	// Task 3
	task3.state = -1;						//Task initial state.
	task3.period = SMHeat_period;			//Task Period.
	task3.elapsedTime = 0;		//Task current elapsed time.
	task3.TickFct = &SMHeat_Tick;			//Function pointer for the tick.
	
	// Task 4
	task4.state = -1;						//Task initial state.
	task4.period = SMInput_period;			//Task Period.
	task4.elapsedTime = SMInput_period;		//Task current elapsed time.
	task4.TickFct = &SMInput_Tick;			//Function pointer for the tick.

	milliseconds = 0;
	
	// Set the timer and turn it on
	TimerSet(GCD);
	TimerOn();
	
	sei();				//Enable Global Interrupt
	
	lcd_init();			//initialize nokia 5110
	ThermoInit();		//initialize MAX31855 temp reading
	
	unsigned char i; // Scheduler for-loop iterator
	delay_ms(100);
	while(1) {
		
		// Scheduler code
		for ( i = 0; i < numTasks; i++ ) {
			// Task is ready to tick
			if ( tasks[i]->elapsedTime >= tasks[i]->period ) {
				// Setting next state for task
				tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
				// Reset the elapsed time for next tick.
				tasks[i]->elapsedTime = 0;
			}
			tasks[i]->elapsedTime += 1;
		}
		
		if (reset) {  //starts from beginning once roast has completed or 
						//return from early termination of roast
			task0.state = -1;
			task1.state = -1;
			task2.state = -1;
			task3.state = -1;
			task4.state = -1;
			reset = 0;
		}
		
		while(!TimerFlag);
		TimerFlag = 0;	
		
	}
	
	//Error: Program should not exit!
	return 0;
}