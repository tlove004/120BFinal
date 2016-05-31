//pwm.h
// By Tyson Loveless, based on pwm library for UCR cs 120B course
// uses phase correct PWM
// replace with commented out sections for fast PWM and set prescalar values


void set_PWM(double count) {
	static double current_count; 
	// Will only update the registers when the output changes
	if (count != current_count) {
		if (count > 255) count = 255;
		else if (count < 0) count = 0;
		
		OCR2A = count; 
		TCNT2 = 0; // resets counter
		current_count = count;
	}
}

void PWM_on() {
	TCCR2A = (1 << COM2A1)|(1 << COM2B1) | (1 << WGM20);  //phase correct pwm
	//TCCR2A = (1 << COM2A1)|(1 << COM2B1) | (1 << WGM20) |(1 << WGM21); //fast PWM
	//TCCR2B = (1 << CS20);  //prescalar = 1 (no prescaling)
	TCCR2B =  (1 << CS21); //prescalar = 8
	set_PWM(0);
}

void PWM_off() {
	TCCR2A = 0x00;
	TCCR2B = 0x00;
}