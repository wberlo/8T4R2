/*
 * tinyrobot.c
 *
 * Created: 2014-02-02 19:56:00
 *  Author: Wim
 *
 * tiny robot with 2 wheel drive
 * temperature sensor sets driving mood
 * sonar detects objects
 * L wheel on PB0, PB2
 * R wheel on PB1, PA7
 * PB0, PB1 are direction pins
 * PB2, PA7 are speed pins (pwm)
 * TMP36 on PA1
 * Sonar on PA3 (trigPin) and PA0 (echoPin)
 * Timer0 is used for speed (pwm)
 * Timer1 is used for timing sonar pulse
 * pin usage:
 * PORTA:
 * PA0: input: SONARread echopin (Timer1)
 * PA1: input: ADClib TMP36 signal
 * PA2: LED
 * PA3: output: SONARread trigpin
 * PA4: available
 * PA5: available
 * PA6: available
 * PA7: output: MOTORright speed (pwm, Timer0)
 * PORTB:
 * PB0: output: MOTORleft direction
 * PB1: output: MOTORright direction
 * PB2: output: MOTORleft speed (pwm, Timer0)
 * PB3: not available: RESET
 */ 
/*
* TODO: implement compass (LSM303D) using I2C;
*		Need SCL and SDA pins (PA4 and PA6?)
* NOTE: can't use any more Pin Change Interrupts, will mess with Sonar
*       Bummer, would need PCINT for wheel encoders
*/

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "motordrive.h"
//#include "adclib.h"
#include "HC_SR04_AT84.h"



/************************************************************************/
/*  function: blink_A2													*/
/*	input: uint8_t freq													*/
/*	output: void														*/
/*	blinks an LED on port A2 a number of times							*/
/*	turns LED on for 0.1 s and off for 0.1 s							*/
/*	repeats freq times													*/
/************************************************************************/
void blink_A2(uint8_t freq)
{
	DDRA |= (1<<PA2);
	uint8_t ii = freq;
	do						// blink LED
	{
		PORTA |= (1<<PA2);
		_delay_ms(100);
		PORTA &= ~(1<<PA2);
		_delay_ms(100);
	} while (ii--);
}	// end blink_A2

void sign_R_A2(void)
{
	DDRA |= (1<<PA2);
		PORTA |= (1<<PA2);
		_delay_ms(200);
		PORTA &= ~(1<<PA2);
		_delay_ms(200);
		PORTA |= (1<<PA2);
		_delay_ms(600);
		PORTA &= ~(1<<PA2);
		_delay_ms(200);
		PORTA |= (1<<PA2);
		_delay_ms(200);
		PORTA &= ~(1<<PA2);
		_delay_ms(600);
}	// end blink_A2

void sign_SOS_A2(void)
{
	DDRA |= (1<<PA2);
	PORTA |= (1<<PA2);
	_delay_ms(200);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(200);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(200);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(600);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(600);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(600);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(200);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(200);
	PORTA &= ~(1<<PA2);
	_delay_ms(200);
	PORTA |= (1<<PA2);
	_delay_ms(200);
	PORTA &= ~(1<<PA2);
	_delay_ms(600);
}	// end blink_A2

long turn2open(int speed)
{
	long maxdistance = 0;
	uint8_t maxii = 0;
	long distarr[12];
	for (uint8_t ii = 0; ii <12; ii++)
	{
		MOTORleft(speed);	//place turn right
		MOTORright(-speed);
		_delay_ms(233);			// 2800 ms is one turn, 2800 / 12 is 30 deg turn
		distarr[ii] = 0;
		MOTORleft(0);	//place turn right
		MOTORright(0);
		_delay_ms(50);			// 2800 ms is one turn, 2800 / 12 is 30 deg turn
		distarr[ii] = SONARavg()/59;
		if (distarr[ii]> maxdistance)
		{
			maxdistance = distarr[ii];
			maxii = ii;
		}
	}
	MOTORleft(0);
	MOTORright(0);
	_delay_ms(300);
	// now turn to max distance
	MOTORleft(speed);	//place turn right
	MOTORright(-speed);
	for (uint8_t ii = 0; ii < maxii; ii++)
	{
		_delay_ms(233);
	}
	return maxdistance;
}

int main()
{
	long distance;
	int CruiseSpeed, TurnSpeed;
	uint8_t ObjectAhead = 0;
	MOTORinit();
	SONARinit();
	CruiseSpeed = MAXSPEED*4/5;
	TurnSpeed = CruiseSpeed/2;
	sign_R_A2();

	DDRA |= (1<<PA2);
/*	
	// test turning speed of vehicle
	while(1) {
		PORTA |= (1<<PA2);
		MOTORleft(TurnSpeed);	//place turn right
		MOTORright(-TurnSpeed);
		_delay_ms(2800);
		PORTA &= ~(1<<PA2);
		MOTORright(0);
		MOTORleft(0);
		_delay_ms(2000);
	}
	// end test turning speed of vehicle
*/
	turn2open(TurnSpeed);
	// should be pointing roughly in most open direction; time to cruise ...

	while(1)
	{	
		// distance = 340 m/s * duration = 0.340 mm/us * timerval / 2 = 0.017 cm/us * timerval
		PORTA |= (1<<PA2);
		distance = SONARavg()/59;
		PORTA &= ~(1<<PA2);
		if (distance < 40)			// object ahead
		{
			ObjectAhead = 1;
		}	
		while (ObjectAhead)	
		{	
			MOTORleft(0);			//stop
			MOTORright(0);
			if (distance < 15)		// too close, back off
			{
				PORTA |= (1<<PA2);
				MOTORleft(-TurnSpeed);
				MOTORright(-TurnSpeed);
				_delay_ms(700);
				PORTA &= ~(1<<PA2);
			}
			MOTORleft(TurnSpeed);	//place turn right
			MOTORright(-TurnSpeed);
			_delay_ms(700);			// time this for approx 90 degree turn
			distance = turn2open(TurnSpeed);
			if (distance > 40)
			{
				ObjectAhead = 0;
			}
			else
			{
				sign_SOS_A2();
			}
			
		}  // end distance check
		MOTORleft(CruiseSpeed);
		MOTORright(CruiseSpeed);
		_delay_ms(100);
	}
	return 1;
}
