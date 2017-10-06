/*
 *  Interrupt and PWM utilities for 16 bit Timer1 on ATmega168/328
 *  Original code by Jesse Tane for http://labs.ideo.com August 2008
 *  Modified March 2009 by JÃ©rÃ´me Despatis and Jesse Tane for ATmega328 support
 *  Modified June 2009 by Michael Polli and Jesse Tane to fix a bug in setPeriod() which caused the timer to stop
 *  Modified June 2011 by Lex Talionis to add a function to read the timer
 *  Modified Oct 2011 by Andrew Richards to avoid certain problems:
 *  - Add (long) assignments and casts to TimerOne::read() to ensure calculations involving tmp, ICR1 and TCNT1 aren't truncated
 *  - Ensure 16 bit registers accesses are atomic - run with interrupts disabled when accessing
 *  - Remove global enable of interrupts (sei())- could be running within an interrupt routine)
 *  - Disable interrupts whilst TCTN1 == 0.  Datasheet vague on this, but experiment shows that overflow interrupt
 *    flag gets set whilst TCNT1 == 0, resulting in a phantom interrupt.  Could just set to 1, but gets inaccurate
 *    at very short durations
 *  - startBottom() added to start counter at 0 and handle all interrupt enabling.
 *  - start() amended to enable interrupts
 *  - restart() amended to point at startBottom()
 * Modiied 7:26 PM Sunday, October 09, 2011 by Lex Talionis
 *  - renamed start() to resume() to reflect it's actual role
 *  - renamed startBottom() to start(). This breaks some old code that expects start to continue counting where it left off
 *
 *  This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  See Google Code project http://code.google.com/p/arduino-timerone/ for latest
 */
#ifndef TIMERONE_cpp
#define TIMERONE_cpp

#include "TimerOne.h"

TimerOne Timer1;              // preinstatiate

ISR(TIMER1_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
	Timer1.isrCallback();
}


void TimerOne::initialize(long microseconds)
{
	/* clear control register A */
	TCCR1A = 0;
	/* set mode 8: phase and frequency correct pwm, stop the timer */
	TCCR1B = _BV(WGM13);
	setPeriod(microseconds);
}

/* AR modified for atomic access */
void TimerOne::setPeriod(long microseconds)
{
	/*
	* the counter runs backwards after TOP,
	* interrupt is at BOTTOM so divide microseconds by 2
	*/
	long cycles = (F_CPU / 2000000) * microseconds;

	if(cycles < RESOLUTION){
		/* no prescale, full xtal */
		clockSelectBits = _BV(CS10);
	}
	else if((cycles >>= 3) < RESOLUTION){
		/* prescale by /8 */
		clockSelectBits = _BV(CS11);
	}
	else if((cycles >>= 3) < RESOLUTION){
		/* prescale by /64 */
		clockSelectBits = _BV(CS11) | _BV(CS10);
	}
	else if((cycles >>= 2) < RESOLUTION){
		/* prescale by /256 */
		clockSelectBits = _BV(CS12);
	}
	else if((cycles >>= 2) < RESOLUTION){
		/* prescale by /1024 */
		clockSelectBits = _BV(CS12) | _BV(CS10);
	}
	else{
		cycles = RESOLUTION - 1;
		/* request was out of bounds, set as maximum */
		clockSelectBits = _BV(CS12) | _BV(CS10);
	}

	oldSREG = SREG;

	/* Disable interrupts for 16 bit register access */
	cli();
	/* ICR1 is TOP in p & f correct pwm mode */
	ICR1 = pwmPeriod = cycles;
	SREG = oldSREG;

	TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
	/* reset clock select register, and starts the clock */
	TCCR1B |= clockSelectBits;
}

void TimerOne::setPwmDuty(char pin, int duty)
{
	unsigned long dutyCycle = pwmPeriod;

	dutyCycle *= duty;
	dutyCycle >>= 10;

	oldSREG = SREG;
	cli();

	if(pin == 1 || pin == 9)
		OCR1A = dutyCycle;
	else if(pin == 2 || pin == 10)
		OCR1B = dutyCycle;

	SREG = oldSREG;
}

/* expects duty cycle to be 10 bit (1024) */

void TimerOne::pwm(char pin, int duty, long microseconds)
{
	if(microseconds > 0)
		setPeriod(microseconds);

	if(pin == 1 || pin == 9) {
		/* sets data direction register for pwm output pin */
		DDRB |= _BV(PORTB1);
		/* activates the output pin */
		TCCR1A |= _BV(COM1A1);
	}

	else if(pin == 2 || pin == 10) {
		DDRB |= _BV(PORTB2);
		TCCR1A |= _BV(COM1B1);
	}

	setPwmDuty(pin, duty);
	/*
	 *	Lex - make sure the clock is running.
	 *
	 *	We don't want to restart the count, in
	 *	case we are starting the second WGM
	 *  and the first one is in the middle of a cycle
	 */
	resume();
}

void TimerOne::disablePwm(char pin)
{
	if(pin == 1 || pin == 9)
		/* clear the bit that enables pwm on PB1 */
		TCCR1A &= ~_BV(COM1A1);

	else if(pin == 2 || pin == 10)
		/* clear the bit that enables pwm on PB2 */
		TCCR1A &= ~_BV(COM1B1);
}

void TimerOne::attachInterrupt(void (*isr)(), long microseconds)
{
	if(microseconds > 0)
		setPeriod(microseconds);

	/* register the user's callback with the real ISR */
	isrCallback = isr;
	/* sets the timer overflow interrupt enable bit */
	TIMSK1 = _BV(TOIE1);
	/*
	* 	might be running with interrupts disabled (eg inside an ISR), so don't touch the global state
	*  	sei();
	*/
	resume();
}

void TimerOne::detachInterrupt()
{
	/*
	* clears the timer overflow interrupt enable bit
	* timer continues to count without calling the isr
	*/
	TIMSK1 &= ~_BV(TOIE1);
}

void TimerOne::resume()
{
	TCCR1B |= clockSelectBits;
}

void TimerOne::restart()
{
	start();
}

/* AR addition, renamed by Lex to reflect it's actual role */
void TimerOne::start()
{
	unsigned int tcnt1;

	/* AR added */
	TIMSK1 &= ~_BV(TOIE1);
	/* AR added - reset prescaler (NB: shared with all 16 bit timers); */
	GTCCR |= _BV(PSRSYNC);

	/* AR - save status register */
	oldSREG = SREG;
	/* AR - Disable interrupts */
	cli();
	TCNT1 = 0;
	/* AR - Restore status register */
	SREG = oldSREG;
	resume();

	/* Wait until timer moved on from zero - otherwise get a phantom interrupt */
	do {
		oldSREG = SREG;
		cli();
		tcnt1 = TCNT1;
		SREG = oldSREG;
  	}
  while (tcnt1==0);
}

void TimerOne::stop()
{
	/* clears all clock selects bits */
	TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
}

/* Returns the value of the timer in microseconds */

unsigned long TimerOne::read()
{
	unsigned long tmp;
	unsigned int tcnt1;

	oldSREG= SREG;
	cli();
	tmp=TCNT1;
	SREG = oldSREG;

	char scale=0;

	switch (clockSelectBits) {
		case 1:
			/* no prescalse */
			scale=0;
		break;
		case 2:
			/* x8 prescale */
			scale=3;
		break;
		case 3:
			/* x64 */
			scale=6;
		break;
		case 4:
			/* x256 */
			scale=8;
		break;
		case 5:
			/* x1024 */
			scale=10;
		break;
	}

	/* Max delay here is ~1023 cycles.  AR modified */
	do {
		oldSREG = SREG;
		cli();
		tcnt1 = TCNT1;
		SREG = oldSREG;
	}
	while (tcnt1==tmp);

	/*
	*	if we are counting down add the top value to how far we have counted down.
	*	AR amended to add casts and reuse previous TCNT1
	 */
	tmp = (  (tcnt1>tmp) ? (tmp) : (long)(ICR1-tcnt1)+(long)ICR1  );

	return ((tmp*1000L)/(F_CPU /1000L))<<scale;
}

#endif /* TIMERONE_cpp */











