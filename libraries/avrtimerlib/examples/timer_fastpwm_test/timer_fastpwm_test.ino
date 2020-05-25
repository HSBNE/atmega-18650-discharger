/*******************************************************************************
 * avr-timer-lib
 * timer_fastpwm_test.ino
 *
 * Test file for Fast PWM mode.
 * I am using an arduino MEGA ADK (AVR atmega2560).
 *
 * To use timers in PWM-mode, check the corresponding timer with pins.
 *
 * @author:		Rob Mertens
 * @date:			18/04/2017
 * @version:	1.1.1
 ******************************************************************************/

//Include avr headers.
#include "avr/io.h"

//Include library headers.
#include "timer8.hpp"
#include "timer16.hpp"

//LED INDICATOR.
#define DDRLED DDRB
#define OUTLED PORTB

using namespace avr;

//TIMER0.
Timer8	t0(t_alias::T0);
Timer16	t1(t_alias::T1);

//INIT.
void setup()
{
	//LED.
	DDRLED |= 0x80;								// Configure PB7 as output.
  OUTLED &= 0x7F;								// Initially turn LED off.

	//TIMER0 SETTINGS.
	t0.initialize(t_mode::PWM_F, t_channel::A, t_inverted::NORMAL);		// t_channel::A sets pwm channels A on T0 (Pin PB7).
																																		// I have a LED connected to PB7.
	t0.setPrescaler(1024);																						// Available prescalers for T0: 1, 8, 64, 256, 1024.
	t0.setDutyCycleA(0.0);																						// Set the dutycycle (OFF=0.0, FULL=1.0).
	t0.reset();																												// Set count value to zero.

	//TIMER1 SETTINGS.
	t1.initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);	// t_channel::BC_TOP sets A as TOP and pwm channels B and C.
	t1.setCompareValueA(0xC8);																						// REMARK: If you specify the TOP, you lose resolution.
	t1.setPrescaler(1024);																								// Available prescalers for T0: 1, 8, 64, 256, 1024.
	t1.setDutyCycleB(0.45);																								// Set the dutycycle (OFF=0.0, FULL=1.0).
	t1.setDutyCycleC(0.78);																								// Set the dutycycle.
	t1.reset();																														// Set count value to zero.
}

//MAIN PROGRAM.
void loop()
{
	for(double i=0; i<=1; i+=0.1)
	{
		t0.setDutyCycleA(i);
	}
}
