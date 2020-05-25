/*******************************************************************************
 * avr-timer-lib
 * timer_interrupt_test.ino
 *
 * Test file for timer interrupts.
 * I am using an arduino MEGA ADK (AVR atmega2560).
 *
 * Available timers for MEGA:
 *
 * 			| 8-bit | 16-bit
 *   T0	|   x   |
 *   T1	|       |   x
 *   T2	|   x   |
 *   T3	|       |   x
 *   T4	|       |   x
 *   T5	|       |   x
 *
 * @author:		Rob Mertens
 * @date:			18/04/2017
 * @version: 	1.1.1
 ******************************************************************************/

//Include library headers.
#include "timer.hpp"
#include "timer8.hpp"
#include "timer16.hpp"
#include "factory.hpp"

using namespace avr;

Timer::Ptr timer[6];

void setup()
{
	//SERIAL.
	Serial.begin(115200);

	//TIMER0 SETTINGS.
	timer[0] = new Timer8(t_alias::T0);
	//timer[0] = TimerFactory::startBelt()->produce(t_alias::T0);
	timer[0]->initialize(t_mode::CTC, t_interrupt::COMPA);		// TIMER0_OVF_vect not available (used by arduino).
	timer[0]->setCompareValueA(200);
	timer[0]->setPrescaler(1); 					// Available prescalers for T0: 1, 8, 64, 256, 1024.
	timer[0]->reset();

	//TIMER1 SETTINGS.
	timer[1] = new Timer16(t_alias::T1);
	timer[1]->initialize(t_mode::NORMAL, t_interrupt::OVF);
	timer[1]->setPrescaler(1); 					// Available prescalers for T1: 1, 8, 64, 256, 1024.
	timer[1]->reset();

	//TIMER2 SETTINGS.
	timer[2] = new Timer8(t_alias::T2);
	timer[2]->initialize(t_mode::NORMAL, t_interrupt::OVF);
	timer[2]->setPrescaler(32); 					// Available prescalers for T2: 1, 8, 32, 64.
	timer[2]->reset();

	//TIMER3 SETTINGS.
	timer[3] = new Timer16(t_alias::T3);
	timer[3]->initialize(t_mode::CTC, t_interrupt::COMPB);
	timer[3]->setCompareValueB(2000);
	timer[3]->setPrescaler(64);         				// Available prescalers for T3: 1, 8, 64, 256, 1024.
	timer[3]->reset();

	//TIMER4 SETTINGS.
	timer[4] = new Timer16(t_alias::T4);
	timer[4]->initialize(t_mode::CTC, t_interrupt::COMPC);
	timer[4]->setCompareValueC(50000);
	timer[4]->setPrescaler(256);         				// Available prescalers for T4: 1, 8, 64, 256, 1024.
	timer[4]->reset();

	//TIMER5 SETTINGS.
	timer[5] = new Timer16(t_alias::T5);
	timer[5]->initialize(t_mode::NORMAL, t_interrupt::NONE);
	timer[5]->setPrescaler(1024);			        	// Available prescalers for T5: 1, 8, 64, 256, 1024.
	timer[5]->reset();
}

	//MAIN PROGRAM.
void loop()
{
	Serial.print("T0::");
	Serial.print(timer[0]->getOverflows());

	Serial.print("\tT1::");
	Serial.print(timer[1]->getOverflows());

	Serial.print("\tT2::");
	Serial.print(timer[2]->getOverflows());

	Serial.print("\tT3::");
	Serial.print(timer[3]->getOverflows());

	Serial.print("\tT4::");
	Serial.print(timer[4]->getOverflows());

	Serial.print("\tT5::");
	Serial.println(timer[5]->getOverflows());
}
