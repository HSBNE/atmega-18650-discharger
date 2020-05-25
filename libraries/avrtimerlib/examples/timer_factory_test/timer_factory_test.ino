/*******************************************************************************
 * avr-timer-lib
 * timer_factory_test.ino
 *
 * Test file for timer interrupts.
 * I am using an arduino MEGA ADK (AVR atmega2560).
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

Timer::Ptr t8_ptr, t16_ptr;

void setup()
{
	//SERIAL.
	Serial.begin(115200);

	//Create the timers.
	t8_ptr = TimerFactory::startBelt()->produce(t_alias::T0);
	t8_ptr->initialize(t_mode::PWM_F, t_channel::B_TOP, t_inverted::NORMAL);
	t8_ptr->setPrescaler(0);
	t8_ptr->reset();

	t16_ptr = TimerFactory::startBelt()->produce(t_alias::T1);
	t16_ptr->initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);
	t16_ptr->setPrescaler(0);
	t16_ptr->reset();
}

	//MAIN PROGRAM.
void loop()
{

}
