/*******************************************************************************
 * avr-timer-lib
 * timer8_doc.ino
 *
 * This file serves as documentation for an avr 8-bit timer. It gives all the
 * basic functionalities for
 *
 * @author:		Rob Mertens
 * @date:			18/04/2017
 * @version: 	1.1.1
 ******************************************************************************/

//Include library headers.
#include "timer.hpp"
#include "timer8.hpp"
#include "factory.hpp"

using namespace avr;

/**
 * [1] Creating timer8 instances.
 *
 * We can do this by constructor, by using timer pointers or by using the
 * factory design pattern.
 */
Timer8 t0(t_alias::T0);
Timer::Ptr t0_ptr = &t0;
Timer::Ptr t2_ptr = TimerFactory::startBelt()->produce(t_alias::T2);

//INIT.
void setup()
{
	//SERIAL.
	Serial.begin(115200);

	/**
	 * [2] Setup the timer.
	 *
	 * Available timer modes for avr timers.
	 * NOTE: for the exact working principles of these modes see manual.
	 * 1. No operation mode 												{NONE},
	 * 2. Normal up counter mode 										{NORMAL},
	 * 3. Clear Timer on Compare										{CTC},
	 * 4. Fast Phase Width Modulation 							{PWM_F},
	 * 5. Phase Correct Phase Width Modulation 			{PWM_PC},
	 * 6. Frequency Correct Phase Width Modulation 	{PWM_FC}
	 */
	t0_ptr->initialize(t_mode::CTC, t_interrupt::COMPA);
	t0_ptr->setCompareValueA(200);
	t0_ptr->setPrescaler(1);
	t0_ptr->reset();

}

/**
 * [3] Runtime functions.
 *
 * Available runtime funtions,
 * set()
 * reset()
 * hardReset()
 * getCount()
 * getNonResetCount()
 *
 */
void loop()
{
	//Set the count value.
	t0_ptr->set(0x00);

	Serial.print("T0::");
	Serial.println(t0_ptr->getOverflows());
}
