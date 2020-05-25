/**
 * avr-timer-lib
 * interrupt.hpp
 *
 * This file contains basic functions for avr interrupts.
 * These functions should be overriden in classes which inherit from it.
 * Different functions describe the different actions for specific interrupts.
 *
 * TODO::make list with all possible avr interrupts for MEGA board.
 *
 * @author	Rob Mertens
 * @date		06/11/2017
 * @version 1.1.1
 */

#ifndef avr_INTERRUPT_HPP
#define avr_INTERRUPT_HPP

namespace avr
{

/**
 * @brief Interrupt interface class.
 */
class Interrupt
{

	public:

		/**
		 * @brief Interrupt handler class.
		 */
		class Handler
		{

			public:

				/** Basic interrupt functionalities ***********************************/
				/**
				 * @brief Interrupt service routine.
				 */
				virtual void interruptServiceRoutine(void) = 0;

				virtual void latchCaptureValue(void) = 0;
				
				/**
				 * @brief Enable interrupts.
				 */
				virtual void enable() = 0;

				/**
				 * @brief Disable interrupts.
				 */
				virtual void disable() = 0;

				/**
				 * @brief Clear all interrupt vectors.
				 */
				virtual void clear() = 0;

		}; //End Handler class.

}; //End Interrupt class.

}; //End namespace avr.

#endif //End avr_INTERRUPT_HPP wrapper.
