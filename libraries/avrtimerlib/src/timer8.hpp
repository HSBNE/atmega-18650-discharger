/**
 * file vmath.hpp
 *
 * @brief This file belongs to the qc_library project. It provides functions
 *        for Vector and rotation math.
 *
 * @author 	Rob Mertens
 * @date		11/11/2017
 */

#ifndef avr_TIMER8_H
#define avr_TIMER8_H

//Include standard headers.
#include <stdint.h>
#include <stddef.h>

//Include avr headers.
#include <avr/interrupt.h>

//Include library headers.
#include "interrupt.hpp"
#include "timer.hpp"

/**
 * @brief The project namespace.
 */
namespace avr
{
#if not defined(ARDUINO)
/**
 * @brief Extern C timer0 overflow vector.
 */
extern "C" void TIMER0_OVF_vect(void) __attribute__((signal));
#endif
/**
 * @brief Extern C timer2 overflow vector.
 */
extern "C" void TIMER2_OVF_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer0 compare A vector.
 */
extern "C" void TIMER0_COMPA_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer2 compare A vector.
 */
extern "C" void TIMER2_COMPA_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer0 compare B vector.
 */
extern "C" void TIMER0_COMPB_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer2 compare B vector.
 */
extern "C" void TIMER2_COMPB_vect(void) __attribute__((signal));

/**
 * @brief Timer8 class definition. This class inherits from the abstract Timer
 *				class for polymorphism.
 */
class Timer8 : public Timer
{

	public:

		/** Typedefs **************************************************************/
		/**
		 * @brief A pointer typedef for Timer8 instance.
		 *				TODO::use smart pointers.
		 */
		typedef Timer8 * Ptr;

		/**
		 * @brief A constant pointer typedef for Timer8 instance.
		 *				TODO::use smart pointers.
		 */
		typedef Timer8 * const CPtr;

		/** Constructors/destructor/overloading ***********************************/
		/**
		 * @brief
		 */
		Timer8(void);

		/**
		 * @brief
		 * @param
		 */
		explicit Timer8(const t_alias);

		/**
		 * @brief
		 * @param
		 */
		Timer8(const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&);

		/**
		 * @brief Explicitly forbid a copy-constructor. We don't want two instances
		 *				interfacing the same hardware addresses.
		 *
		 * The "= delete" argument since c++11. Otherwise set this funtion private.
		 */
		Timer8(const Timer8&) = delete;

		/**
		 * @brief
		 */
		virtual ~Timer8(void);

		/**
		 * @brief NOTE::removed private.
		 */
		//Timer8& operator=(const Timer8&);

		/** Timer setting functions ***********************************************/
		/**
		 * @brief
		 */
		int8_t setAlias(const t_alias) override;

		/**
		 * @brief Method for setting the timer mode 2 NORMAL or CTC. This method
		 *				cannot set mode 2 PWM.
		 * @param mode The timer operation mode, only NORMAL or CTC.
		 * @param interrupt The interrupt mode, default is NONE.
		 * @return
		 */
		int8_t initialize(const t_mode, const t_interrupt) override;

		/**
		 * @brief Method for setting the timer mode 2 PWM. This method cannot set
		 * 				mode 2 NORMAL or CTC.
		 * @param mode The timer operation mode, only PWM.
		 * @param interrupt The interrupt mode, default is NONE.
		 * @param inverted Is the PWM-channel inverted?
		 * @return
		 */
		int8_t initialize(const t_mode, const t_channel,
			const t_inverted) override;

		/**
		 * @brief Method for setting the timer prescaler value.
		 *
		 *    VALUE | TIMER 0 | TIMER 2
		 *     	  1 |    x    |   x
		 * 	  		8 |    x    |   x
		 *       32 |	      	|   x
		 *       64 |    x    |   x
		 *      256 |    x    |
		 *     1024 |    x    |
		 *
		 * TODO::don't allow prescaler with wrong timer.
		 *
		 * @param The prescaler value.
		 * @return
		 */
		int8_t setPrescaler(const uint16_t) override;

		/** Timer runtime functions ***********************************************/
		/**
		 * @brief
		 */
		void set(const size_t) override;

		/**
		 * @brief
		 */
		void reset(void) override;

		/**
		 * @brief
		 */
		void hardReset(void) override;

		/**
		 * @brief Method for setting the OCRxA register.
		 * @param compare The compare value (unsigned byte).
		 */
		void setCompareValueA(const size_t) override;

		/**
		 * @brief
		 * @param[out]
		 */
		int8_t setDutyCycleA(double) override;

		/**
		 * @brief Method for setting the OCRxB register.
		 * @param compare The compare value (unsigned byte).
		 */
		void setCompareValueB(const size_t) override;

		/**
		 * @brief
		 * @param[out]
		 */
		int8_t setDutyCycleB(double) override;

		/**
		 * @brief
		 */
		size_t getCount(void) override;

		/**
		 * @brief NOTE::moved to ABC.
		 */
		//uint32_t getOverflows(void);

		/**
		 * @brief Method for obtaining the total summized count since the last
		 *				reset. Thus overflows are accounted.
		 *
		 * NOTE: This method only works if the timer interrupts with an timer_ovf_vect()
		 *
		 * @return The count value since last reset.
		 */
		uint32_t getNonResetCount(void) override;

		void latchCaptureValue(void) override { return; };

		void overflowCaptureValue(void) override { return; };
		
		uint16_t getCaptureCount(void) override { return 0; };
		
		uint16_t getCaptureDelta(void) override { return 0; };
		
		/** Interrupt functionality overrides *************************************/
		/**
		 * @brief NOTE::moved to ABC.
		 */
		//void enable(void) override;

		/**
		 * @brief NOTE::moved to ABC.
		 */
		//void disable(void) override;

		/**
		 * @brief NOTE::moved to ABC.
		 */
		//void clear(void) override;

	private:

		/** Constructors/destructor/overloading ***********************************/
		/**
		 * @brief The equals operator is used locally. We don't want to make two
		 * 				instances at global scale interfacing with the same hardware
		 *				registers. However, this function is used locally for the ISR
		 *				mapping.
		 * @param The Timer8 instance to copy.
		 * @return The copied Timer8 instance.
		 */
		Timer8& operator=(const Timer8&);

		/** Interrupt functionality overrides *************************************/
		/**
		 * @brief NOTE::moved to ABC.
		 */
		//void interruptServiceRoutine(void) override;

		/** Registers *************************************************************/
		/**
		 * @brief
		 */
		volatile uint8_t * tcntx_;			// TIMER COUNT

		/**
		 * @brief
		 */
		volatile uint8_t * tccrxa_;			// PRESCALER

		/**
		 * @brief
		 */
		volatile uint8_t * tccrxb_;			// PRESCALER

		/**
		 * @brief
		 */
		volatile uint8_t * timskx_;			// Timer Interrupt Mask register.

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxa_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxb_;

		/**
		 * @brief Sets the register pointers to the appropriate values for timer0.
		 */
		void setRegistersT0(void);

		/**
		 * @brief Sets the register pointers to the appropriate values for timer2.
		 */
		void setRegistersT2(void);

		/** Timer mode functionalities ********************************************/
		/**
		 * @brief Sets the timer mode. It uses the private mode functions below.
		 * @param The desired timer mode.
		 * @return Is the operation successful ? [0] YES : [-1] NO.
		 */
		int8_t setMode(const t_mode) override;

		/**
		 * @brief Sets the timer mode to normal.
		 *				NOTE::this function sets only two bits, consider removing?
		 */
		void setMode2Normal(void);

		/**
		 * @brief Sets the timer mode to Clear Timer on Compare (CTC).
		 *				NOTE::this function sets only two bits, consider removing?
		 */
		void setMode2Ctc(void);

		/**
		 * @brief Sets the timer mode to Fast PWM.
		 *				NOTE::this function sets only two bits, consider removing?
		 */
		void setMode2FastPwm(void);

		/**
		 * @brief Sets the timer mode to Phase Correct PWM.
		 *				NOTE::this function sets only two bits, consider removing?
		 */
		void setMode2PhaseCorrectPwm(void);

		/**
		 * @brief Sets the timer interrupt mode. This is only possible if the timer
		 * 				is set to the corresponding modes {NORMAL, CTC}.
		 * @param The desired timer interrupt mode.
		 * @return Is the operation successful ? [0] YES : [-1] NO.
		 */
		int8_t setInterruptMode(const t_interrupt) override;

		/**
		 * @brief Sets the timer PWM-channel. This is only possible if the timer is
		 * 				set to the corresponding modes {PWM_F, PWM_PC}.
		 * @param The desired timer PWM-channel.
		 * @return Is the operation successful ? [0] YES : [-1] NO.
		 */
		int8_t setPwmChannel(const t_channel, const t_inverted) override;

		/** Timer runtime functions ***********************************************/
		/**
		 * @brief This is set to private since this timer has no channel C.
		 *
		 * HACK::this method is only used in one derived class...
		 */
		void setCompareValueC(const size_t) override {}

		/**
		 * @brief This is set to private since this timer has no channel C.
		 *
		 * HACK::this method is only used in one derived class...
		 */
		int8_t setDutyCycleC(double) override { return 0; }

		/** Interrupt functions ***************************************************/
		/**
		 * @brief Static array of Timer8 instances. For each interrupt vector
	 	 * 				memory is allocated. This list is defined at global scope to Links
		 *				it with the appropriate ISR.
		 */
		static Timer8::Ptr __T8__[7];
#if not defined(ARDUINO)
		/**
		 * @brief Friend timer0 overflow vector. This vector is standard for each
		 * 				arduino and thus standard friend.
		 */
		friend void TIMER0_OVF_vect(void);
#endif
		/**
		 * @brief Friend timer2 overflow vector.
		 */
		friend void TIMER2_OVF_vect(void);

		/**
		 * @brief Friend timer0 compare A vector.
		 */
		friend void TIMER0_COMPA_vect(void);

		/**
		 * @brief Friend timer2 compare A vector.
		 */
		friend void TIMER2_COMPA_vect(void);

		/**
		 * @brief Friend timer0 compare B vector.
		 */
		friend void TIMER0_COMPB_vect(void);

		/**
		 * @brief Friend timer2 compare B vector.
		 */
		friend void TIMER2_COMPB_vect(void);

}; //End Timer8 class.

}; //End AVR namespace.

#endif
