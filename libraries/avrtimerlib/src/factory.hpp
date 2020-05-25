/**
 * avr-timer-lib
 * file factory.hpp
 *
 * @brief This file belongs to the avr_timer_lib project.
 *
 * @author 	Rob Mertens
 * @date		27/10/2017
 */

#ifndef avr_TIMER_FACTORY_HPP
#define avr_TIMER_FACTORY_HPP

//Include standard headers.
#include <stdint.h>

//Include library headers.
#include "timer.hpp"
#include "timer8.hpp"
#include "timer16.hpp"

/**
 * @brief The project namespace.
 */
namespace avr
{

	/**
	 * @brief A Timer factory function.
	 *
	 * This is also implemented as a class below which follows a tidy design
	 * scheme. However, I don't like the implementation class since it only maps
	 * one function and this is obvious for inexperienced arduino users.
	 *
	 * For now I leave this function marked deprecated.
	 *
	 * @param
	 * @param[out]
	 * @return
	 */
	/*Timer::Ptr create(const t_alias alias) //__attribute__((deprecated))
	{
		switch(alias)
		{
			case t_alias::T0 :
				return new Timer8(alias);
				break;
			case t_alias::T1 :
				return new Timer16(alias);
				break;
			case t_alias::T2 :
				return new Timer8(alias);
				break;
			case t_alias::T3 :
				return new Timer16(alias);
				break;
			case t_alias::T4 :
				return new Timer16(alias);
				break;
			case t_alias::T5 :
				return new Timer16(alias);
				break;
			case t_alias::NONE :
			case t_alias::TX :
			default :
				//TODO::return NULL pointer.
				break;
		}
	};*/

/**
 * @brief TimerFactory Class. This class is the replacement for the factory
 *				function below. This class serves as a singleton since one factory
 *				suffices to create all necessary timers.
 */
class TimerFactory
{

	public:

		/**
		 * @brief
		 */
    virtual ~TimerFactory() {}

		/**
		 * @brief TimerFactory is a singleton class.
		 * @return The static timer factory singleton instance.
		 */
		static TimerFactory *startBelt(void) { return &factory_; }

		/**
		 * @brief A Timer factory function.
		 *
		 * @param
		 * @param[out]
		 * @return
		 */
		Timer::Ptr produce(const t_alias alias)
		{
			switch(alias)
			{
				case t_alias::T0 :
					return new Timer8(alias);
					break;
				case t_alias::T1 :
					return new Timer16(alias);
					break;
				case t_alias::T2 :
					return new Timer8(alias);
					break;
				case t_alias::T3 :
					return new Timer16(alias);
					break;
				case t_alias::T4 :
					return new Timer16(alias);
					break;
				case t_alias::T5 :
					return new Timer16(alias);
					break;
				case t_alias::NONE :
				case t_alias::TX :
				default :
					//TODO::return NULL pointer.
					break;
			}
		}

	private:

			/**
			 * @brief
			 */
	    TimerFactory(void);

			/**
			 * @brief
			 */
	    TimerFactory(const TimerFactory&) {}

			/**
			 * @brief
			 */
			TimerFactory &operator=(const TimerFactory&) { return (*this); }

			/**
			 * @brief The static factory instance.
			 */
			static TimerFactory factory_;

};

}; //End avr namespace.

#endif //End avr_TIMER_FACTORY_HPP wrapper.
