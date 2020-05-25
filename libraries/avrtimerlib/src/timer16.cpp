/******************************************************************************
 * avr-timer-lib
 * Timer16.CPP
 *
 * This file contains functions for 16bit avr-timers.
 *
 * TODO::auto pin-map w/ alias function for different arduino's (UNO/MEGA/...).
 *
 * @author:	Rob Mertens
 * @version: 	1.1.1
 * @date: 	20/04/2017
 ******************************************************************************/

#include "timer16.hpp"

/**
 * @brief The project namespace.
 */
namespace avr
{

/** Constructors/destructor/overloading ***************************************/
Timer16::Timer16(void) : Timer()
{
	alias_ = t_alias::NONE;

	overflows_ = 0;
	capture_ = 0;
	captureN_ = 0;
	captureOvf_ = 0;
}

Timer16::Timer16(const t_alias alias) : Timer()
{
	setAlias(alias);

	overflows_ = 0;
	capture_ = 0;
	captureN_ = 0;
}

Timer16::Timer16(const volatile uint8_t * const& tccrxa,
	const volatile uint8_t * const& tccrxb,
	const volatile uint8_t * const& tcntxh,
	const volatile uint8_t * const& tcntxl,
	const volatile uint8_t * const& timskx,
	const volatile uint8_t * const& ocrxah,
	const volatile uint8_t * const& ocrxbh,
	const volatile uint8_t * const& ocrxal,
	const volatile uint8_t * const& ocrxbl,
	const volatile uint8_t * const& icrxl,
	const volatile uint8_t * const& icrxh) : Timer()
{
	tccrxa_ = tccrxa;						// Timer Count Control Register.
	tccrxb_ = tccrxb;						// Timer Count Control Register.
	tcntxl_ = tcntxl;						// Timer Count register.
	tcntxh_ = tcntxh;
	timskx_ = timskx;						// Timer Interrupt Mask register.
	ocrxal_ = ocrxal;
	ocrxah_ = ocrxah;
	ocrxbl_ = ocrxbl;
	ocrxbh_ = ocrxbh;
	icrxl_  = icrxl;
	icrxh_  = icrxh;

	alias_ = t_alias::TX;
	__T16__[20] = this;						// Instance of itself for ISR.

	overflows_ = 0;
	capture_ = 0;
	captureN_ = 0;
	captureOvf_ = 0;
}

Timer16::~Timer16(void)
{
	//Free the register pointers.
	delete tccrxa_;
	delete tccrxb_;
	delete tcntxl_;
	delete tcntxh_;
	delete timskx_;
	delete ocrxal_;
	delete ocrxah_;
	delete ocrxbl_;
	delete ocrxbh_;
	delete ocrxcl_;
	delete ocrxch_;
	delete icrxl_;
	delete icrxh_;
	
	//Delete this instance from static list.
	for(size_t n = 0; n < sizeof(__T16__) ; ++n)
	{
		//Do both pointers point to the same adress?
		//If so, delete pointer in the static list.
		if(__T16__[n]==this)delete __T16__[n];
	}
}

Timer16& Timer16::operator=(const Timer16& other)
{
	if(this != &other)
	{
		//Timer settings.
		alias_ = other.alias_;
		mode_ = other.mode_;
		interrupt_ = other.interrupt_;
		channel_ = other.channel_;
		inverted_ = other.inverted_;
		prescale_ = other.prescale_;
		
		//Timer runtime variables.
		overflows_ = other.overflows_;
		capture_ = other.capture_;
		captureN_ = other.captureN_;
		captureOvf_ = other.captureOvf_;
		
		//Free the register pointers.
		uint8_t *tccrxa(new volatile uint8_t(*other.tccrxa_));
		delete tccrxa_;
		tccrxa_ = tccrxa;
		uint8_t *tccrxb(new volatile uint8_t(*other.tccrxb_));
		delete tccrxb_;
		tccrxb_ = tccrxb;
		uint8_t *tcntxl(new volatile uint8_t(*other.tcntxl_));
		delete tcntxl_;
		tcntxl_ = tcntxl;
		uint8_t *tcntxh(new volatile uint8_t(*other.tcntxh_));
		delete tcntxh_;
		tcntxh_ = tcntxh;
		uint8_t *timskx(new volatile uint8_t(*other.timskx_));
		delete timskx_;
		timskx_ = timskx;
		uint8_t *ocrxal(new volatile uint8_t(*other.ocrxal_));
		delete ocrxal_;
		ocrxal_ = ocrxal;
		uint8_t *ocrxah(new volatile uint8_t(*other.ocrxah_));
		delete ocrxah_;
		ocrxah_ = ocrxah;
		uint8_t *ocrxbl(new volatile uint8_t(*other.ocrxbl_));
		delete ocrxbl_;
		ocrxbl_ = ocrxbl;
		uint8_t *ocrxbh(new volatile uint8_t(*other.ocrxbh_));
		delete ocrxbh_;
		ocrxbh_ = ocrxbh;
		uint8_t *ocrxcl(new volatile uint8_t(*other.ocrxcl_));
		delete ocrxcl_;
		ocrxcl_ = ocrxcl;
		uint8_t *ocrxch(new volatile uint8_t(*other.ocrxch_));
		delete ocrxch_;
		ocrxch_ = ocrxch;
		uint8_t *icrxl(new volatile uint8_t(*other.icrxl_));
		delete icrxl_;
		icrxl_ = icrxl;
		uint8_t *icrxh(new volatile uint8_t(*other.icrxh_));
		delete icrxh_;
		icrxh_ = icrxh;

	}
	return (*this);
}

/** Timer setting functions ***************************************************/
int8_t Timer16::setAlias(const t_alias alias)
{
	int8_t ret = 0;
	alias_ = t_alias::NONE;

	switch(alias)
	{
		case t_alias::T1 :
			setRegistersT1();
			break;

		case t_alias::T3 :
			setRegistersT3();
			break;

		case t_alias::T4 :
			setRegistersT4();
			break;

		case t_alias::T5 :
			setRegistersT5();
			break;

		case t_alias::NONE :
		case t_alias::T0 :
		case t_alias::T2 :
		case t_alias::TX :
		default :
			ret = -1;
			return ret;
	}

	alias_ = alias;

	return ret;
}

void Timer16::setRegistersT1(void)
{
	tccrxa_ = (volatile uint8_t *)&TCCR1A;
	tccrxb_ = (volatile uint8_t *)&TCCR1B;
	tcntxl_ = (volatile uint8_t *)&TCNT1L;
	tcntxh_ = (volatile uint8_t *)&TCNT1H;
	timskx_ = (volatile uint8_t *)&TIMSK1;
	ocrxal_ = (volatile uint8_t *)&OCR1AL;
	ocrxah_ = (volatile uint8_t *)&OCR1AH;
	ocrxbl_ = (volatile uint8_t *)&OCR1BL;
	ocrxbh_ = (volatile uint8_t *)&OCR1BH;
	ocrxcl_ = (volatile uint8_t *)&OCR1CL;
	ocrxch_ = (volatile uint8_t *)&OCR1CH;
	icrxl_  = (volatile uint8_t *)&ICR1L;
	icrxh_  = (volatile uint8_t *)&ICR1H;
}

void Timer16::setRegistersT3(void)
{
	tccrxa_ = (volatile uint8_t *)&TCCR3A;
	tccrxb_ = (volatile uint8_t *)&TCCR3B;
	tcntxl_ = (volatile uint8_t *)&TCNT3L;
	tcntxh_ = (volatile uint8_t *)&TCNT3H;
	timskx_ = (volatile uint8_t *)&TIMSK3;
	ocrxal_ = (volatile uint8_t *)&OCR3AL;
	ocrxah_ = (volatile uint8_t *)&OCR3AH;
	ocrxbl_ = (volatile uint8_t *)&OCR3BL;
	ocrxbh_ = (volatile uint8_t *)&OCR3BH;
	ocrxcl_ = (volatile uint8_t *)&OCR3CL;
	ocrxch_ = (volatile uint8_t *)&OCR3CH;
	icrxl_  = (volatile uint8_t *)&ICR3L;
	icrxh_  = (volatile uint8_t *)&ICR3H;
}

void Timer16::setRegistersT4(void)
{
	tccrxa_ = (volatile uint8_t *)&TCCR4A;
	tccrxb_ = (volatile uint8_t *)&TCCR4B;
	tcntxl_ = (volatile uint8_t *)&TCNT4L;
	tcntxh_ = (volatile uint8_t *)&TCNT4H;
	timskx_ = (volatile uint8_t *)&TIMSK4;
	ocrxal_ = (volatile uint8_t *)&OCR4AL;
	ocrxah_ = (volatile uint8_t *)&OCR4AH;
	ocrxbl_ = (volatile uint8_t *)&OCR4BL;
	ocrxbh_ = (volatile uint8_t *)&OCR4BH;
	ocrxcl_ = (volatile uint8_t *)&OCR4CL;
	ocrxch_ = (volatile uint8_t *)&OCR4CH;
	icrxl_  = (volatile uint8_t *)&ICR4L;
	icrxh_  = (volatile uint8_t *)&ICR4H;
}

void Timer16::setRegistersT5(void)
{
	tccrxa_ = (volatile uint8_t *)&TCCR5A;
	tccrxb_ = (volatile uint8_t *)&TCCR5B;
	tcntxl_ = (volatile uint8_t *)&TCNT5L;
	tcntxh_ = (volatile uint8_t *)&TCNT5H;
	timskx_ = (volatile uint8_t *)&TIMSK5;
	ocrxal_ = (volatile uint8_t *)&OCR5AL;
	ocrxah_ = (volatile uint8_t *)&OCR5AH;
	ocrxbl_ = (volatile uint8_t *)&OCR5BL;
	ocrxbh_ = (volatile uint8_t *)&OCR5BH;
	ocrxcl_ = (volatile uint8_t *)&OCR5CL;
	ocrxch_ = (volatile uint8_t *)&OCR5CH;
	icrxl_  = (volatile uint8_t *)&ICR5L;
	icrxh_  = (volatile uint8_t *)&ICR5H;
}

int8_t Timer16::initialize(const t_mode mode, const t_interrupt interrupt)
{
	int8_t ret = 0;

	//t_mode
	if(mode==t_mode::NORMAL 
	or mode==t_mode::CTC 
	or mode==t_mode::TIN
	or mode==t_mode::CAPTURE) {
		ret = setMode(mode);
	} else {
		ret=-1;
	} 
	
	if(ret==-1)return ret;

	//t_channel.
	ret = setPwmChannel(t_channel::NONE, t_inverted::NONE);
	if(ret==-1)return ret;

	//t_interrupt.
	ret = setInterruptMode(interrupt);
	if(ret==-1)return ret;

	return ret;
}

int8_t Timer16::initialize(const t_mode mode, const t_channel channel,
	const t_inverted inverted)
{
	int8_t ret = 0;

	//t_mode
	if(mode==t_mode::PWM_F or mode==t_mode::PWM_PC)ret = setMode(mode);
	else{ret=-1;}
	if(ret==-1)return ret;

	//t_interrupt.
	ret = setInterruptMode(t_interrupt::NONE);
	if(ret==-1)return ret;

	//t_channel.
	ret = setPwmChannel(channel, inverted);
	return ret;
}

int8_t Timer16::setMode(const t_mode mode)
{
	int8_t ret = 0;

	//Non-operational mode.
	*tccrxb_ &= 0xF8;
	mode_ = t_mode::NONE;

	//MODES.
	switch(mode)
	{
		case t_mode::CAPTURE :
			setMode2InputCapture();
			break;
		
		case t_mode::TIN :
			setMode2TimerInput();
			break;
		
		case t_mode::NORMAL :
			setMode2Normal();
			break;

		case t_mode::CTC :
			setMode2Ctc();
			break;

		case t_mode::PWM_F :
			setMode2FastPwm();
			break;

		case t_mode::PWM_PC :
			setMode2PhaseCorrectPwm();
			break;

		case t_mode::PWM_FC :
			setMode2FrequencyCorrectPwm();
			break;

		case t_mode::NONE :
			//Nothing.
			break;

		default :
			ret = -1;
			return ret;
	}

	return ret;
}

void Timer16::setMode2Normal(void)
{
	//Normal mode.
	// WGMx0  = 0;
	// WGMx1  = 0;
	*tccrxa_ = 0x00;
	// COMxB0 = 0;
	// COMxB1 = 0;
	// COMxA0 = 0;
	// COMxA1 = 0;
	// WGMx2  = 0;
	// WGMx3  = 0;
	*tccrxb_ = 0x00;

	//Set var.
	mode_ = t_mode::NORMAL;
}

void Timer16::setMode2TimerInput(void)
{
	//Normal mode.
	// WGMx0  = 0;
	// WGMx1  = 0;
	*tccrxa_ = 0x00;
	// WGMx1  = 0;
	// COMxB0 = 0;
	// COMxB1 = 0;
	// COMxA0 = 0;
	// COMxA1 = 0;
	// WGMx2  = 0;
	// WGMx3  = 0;
	*tccrxb_ = 0x00;

	//Set var.
	mode_ = t_mode::TIN;
}

void Timer16::setMode2InputCapture(void)
{
	// Normal mode.
	// WGMx0  = 0;
	// WGMx1  = 0;
	*tccrxa_ = 0x00;
	// COMxB0 = 0;
	// COMxB1 = 0;
	// COMxA0 = 0;
	// COMxA1 = 0;
	// WGMx2  = 0;
	// WGMx3  = 0;
	// ICNCx  = 1;
	// ICESx  = 0;
	*tccrxb_ = (1 << ICNC1);

	//Set var.
	mode_ = t_mode::CAPTURE;
}

void Timer16::setMode2Ctc(void)
{
	//CTC Mode.
	// WGMx0  = 0;
	// WGMx1  = 0;
	*tccrxa_ =  0;
	// COMxB0 = 0;
	// COMxB1 = 0;
	// COMxA0 = 0;
	// COMxA1 = 0; 
	// WGMx2  = 0;
	// CTCx   = 1;
	*tccrxb_ = (1 << WGM12);

	//Set var.
	mode_ = t_mode::CTC;
}

int8_t Timer16::setInterruptMode(const t_interrupt interrupt)
{
	int8_t ret = 0;

	//RESET VARS.
	interrupt_ = t_interrupt::NONE;
	
	//*__T16__[21] = {};
	//NOTE::This is bad...

	//CHECK ALIAS.
	if(alias_!=t_alias::T1 and alias_!=t_alias::T3 and alias_!=t_alias::T4
		and alias_!=t_alias::T5 and alias_!=t_alias::TX)
	{
		ret = -1;
		return ret;
	}

	//Delete this from static list.
	//This could be the case if the timer is initilized for the second time
	//during runtime.
	for(size_t n = 0; n < sizeof(__T16__); ++n)
	{
		//Do both pointers point to the same adress?
		//If so, delete pointer in the static list.
		if(__T16__[n]==this)delete __T16__[n];
	}

	//INTERRUPT MODE.
	switch(interrupt)
	{
		case t_interrupt::NONE :
			*timskx_ = 0x00;
			break;

		case t_interrupt::OVF :
			*timskx_ = 0x01;
			if(alias_==t_alias::T1)__T16__[0]=this;
			else if(alias_==t_alias::T3)__T16__[1]=this;
			else if(alias_==t_alias::T4)__T16__[2]=this;
			else{__T16__[3]=this;}
			break;

		case t_interrupt::COMPA :
			*timskx_ = 0x02;
			if(alias_==t_alias::T1)__T16__[4]=this;
			else if(alias_==t_alias::T3)__T16__[5]=this;
			else if(alias_==t_alias::T4)__T16__[6]=this;
			else{__T16__[7]=this;}
			break;

		case t_interrupt::COMPB :
			*timskx_ = 0x04;
			if(alias_==t_alias::T1)__T16__[8]=this;
			else if(alias_==t_alias::T3)__T16__[9]=this;
			else if(alias_==t_alias::T4)__T16__[10]=this;
			else{__T16__[11]=this;}
			break;

		case t_interrupt::COMPC :
			*timskx_ = 0x08;
			if(alias_==t_alias::T1)__T16__[12]=this;
			else if(alias_==t_alias::T3)__T16__[13]=this;
			else if(alias_==t_alias::T4)__T16__[14]=this;
			else{__T16__[15]=this;}
			break;

		case t_interrupt::CAPT :
			*timskx_ = 0x20 | 0x02;
			if(alias_==t_alias::T1) { __T16__[16]=this;__T16__[4]=this; }
			else if(alias_==t_alias::T3) { __T16__[17]=this;__T16__[5]=this ;}
			else if(alias_==t_alias::T4) { __T16__[18]=this;__T16__[6]=this;} 
			else{__T16__[19]=this;__T16__[7]=this;}
			break;

		default :
			ret = -1;
			return ret;
	}

	interrupt_ = interrupt;

	return ret;
}

void Timer16::setMode2FastPwm(void)
{
	//Fast PWM.
	// WGMx0  = 1;
	*tccrxa_ &= 0x0C;
	// WGMx1 = 1;
	*tccrxa_ |=  (1 << 0);
	// COMxB0 = 0;
	*tccrxa_ |=  (1 << 1);
	// COMxB1 = 0;
	// COMxA0 = 0;
	// COMxA1 = 0; TODO::check 0xF7
	*tccrxb_ &= 0xF7;
	// WGMx2  = 1;
	//*tccrxb_ |= (1 << 3);

	//Set var.
	mode_ = t_mode::PWM_F;
}

void Timer16::setMode2PhaseCorrectPwm(void)
{
	//Phase Correct PWM.
	// WGMx0  = 1;
	*tccrxa_ &= 0x0C;
	// WGMx1  = 0;
	*tccrxa_ |=  (1 << 0);
	// COMxB0 = 0;
	// COMxB1 = 0;
	// COMxA0 = 0;
	// COMxA1 = 0; TODO::check 0xF7
	*tccrxb_ &= 0xF7;
	// WGMx2  = 1;
	//*tccrxb_ |= (1 << 3);

	//Set var.
	mode_ = t_mode::PWM_PC;
}

void Timer16::setMode2FrequencyCorrectPwm(void)
{
	//Phase Correct PWM.
	*tccrxa_ &= 0x0C;					// WGMx0  = 1;
	//*tccrxa_ |= (1 << 0);			// WGMx1  = 0;
														// COMxB0 = 0;
														// COMxB1 = 0;
														// COMxA0 = 0;
	*tccrxb_ &= 0xF7;					// COMxA1 = 0; TODO::check 0xF7
	//*tccrxb_ |= (1 << 3); 		// WGMx2  = 1;

	//Set var.
	mode_ = t_mode::PWM_FC;
}

int8_t Timer16::setPwmChannel(const t_channel channel,
	const t_inverted inverted)
{
	int8_t ret = 0;

	//RESET VARS.
	channel_ = t_channel::NONE;
	inverted_ = t_inverted::NONE;
	*tccrxa_ &= 0x0F;
	*tccrxb_ &= 0xFB;

	//PWM channel.
	switch(channel)
	{
		case t_channel::NONE :
			//Nothing.
			break;

		case t_channel::A :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x80;
			else{*tccrxa_ |= 0xC0;}
			break;

		case t_channel::B :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x20;
			else{*tccrxa_ |= 0x30;}
			break;

		case t_channel::B_TOP : //TODO::check tccrxa
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		case t_channel::C :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x08;
			else{*tccrxa_ |= 0x0C;}
			break;

		case t_channel::C_TOP : //TODO::check tccrxa
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		case t_channel::AB :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		case t_channel::AC :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x88;
			else{*tccrxa_ |= 0xCC;}
			break;

		case t_channel::BC :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x28;
			else{*tccrxa_ |= 0x3C;}
			break;

		case t_channel::BC_TOP : //TODO::check tccrxa
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x28;
			else{*tccrxa_ |= 0x3C;}
			break;

		case t_channel::ABC :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA8;
			else{*tccrxa_ |= 0xFC;}
			break;

		default :
			ret = -1;
			return ret;
	}

	channel_ = channel;
	inverted_ = inverted;

	return ret;
}

int8_t Timer16::setPrescaler(const uint16_t prescale)
{
	int8_t ret = 0;

	prescale_ = 0;
	*tccrxb_ &= 0xF8;

	if (mode_==t_mode::TIN) {
		if (prescale==1) {
			*tccrxb_ |= 0x07; 	// Rising Edge Tn pin
		} else if(prescale==0xFFFF) { // '-1'
			*tccrxb_ |= 0x06;	// Falling Edge Tn pin
		}
	} else {
		switch(prescale)
		{
			case 0xFFFF:
				if (mode_==t_mode::CTC) {
					*tccrxb_ |= 0x07; 	// Rising Edge Tn pin
				}
				break;

			case 0 :
				if (mode_==t_mode::CTC) {
					*tccrxb_ |= 0x06;	// Falling Edge Tn pin
				}
				//Timer inactive.
				//Nothing.
				break;

			case 1 :
				*tccrxb_ |= 0x01;
				break;

			case 8 :
				*tccrxb_ |= 0x02;
				break;

			case 64 :
				*tccrxb_ |= 0x03;
				break;

			case 256 :
				*tccrxb_ |= 0x04;
				break;

			case 1024 :
				*tccrxb_ |= 0x05;
				break;

			default :
				ret = -1;
				return ret;
		}
	}
	
	prescale_ = prescale;

	return ret;

}

void Timer16::setCompareValueA(const size_t compare)
{
	*ocrxah_ = (uint8_t)((static_cast<uint16_t>(compare) >> 8) & 0xFF);
	*ocrxal_ = (uint8_t)(static_cast<uint16_t>(compare) & 0xFF);
}

void Timer16::setCompareValueB(const size_t compare)
{
	*ocrxbh_ = (uint8_t)((static_cast<uint16_t>(compare) >> 8) & 0xFF);
	*ocrxbl_ = (uint8_t)(static_cast<uint16_t>(compare) & 0xFF);
}

void Timer16::setCompareValueC(const size_t compare)
{
	*ocrxch_ = (uint8_t)((static_cast<uint16_t>(compare) >> 8) & 0xFF);
	*ocrxcl_ = (uint8_t)(static_cast<uint16_t>(compare) & 0xFF);
}

int8_t Timer16::setDutyCycleA(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC or mode_==t_mode::PWM_FC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;

		compare = uint8_t(dutyCycle*0xFFFF-1);

		setCompareValueA(compare);
	}
	else
	{
		ret = -1;
	}

	return ret;
}

int8_t Timer16::setDutyCycleB(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC or mode_==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;

		if(*ocrxal_==0x00)compare = uint8_t(dutyCycle*0xFFFF-1);
		else{compare = uint8_t(dutyCycle*((*ocrxah_ << 8) | (*ocrxal_ & 0xFF))-1);}

		setCompareValueB(compare);
	}
	else
	{
		ret = -1;
	}

	return ret;
}

int8_t Timer16::setDutyCycleC(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC or mode_==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;

		if(*ocrxal_==0x00)compare = uint8_t(dutyCycle*0xFFFF-1);
		else{compare = uint8_t(dutyCycle*(((*ocrxah_ & 0xFF )<< 8) | (*ocrxal_ & 0xFF))-1);}

		setCompareValueC(compare);
	}
	else
	{
		ret = -1;
	}

	return ret;
}

void Timer16::set(const size_t value)
{
	*tcntxh_ = (uint8_t)((static_cast<uint16_t>(value) >> 8) & 0xFF);
	*tcntxl_ = (uint8_t)(static_cast<uint16_t>(value) & 0xFF);
}

void Timer16::reset(void)
{
	set(0x0000);
	overflows_ = 0x00000000;
	capture_ = 0;
	captureN_ = 0;
	captureOvf_ = 0;
}

void Timer16::hardReset(void)
{
	if(interrupt_!=t_interrupt::NONE)setInterruptMode(t_interrupt::NONE);
	if(channel_!=t_channel::NONE)setPwmChannel(t_channel::NONE, t_inverted::NONE);
	setMode(t_mode::NORMAL);
	reset();

	//Delete this instance from the list.
	for(size_t n = 0; n < sizeof(__T16__); ++n)
	{
		if(__T16__[n]==this)delete __T16__[n];
	}
}

size_t Timer16::getCount(void)
{
	uint16_t count;
	count  = *tcntxl_ & 0xFF;
	count |= ((uint16_t)*tcntxh_ & 0xFF) << 8;
	return static_cast<size_t>(count);
}

uint32_t Timer16::getNonResetCount(void)
{
	uint32_t nonResetCount = 0x00000000;

	switch (mode_) {
		case  t_mode::NORMAL:
		case  t_mode::TIN:
		case  t_mode::CAPTURE:
			nonResetCount  = *tcntxl_ & 0xFF;
			nonResetCount |= ((uint16_t)*tcntxh_ & 0xFF) << 8;
			nonResetCount |= (overflows_ << 16);
			break;

		case t_mode::CTC:
		{
			uint32_t overflowPre = overflows_;
			nonResetCount = *tcntxl_;
			nonResetCount += ((uint16_t)*tcntxh_ & 0xFF) << 8;
			nonResetCount *= 8000UL;
			nonResetCount >>= 16;
			// Catch any case where an interrupt casued a change in overflow
			// around the same time as the count 
			if (nonResetCount >= 4000UL) { 
				nonResetCount += overflowPre * 8000UL;
			} else {
				nonResetCount += overflows_ * 8000UL;
			}
			break;
		}
	}
	
	return nonResetCount;
}

/**
 * @brief Gets the last capture value
 */
uint16_t Timer16::getCaptureCount(void)
{
	return capture_;
}

uint16_t Timer16::getCaptureDelta(void)
{
	return capture_ - captureN_;
}

/**
 * @brief Latch the input capture value to local var
 */
void Timer16::latchCaptureValue(void) 
{
	captureN_ 	= capture_;
	captureOvf_ = 0;
	capture_  	= *icrxl_ & 0xFF;
	capture_   |= ((uint16_t)*icrxh_ & 0xFF) << 8;
	*ocrxah_ = ((capture_-1) >> 8 ) & 0xFF;
	*ocrxal_ = ((capture_-1) & 0xFF);
}

void Timer16::getLastCapture(tmr16Capture_t * pCapture) {
	pCapture->count  	= *tcntxl_ & 0xFF;
	pCapture->count    |= ((uint16_t)*tcntxh_ & 0xFF) << 8;
	pCapture->period 	= capture_ - captureN_;
	pCapture->capture 	= capture_;
	pCapture->overflow  = captureOvf_;
}

void Timer16::overflowCaptureValue(void)
{
	if (captureN_ != capture_) {
		captureOvf_++;
	}
}

/** Interrupt functionality overrides *****************************************/
//NOTE::moved to ABC.
//void Timer8::interruptServiceRoutine(void) { overflows_++; }

//void Timer16::enable(void) {}

//void Timer16::disable(void) {}

//void Timer16::clear(void) {}

/** Interrupt preprocessor functions ******************************************/
/**
 * @brief The private static list of timers. Each timer interrupt vector has
 * 				it's own instance in the static list. If the corresponding
 *				interrupt vector is set, the instance is added to this list.
 *
 * We initiate the private function in de .source file which does has acces to
 * the static private member list.
 *
 * TODO::different avrs.
 */
Timer16 * Timer16::__T16__[21] = {};

/**
 * @brief This preporcessor method creates an ISR mapping function for each
 *				available timer interrupt vector. This is now done for the arduino
 *				MEGA2560 but in the future UNO will be supported.
 *
 * TODO::different avrs.
 */
#define TIMER_ISR(t, vect, n)													\
ISR(TIMER ## t ## _ ## vect ## _vect)											\
{																				\
	if(Timer16::__T16__[n])Timer16::__T16__[n] -> interruptServiceRoutine();	\
}

#define TIMER_ISR_CAPTURE(t, vect, n)											\
ISR(TIMER ## t ## _ ## vect ## _vect)											\
{																				\
	if(Timer16::__T16__[n])Timer16::__T16__[n] -> latchCaptureValue();			\
}

#define TIMER_ISR_CAPTURE_OVF(t, vect, n)										\
ISR(TIMER ## t ## _ ## vect ## _vect)											\
{																				\
	if(Timer16::__T16__[n])Timer16::__T16__[n] -> overflowCaptureValue();		\
}

#if defined(TIMER1_OVF_vect)
TIMER_ISR(1, OVF, 0)
#endif
#if defined(TIMER3_OVF_vect)
TIMER_ISR(3, OVF, 1)
#endif
#if defined(TIMER4_OVF_vect)
TIMER_ISR(4, OVF, 2)
#endif
#if defined(TIMER5_OVF_vect)
TIMER_ISR(5, OVF, 3)
#endif

#if defined(TIMER1_COMPA_vect)
TIMER_ISR_CAPTURE_OVF(1, COMPA, 4)
#endif
#if defined(TIMER3_COMPA_vect)
TIMER_ISR_CAPTURE_OVF(3, COMPA, 5)
#endif
#if defined(TIMER4_COMPA_vect)
TIMER_ISR_CAPTURE_OVF(4, COMPA, 6)
#endif
#if defined(TIMER5_COMPA_vect)
TIMER_ISR_CAPTURE_OVF(5, COMPA, 7)
#endif

#if defined(TIMER1_COMPB_vect)
TIMER_ISR(1, COMPB, 8)
#endif
#if defined(TIMER3_COMPB_vect)
TIMER_ISR(3, COMPB, 9)
#endif
#if defined(TIMER4_COMPB_vect)
TIMER_ISR(4, COMPB, 10)
#endif
#if defined(TIMER5_COMPB_vect)
TIMER_ISR(5, COMPB, 11)
#endif

#if defined(TIMER1_COMPC_vect)
TIMER_ISR(1, COMPC, 12)
#endif
#if defined(TIMER3_COMPC_vect)
TIMER_ISR(3, COMPC, 13)
#endif
#if defined(TIMER4_COMPC_vect)
TIMER_ISR(4, COMPC, 14)
#endif
#if defined(TIMER5_COMPC_vect)
TIMER_ISR(5, COMPC, 15)
#endif

#if defined(TIMER1_CAPT_vect)
TIMER_ISR_CAPTURE(1, CAPT, 16)
#endif
#if defined(TIMER3_CAPT_vect)
TIMER_ISR_CAPTURE(3, CAPT, 17)
#endif
#if defined(TIMER4_CAPT_vect)
TIMER_ISR_CAPTURE(4, CAPT, 18)
#endif
#if defined(TIMER5_CAPT_vect)
TIMER_ISR_CAPTURE(5, CAPT, 19)
#endif


}; //End AVR namespace.
