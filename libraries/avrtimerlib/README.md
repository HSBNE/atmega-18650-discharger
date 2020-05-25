## avr-timer-lib

Timer library for Arduino Mega (atmega2560). The available timers on the atmega2560 are shown in the following table. The extern timer is not really a atmega2560 timer.

| Timer     		| Code specifier 		| Bitness	|
| ------------- | ----------------- | ------- |
| Timer 0    		| `t_alias::T0`			| 8				|
| Timer 1				| `t_alias::T1` 		| 16			|
| Timer 2				| `t_alias::T2` 		| 8				|
| Timer 3				| `t_alias::T3` 		| 16			|
| Timer 4				| `t_alias::T4` 		| 16			|
| Timer 5				| `t_alias::T5` 		| 16			|
| Timer extern	| `t_alias::TX` 		| 8/16		|

### Easy to use

A hardware timer (8/16-bit) can be set in a few lines of code as shown in the
example below:

```c++
// Create the timer instance.
// We use the Arduino MEGA hardware Timer0.
Timer8 t0(t_alias::T0);

// Enter the timer settings.
// We set the timer to count to Normal TOP value (0xFF).
// And set an interrupt when TOP is reached.
t0.initialize(t_mode::NORMAL, t_interrupt::OVF0);

// The timer prescaler value determines the actual time it takes to up the timer
// count value. More on prescalers is explained in detail below.
t0.setPrescaler(1);

// Reset the timer and start counting.
t0.reset();
```

Pointer to Timer instances are supported as shown in the following example. A factory pattern is added to create timers without even specifying the timer bitness.

```c++
// Create a base class timer pointer to the 8-bit timer from the code block above.
Timer::Ptr t0_ptr = &t0;

// Create a const base class timer pointer to an 16-bit timer.
// This is a const pointer and NOT pointer to const.
Timer::CPtr t1_ptr(new Timer16(t_alias::T1));

// Instead of determining bitness a Timer Factory is implemented.
// Create an array for all the hardware timers of the arduino MEGA.
Timer::Ptr timer[6];
timer[0] = t0_ptr;
timer[1] = t1_ptr;
timer[2] = TimerFactory::startBelt()->produce(t_alias::T2);
timer[3] = TimerFactory::startBelt()->produce(t_alias::T3);
timer[4] = TimerFactory::startBelt()->produce(t_alias::T4);
timer[5] = TimerFactory::startBelt()->produce(t_alias::T5);
```

### Some theory

Ok, some timer theory first. To do some useful stuff with AVR-timers we have to understand how these things work under the hood of our Arduino. Let's start with basics of the basics, namely the concepts of *ticks* and *time*. The Arduino MEGA runs on a clock frequency of 16 MHz or 16.000.000 Hz. Each clock cycle the timer ups the count value, this is referred to as a tick. The time for this tick depends on the clock frequency and is equal to 0.0625e-8[!] us which is *very very* fast. An 8-bit timer uses one byte of information. The number of ticks to reach the maximum count value equals  255 (2^8 - 1). This means that the timer reaches its TOP count value in 15.94 us. For an 16-bit timer the number of ticks is equal to 65355 (2^16 - 1). Speaking in terms of time, this will take approximately 4084.69 us. Obviously we don't want our timer to count to either 15.94 us or 4084.69 us all the time. Luckely for us, we can play a little bit with these values. The engineers of AVR provided a parameter called the *prescaler* to modify the time for each tick. The formula to calculate the timer for each tick is given as follows:

//TODO::formula .gif

The possibilities for the prescaler values is different for each timer. Next table gives an overview for all the available timers on the Atmega2560.

| Prescaler | Timer 0 | Timer 1 | Timer 2 | Timer 3 | Timer 4 | Timer 5 |
| --------- | -------:| -------:| -------:| -------:| -------:| -------:|
| 1    			| X			 :| X				| X				| X				| X				| X				|
| 8    			| X			 :| X				| X				| X				| X				| X				|
| 32				| 				| 				| X				| 				| 				| 				|
| 64				| X				| X				| X				| X				| X				| X				|
| 256				| X				| X				| 				| X				| X				| X				|
| 1024			| X				| X				| 				| X				| X				| X				|


### Timer interrupts

AVR timers have two distict operation modes. Firstly the **interrupt** mode. In this mode the timer count to a specific value, either TOP or a manually set value. When the timer reaches this value an interrupt is fired. These timer modes are shown in the table below.

| Interrupt modes        													| Code specifier 		| Bitness	|
| ----------------------------------------------- | ----------------- | ------- |
| None      																			| `t_mode::NONE`		| 8/16		|
| Normal mode counts to TOP 											| `t_mode::NORMAL` 	| 8/16		|
| Clear Timer on Compare (CTC) 										| `t_mode::CTC` 		| 8/16		|

The next example explains how to set a timer to an interrupt mode.

```c++
// We use the timer2 instance from the code block above.
// The NORMAL specifier indicates that the timer counts to TOP.
// Since timer2 is an 8-bit timer, the maximum count value is 0xFE.
// The OVF0 specifier sets the interrupt flag on TOP.
timer[2]->initialize(t_mode::NORMAL, t_interrupt::OVF0);

// For the next example we want the timer to count to a desired value.
// Let's say that this value is 20000 clocks, e.g.: 0x4E20 (HEX).
// The CTC specifier clears the timer count on compare value.
// The COMPA specifier fires an interrupt on overflow.
timer[3]->initialize(t_mode::CTC, t_interrupt::COMPA);
timer[3]->setCompareValueA(0x4E20);

// The next line sets no interruptflag.
// NOTE::the InterruptServiceRoutine() ups the timer overflow counter.
// The nonResetCount() funtion does not work here.
timer[3]->initialize(t_mode::CTC, t_interrupt::NONE);

// The following example shows how to _NOT_ set a timer.
// We set the timer to clear on compare with 20000 and set an interrupt on TOP.
// This means that the interrupt is never fired.
timer[3]->initialize(t_mode::CTC, t_interrupt::OVF);
timer[3]->setCompareValueA(0x4E20);
```

Supported interrupt vectors are `OVF`, `COMPA`, `COMPB` for 8-bit timers and additionally , `COMPC` for 16-bit timers.

### Pulse Width Modulation

The second distinct operation modes are **Pulse Width Modulation** (PWM). In these modes interrupts and corresponding vectors do not work. This library provides easy-to-use funtions to specify a timer as PWM-signal. The supported modes are listed below. The Fast PWM is considered as the most standard form of PWM with the recognizable saw-tooth curve. For the other modes I refer to the AVR-manual. An 8-bit timer has two channels A and B, and an 16-bit timer has three channels A,B and C. This means you can obtain either two or three PWM-signals from one timer!

| Pulse Width Modulation (PWM) modes  						| Code specifier 		| Bitness	|
| ----------------------------------------------- | ----------------- | ------- |
| None      																			| `t_mode::NONE`		| 8/16		|
| Pulse Width Modulation (PWM) Fast  							| `t_mode::PWM_F` 	| 8/16		|
| Pulse Width Modulation (PWM) Pulse Correct 			| `t_mode::PWM_PC` 	| 8/16		|
| Pulse Width Modulation (PWM) Frequency Correct 	| `t_mode::PWM_FC` 	| 16			|

```c++
// Since timer4 is an 16-bit timer the maximum count value is 65535 (2^16 - 1).
// This means that the timer counts to 4095.94 us.
// Setting the duty cycle to 0.75% gives ON-time of 0.75*4095.9375 = 3071.95 us.
timer[4]->initialize(t_mode::PWM_F, t_channel::A, t_inverted::NORMAL);
timer[4]->setPrescaler(1);
timer[4]->setDutyCycleA(0.75);

// To invert the channel we set the INV specifier which gives us 3071.95 us OFF-time.
timer[4]->initialize(t_mode::PWM_F, t_channel::A, t_inverted::INV);
```

The period from the latter example is a bit tedious to use. So let's create a timer with a desired period. Let's say we want a PWM-signal with a period of 4 ms. The number of the timer TOP value can be found by the formulas above. For our example this means to set the top value to 63999 ticks with a prescaler of 1. This fits within the 16-bit timer maximum count value (65355). Which prescaler and TOP value must be set for an 8-bit timer? Try it yourself!

To set the 16-bit timer to a PWM-signal with period of 4 ms we look at the next code block example. The TOP specifier after the channel declaration sets the timer with desired TOP value. Note that you lose one PWM-channel since the TOP value is set in the OCRA register.

```c++
// Set timer5 to a PWM-signal with a period of 4000 us.
// The PWM-outputs are specified at channel B and C.
timer[5]->initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);
timer[5]->setPrescaler(1);
timer[5]->setCompareValueA(63999);
timer[5]->setDutyCycleB(0.75);
timer[5]->setDutyCycleC(0.67);
```

Supported PWM-channels are `A`, `B`, `B_TOP` and `AB` for 8-bit timers and additionally , `C`, `C_TOP`, `BC_TOP`, `AC`, `BC` and `ABC` for 16-bit timers.

## Future extensions

* Arduino UNO support.
* Extern timer support `t_alias::TX` and additional functions for this timer.
* The current interrupt service routine ups the overflow counter. However, a lot more ISR's could be usefull. In the future I'll provide some method to override the class member ISR() function.
* The CAPTURE interrupt vectors are not supported.
