# atmega-18650-discharger


An arduino. atmega1280/2560 18650 battery discharger and capacity tester supporting 30 concurrent battery's being discharged, simple mobile phone interface using wifi, OSC protocol and UDP messages to off-the-shelf phone application called 'Touch OSC'. 

Discharges what is assumed ot be a fully-charged 18650 down till a given threshold, then stops the discharging, measuring the actual discharge profile over time and tells user the measure capacity at hte end of the discharge cycle.

Original code by @davidbuzz, working with esp8266/esp12 as wifi interface and master controller and 2x Arduino Mega 2560 slaves.


