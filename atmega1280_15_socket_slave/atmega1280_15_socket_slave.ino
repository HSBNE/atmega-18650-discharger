// Slave project... will add info later :)
#include <Wire.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <TimeLib.h>
#define ARDUINO

#include <timer.hpp>
#include <timer8.hpp>
#include <timer16.hpp>
#include <util/atomic.h>
#include <avr/sleep.h>
#include <avr/power.h>

// I happen to use an atmega1280 and an atmega2560 so this is a neat way to decide which
// one gets what ID... otherwise use EEPROM or something
#if defined(__AVR_ATmega1280__)
#define DEVICEID 2
#define POSITIONS 15
#elif defined(__AVR_ATmega2560__)
#define DEVICEID 3
#define POSITIONS 15
#endif

// I2C device IDs start at 8 and go up.
#define I2C_DEVICEID DEVICEID+8

// earlier coe had only 3 on Nano, etc.

// I2C packet......
typedef struct sensorData_t {
  uint8_t     Header;
  uint8_t     DeviceID;       // device ID.
  uint16_t    BatteryVoltage;
  uint8_t     Running:1;      // lowest bit, compatile with full byte
  uint8_t     Complete:1;
  uint8_t     Fault:1;
  uint8_t     Idle:1;
  uint16_t    Current;
  uint16_t    CurrentTotal;
  uint16_t    RunTime;
  uint8_t     socket;
  uint16_t    Power;
  uint16_t    PowerTotal;
  uint16_t    SocketLoss;
  uint32_t    timeStamp;
  uint32_t    sampleTotal;
} __attribute__ ((packed));
#define PACKET_SIZE sizeof(sensorData_t)

// as bytes.....
typedef union I2C_Packet_t {
  sensorData_t sensor;
  byte I2CPacket[sizeof(sensorData_t)];
};
I2C_Packet_t batteryinfo;

typedef enum {
  eChgWaitDetect = 0,
  eChgDischg,
  eChgDischgSktProb,
  echgWaitRemove
} chgStates;

typedef enum {
  eStateStartup = 0,
  eStateIdle, // Green Flash
  eStateRunning, // Blue
  eStateSomeComplete, // Blue Flash
  eStateAllComplete,   // Green
  eStateAnyError  // Red Flash
} eRunState;

typedef struct calibData_t {
  uint8_t      cmd;
  uint8_t      pos;
  uint16_t     data;
} __attribute__ ((packed));

typedef struct setTime_t {
  uint8_t      cmd;
  uint32_t     time;
} __attribute__ ((packed));

typedef struct dataRequest_t {
  uint8_t       pos;
}  __attribute__ ((packed));

typedef union I2C_Request_t {
  dataRequest_t   request;
  calibData_t     calibration;
  setTime_t       time;
  uint8_t         I2CPacket[16];
} __attribute__ ((packed));

typedef struct _typBatCtx {
  chgStates       state;          // Charge State
  uint8_t         confirm;        // Concfirm count for leaving some states
  uint16_t        batteryVoltage; // MilliVolts
  uint16_t        batteryCurrent; // MilliAmps
  uint16_t        batteryWatts;   // MilliWatts
  uint16_t        prevVoltage;    // Last Known good voltage
  uint32_t        socketVoltage;  // Voltage msr with Current Draw
  uint16_t        socketLoss;     // Voltage loss in the socket
  uint32_t        totalCurrent;   // Total microAmpHours
  uint32_t        totalWatts;     // Total microWattHours
  uint32_t        startTime;      // time_since_starts
  uint32_t        lastTime;       // previousTime
  uint32_t        runTime;        // time_ends
  uint32_t        microAdjust;    // Micro adjustment
  uint16_t        sampleTime;     // Sample time 
  uint32_t        sampleTotal;    // Total Sample time
  uint8_t         ledState: 1;    // ledstates
  uint8_t         complete: 1;    // dones
  uint8_t         present: 1;     // batterypresents
  uint8_t         fetState: 1;
  uint8_t         fault: 1;
} ;

int DEBUG = 0;
eRunState runState = eStateStartup;

// Load resistors are ~2.2ohms with a 29mOhm FET... Calibrate at room temperature. Calibration store in EEPROM
uint32_t resistor_val[POSITIONS] =
#if defined(__AVR_ATmega1280__)
  { 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229 };
#elif defined(__AVR_ATmega2560__)
  { 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229, 2229 };
#else
#error "Device type to resisotr value lookup is not defined!"
#endif

uint16_t resistor_time[POSITIONS] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 } ;
uint32_t resistor_comp[POSITIONS] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 } ;

int i;

//16 way! - the most important thing is 16 available ADCs, and 16 digital IO for mosfets, as a bonus, we have 16 digital io for LEDs too. and still spares after that. :
// constants
const unsigned int sensor1Pins[POSITIONS] = 
#if defined(__AVR_ATmega1280__)
  { A0,A1,A2,A3, A4,A5,A6,A7, A8,A9,A10, A11,A12,A13,A14 };
#elif defined(__AVR_ATmega2560__)
  { A0,A1,A2, A3,A4,A5, A6,A7,A8, A9,A10,A11, A12,A13,A14 };
#endif

// don't use 20 or 21 for IO, as they are for I2C comms on a 2560.
// Pin 44 doesn't work as an output for some reason... NFI.. 
// just dont use it.. maybe its setup as a timer output of some crap
const unsigned int FETPins[POSITIONS] = 
#if defined(__AVR_ATmega1280__)
  { 30,31,32,33, 34,35,36,37, 43,42,45, 38,39,40,41  };  
#elif defined(__AVR_ATmega2560__)
  { 31,32,33, 35,36,37, 39,40,41, 43,42,45, 27,28,29 }; 
#endif

// battery constant characteristics
const int LiMinThreshold = 2700; //3900; //2700; // 2700 = Lithium Minimal Voltage for load removal, 3900 = minimum needed for re-charger to kick in.
const int LiMaxThreshold = 4250; // Lithium Max Voltage for load removal
const int LiNewDischargeThreshold = 4000;
const int LiResumeWithLastKnown = 250;
const int LiRemovedThreshold = 500;
const int LiDeadBand = 500;
const int LiSocketLossInitialLimit = 300;
const int LiSocketLossLimit = 600;

_typBatCtx BatContext[POSITIONS] = {0};

 // Nominal 5v regulated Vcc line.. A default
#define ADC_NOM_INIT      5025UL     
// Nominal Voltage band-gap.. measured and calibration stored if no Vref
#define ADC_BG_INIT       1060UL      
// Vref on A15 used to self-calirate the ADC_NOM continously as to not lose a mV
#define ADC_EX_INIT       2500UL
uint32_t ADC_NOM      = ADC_NOM_INIT;
uint32_t ADC_BG       = ADC_BG_INIT;
uint32_t ADC_EX_REF   = ADC_EX_INIT;
#define ADC_DEC           4095UL

#define LED_BLUE    10
#define LED_GREEN   11
#define LED_RED     12

byte REQUESTEDPOS = 0x00;

SimpleTimer  timer;
int dataTmrID;
int calibTmrID;
int ledTimerID;
int compTimerID;

avr::Timer16::Ptr  tmrSec;
avr::Timer16::Ptr  tmrMicros;
avr::Timer16::Ptr  tmrMillis;

uint32_t           time_offset;
uint32_t           millis_offset;

int read_bandGap() {  
  int i = 16;
  uint32_t data = 0;
  uint8_t ADCSRA_buf = ADCSRA;

  ADCSRA = 0;
  ADCSRB = 0;
  ADCSRA = 0x87;
  
  // REFS1 REFS0          --> 0 1 AREF, Internal Vref turned off
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)
  ADMUX = (0<<REFS1) | (0<<REFS0) | (0<<ADLAR) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

  // Start a conversion  
  ADCSRA |= _BV( ADSC );
  
  // Delay just a moment
  delay(5);

  do {
    // Start a conversion  
    ADCSRA |= _BV( ADSC );
    
    // Wait for it to complete
    while( ( (ADCSRA & (1<<ADSC)) != 0 ) );

    data += ADC;
  } while (--i);

  data = 0;
  i = 64;
  
  do {
    // Start a conversion  
    ADCSRA |= _BV( ADSC );
    
    // Wait for it to complete
    while( ( (ADCSRA & (1<<ADSC)) != 0 ) );

    data += ADC;
  } while (--i);

  ADCSRA = ADCSRA_buf;

  return (int)(((uint32_t)(data >> 3) * ADC_NOM) / 8191);
}

typedef enum eCmdHdr {
  eCmdSetCalibration = 0,
  eCmdSetDateTime
} ;

void receiveEvent(int bytes)
{
  I2C_Request_t req;
  memset(req.I2CPacket, 0, sizeof(req.I2CPacket));
  
  if (Wire.available() != 0)
  {
    int i;
    
    for (i = 0; i < bytes && Wire.available(); i++)
    {
      req.I2CPacket[i] = Wire.read();
    }

    if (i == 1) {
      REQUESTEDPOS = req.request.pos;

      /*if (DEBUG > 0 ) {
        Serial.print("Received: ");
        Serial.print(REQUESTEDPOS, HEX);
        Serial.print("\n");
      }*/
    } else {
      switch (req.I2CPacket[0]) {
        case eCmdSetCalibration:
          // Calibration position number starts with 0x80, remove it for proper channel offset
          if (i >= sizeof(calibData_t)) {
            digitalWrite(LED_BLUE, HIGH );
            //Serial.println("--=={ Calibration Event }==-- ");
            if (req.calibration.pos & 0x80)
            {
              uint8_t channel;
              req.calibration.pos &= ~0x80;

              channel = req.calibration.pos & 0x7F;

              if (channel == 0) {
                // Calibrate the vRef
                ADC_NOM = req.calibration.data;
                ADC_BG  = read_bandGap();
                ADC_EX_REF = SuperSuperSampleAnalogRead(A15);
                
                // Record the ADC_NUM to begin with
                EEPROM.update(0, (uint8_t)(ADC_NOM & 0xFF));
                EEPROM.update(1, (uint8_t)(ADC_NOM >> 8));

                // Bandgap gets used for future self-calibration
                EEPROM.update(2, (uint8_t)(ADC_BG & 0xFF));
                EEPROM.update(3, (uint8_t)(ADC_BG >> 8));

                if ( ADC_EX_REF >= (ADC_EX_INIT - 100)
                  && ADC_EX_REF <= (ADC_EX_INIT + 100) ) {
                  EEPROM.update(4, (uint8_t)(ADC_EX_REF & 0xFF));
                  EEPROM.update(5, (uint8_t)(ADC_EX_REF >> 8));
                }
              } else if (channel <= POSITIONS) {
                uint8_t pos = channel - 1;
                uint32_t rawData;
                uint32_t current = req.calibration.data;
                
                //Serial.println("---------------{ Calibration } ------------------");
                //Serial.print("\tCalibrating: ");
                //Serial.print(channel);
                //Serial.print(" @ ");
                //Serial.print(current);
                //Serial.println("mA");
                //digitalWrite(FETPins[pos], HIGH);
                delay(1000);

                rawData = SuperSuperSampleAnalogRead(sensor1Pins[pos]);
                digitalWrite(FETPins[pos], LOW);

                rawData *= 1000UL;
                rawData /= current;

                Serial.print("\mOhm = ");
                Serial.println(rawData);
                
                if (rawData >= 1900 && rawData <= 2500) {
                  resistor_val[pos] = rawData;
                  // Commit to memory
                  EEPROM.update(10 + (pos * 2), (uint8_t)(rawData & 0xFF));
                  EEPROM.update(11 + (pos * 2), (uint8_t)(rawData >> 8));

                  //Serial.println("--------------{ COMPLETE }-----------");
                } else {
                  //Serial.println("----------{ FAILED TOO HI/LO }-------");
                }
              }
            }
          }
          break;
        case eCmdSetDateTime:
          if (i >= sizeof(setTime_t)) {
            //Serial.println("--=={ Calibration Event }==-- ");
            if (time_now() == 0) {
              digitalWrite(LED_BLUE, HIGH );
              digitalWrite(LED_RED, LOW );
              digitalWrite(LED_GREEN, LOW );
              //Serial.println("--=={ Time Start }==--");
              start_hw_timers();
            }

            time_offset = req.time.time - (time_now() / 1000UL);
            millis_offset = req.time.time - (millis() / 1000UL);
          }

          break;
      }
    }
  }
}

void load_calibration() {
  uint16_t eeADC_NOM;
  uint16_t eeADC_BG;
  uint16_t eeADC_EX_REF;
  uint16_t res;
  uint8_t pos;

  eeADC_NOM =             EEPROM.read(0);
  eeADC_NOM |= ((uint16_t)EEPROM.read(1)) << 8;
  
  // Bandgap gets used for future self-calibration
  eeADC_BG =             EEPROM.read(2);
  eeADC_BG |= ((uint16_t)EEPROM.read(3)) << 8;

  eeADC_EX_REF =             EEPROM.read(4);
  eeADC_EX_REF |= ((uint16_t)EEPROM.read(5)) << 8;

  Serial.println("---------------{ Calibration Load } ------------------");

  if (eeADC_NOM >= 4500 && eeADC_NOM <= 5150) {
    ADC_NOM = eeADC_NOM;
    
    Serial.print("\tADC_NOM: ");
    Serial.print(ADC_NOM);
    Serial.println("mV");
  } else {
    ADC_NOM = ADC_NOM_INIT;
    Serial.print("\tFACTORY ADC_NOM: ");
    Serial.print(ADC_NOM);
    Serial.println("mV");
  }

  if (eeADC_BG >= 900 && eeADC_BG <= 1300) {
    ADC_BG = eeADC_BG;
    Serial.print("\tADC_BAND_GAP: ");
    Serial.print(ADC_BG);
    Serial.println("mV");
  } else {
    ADC_BG = read_bandGap();
    Serial.print("\tFACTORY ADC_BAND_GAP: ");
    Serial.print(ADC_BG);
    Serial.println("mV");
  }

  if ( eeADC_EX_REF >= (ADC_EX_INIT - 100)
    && eeADC_EX_REF <= (ADC_EX_INIT + 100) ) {
    ADC_EX_REF = eeADC_EX_REF;
    Serial.print("\tADC_EX_REF: ");
    Serial.print(ADC_EX_REF);
    Serial.println("mV");
  } else {
    eeADC_EX_REF = SuperSuperSampleAnalogRead(A15);

    if (eeADC_EX_REF >= (ADC_EX_INIT - 400) 
     && eeADC_EX_REF <= (ADC_EX_INIT + 400)) {
      ADC_EX_REF = eeADC_EX_REF;
      Serial.print("\tFACTORY AD_EX_REF: ");
      Serial.print(ADC_EX_REF);
      Serial.println("mV");
    } else {
      ADC_EX_REF - 0;
      Serial.print("\tAD_EX_REF: NOT AVAILABLE A15 = ");
      Serial.print(eeADC_EX_REF);
      Serial.println("mV");
    }
  }

  for (i=0;i<POSITIONS;i++)
  {
    res =            EEPROM.read(10 + (i * 2));
    res |= (uint16_t)EEPROM.read(11 + (i * 2)) << 8 ;

    if (res >= 1900 && res <= 2500) {
      resistor_val[i] = res;
      
      Serial.print("\Position: ");
      Serial.print(i+1);
      Serial.print(" @ ");
      Serial.print(res);
      Serial.println("mOhm");
      
    }
  }
}

// Self-Calibrate the ADC_NOM based upon saved ADC BandGap voltage
void self_calibrate() {
  if (ADC_EX_REF >= (ADC_EX_INIT - 100) 
   && ADC_EX_REF <= (ADC_EX_INIT + 100)) {
    // Use the external reference calibration method
    uint32_t  exRef =  SuperSuperSampleAnalogRead(A15);

    Serial.println("=======[self_calibrate()]========");
    Serial.print("\tpre  { ADC_NOM: ");
    Serial.print(ADC_NOM);
    Serial.print("mv ADC_EX_REF: ");
    Serial.print(exRef);
    Serial.println("mV }");

    // Sanity check calibration
    if (exRef >= (ADC_EX_INIT - 400)
     && exRef <= (ADC_EX_INIT + 400)) {
      ADC_NOM = (ADC_NOM * 1000UL) / ( (exRef * 1000UL) / ADC_EX_REF );
  
      Serial.print("\tpost { ADC_NOM: ");
      Serial.print(ADC_NOM);
      Serial.print("mv ADC_EX_REF: ");
      Serial.print(SuperSuperSampleAnalogRead(A15));
      Serial.println("mV }");
    
      Serial.println("=======[****************]========");
    } else {
      Serial.println("\t ADC EX REF OUTSIDE EXPECTED RANGE");
      Serial.println("=======[****************]========");
    }
    
  } else {
    // Use the bandgap calibration method
    uint32_t  bandGap = read_bandGap();
  
    Serial.println("=======[self_calibrate()]========");
    Serial.print("\tpre  { ADC_NOM: ");
    Serial.print(ADC_NOM);
    Serial.print("mv ADC_BG: ");
    Serial.print(bandGap);
    Serial.println("mV }");
    
    ADC_NOM = (ADC_NOM * 1000UL) / ( (bandGap * 1000UL) / ADC_BG );
  
    Serial.print("\tpost { ADC_NOM: ");
    Serial.print(ADC_NOM);
    Serial.print("mv ADC_BG: ");
    Serial.print(read_bandGap());
    Serial.println("mV }");
  
    Serial.println("=======[****************]========");
  }
}

// Return back Millivolts
int SuperSuperSampleAnalogRead(int pin) {
  uint32_t data =  0;
  int read ;
  int i = 16;

  do {
    data = analogRead(pin);
  } while (--i);

  i = 64;
  data = 0;


   do {
    read = analogRead(pin) ;
    if (read < 0 || read >= 0x400) {
      i++;
    } else {
      data += read;
    }
  } while (--i);

  analogRead(A8 - 1); // Should be GND

  // Remove lowest two bits and scale valuess
  return (int)(((uint32_t)(data >> 3) * ADC_NOM) / 8191UL);
}

// Return back Millivolts
int SuperSampleAnalogRead(int pin) {
  uint16_t data =  0;
  int read ;
  int i = 16;

  do {
    data = analogRead(pin);
  } while (--i);

  i = 16;
  data = 0;

   do {
    read = analogRead(pin) ;
    if (read < 0 || read >= 0x400) {
      i++;
    } else {
      data += read;
    }
  } while (--i);

  analogRead(A8 - 1); // Should be GND

  // Remove lowest two bits and scale valuess
  return (int)(((uint32_t)(data >> 2) * ADC_NOM) / ADC_DEC);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() { 
  if ((REQUESTEDPOS < 1) or ( REQUESTEDPOS > POSITIONS )) {
    REQUESTEDPOS = 1;  // if not given a decent offset, assume the first one.
  }

  // turn human readable 1-based offset back to 0-based array offset:
  int __REQUESTEDPOS = REQUESTEDPOS - 1;
  _typBatCtx * pCtx = &BatContext[__REQUESTEDPOS];

  // fill the packet
  batteryinfo.sensor.Header          = 4;
  batteryinfo.sensor.DeviceID        = DEVICEID;
  batteryinfo.sensor.BatteryVoltage  = pCtx->batteryVoltage;
  batteryinfo.sensor.SocketLoss      = pCtx->socketLoss; 
  batteryinfo.sensor.Current         = pCtx->batteryCurrent;
  batteryinfo.sensor.CurrentTotal    = pCtx->totalCurrent / 100000UL;
  batteryinfo.sensor.Power           = pCtx->batteryWatts;
  batteryinfo.sensor.PowerTotal      = pCtx->totalWatts / 100000UL;
  batteryinfo.sensor.RunTime         = pCtx->runTime;
  batteryinfo.sensor.Running         = (pCtx->state == eChgDischg )|| (pCtx->state == eChgDischgSktProb); 
  batteryinfo.sensor.Complete        = pCtx->complete && (pCtx->state == echgWaitRemove);
  batteryinfo.sensor.Fault           = pCtx->fault;
  batteryinfo.sensor.Idle            = !pCtx->present;
  batteryinfo.sensor.socket          = REQUESTEDPOS;
  batteryinfo.sensor.timeStamp       = pCtx->lastTime;
  batteryinfo.sensor.sampleTotal     = pCtx->sampleTotal;

  // send the packet.
  Wire.write(batteryinfo.I2CPacket, PACKET_SIZE);
}

// Read the battery voltage from the ADC and adjust it
void battery_read_voltage(uint8_t pos, boolean toggleFET) {
  _typBatCtx * pCtx = &BatContext[pos];

  if (toggleFET && pCtx->fetState) {
    digitalWrite(FETPins[pos], LOW);
  }

  // Read the battery voltage if we're toggling or the current value otherwise
  if ( (toggleFET && pCtx->fetState) || !pCtx->fetState) {
    pCtx->batteryVoltage = SuperSampleAnalogRead(sensor1Pins[pos]);
  } else {
    pCtx->socketVoltage  = SuperSampleAnalogRead(sensor1Pins[pos]);
  }

  if (toggleFET && pCtx->fetState) {
    digitalWrite(FETPins[pos], HIGH);
  }
}

void battery_calc_current(uint8_t pos) {
  _typBatCtx * pCtx = &BatContext[pos];

  // FETs are aprox 29mOhm
  pCtx->batteryCurrent = (pCtx->socketVoltage * 1000UL) / (resistor_val[pos] + resistor_comp[pos]);
  pCtx->batteryWatts   = ((uint32_t)pCtx->batteryVoltage * (uint32_t)pCtx->batteryCurrent) / 1000UL;
  pCtx->socketLoss     = pCtx->batteryVoltage > pCtx->socketVoltage ? pCtx->batteryVoltage - pCtx->socketVoltage : 0;
}

void battery_set_discharge(uint8_t pos, boolean state) {
  _typBatCtx * pCtx = &BatContext[pos];

  if (state) {
    digitalWrite(FETPins[pos], HIGH);   // set the Discharge FET on
    pCtx->ledState = pCtx->fetState = 1;
  } else {
    digitalWrite(FETPins[pos], LOW);   // set the Discharge FET off - stop loading
    pCtx->ledState = pCtx->fetState = 0;
  }
}

void battery_reset_context(uint8_t pos) {
  _typBatCtx * pCtx = &BatContext[pos];

  pCtx->state           = eChgWaitDetect;
  pCtx->confirm         = 0;
  pCtx->batteryVoltage  = SuperSampleAnalogRead(sensor1Pins[pos]);
  pCtx->batteryCurrent  = 0;
  pCtx->socketLoss      = 0;
  pCtx->prevVoltage     = 0;
  pCtx->socketVoltage   = 0;
  pCtx->totalCurrent    = 0;
  pCtx->startTime       = 0;
  pCtx->runTime         = 0;
  pCtx->ledState        = 0;
  pCtx->complete        = 0;
  pCtx->present         = 0;
  pCtx->lastTime        = 0;
  pCtx->microAdjust     = 0;
  pCtx->fault           = false;
  pCtx->totalWatts      = 0;
  pCtx->sampleTotal     = 0;
  pCtx->sampleTime      = 0;
  pCtx->batteryWatts    = 0;
  pCtx->ledState = pCtx->fetState = 0;
  digitalWrite(FETPins[pos], LOW);
}

void battery_print_position (struct _typBatCtx * pCtx, uint8_t pos, uint8_t isRunning) {
  if (DEBUG >= 3) return;
  Serial.print("{'Device':");
  Serial.print(DEVICEID); // print voltage value
  Serial.print(".");
  Serial.print(pos + 1); // print human readable starting from 1

  Serial.print(",'State':");
  if (pCtx->fault) {
    Serial.print("FAULT");
  } else {
    switch (pCtx->state) {
      case eChgWaitDetect:
        Serial.print("IDLE");
        break;
      case eChgDischg:
        if (pCtx->fetState)
          Serial.print("RUNNING");
        else
          Serial.print("CONFIRM");
        break;
      case eChgDischgSktProb:
        Serial.print("SKTPROB");
        break;
      case echgWaitRemove:
        if (pCtx->fault) {
          Serial.print("FAULT");
        } else if (pCtx->complete) {
          Serial.print("CMPLT");
        } else {
          Serial.print("REMOVE");
        }
        break;
    }
  }
  
  Serial.print(",'mV':");
  Serial.print(pCtx->batteryVoltage); // print voltage value

  Serial.print(",'SktLoss':");
  Serial.print(pCtx->socketLoss); // print value

  Serial.print(",'mA':");
  Serial.print(pCtx->batteryCurrent); // print value

  Serial.print(",'mW':");
  Serial.print(pCtx->batteryWatts); // print value

  Serial.print(",'mAh':");
  Serial.print(pCtx->totalCurrent / 100000);

  Serial.print(",'mWh':");
  Serial.print(pCtx->totalWatts / 100000);

  Serial.print(",'RunTime':");    // prints a tab
  Serial.print(pCtx->runTime);

  Serial.print(",'SampleTotal':");    // prints a tab
  Serial.print(pCtx->sampleTotal);

  Serial.print(",'SampleTime':");    // prints a tab
  Serial.print(pCtx->sampleTime);

  if (pCtx->state == eChgDischgSktProb ) {
    Serial.print(",'SktDrop':");
    Serial.print((int)pCtx->batteryVoltage - (int)pCtx->socketVoltage);
  }
  Serial.print("}");
}

#define FET_OFF(x)  digitalWrite(FETPins[x], LOW); BatContext[x].fetState = 0;
#define FET_ON(x)   digitalWrite(FETPins[x], HIGH); BatContext[x].fetState = 1;

#define LED_OFF(x)  BatContext[x].ledState = 0; // digitalWrite(ledPins[x], LOW); 
#define LED_ON(x)   BatContext[x].ledState = 1; // digitalWrite(ledPins[x], HIGH);

void loop() {
  timer.run();
}

void ledDisplay() {
  static int pulse = 0;
  
  switch (runState) {
    case eStateStartup: // Green
      if (DEBUG>4) Serial.println("LED Mode: eStateStartup: // Green");
      digitalWrite(LED_RED,   LOW);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE,  LOW);
      break;

    case eStateIdle: // Green Flash
      if (DEBUG>4) Serial.println("LED Mode: eStateIdle: // Green Flash");
      digitalWrite(LED_RED,   LOW);
      digitalWrite(LED_GREEN, pulse ? HIGH : LOW);
      digitalWrite(LED_BLUE,  LOW);
      break;

    case eStateRunning: // Blue
      if (DEBUG>4) Serial.println("LED Mode:  eStateRunning: // Blue");
      digitalWrite(LED_RED,   LOW);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE,  LOW);
      break;

    case eStateSomeComplete: // Blue Flash
      if (DEBUG>4) Serial.println("LED Mode: eStateSomeComplete: // Blue Flash");
      digitalWrite(LED_RED,   LOW);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE,  pulse ? HIGH : LOW);
      break;

    case eStateAllComplete: // White Flash
      if (DEBUG>4) Serial.println("LED Mode: eStateAllComplete: // Green");
      digitalWrite(LED_RED,   LOW);
      digitalWrite(LED_GREEN, pulse ? HIGH : LOW);
      digitalWrite(LED_BLUE,  pulse ? LOW : HIGH);
      break;

    case eStateAnyError: // Red Flash
      if (DEBUG>4) Serial.println("LED Mode: eStateAnyError: // Red Flash");
      digitalWrite(LED_RED,   pulse ? HIGH : LOW);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE,  LOW);
      break; 
  }

  pulse = ++pulse % 2;
}

void sensorDataSend() {
  _typBatCtx * pCtx = NULL;
  uint32_t integration;
  uint8_t skip;
  uint32_t  timeNow;
  uint32_t  microNow;
  static uint32_t timeLast = 0;
  bool allRunning = true;
  bool allComplete = true;
  bool anyError = false;
  bool someComplete = false;
  bool someRunning = false;
  static bool warningSent = false;

  // Ensure that the clock is running
  if ((timeNow = time_now()) <= 1000) {
    runState = eStateAnyError;

    if (!warningSent)
      Serial.println("**** 1Hz External clock NOT running *****");

    warningSent = true;

    digitalWrite(LED_RED,  HIGH );

    for ( int pos = 0 ; pos < POSITIONS ; pos++ ) {
      // Disable all FETs as our time-keeping is fucked up
      FET_OFF(pos);
    }

    return;
  } 

  warningSent = false;

  Serial.print("----------------------- ");
  Serial.print(timeNow);
  Serial.println("ms--------------------------");

  for ( int pos = 0 ; pos < POSITIONS ; pos++ ) {
    pCtx = &BatContext[pos];

    switch (pCtx->state) {
      case eChgWaitDetect:
      
        FET_OFF(pos);
        // Check the battery voltage first
        battery_read_voltage(pos, false);

        if ( pCtx->batteryVoltage >= LiMaxThreshold ) {
          pCtx->fault = true;
          pCtx->present = false;
          pCtx->complete = false;
          
        } else if (pCtx->batteryVoltage >= (LiMinThreshold + LiResumeWithLastKnown)) {
          // Check socket resistance fist. Then we move on
          FET_ON(pos);
          battery_read_voltage(pos, false);
          FET_OFF(pos);
          // Calculate the socketLoss and currents/watts
          battery_calc_current(pos);

          timeNow = time_now();

          if (timeNow == 0) {
            pCtx->fault = true;
            pCtx->batteryWatts   = 0;
            pCtx->batteryCurrent = 0;

            Serial.print("--=={ TIME FAULT }==--");
          } else if ( pCtx->socketLoss >= LiSocketLossInitialLimit ) {
            pCtx->fault = true;
            pCtx->present = true;
            pCtx->batteryWatts   = 0;
            pCtx->batteryCurrent = 0;

            Serial.print("--=={ EXCESS SOCKET LOSS }==--");
            
          } else if ( ( pCtx->complete == true && pCtx->batteryVoltage >= LiNewDischargeThreshold )
          || ( pCtx->complete == false && pCtx->batteryVoltage >= LiNewDischargeThreshold && pCtx->batteryVoltage >= ( pCtx->prevVoltage + LiResumeWithLastKnown ) ) ) {
            // Enable the discharge cycle
            battery_set_discharge(pos, true);
            // Set state the discharge now
            pCtx->state         = eChgDischg;
            // Record some basic statistics
            pCtx->startTime      = pCtx->lastTime = timeNow;
            pCtx->runTime        = 0;
            pCtx->totalCurrent   = 0;
            pCtx->totalWatts     = 0;
            pCtx->batteryWatts   = 0;
            pCtx->socketVoltage  = 0;
            pCtx->batteryCurrent = 0;
            pCtx->microAdjust    = 0;
            pCtx->sampleTotal    = 0;
            pCtx->sampleTime     = 0;
            pCtx->prevVoltage    = pCtx->batteryVoltage;
            pCtx->complete       = false;
            pCtx->fault          = false;
            pCtx->present        = true;
            pCtx->confirm = 10;
            Serial.print("\tNew battery Detected! ");
            
          } else if ( pCtx->complete == false && pCtx->runTime > 0
          && ( ( ( pCtx->batteryVoltage >= pCtx->prevVoltage ) 
              && ( pCtx->batteryVoltage < (pCtx->prevVoltage + LiResumeWithLastKnown)
              &&  pCtx->batteryVoltage >=  (LiMinThreshold) ) ) 
            || ( pCtx->batteryVoltage < LiNewDischargeThreshold
              &&  pCtx->batteryVoltage >=  (LiMinThreshold + LiDeadBand) ) ) ) 
          {
            // Enable the discharge cycle
            battery_set_discharge(pos, true);
            // Set state the discharge now
            pCtx->state         = eChgDischg;
            // Record some basic statistics
            pCtx->lastTime       = timeNow;
            pCtx->startTime      = pCtx->lastTime - (pCtx->runTime * 1000);
            pCtx->microAdjust    = 0;
            //pCtx->runTime        = 0;
            //pCtx->totalCurrent   = 0;
            //pCtx->totalWatts     = 0;
            //pCtx->batteryWatts   = 0;
            //pCtx->socketVoltage  = 0;
            //pCtx->batteryCurrent = 0;
            pCtx->prevVoltage    = pCtx->batteryVoltage;
            // Resume Previous Discharge
            pCtx->fault          = false;
            pCtx->present        = true;
            pCtx->confirm = 10;
            Serial.print("\tResuming Previous Discharge! ");
          } 
        } else {
          pCtx->fault = false;
          pCtx->present = false;

        }

        allRunning = false;
        if (!pCtx->complete) {
          allComplete = false;
        } else {
          someComplete = true;
        }
        if (pCtx->fault || timeNow == 0) {
          anyError = true;
        }  

        if (DEBUG < 3) {
          battery_print_position(pCtx, pos, false);
          
          Serial.println();
        }

        break;

      case eChgDischg:
        someRunning = true;
        allComplete = false;
        
        // Read voltage to calculate current (socketVoltage)
        battery_read_voltage(pos, false);

        timeNow = time_now();

        // Time error, show it
        if (timeNow == 0 || timeNow <= pCtx->lastTime) {
            FET_OFF(pos);
            pCtx->socketVoltage  = 0;
            pCtx->confirm = 10;
            pCtx->state = eChgDischgSktProb;
            pCtx->fault = true;
        } else {
          pCtx->sampleTime = micro_now();

          // Read voltage without the current draw (batteryVoltage)
          battery_read_voltage(pos, true);

          // Calculate the Micro time used in the Voltage call
          pCtx->sampleTime = micro_now() - pCtx->sampleTime;

          pCtx->sampleTime >>= 1; // 2Tiks per uS

          battery_calc_current(pos);
          
          pCtx->fault = false;
          pCtx->present = true;
          pCtx->complete = false;

          // Integrate the current now
          if (pCtx->fetState) {
            uint32_t period;
            uint32_t adjustTime = pCtx->sampleTime + pCtx->microAdjust ;

            pCtx->sampleTotal += pCtx->sampleTime;

            if (adjustTime > 10000UL) {
              adjustTime = 0;
              if (DEBUG) Serial.println("****Micro CLOCK ERROR******");
            }

            // Calculte the period to the 1/10th milli secons from RTC input
            period = (( timeNow - pCtx->lastTime ) * 10UL ) - (adjustTime / 100UL);

            // Carry the remainder for next time.. good enough
            pCtx->microAdjust = adjustTime % 100UL;

            pCtx->totalCurrent += ( period * (uint32_t)pCtx->batteryCurrent) / 360UL;
            pCtx->totalWatts   += ( period * (uint32_t)pCtx->batteryWatts) / 360UL;
          }

          // Calculate the total runtime
          pCtx->runTime  = (timeNow - pCtx->startTime) / 1000UL ;
          pCtx->lastTime = timeNow;

          if ( pCtx->fetState 
          && (pCtx->socketLoss >= LiSocketLossLimit)
          && (pCtx->batteryVoltage >= LiMinThreshold) ) {
            FET_OFF(pos);
            pCtx->socketVoltage  = 0;
            pCtx->confirm = 10;
            pCtx->state = eChgDischgSktProb;
            pCtx->fault = true;

          } else if (pCtx->batteryVoltage >= LiMinThreshold 
          && pCtx->batteryVoltage < LiMaxThreshold) {
            // This is the only place prevVoltage is stored.. last know good voltage
            pCtx->prevVoltage = pCtx->batteryVoltage;
            LED_ON(pos);
            // Also re-enable FET in case it was every turned off
            FET_ON(pos);
            pCtx->confirm = 10;

          } else if ( --pCtx->confirm == 0 ) {
            pCtx->state = echgWaitRemove;
            pCtx->complete  = true;

            battery_set_discharge(pos, false);
          }

          if (DEBUG < 3) {
            battery_print_position(pCtx, pos, true);
    
            Serial.println();
          }
        }
        break;

      case eChgDischgSktProb:
        timeNow = time_now();

        pCtx->fault = true;
        pCtx->present = true;
        pCtx->complete = false;

        if (timeNow == 0 || timeNow <= pCtx->lastTime) {
          // Ensure FET is OFF please
          FET_OFF(pos);
        } else {
          // Remove this time from the run-time
          pCtx->startTime += (timeNow - pCtx->lastTime);

          // Calculate the total runtime
          pCtx->runTime  = ((timeNow - pCtx->startTime) / 1000UL) ;
          pCtx->lastTime = timeNow;

          // Ensure FET is OFF please
          FET_OFF(pos);

          // Read voltage without the current draw
          battery_read_voltage(pos, true);

          if (pCtx->batteryVoltage >= (LiMinThreshold + LiResumeWithLastKnown) 
          && (pCtx->batteryVoltage < LiMaxThreshold) ) { 
            FET_ON(pos);

            // read with discharger going
            battery_read_voltage(pos, false);

            battery_calc_current(pos);

            if ( pCtx->socketLoss < LiSocketLossInitialLimit ) {
              LED_ON(pos);

              // OK to go back to discharge
              pCtx->state = eChgDischg;
              pCtx->confirm = 10;
              pCtx->fault = false;
              pCtx->prevVoltage = pCtx->batteryVoltage;

            } else {
              // Still have an issue
              FET_OFF(pos);
              
              // No current
              pCtx->batteryCurrent = 0;
              pCtx->batteryWatts = 0;

              // Flash LED pulsing to show a connection error here
              if ( (--pCtx->confirm % 4) == 0 )
              {
                LED_OFF(pos);
                pCtx->socketVoltage  = 0;
                pCtx->confirm = 10;
              } else {
                LED_ON(pos);
              }
            }
          } else if ( --pCtx->confirm == 0 ) {
            pCtx->state = echgWaitRemove;

            pCtx->batteryCurrent = 0;
            pCtx->batteryWatts = 0;

            battery_set_discharge(pos, false);
          }
        }

        allComplete = false;
        if (pCtx->fault) {
          anyError = true;
        }  

        if (DEBUG < 3) {
          battery_print_position(pCtx, pos, false);
  
          Serial.println();
        }

      case echgWaitRemove:

        FET_OFF(pos);
        
        battery_read_voltage(pos, false);

        pCtx->batteryWatts    = 0;
        pCtx->batteryCurrent  = 0;
        pCtx->socketVoltage   = 0;
        pCtx->socketLoss      = 0;
        pCtx->present         = true;

        allRunning = false;
        if (!pCtx->complete) {
          allComplete = false;
        } else {
          someComplete = true;
        }
        if (pCtx->fault) {
          anyError = true;
        }  

        if ( pCtx->complete == true
        && ( pCtx->batteryVoltage >= (LiMinThreshold - LiDeadBand) ) 
        && ( pCtx->batteryVoltage < (LiMinThreshold + LiDeadBand) ) ) {
          // Flash the LED
          if ( (pCtx->confirm = (++pCtx->confirm % 2)) ) {
            LED_ON(pos);
          } else {
            LED_OFF(pos);
          }
        } else {
          pCtx->state = eChgWaitDetect;
          LED_OFF(pos);
        } 

        if (DEBUG < 3) {
          battery_print_position(pCtx, pos, false);
          Serial.println();
        }

        break;

      default:
        battery_reset_context(pos);

        break;
    }
  }

  if (anyError) {
    runState = eStateAnyError;
  } else if (allComplete) {
    runState = eStateAllComplete;
  } else if (someRunning) {
    runState = eStateRunning;
  } else if (someComplete) {
    runState = eStateSomeComplete;
  } else {
    runState = eStateIdle;
  }
}

/*
 * micro_now has a 2Mhz clock rate 
 */
uint16_t micro_now(void) {
  return tmrMicros->getCount();
}

/*
 * time_now uses a 8,192Hz external clock and capture timer
 * 
 * If the frequency is higher than this, then we ignore it 
 * and return zero.
 */
uint32_t time_now(void) {
  return tmrSec->getNonResetCount();
}

void start_hw_timers() {
  tmrSec->setPrescaler(0);                    // Use 1 for rising edge or 0xFFFF for falling edge of TIN
  tmrSec->reset();

  tmrMicros->setPrescaler(8);                 // Use 1 for rising edge or 0xFFFF for falling edge of TIN
  tmrMicros->reset();
}

void setup_hw_timers() {
  // Timer overflow every 1s and tick at 32.768kHz
  tmrSec= new avr::Timer16(avr::t_alias::T5);
  tmrSec->initialize(avr::t_mode::CTC, avr::t_interrupt::COMPB);
  tmrSec->setCompareValueA(0xFFFF);
  tmrSec->setCompareValueB(0);
  tmrSec->reset();

  // Timer will expire every 32ms aprox
  tmrMicros= new avr::Timer16(avr::t_alias::T1);
  tmrMicros->initialize(avr::t_mode::NORMAL, avr::t_interrupt::NONE);
  tmrMicros->reset();
}//end setup

void resistor_heat_comp(void) {
  _typBatCtx * pCtx;

  for ( int pos = 0 ; pos < POSITIONS ; pos++ ) {
    pCtx = &BatContext[pos];

    if (pCtx->fetState && resistor_time[pos] < 600U) {
      resistor_time[pos]++;
    } if (!pCtx->fetState && resistor_time[pos]) {
       resistor_time[pos]--;
    }

    if (resistor_time[pos]) {
      resistor_comp[pos] = resistor_time[pos] / 100U;
    } else {
      resistor_comp[pos] = 0;
    }
  } 
}

void setup() {
  _typBatCtx * pCtx;

  analogReference(EXTERNAL);

  // Disable the digital input buffers on the Analogic input ports
  DIDR0 = 0xFF; // ADC0D - ADC7D
  DIDR2 = 0xFF; // ADC8D - ADC15D

  Serial.begin(250000);// start serial port to send data during run to the PC
  Serial.println("Bat PWR Tester[Active]");  // Print a message to the //lcd.
  Serial.println("Detecting Bat .... please put one in now if not already...."); // print voltage value
  Serial.println("                        ");

  load_calibration();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  /* 
   * Pins used for the Timers
   */
  pinMode(49, INPUT_PULLUP);
  pinMode(47, INPUT_PULLUP);

  setup_hw_timers();
  //start_hw_timers();

  // init all the IO and pullups and lEDs etc.
  for ( int pos = 0 ; pos < POSITIONS ; pos++ ) {
    pCtx = &BatContext[pos];

    //pinMode(ledPins[pos], OUTPUT);//activation led and enable for FET
    pinMode(FETPins[pos], OUTPUT);//activation led and enable for FET
    //pinMode(SPKPins[pos], OUTPUT);//activation led and enable for FET
    digitalWrite(FETPins[pos], LOW);   // set the LED off

    battery_reset_context(pos);

    Serial.print("\tDevice=");
    Serial.print(DEVICEID); // raw ADC
    Serial.print(".");
    Serial.print(pos + 1); // print displayable position, starting from .1

    Serial.print("\t");

    resistor_comp[pos] = 0;
    resistor_time[pos] = 0;

    if (pCtx->batteryVoltage > LiMaxThreshold) {
      Serial.print("Warning high-V! V > ");
      Serial.print(LiMaxThreshold);
      Serial.print(" \t");

    } else if (pCtx->batteryVoltage > LiMinThreshold) {
      Serial.print("Type:Li-Ion Bat. \t");

    } else {
      Serial.print("Unknown or FLAT BATTERY V < ");
      Serial.print(LiMinThreshold);
      Serial.print(" \t");

    }

    Serial.print("\tV=");
    Serial.print(pCtx->batteryVoltage); // print voltage value
    Serial.println("mV");
  }

  // after all the GPIOs are init-ed to their correct state, we can start listening for wire transfers.
  Wire.begin(I2C_DEVICEID);     // join i2c bus with an address above #8
  Wire.onReceive(receiveEvent); // for incoming data TO slave
  Wire.onRequest(requestEvent); // register event to trigger outgoing data FROM slave

  dataTmrID  = timer.setInterval(1000, sensorDataSend);  //timer will run every sec 
  calibTmrID = timer.setInterval(DEBUG == 3 ? 10000 : 60000, self_calibrate); //timer will run every minute 
  ledTimerID = timer.setInterval(1000, ledDisplay, time_now);
  compTimerID = timer.setInterval(1000, resistor_heat_comp);

  Serial.println("-------------------------------------------------------------");
}
