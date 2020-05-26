// with I2C slave support!  see other sketch called esp8266_wifi_bridge.ino for master, with OSC etc. 
#include <Wire.h>

// 1 -> whatever
#define DEVICEID 2
// I2C device IDs start at 8 and go up.
#define I2C_DEVICEID DEVICEID+8

// earlier code had only 3 on Nano, etc.
#define POSITIONS 16

int DEBUG = 1;

// battery constant characteristics
float LiMinThreshold = 2700; //3900; //2700; // 2700 = Lithium Minimal Voltage for load removal, 3900 = minimum needed for re-charger to kick in.
float LiMaxThreshold = 4220; // Lithium Max Voltage for load removal
int power_resistor_value = 2.2; // needs to be in the range 1.8-2.5 ohms. best is 2 or 2.2 ohms, typically approx 10W  ( eg, 2x1ohm 5w in series will do , or a 2.2ohm 10W) 

// 5V NANO ADC to mV VOLTAGE FACTOR is 4.887
// 5V MEGA1280 ADC to mV VOLTAGE FACTOR is 13.53
#define ADC 4.887
//#define ADC 130.1

int i;

//16 way! - the most important thing is 16 available ADCs, and 16 digital IO for mosfets, as a bonus, we have 16 digital io for LEDs too. and still spares after that. :
// constants
unsigned int sensor1Pins[POSITIONS] = { A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15 };
//int sensor2Pins[] = { 0, 4, 5 }; // we don't have 3 separate analog pins avail here because we use I2C on A4 and A5.
unsigned int ledPins[POSITIONS] = { 22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37 };
unsigned int FETPins[POSITIONS] = { 4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19 };  // don't use 20 or 21 for IO, as they are for I2C comms on a 2560.
unsigned int SPKPins[POSITIONS] = {  3, 3 , 3,3,3,3,3,3,3,3,3,3,3,3,3,3 };
// variables
unsigned int sensor1Values[POSITIONS] = { 0 , 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 , 0 };
//int sensor2Values[] = { 0 , 0 , 0 };
unsigned int BatVoltages[POSITIONS] = { 5000, 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000, 5000};
unsigned int FetVoltages[POSITIONS] = { 40,40,40,40,40,40,40,40,40,40,40,40,40,40,40, 40}; // with enough spare ADCs we can calculate these, but for now we know its close to 40mV most of the time
unsigned int MilliAmps[POSITIONS] = { 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0};    // these ints are 16bit values
unsigned long TotalCurrents[POSITIONS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // LONG, as we go over 65k

boolean batterypresents[POSITIONS] = { false, false,false,false,false,false,false,false,false,false,false,false,false,false,false, false};
boolean dones[POSITIONS] =           { false, false,false,false,false,false,false,false,false,false,false,false,false,false,false, false} ;
boolean ledstates[POSITIONS] =       { false, false,false,false,false,false,false,false,false,false,false,false,false,false,false, false} ;
unsigned long PrevMilliss[POSITIONS]  =      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long MillisPasseds[POSITIONS] =     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long time_since_starts[POSITIONS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
unsigned long time_ends[POSITIONS] =         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 

// I2C packet...... 
typedef struct sensorData_t{
  byte stx;
  byte DeviceID;   // device ID.   
  byte batvoltage1_low;
  byte batvoltage1_hi;
  byte Running1;
  byte MilliAmps1_low;
  byte MilliAmps1_hi;
  byte TotalCurrents1_low;
  byte TotalCurrents1_hi;
  byte time_ends1_low;
  byte time_ends1_hi;
  byte etx;
};
#define PACKET_SIZE 12  // can NEVER exceeed 32, hardware limit.
//#define PACKET_SIZE sizeof(sensorData_t)

// as bytes.....
typedef union I2C_Packet_t{
sensorData_t sensor;
byte I2CPacket[sizeof(sensorData_t)];
};
I2C_Packet_t batteryinfo;


void shortbeep(int pos){
            digitalWrite(SPKPins[pos], HIGH);
            delay(1);
            digitalWrite(SPKPins[pos], LOW);
}

byte REQUESTEDPOS = 0x00;
void receiveEvent(int bytes)
{
  if(Wire.available() != 0)
  {
    for(int i = 0; i< bytes; i++)
    {
      REQUESTEDPOS = Wire.read();
      if (DEBUG > 0 ) Serial.print("Received: ");
      if (DEBUG > 0 ) Serial.print(REQUESTEDPOS, HEX);
      if (DEBUG > 0 ) Serial.print("\n");
    }
  }
}

// just like analogRead only it does 2 slow reads and makes sure they are within 10% of each other
int debouncedAnalogRead(int pin){ 
  int trydata[] = { 0,100 } ;
  //for(int _try = 0; _try < 2 ; _try++ ) { 
      trydata[0] = analogRead(pin);
   //delay(50);
      trydata[1] = analogRead(pin);
   //delay(50);
  //}
  // ADC values are out of 1024, so 10% is about 100
  while (abs(trydata[0]-trydata[1]) > 10 ) {
    //Serial.println("poor ADC value");
    //Serial.println(abs(trydata[0]-trydata[1]));
   delay(50);

      trydata[0] = analogRead(pin);
   //delay(50);
      trydata[1] = analogRead(pin);
   //delay(50);
    
//    Serial.println
  }
  return trydata[1]; // second try is probably better. 
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {

    if ((REQUESTEDPOS < 0) or ( REQUESTEDPOS > POSITIONS )) { REQUESTEDPOS = 0; } // if not given a decent offset, assume the first one.

    // turn human readable 1-based offset back to 0-based array offset:
    int __REQUESTEDPOS = REQUESTEDPOS-1;

    // fill the packet 
    batteryinfo.sensor.stx=2; // just becasue, we don't use this other than to tel that a packet is not all zeros
    batteryinfo.sensor.DeviceID=DEVICEID; // human-readable of DeviceID is passed back too.
    batteryinfo.sensor.batvoltage1_low = (byte)BatVoltages[__REQUESTEDPOS];
    batteryinfo.sensor.batvoltage1_hi = BatVoltages[__REQUESTEDPOS]  >> 8;
    batteryinfo.sensor.Running1= dones[__REQUESTEDPOS]?0:1;
    batteryinfo.sensor.MilliAmps1_low = (byte)MilliAmps[__REQUESTEDPOS];
    batteryinfo.sensor.MilliAmps1_hi = MilliAmps[__REQUESTEDPOS]  >> 8;
    batteryinfo.sensor.TotalCurrents1_low = (byte)(TotalCurrents[__REQUESTEDPOS]/1000);
    batteryinfo.sensor.TotalCurrents1_hi = (TotalCurrents[__REQUESTEDPOS]/1000)  >> 8;
    batteryinfo.sensor.time_ends1_low = (byte)time_ends[__REQUESTEDPOS];
    batteryinfo.sensor.time_ends1_hi = time_ends[__REQUESTEDPOS]  >> 8;
    batteryinfo.sensor.etx=REQUESTEDPOS; // pass back the human-readable of REQUESTEDPOS

    // send the packet.
    Wire.write(batteryinfo.I2CPacket, PACKET_SIZE);

  
}

void cycle_leds(){
  Serial.println("Cycling LEDs");

   for( int pos = 0 ; pos < POSITIONS ; pos++ ) { 
     digitalWrite(ledPins[pos], HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(200);              // wait for a second
      digitalWrite(ledPins[pos], LOW);    // turn the LED off by making the voltage LOW
      delay(200);              // wait for a second
   }
  Serial.println("LEDs test done");
}

void setup() {
  
  Serial.begin(57600);// start serial port to send data during run to the PC
  Serial.println("Bat PWR Tester[Active]");  // Print a message to the //lcd.
  Serial.println("Detecting Bat .... please put one in now if not already...."); // print voltage value
  Serial.println("                        ");
  Serial.println(PACKET_SIZE);

  // init all the IO and pullups and lEDs etc. 
  for( int pos = 0 ; pos < POSITIONS ; pos++ ) { 
    
      pinMode(ledPins[pos], OUTPUT);//activation led and enable for FET
      pinMode(FETPins[pos], OUTPUT);//activation led and enable for FET
      pinMode(SPKPins[pos], OUTPUT);//activation led and enable for FET
   
      //elay(200);  //s[pos]
      digitalWrite(ledPins[pos], HIGH);   // set the LED on
      digitalWrite(FETPins[pos], HIGH);   // set the LED on
      digitalWrite(ledPins[pos], LOW);   // set the LED off
      digitalWrite(FETPins[pos], LOW);   // set the LED off

      // read ADC
      sensor1Values[pos] = debouncedAnalogRead(sensor1Pins[pos]);   // read the value from the sensor:
  
      // Detecting battery type
      BatVoltages[pos] = sensor1Values[pos]*ADC;
      
      Serial.print("\tDevice=");

      Serial.print(sensor1Values[pos]); // raw ADC 
      Serial.print("\t");
      
      Serial.print(pos+1); // print displayable position, starting from .1
      Serial.print("\t");
                 
      if (BatVoltages[pos] > LiMaxThreshold){
        Serial.print("Warning high-V! V > 4.5 \t");
        dones[pos] = true;
        batterypresents[pos] = true;
      }
    
      if (BatVoltages[pos] > LiMinThreshold){
        Serial.print("Type:Li-Ion Bat. \t");
        batterypresents[pos] = true;
       }
    
      if (BatVoltages[pos] < 2){
        Serial.print("Unknown or FLAT BATTERY V<2 \t");
        dones[pos] = true;
        batterypresents[pos] = false;
      }
      
      Serial.print("\tV=");
      Serial.print(sensor1Values[pos]*ADC); // print voltage value
      Serial.println("mV");

  PrevMilliss[pos]  = millis();
  time_since_starts[pos] = millis();

  } 

  cycle_leds(); // demo/tester

  // after all the GPIOs are init-ed to their correct state, we can start listening for wire transfers.   
  Wire.begin(I2C_DEVICEID);     // join i2c bus with an address above #8
  Wire.onReceive(receiveEvent); // for incoming data TO slave 
  Wire.onRequest(requestEvent); // register event to trigger outgoing data FROM slave


  //delay(1000);
  Serial.println("-------------------------------------------------------------");
  
  shortbeep(0);
}


void loop() {
      Serial.println("-------------------------------------------------------------");
            delay(1000);

    for( int pos = 0 ; pos < POSITIONS ; pos++ ) { 
  
          if (BatVoltages[pos] > LiMinThreshold && !dones[pos]) {
            digitalWrite(ledPins[pos], HIGH);   // set the LED on
            digitalWrite(FETPins[pos], HIGH);   // set the LED on
            sensor1Values[pos] = debouncedAnalogRead(sensor1Pins[pos]);   // read the value from the sensor:
            BatVoltages[pos] = (sensor1Values[pos]*ADC);
            //sensor2Values[pos] = analogRead(sensor2Pins[pos]);   // read the value from the FET:
            //FetVoltages[pos] = (sensor2Values[pos]*ADC);  // this is the way to calculate it if you don't use the default value

            Serial.print("\tDevice=");
            Serial.print(DEVICEID); // print voltage value
            Serial.print(".");
            Serial.print(pos+1); // print human readable starting from 1
            
 
            Serial.print("\t[RUNNING] BattmV=");
            Serial.print(BatVoltages[pos]); // print voltage value
            
   
            MilliAmps[pos] = (BatVoltages[pos]-FetVoltages[pos])/power_resistor_value; 
            if ( MilliAmps[pos] < 0 ) { MilliAmps[pos] = 0; } // dissallow neg values
            if ( MilliAmps[pos] > 32000 ) { MilliAmps[pos] = 0; } // dissallow neg values
            Serial.print("\tmilliAmps=");
            Serial.print(MilliAmps[pos]); // print value        
            
            TotalCurrents[pos]=TotalCurrents[pos]+MillisPasseds[pos]/1000*MilliAmps[pos]/3.6; 
            Serial.print("  \tI=");
            Serial.print(TotalCurrents[pos]/1000);
            Serial.print("mAH  ");

            MillisPasseds[pos] = millis()- PrevMilliss[pos] ;
            PrevMilliss[pos] = millis();
    
            Serial.print("\tSecs:");    // prints a tab
            time_ends[pos] = int((millis()-time_since_starts[pos])/1000);
            Serial.print(time_ends[pos]);
 
            Serial.print("\t FETmV=");
            Serial.print(FetVoltages[pos]); // print value
 
            Serial.println();
          }
          else
          {
            // if we've "finished" ( moving to the done state ) , beep once.
            if (dones[pos] == false ) {
              shortbeep(pos);
              //time_ends[pos] = int((millis()-time_since_starts[pos])/1000);
            }
            dones[pos]=true;

            
            digitalWrite(ledPins[pos], LOW);   // set the LED off - stop loading
            digitalWrite(FETPins[pos], LOW);   // set the LED off - stop loading

            Serial.print("\tDevice=");
            Serial.print(DEVICEID); // print voltage value
            Serial.print(".");
            Serial.print(pos+1); // print voltage value
            Serial.print("\t[ DONE  ] ");  // Print a message to the //lcd.
    
            sensor1Values[pos] = debouncedAnalogRead(sensor1Pins[pos]);   // read the value from the sensor:
            BatVoltages[pos] = (sensor1Values[pos]*ADC);
    
            Serial.print("BattmV=");
            Serial.print(BatVoltages[pos]); // print voltage value

            Serial.print("\tmilliAmps=");
            Serial.print(MilliAmps[pos]); // print value        
 
            Serial.print("  \tI=");
            Serial.print(TotalCurrents[pos]/1000);    
            Serial.print("mAH  ");

            Serial.print("\tSecs:");    // prints a tab
            Serial.print(time_ends[pos]);

  
            // if battery is still in holder, but we are done with it... flash LED of appropriate slot..
            if ( batterypresents[pos] ) { 
                  if ( ledstates[pos] == LOW ) { 
                      digitalWrite(ledPins[pos], HIGH);
                      ledstates[pos] = HIGH;
                  } else { 
                      digitalWrite(ledPins[pos], LOW);
                      ledstates[pos] = LOW; 
                  }
                  //delay(100);
            } 

            bool restarting = false;
            
            // if the battery voltage is suddenly LOWER than 1v, it was clearly taken OUT. 
            if (( batterypresents[pos] == true) and (BatVoltages[pos] < 1000 )  ) { 
              batterypresents[pos]  = false;
              restarting = true;
              Serial.print("\tLOWER ");
            }
            // if the battery voltage is suddenly HIGHER than 1v when it wasn't before, we've clearly put a battery IN.
            if ( ( batterypresents[pos] == false) and ( BatVoltages[pos] > 1000 ) ) { 
              batterypresents[pos] = true; 
              restarting = true;
              Serial.print("\tHIGHER ");

            }

            // we restart based on HIGH or LOW voltage change....
            if (restarting == true ) { 
                Serial.print("\tDetected ADD/REMOVE .. restarting! "); 
                TotalCurrents[pos] = 0;
                dones[pos] = false;   
                time_since_starts[pos] = millis();
                shortbeep(pos);  // if we've "started" ( moving to the discharging state ) , beep once.
                time_ends[pos] = 0;
            }
            Serial.println();
          }
    }
}
