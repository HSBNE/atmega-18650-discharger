/*
  esp32_wifi_bridge.ino creates either a WiFi AP or connects to a WiFi network
  where it will transmit via OSC (UDP:9000) to a set hot (need a better config for this)
  where it can be processed, logged and analysed later on for battery sorting for large 
  power walls.
   
  Work in progress. Combination of Hassio, influx DB, node-red, atmega1280 and esp32 to create 
  a full discharge, record, barcode scan measurement system with minimal measurement error given
  the somewhat sub-par ardunio boards :)

  Include your WiFi AP or client details in "wifi.ssid.h" as:

    // Client WiFi details
    const char *wifi_cli_ssid 		  = "<Client Wifi SSID to connect to>";
    const char *wifi_cli_password 	= "<Client WiFi Pasword>";
    const char *wifi_cli_hostname 	= "<Your chosen hostname>";

    // Access Point WiFi details
    const char *wifi_ap_ssid 		    = "<WiFi AP SSID name for other clients to connect to>";
    const char *wifi_ap_password 	  = "<WiFi AP password>";

    // Defailt IP address and ports to transmit data to
    const IPAddress outIp(XX.XX.XX.XX);         
    const unsigned int outPort = 9000;     
    // local port to listen for OSC packets (Calibration packets and commands) 
    const unsigned int localPort = 8888;        

  Created for arduino-esp32 on Feb 2020
  by ArakniD modifed from Buzz code
*/

// CODE WORKS IN CONJUNCTION WITH atmega1280_15_socket_slave and configured to read their slave addresses
// and offer up the slots for sampling
#include <WiFi.h>
#include <WiFiClient.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <SimpleTimer.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiUdp.h>
#include <NTPClient.h>

int DEBUG = 0; // 0 = less, 1 or higher = more

#include <Wire.h>
#define I2CAddressESPWifi 8

int connected = 0;

// I2C packet......
typedef struct sensorData_t {
  uint8_t     Header;
  uint8_t     DeviceID;   // device ID.
  uint16_t    BatteryVoltage;
  uint8_t     Running: 1; // lowest bit, compatile with full byte
  uint8_t     Complete: 1;
  uint8_t     Fault: 1;
  uint8_t     Idle: 1;
  uint16_t    Current;
  uint16_t    CurrentTotal;
  uint16_t    RunTime;
  uint8_t     socket;
} __attribute__ ((packed));
#define PACKET_SIZE sizeof(sensorData_t)

typedef struct sensorData_v2_t {
  sensorData_t  v1;
  uint16_t  Power;
  uint16_t  PowerTotal;
  uint16_t  SocketLoss;
  uint32_t  timeStamp;
  uint32_t  sampleTotal;
} __attribute__ ((packed));
#define PACKET_SIZE_V2 sizeof(sensorData_v2_t)

// as bytes.....
typedef union I2C_Packet_t {
  sensorData_t    data;
  sensorData_v2_t data_v2;
  byte            I2CPacket[sizeof(sizeof(sensorData_v2_t))];
} __attribute__ ((packed));
// a single incoming packet....
I2C_Packet_t batteryinfo;


typedef struct calibData_t {
  uint8_t      cmd;
  uint8_t      pos;
  uint16_t     data;
} __attribute__ ((packed));

typedef struct setTime_t {
  uint8_t      cmd;
  uint32_t     time;
} __attribute__ ((packed));

typedef struct calibrationData_t {
  uint8_t   position;
  uint16_t  data;
} __attribute__ ((packed));

typedef struct calibrationReq_t {
  uint32_t  hdr;
  uint8_t   slot;
  uint16_t  data;
}  __attribute__ ((packed));

typedef struct timeReq_t {
  uint32_t  hdr;
  uint32_t  time;
}  __attribute__ ((packed));

typedef enum eCmdHdr {
  eCmdSetCalibration = 0,
  eCmdSetDateTime
} ;

typedef struct ds1307cfg_t {
  uint8_t   address;
  uint32_t  time;
  uint8_t   alarm0;
  uint8_t   alarm1;
  uint8_t   alarm2;
  uint8_t   control;
  uint8_t   status;
} __attribute__ ((packed));

WiFiUDP udp;
WiFiUDP ntpUDP;
bool timeSyncSentToSlaves = false;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

SimpleTimer  timer;
int dataTmrID;
int wifiTmrID;
int ntpTmrID;

typedef struct slotPosition_t {
  int deviceId;
  int position;
  int slot;
};

//===================================================================
// Configuration in headers attached

// The slots are configured in the slave_slots.h file
#include "slave_slots.h"

// WiFi details are in local header file NOT uploaded to GitHub
#include "wifi.ssid.h"

//===================================================================

void send_OSC_msg( const slotPosition_t * pSlot, I2C_Packet_t * pData ) {
  static char path[24];
  static char data[24];
  static OSCBundle bndl;

  bndl.empty();

  sprintf(path, "/%d/volts", pSlot->slot);
  sprintf(data, "%0000dmV", pData->data.BatteryVoltage);
  bndl.add(path).add(data);

  sprintf(path, "/%d/mah", pSlot->slot);
  sprintf(data, "%0000dmA", pData->data.CurrentTotal);
  bndl.add(path).add(data);

  sprintf(path, "/%d/voltage", pSlot->slot);
  bndl.add(path).add((float)pData->data.BatteryVoltage);

  sprintf(path, "/%d/current", pSlot->slot);
  bndl.add(path).add((float)pData->data.Current);

  sprintf(path, "/%d/currentTot", pSlot->slot);
  bndl.add(path).add((float)pData->data.CurrentTotal);

  sprintf(path, "/%d/runtime", pSlot->slot);
  bndl.add(path).add((float)pData->data.RunTime);

  sprintf(path, "/%d/running", pSlot->slot);
  bndl.add(path).add((float)pData->data.Running);

  // New fields for later firmware
  if (pData->data.Header >= 3) {
    sprintf(path, "/%d/power", pSlot->slot);
    bndl.add(path).add((float)pData->data_v2.Power);

    sprintf(path, "/%d/powerTot", pSlot->slot);
    bndl.add(path).add((float)pData->data_v2.PowerTotal);

    sprintf(path, "/%d/socketloss", pSlot->slot);
    bndl.add(path).add((float)pData->data_v2.SocketLoss);

    sprintf(path, "/%d/complete", pSlot->slot);
    bndl.add(path).add((float)pData->data.Complete);

    sprintf(path, "/%d/fault", pSlot->slot);
    bndl.add(path).add((float)pData->data.Fault);

    sprintf(path, "/%d/idle", pSlot->slot);
    bndl.add(path).add((float)pData->data.Idle);
  }

  if (pData->data.Header >= 4) {
    sprintf(path, "/%d/timestamp", pSlot->slot);
    bndl.add(path).add((float)pData->data_v2.timeStamp);

    sprintf(path, "/%d/sampleTotal", pSlot->slot);
    bndl.add(path).add((float)pData->data_v2.sampleTotal);
  }

  static bool connected_reported = false;

  if (connected > 0) {
    connected_reported = false;
    if (DEBUG > 0) {
      Serial.printf("send_OSC_ms DeviceID:%d pos:%d slot:%d time:%d\n",
                    pData->data.DeviceID, pSlot->position, pSlot->slot, pData->data.RunTime );
    }
    
    if (!udp.beginPacket(outIp, outPort)) {
      Serial.print("!!!! udp.beginPacket(outIp, outPort) FAIL ");
    } else {
      bndl.send(udp);
      if (!udp.endPacket() && DEBUG > 0) {
        Serial.println("!!!! udp.endPacket() FAIL ");
      }
    }
  } else if (connected_reported == false) {
    Serial.printf("send_OSC_ms DeviceID:%d pos:%d slot:%d No Clients\n",
                    pData->data.DeviceID, pSlot->position, pSlot->slot );
    connected_reported = true;
  }

  bndl.empty();
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    static char gatewayName[24];

    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          timer.disable(wifiTmrID);
          Serial.println("--=={ STA_CONNECTED }==--");
          Serial.print("\tIp:");
          Serial.println(WiFi.localIP());
          //case SYSTEM_EVENT_AP_STACONNECTED:
          // Starr the UDP session
          if ( connected++ == 0 ) {
            udp.begin(localPort);
            Serial.print("WiFi Sending Started");
            digitalWrite(5, LOW);
            sprintf(gatewayName, "%d.%d.%d.%d", WiFi.gatewayIP()[0],WiFi.gatewayIP()[1],WiFi.gatewayIP()[2],WiFi.gatewayIP()[3]);
            timeClient.setPoolServerName((const char *)gatewayName);
            timeClient.begin();
            // Update the Time Sync on startup
            update_rtc();
          }
          break;
      //case SYSTEM_EVENT_AP_STADISCONNECTED:
      case SYSTEM_EVENT_STA_DISCONNECTED:
          if (connected > 0) {
            if (--connected == 0) {
              Serial.println("WiFi lost connection, no more sends");
              udp.stop();
              digitalWrite(5, HIGH);
              timer.enable(wifiTmrID);
              timeClient.end();
            }
          }
          break;
    }
}

void update_rtc(void) {
  if (timeClient.update())
  {
    transmit_timeSync( timeClient.getEpochTime());
  }
}

void transmit_timeSync(uint32_t timestamp) {
  setTime_t data = { eCmdSetDateTime, timestamp };
  ds1307cfg_t cfg = { 0, timestamp, 0, 0, 0, 0x44, 0x00 };

  Serial.println();

  Serial.println("--=={ Time Sync }==--");

  Serial.print("--=={ Timestamp: ");
  Serial.print(timestamp);
  Serial.println(" }==--");

  // Configure the time for the boards
  Wire.beginTransmission(2 + I2CAddressESPWifi);
  Wire.write((const uint8_t *)&data, sizeof(setTime_t));
  Wire.endTransmission(true);

  Serial.println("-{ Board #2 Written }-");
  delay(5);

  // Configure the time for the boards
  Wire.beginTransmission(3 + I2CAddressESPWifi);
  Wire.write((const uint8_t *)&data, sizeof(setTime_t));
  Wire.endTransmission(true);

  Serial.println("-{ Board #3 Written }-");
  delay(5);

  // Write to the DS1371 RTC chip, starts the timer
  Wire.beginTransmission(0x68);
  Wire.write((const uint8_t *)&cfg, sizeof(ds1307cfg_t));
  Wire.endTransmission(true);

  Serial.println("-{ DS1371 Written }-");
  delay(5);
}

void reconnect_wifi(void) {
   WiFi.begin( );
}

void checkForCalibrationCmd() {
  udp.parsePacket();

  if ( udp.available() == sizeof(calibrationReq_t )) {
     calibrationReq_t calib;

     udp.read((char *) &calib, sizeof(calibrationReq_t));

     if ( calib.hdr == 0xDEADBEEF && calib.slot <= POSITIONS ) {
      const slotPosition_t * pSlot = &slotPos[calib.slot-1];
      
      calibData_t data = { eCmdSetCalibration, (pSlot->position | 0x80), calib.data };

      // This is a voltage calibration.. not current.. mock position to zero
      if (calib.data > 4800 && calib.data <= 5200) {
        data.pos = 0x80;
      }

      Serial.println("!!!!!!!!!!!!!Calibration");
      
      // send the client a "position" or "offset" request  ( typically 0-3 on a nano, or 0-16 on a mega )
      Wire.beginTransmission(pSlot->deviceId + I2CAddressESPWifi);
      Wire.write((const uint8_t *)&data, sizeof(calibData_t));
      Wire.endTransmission(true);
    }
  } else if (udp.available() == sizeof(timeReq_t ) ) {
    timeReq_t time;

    udp.read((char *) &time, sizeof(timeReq_t));
    if (time.hdr == 0xDE0000AD) {
      transmit_timeSync(time.time);
    }

  } else {
    udp.flush();
  }
}

void setup() {
  Wire.begin(21, 22);       // GPIO4,GPIO5 = ~D2,~D1 => SDA,SCL -> join i2c bus, for ESP.
  Serial.begin(250000);  // start serial for output

  if ( false ) {	  
	  WiFi.mode(WIFI_AP); // without this softap can't do multicast udp.
	  WiFi.softAP(wifi_ap_ssid, wifi_ap_password);
  } else {
	WiFi.setHostname(wifi_cli_hostname);

	WiFi.begin(wifi_cli_ssid, wifi_cli_password);
  }
  
  digitalWrite(5, LOW);
  pinMode(5, OUTPUT);

  //register event handler
  WiFi.onEvent(WiFiEvent);

  // start UDP server
  Serial.println("Starting UDP");
  Serial.print("Local port: ");
  Serial.println(localPort);

  ds1307cfg_t cfg = { 0, 0, 0, 0, 0, 0x80, 0x80 };

  // Write to the DS1371 RTC chip, starts the timer
  Wire.beginTransmission(0x68);
  Wire.write((const uint8_t *)&cfg, sizeof(ds1307cfg_t));
  Wire.endTransmission(true);

  Serial.println("-{ DS1371 Stopped - Wait for TimeSync }-");

  dataTmrID  = timer.setInterval(2000 / POSITIONS, sensorDataSend);  //timer will run every sec 
  wifiTmrID  = timer.setInterval(15000 , reconnect_wifi);  //timer will run every sec 
  ntpTmrID   = timer.setInterval(3600000, update_rtc);  //timer will run every sec 

  timer.disable(wifiTmrID);
}

void loop() {
  timer.run();
  checkForCalibrationCmd();
}

int currentSlotPos = 0;

void sensorDataSend() {
  const slotPosition_t * pSlot = &slotPos[currentSlotPos];
  int slotNum = currentSlotPos + 1;
  int i2caddr = pSlot->deviceId + I2CAddressESPWifi;
  int POS = pSlot->position;
  uint8_t d = 0;

  memset( batteryinfo.I2CPacket, 0, sizeof(batteryinfo.I2CPacket) );

  // send the client a "position" or "offset" request  ( typically 0-3 on a nano, or 0-16 on a mega )
  Wire.beginTransmission(i2caddr);
  Wire.write(POS);
  Wire.endTransmission(true);

  delay(2); // a delay
  
  // request   bytes from slave device starting with offset of 8, and going up from there 8-18
  Wire.requestFrom(i2caddr, PACKET_SIZE_V2);

  // drag data over i2c and put into byteArray
  while (Wire.available()) { // slave may send less than requested
    batteryinfo.I2CPacket[d++] = Wire.read(); // receive a byte
  }

  switch (d) {
    case 0:
      if (DEBUG > 2 )  Serial.println("\tunable to get ANY bytes, going to next device");
      break;
    case PACKET_SIZE:
      if (DEBUG > 2 )  Serial.println("*****LEGACY");
      break;
    case PACKET_SIZE_V2:
      if (DEBUG > 2 ) {
        if (batteryinfo.data.Header == 2)
          Serial.println("**===LEGACY");
        else
          Serial.println("*****NEW");
      }
      break;
    default:
      if (DEBUG > 2 ) { Serial.print("\tNFI: "); Serial.println(d); }
      break;
  }

  if (DEBUG > 1 ) Serial.printf("\tProbing for a device (%d) with i2c address(%d) and offset(%d).\n", pSlot->deviceId, i2caddr, POS);

  // quick validate that the device we asked for is the one claiming to respond, otherwise ignore it.
  // TODO use a real packet checksum if you like.
  if ((pSlot->deviceId != batteryinfo.data.DeviceID ) or (batteryinfo.data.DeviceID == 0) ) {
    if (DEBUG > 1 ) Serial.println("\tpacket DeviceID error, dropped.");
  } else if ((POS != batteryinfo.data.socket) or (batteryinfo.data.socket == 0) ) {
    if (DEBUG > 1 ) Serial.printf("\tpacket reportedpos error, dropped POS:%d  reportedpos:%d\n", POS, batteryinfo.data.socket);
  } else {
    if (DEBUG > 0 ) Serial.printf("\tdeviceid:%d - mV:%d ma:%d - cumulative:%d - running:%d \n"
                                  , batteryinfo.data.socket, batteryinfo.data.BatteryVoltage, batteryinfo.data.Current, batteryinfo.data.CurrentTotal, batteryinfo.data.RunTime);

    // and now finally send it. !
    send_OSC_msg(pSlot, &batteryinfo );  // pulls from batteryinfo and clientips
  }
    
  currentSlotPos = ++currentSlotPos % POSITIONS;
}
