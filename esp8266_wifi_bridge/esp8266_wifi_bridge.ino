// we install this library and then dont use it: https://github.com/tzapu/WiFiManager
// very helpful: see also http://www.elinistech.com/images/esp8266/The-ESP8266-Book-September-2015.pdf
// and comments at : https://github.com/esp8266/Arduino/issues/471
// and https://github.com/sandeepmistry/esp8266-OSC/blob/master/examples/UDPSendBundle/UDPSendBundle.ino
// NOTE:   code tested using arduino IDE 1.6.7 AND esp8266 board plugin version 2.0.0  ( which is def not the latest, but will do ) 

//CODE WORKS IN CONJUNCTION WITH _18650_discharger_three_way programmed into upto 10x Arduino Nanos used for smart-discharger/s


#include <ESP8266WiFi.h>

#include <OSCMessage.h>
#include <OSCBundle.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
//#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <WiFiUdp.h>


int DEBUG = 3; // 0 = less, 1 or higher = more

// BUZZ MASTER I2C on ESP8266  - WILL NOT COMPILE FOR AVR as-is, make sure your target is an esp8266 board.


#include <Wire.h>
#define I2CAddressESPWifi 8


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
#define PACKET_SIZE 12
//#define PACKET_SIZE sizeof(sensorData_t)

// as bytes.....
typedef union I2C_Packet_t{
sensorData_t sensor;
byte I2CPacket[sizeof(sensorData_t)];
};
// a single incoming packet....
I2C_Packet_t batteryinfo;


  //the message wants an OSC address as first argument
  //OSCMessage msg("/analog/0");
  //msg.add((int32_t)analogRead(0));

int NeedRefresh = 0;

extern "C" {
  #include "user_interface.h"
//  #include "ip_addr.h"
//  #include "mem.h"

  // callback context should NOT have blocking ( Serial.println ) calls. bad. 
  void _onWiFiEvent(System_Event_t *event){
    NeedRefresh = 1;
    if(event->event == EVENT_SOFTAPMODE_STACONNECTED){
      Event_SoftAPMode_StaConnected_t *data = (Event_SoftAPMode_StaConnected_t *)(&event->event_info.sta_connected);
      //Serial.printf("Station Connected: id: %d, mac: " MACSTR "\n", data->aid, MAC2STR(data->mac));
    } else if(event->event == EVENT_SOFTAPMODE_STADISCONNECTED){
      Event_SoftAPMode_StaDisconnected_t *data = (Event_SoftAPMode_StaDisconnected_t *)(&event->event_info.sta_disconnected);
      //Serial.printf("Station Disconnected: id: %d, mac: " MACSTR "\n", data->aid, MAC2STR(data->mac));
    }
  }
}
// the connected CLIENT IP address/s, if any. ( populated by count_stations), and 
// the target for where all teh UDP OSC messages go. 
IPAddress clientips[4];

//const IPAddress outIp(192,168,192,28);        // remote IP of your computer
const unsigned int outPort = 9000;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

WiFiUDP udp;


// count devices that have fully DHCPd and have IP adresses:
int count_stations(){
    struct station_info *stat_info;
    struct ip_addr *ipaddr;
    //IPAddress ip; see clientips
    int retries = 0;
    int count = 0;
    stat_info = NULL;
        
    // get a non-null stat_info
    while ((stat_info == NULL) and ( retries < 10)) {  // wait for the dhcp to finish and get an actual IP.
      if (DEBUG >0 ) Serial.print("X");
      stat_info = wifi_softap_get_station_info();
      delay(500);
      retries++;
    }
    // iterate over the stat_info to see how many we have..? 
    while (stat_info != NULL) {
      ipaddr = &stat_info->ip;
      clientips[count] = ipaddr->addr;
      //ip=ipaddr->addr;
      if (DEBUG >0 ) Serial.println(clientips[count].toString());
      stat_info = STAILQ_NEXT(stat_info, next);
      count++;
    }
    int zerofill = 0;
    zerofill = count;
    while(zerofill < 4){
      clientips[count][0] = 0; // 4 octets of an IP.
      clientips[count][1] = 0;
      clientips[count][2] = 0;
      clientips[count][3] = 0;
      zerofill++;
    }
    //Serial.printf("count: %d\n",count);
    return count;
}

/* 
 *  this works, but it relies on the wifi_softap_get_station_num() function which returns BEFORE dhcp has issues IPs to all connected.
// this should only be called "moments" after a System_Event_t wifi connect or disconnect event has occurrred on the AP.
// and it does not wait for long...
void _oldwait_for_new_connection(){
  static unsigned int last_time_we_were_called_we_had_how_many_devices = 0;  // safe to assume zero clients on startup, but not at any other time.  :-) 
  unsigned int softap_stations_cnt = 255;
  int retries = 0;
  Serial.printf("was clients: %d   \n", last_time_we_were_called_we_had_how_many_devices);

  //  retries limit
  softap_stations_cnt = wifi_softap_get_station_num();
  while ((softap_stations_cnt == last_time_we_were_called_we_had_how_many_devices) and (retries < 5)) { // this maxes out at around 5 seconds
    softap_stations_cnt = wifi_softap_get_station_num();
    retries++;
    delay(1000);
    //Serial.printf("retries: %d %d\n",retries,softap_stations_cnt);
    Serial.print(".");
  }
  last_time_we_were_called_we_had_how_many_devices = softap_stations_cnt; // new count
  Serial.printf("now clients: %d    \n", last_time_we_were_called_we_had_how_many_devices);
}
*/

// this should only be called "moments" after a System_Event_t wifi connect or disconnect event has occurrred on the AP.
// and it does not wait for long...
void wait_for_new_connection(){
  static unsigned int last_time_we_were_called_we_had_how_many_devices = 0;  // safe to assume zero clients on startup, but not at any other time.  :-) 
  unsigned int softap_stations_cnt = 255;
  int retries = 0;
  if (DEBUG >0 ) Serial.printf("conencted clients: %d   \n", last_time_we_were_called_we_had_how_many_devices);

  // retries limit .. 
  softap_stations_cnt = count_stations();
  while ((softap_stations_cnt == last_time_we_were_called_we_had_how_many_devices) and (retries < 5)) { // this maxes out at around 5 seconds ( 100x100ms)
    softap_stations_cnt = count_stations();
    retries++;
    delay(1000);
    //Serial.printf("retries: %d %d\n",retries,softap_stations_cnt);
    if (DEBUG >0 ) Serial.print(".");
  }
  last_time_we_were_called_we_had_how_many_devices = softap_stations_cnt; // new count
  if (DEBUG >0 ) Serial.printf("conencted clients: %d    \n", last_time_we_were_called_we_had_how_many_devices);
}

//The following code will convert an int to a string with up to nine decimals.
/* char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}*/

int convert_to_slot(int DeviceID,int pos){
     // we are passed in a device ID and a internal position of that device, we turn that into a position in touch OSC in the 1-30 range.
     //deviceid  =nano number/s 1..10  
     //pos = 1 , 2 or 3 

     //slot = 1..30 ( total avail in OSC )
 
     // so deviceid.pos          1.1,1.2,1.3, 2.1,2.2,2.3, 3.1,3.2,3.3, 10.1,
     // gives this               4.1,4.2,4.3, 5.1,5.2,5.3, 6.1,6.2,6.3, 10.2,
     // pyhsical layout          7.1,7.2,7.3, 8.1,8.2,8.3, 9.1,9.2,9.3, 10.3,

     int slot = 0;
     switch( DeviceID) {
       // MOST OF ROW 1
        case 1:
          switch(pos){
            case 1:   slot=28;  break;
            case 2:   slot=29;  break;
            case 3:   slot=30;  break;
          }
        break;
        case 2:
          switch(pos){
            case 1:   slot=25;  break; 
            case 2:   slot=26;  break;
            case 3:   slot=27;  break;
            
            case 4:   slot=21;  break;
            case 5:   slot=22;  break;
            case 6:   slot=23;  break;
            case 7:   slot=24;  break;
            
            case 8:   slot=11;  break;
            case 9:   slot=12;  break;
            case 10:   slot=13;  break;
            case 11:   slot=14;  break;
            case 12:   slot=15;  break;
            case 13:   slot=16;  break;
            case 14:   slot=17;  break;
            case 15:   slot=18;  break;
            case 16:   slot=19;  break;
            
            //case X:   slot=20;  break;
            
          }
        break;
        case 3:
          switch(pos){
            //case 1:   slot=7;  break;
            //case 2:   slot=8;  break;
            //case 3:   slot=9;  break;
          }
        break;
        // MOST OF ROW 2
        case 4:
          switch(pos){
            //case 1:  slot=11;  break;
            //case 2:  slot=12;  break;
            //case 3:  slot=13;  break;
          }
        break;
        case 5:
          switch(pos){
            //case 1:  slot=14;  break;
            //case 2:  slot=15;  break;
            //case 3:  slot=16;  break;
          }
        break;
        case 6:
          switch(pos){
            //case 1:  slot=17;  break;
            //case 2:  slot=18;  break;
            //case 3:  slot=19;  break;
          }
        break;
        // MOST OF ROW 3
       case 7:
          switch(pos){
            //case 1:  slot=21;  break;
            //case 2:  slot=22;  break;
            //case 3:  slot=23;  break;
          }
        break;
        case 8:
          switch(pos){
            //case 1:  slot=24;  break;
            //case 2:  slot=25;  break;
            //case 3:  slot=26;  break;
          }
        break;
        case 9:
          switch(pos){
            //case 1:  slot=27;  break;
            //case 2:  slot=28;  break;
            //case 3:  slot=29;  break;
          }
        break;
        // last column...
       case 10:
          switch(pos){
            //case 1:  slot=10;  break;
            //case 2:  slot=20;  break;
            //case 3:  slot=30;  break;
          }
        break;

     }

return slot;
}

void send_OSC_msg(unsigned int DeviceID,int pos,int v1, unsigned int ma1,  unsigned int c1, unsigned int t1,  unsigned int r1 ) {
  

  int slot = 0;
  slot = convert_to_slot(DeviceID,pos);
  
  if (DEBUG > 0) Serial.printf("send_OSC_ms DeviceID:%d pos:%d slot:%d time:%d\n",DeviceID,pos,slot,t1);

  static int testdata = 2000; // 2v
  
      OSCBundle bndl;
        String s;
        char slotnum[3];        
        s = String(slot);
        s.toCharArray(slotnum,3);
        char buf[10];
        
        //VS from fv1 ---------------
        char vs[] = "/1/volts  ";
        vs[8] = slotnum[0];
        vs[9] = slotnum[1];
        itoa(v1,buf,10); //INTEGER base 10 to string in millivolts:  eg: buf = "4231"
        //turn '123' into '0123'
        if (v1 < 1000 ) { buf[3] = buf[2]; buf[2] = buf[1] ;buf[1] = buf[0] ; buf[0] = '0';  }
        //turn '12' into '0012'
        else if (v1 < 100 ) { buf[3] = buf[1]; buf[2] = buf[0] ; buf[1] = '0'; buf[0] = '0';  } 
        //turn '1' into '0001'
        else if (v1 < 10 ) { buf[3] = buf[0]; buf[2] = '0';buf[1] ='0' ; buf[0] = '0';  } 
        
        buf[2] = buf[1]; // move the second digit over one, drop the third.
        buf[1] = '.';   // shove in the decimal
        buf[3] = 'v';
        buf[4] = '\0'; //"1.23v" as a terminated string
        bndl.add(vs).add(buf);

        //MS from c1 ---------------
        char ms[] = "/1/mah  ";
        ms[6] = slotnum[0];
        ms[7] = slotnum[1];
        itoa(c1, buf, 10); //INTEGER
        //Serial.println(c1);
        //Serial.println(buf);
        int offset = 0;
        if (c1 < 10000 ) offset = 4;
        if (c1 < 1000 ) offset = 3;
        if (c1 < 100 ) offset = 2;
        if (c1 < 10 ) offset = 1;
        buf[offset++] = 'm'; //"1234mAh" as a string
        buf[offset++] = 'A';  
        buf[offset++] = 'h';  
        buf[offset++] = '\0';  
        //Serial.println(buf);

        bndl.add(ms).add(buf);

        //FS from fv1 ( as per text above ) ---------------
        char fs[] = "/1/fader  ";
        fs[8] = slotnum[0];
        fs[9] = slotnum[1];
        //if ( v1 < 1000 ) {   testdata += 100 ; if (testdata > 4200 ) { testdata = 2000; } v1 = testdata; }  //NON-ZERO TEST DATA
        float fv1 = v1/1000.0;
        float sliderval =   (fv1-2)/2.2;   // scale ranges from 2-4.2v
        //Serial.println(sliderval);
        bndl.add(fs).add((float)sliderval);
        
        char gs[] = "/1/greenled  ";
        gs[11] = slotnum[0];
        gs[12] = slotnum[1];
        bndl.add(gs).add((float)r1);

        char rs[] = "/1/redled  ";
        rs[9] = slotnum[0];
        rs[10] = slotnum[1];
        bndl.add(rs).add((float)0);
        
    
      IPAddress ip;
      for ( int ipcount = 0 ; ipcount < 4 ; ipcount++ ) { // try to send to all clients on AP
          ip = clientips[ipcount];
          if (ip[0] != 0) {  // don't send to 0.0.0.0 addresses
              //Serial.print("sending to: ");
              //Serial.println(ip.toString());
    
             udp.beginPacket(ip, outPort);
            bndl.send(udp);
            udp.endPacket();
            bndl.empty();
            delay(20); // slow sending to make recieving easier.
            yield();
            
          }
       }

/*
      // just graph the first slot...
      if (slot == 1) { 
        Serial.println("slot 1");
        static int last_minute = 0; // report in once each minute to the graph the max voltage seen in that period. 
        static float highest_voltage_this_period = 0.0;

        char bbb[12];
        ftoa(bbb,highest_voltage_this_period,2); //FLOAT to 2 significant places.
        Serial.println(bbb);

        if ((t1 < 60)  and (last_minute > 0)) {  // reset last_minute when we see t1 resets
          last_minute =0;
          highest_voltage_this_period = 0;
          Serial.println("slot 1a");
        }
        

        if (fv1 > highest_voltage_this_period ) { 
          highest_voltage_this_period = fv1;
          Serial.println("slot 1b");
        }
          if (t1 > 70 ) t1 = t1%70;
          if (t1 < 10 ) t1 = 10;
 
        int mins = t1/60;
        char oscgraph[] = "/2/graph1/  ";  // ONE space = 1 character wide! 
        if (t1 > last_minute ) { 
          Serial.println("slot 1c");
          // graph highest_voltage_this_period at an offset of last_minute to the OSC chart:
          itoa(t1, buf, 10); //INTEGER
          if ((buf[0] != '\0' ) and (buf[0] != 0 )) {
            oscgraph[10] = buf[0];
            oscgraph[11] = buf[1];
          } else { 
            oscgraph[10] = ' ';
            oscgraph[11] = ' ';
          }
          oscgraph[12] = '\0'; // null
 
          // then restart the next minute
          last_minute = t1;
          highest_voltage_this_period = 0;
        }
        
 
        for ( int ipcount = 0 ; ipcount < 4 ; ipcount++ ) { // try to send to all clients on AP
          ip = clientips[ipcount];
          if (ip[0] != 0) {  // don't send to 0.0.0.0 addresses

            if (highest_voltage_this_period < 2.7 ) highest_voltage_this_period = 2.8;
            if (highest_voltage_this_period > 4.2 ) highest_voltage_this_period = 4.1;

            Serial.println(oscgraph);
            Serial.println(highest_voltage_this_period);

            //OSCMessage bb("/2/graph1/5");
            OSCMessage bb(oscgraph);
            //bb.add(oscgraph).add((float)0.3);//highest_voltage_this_period)
            //bb.add((float)3.9);
            float x = 4.2-highest_voltage_this_period;
            bb.add(x);
            udp.beginPacket(ip, outPort);
            bb.send(udp);
            udp.endPacket();
            bb.empty();
            delay(20); // slow sending to make recieving easier.
            yield();
          }
        }
 
        
      }
      */

}

void setup() {
  Wire.begin(4,5);        // GPIO4,GPIO5 = ~D2,~D1 => SDA,SCL -> join i2c bus, for ESP.
  Serial.begin(57600);  // start serial for output

// access point only:
  const char *ssid = "Buzzs-Discharger";
  const char *password = "qwer1234";
  WiFi.mode(WIFI_AP); // without this softap can't do multicast udp.
  WiFi.softAP(ssid, password); 
  wifi_set_event_handler_cb(_onWiFiEvent);

  //WiFi.softAPConfig(IPAddress local_ip, IPAddress gateway, IPAddress subnet)
  
  IPAddress clientips = WiFi.softAPIP();
  Serial.println(clientips);


   // start UDP server
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  
/*  Serial.println("starting wifi manager"); 
  
  WiFiManager wifiManager;
  
  //wifiManager.resetSettings();
   
   wifiManager.setTimeout(180); // if we don't get properly configured wifi within this time, continue onto our true battery purpose anyway....

     if(!wifiManager.autoConnect("Buzzs-Discharger")) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
     }
     */
  Serial.println("done wifi manager"); 
  //wait_for_new_connection(); // to init the static  variable and then timeout at 10 secs
   
}

byte byteArray[PACKET_SIZE];
//int readstatus = I2c.read(addrSlaveI2C, PACKET_SIZE, byteArray ); //request data and store directly to i2CData array

//  for (int k=0; k < PACKET_SIZE; k++)
//  {  leakinfo.I2CPacket[k] = byteArray[k]; }

void loop() {


  if (NeedRefresh==1){
    Serial.println("NeedRefresh"); 
    wait_for_new_connection(); // waits for new client to finish dhcp and updates our list of IPs of connected clients.
    NeedRefresh=0;
  }


   // nano/s we'll communicate with.... 
  for ( int deviceid = 1 ; deviceid <=10 ; deviceid++ ) { 

      int i2caddr = I2CAddressESPWifi+deviceid;

      if (DEBUG >2 ) Serial.printf("Probing for a device (%d) with i2c address(%d).\n", deviceid, i2caddr);

      #define MAXPOS 16
      
      for ( int POS = 1 ; POS <= MAXPOS ; POS++ )  {  // TBA, more pos thatn 3 will exist, starts at 1.  

          // zero any earlier data that might be lft...
          for (int k=0; k < PACKET_SIZE; k++)
          {  
            batteryinfo.I2CPacket[k] = 0;
          }
     
          // send the client a "position" or "offset" request  ( typically 0-3 on a nano, or 0-16 on a mega ) 
          Wire.beginTransmission(i2caddr);   
          Wire.write(POS);                
          Wire.endTransmission();   
          
          delay(100); // a delay
    
      
          Wire.requestFrom(i2caddr,PACKET_SIZE);    // request   bytes from slave device starting with offset of 8, and going up from there 8-18
        
          // drag data over i2c and put into byteArray
          int d = 0;  
          while (Wire.available()) { // slave may send less than requested
            byte c = Wire.read(); // receive a byte
            byteArray[d] = c;
            if (DEBUG >0 )  Serial.print(c,DEC);         // print the character
            if (DEBUG >0 )  Serial.print(" ");
            d++;
          }
          if ( (d > 0 ) and (DEBUG >0 )  ) Serial.println();
          
          if (d == 0 ) { 
            if (DEBUG >2 )  Serial.println("\tunable to get ANY bytes, going to next device");
            break; // skips rest of inner 'for' loop and goes to next deviceid
          }

          if (DEBUG >1 ) Serial.printf("\tProbing for a device (%d) with i2c address(%d) and offset(%d).\n", deviceid, i2caddr,POS);

        
          // without endian issues.... we convert byte-array packet into proper format like this:
          // but only if all the values are BYTE, nothing bigger. 
          for (int k=0; k < PACKET_SIZE; k++)
          {  
            batteryinfo.I2CPacket[k] = byteArray[k]; 
            if (DEBUG >1 ) Serial.print(byteArray[k],DEC);         // print the character
            if (DEBUG >1 ) Serial.print(" ");
          }
            if (DEBUG >1 ) Serial.println();
        
          // the we convert the bytes back into INTS ( avoids endianness issues trying to send uint16_t or anything bigger. )
          unsigned int v1 = batteryinfo.sensor.batvoltage1_low | ( batteryinfo.sensor.batvoltage1_hi << 8) ;
          //float fv1 = v1/1000.0;
          unsigned int ma1 = batteryinfo.sensor.MilliAmps1_low  | ( batteryinfo.sensor.MilliAmps1_hi << 8) ;
          unsigned int c1 = batteryinfo.sensor.TotalCurrents1_low | ( batteryinfo.sensor.TotalCurrents1_hi << 8)  ;
          unsigned int t1 = batteryinfo.sensor.time_ends1_low | ( batteryinfo.sensor.time_ends1_hi << 8)  ;
          int DeviceID = batteryinfo.sensor.DeviceID;
          int r1 = batteryinfo.sensor.Running1;
          int reportedpos = batteryinfo.sensor.etx; 

          // quick validate that the device we asked for is the one claiming to respond, otherwise ignore it.
          // TODO use a real packet checksum if you like.
          if ((deviceid != DeviceID ) or (DeviceID == 0) ){
            if (DEBUG >1 ) Serial.println("\tpacket DeviceID error, dropped.");
            continue;
          }
          if ((POS != reportedpos) or (reportedpos == 0) ) {
            if (DEBUG >1 ) Serial.printf("\tpacket reportedpos error, dropped POS:%d  reportedpos:%d\n",POS,reportedpos);
            continue;
          }
        
          if (DEBUG >0 ) Serial.printf("\tdeviceid:%d - mV:%d ma:%d - cumulative:%d - time:%d - running:%d \n",DeviceID,v1,ma1,c1,t1,r1);
   

          // and now finally send it. ! 
          send_OSC_msg(DeviceID,POS,v1, ma1, c1, t1 ,r1 );  // pulls from batteryinfo and clientips
          
          delay(20);
          yield();

      }

  } 
  yield();
  //delay(500);
}
