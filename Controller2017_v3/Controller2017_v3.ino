// Controller2017_v3
// This code is loaded into the Huzzah Feather that has a FeatherWing RFM69.  It communicates with the Moisture Puck
// out in the Garden as well as Adafruit.io  I wrote a blog post on this (http://bitknitting.wordpress.com)
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <TimeLib.h>

// RFM69 ID numbers
#define PUCK_ADDRESS           4
#define CONTROLLER_ADDRESS     1
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

//*****************************************************
// I first get code running on a Feather M0 RFM69.  Once the RFM69 code works,
// I move on to the ESP826 and a FeatherWing RFM69.
#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio - I tested with this
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#else //using a Featherwing RFM69 w/ Huzzah running Arduino IDE
#define LED           BUILTIN_LED
/*
   From https://learn.adafruit.com/radio-featherwing/wiring#esp8266-wiring
*/
#define RFM69_CS  2    // "E"
#define RFM69_RST 16   // "D"
#define RFM69_INT 15   // "B"
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, CONTROLLER_ADDRESS);
/*************************************************************************/
/* Controller / Moisture Puck stuff

*/
const uint8_t packetMoistureInfo = 2;
struct moistureInfo_t
{
  uint8_t packetType;
  int     moistureReading;
  int8_t  temperatureOfRadioChip;
  float   batteryLevel;
};
union moistureUnion_t
{
  moistureInfo_t values;
  uint8_t b[sizeof(moistureInfo_t)];
} moistureInfo;
struct timeInfo_t
{
  uint8_t packetType;  // packetTime
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t AM_wateringHour;
  uint8_t PM_wateringHour;
};
union timeUnion_t
{
  timeInfo_t values;
  uint8_t b[sizeof(timeInfo_t)];
} timeInfo;
/*************************************************************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* WiFi Access Point *********************************/


// Adafruit IO subscriptions and data
#define WLAN_SSID       "XXXXX"
#define WLAN_PASS       "XXXXX"

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883
#define AIO_USERNAME    "XXXXX"
#define AIO_KEY         "XXXXX"
WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish moisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureReadingNode2");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatureNode2");
Adafruit_MQTT_Publish batteryLevel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/batteryLevelNode2");
Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, "time/seconds");
//Adafruit_MQTT_Subscribe water = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/water");
// set timezone offset from UTC
;
int timeZone = -7; // UTC - 4 eastern daylight time (nyc)
bool isTimeSet = false;
/*************************************************************************/
/********************************************************
   SETUP
 ********************************************************/
void setup()
{
  //Serial.begin(115200);
  //while (!Serial);
  initStuff();
}
/********************************************************
   LOOP
 ********************************************************/
void loop()
{
  doMqttLoopStuff();
  //Not talking with the Moisture Puck until Adafruit.io has given the Controller the current time.
  if (isTimeSet) {
    //Serial.println("Checking if there is moisture info available.");
    if (rf69_manager.available())
    {
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(moistureInfo);
      uint8_t from;
      if (rf69_manager.recvfromAck(moistureInfo.b, &len, &from))
      {
        Blink(LED, 400, 3);
        //Serial.print("got moisture Info packet from : 0x");
        //Serial.println(from, HEX);
        printMoistureInfo();

        // Send a reply back to the originator client
        setUpATimeInfoPacket();
        if (!rf69_manager.sendtoWait(timeInfo.b, sizeof(timeInfo), from)) {
          //Serial.println("Failed to send a timeInfo packet.");
        }else {
                  publishMoistureInfo(); //publish if complete with timeInfo send/receive
                  //Note: publishing takes longer than the traffic
        }
      } 
    }
  }
}
/********************************************************
   SETUPTIMEINFOPACKET
 ********************************************************/
void setUpATimeInfoPacket() {
  timeInfo.values.hour = hour();
  timeInfo.values.minute = minute();
  timeInfo.values.second = second();
  timeInfo.values.AM_wateringHour = 4;
  timeInfo.values.PM_wateringHour = 21;
}
/********************************************************
   PRINTMOISTUREINFO
 ********************************************************/
void printMoistureInfo() {
  //Serial.println("\n ---> Moisture Info <---");
  //Serial.print("Moisture Reading: ");
  //Serial.print(moistureInfo.values.moistureReading);
  //Serial.print("| Battery level: ");
  //Serial.print(moistureInfo.values.batteryLevel);
  //Serial.print("| Radio Temperature: ");
  //Serial.println(moistureInfo.values.temperatureOfRadioChip);
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  initRadio();
  initNetwork();
  Blink(LED,400,6);
  digitalWrite(LED,HIGH);
}
/********************************************************
  INITRADIO
  this is mostly copy/pasted from an Adafruit example
********************************************************/
void initRadio() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    ////Serial.println(F("Could not initialize RH manager."));
    while (1) ;
  }
  rf69.setFrequency(RF69_FREQ);

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);
  // Wait a little longer for an ACK after sending packets
  rf69_manager.setTimeout(600); //default is 200ms, see: http://www.airspayce.com/mikem/arduino/RadioHead/classRHReliableDatagram.html#ad282ac147986a63692582f323b56c47f
  rf69_manager.setRetries(5); // default is 3...probably overkill...but want to give the Controller time to respond given it's also dealing with MQTT.
  //Serial.print("RFM69 radio @");  //Serial.print((int)RF69_FREQ);  //Serial.println(" MHz");
}
/********************************************************
   INITNETWORK
 ********************************************************/
void initNetwork() {
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  //Serial.println(" WiFi connected.");
  timefeed.setCallback(timecallback);
  mqtt.subscribe(&timefeed);
}
/********************************************************
   DOMQTTLOOPSTUFF
 ********************************************************/
void doMqttLoopStuff() {
  MQTT_connect();
  mqtt.processPackets(500);
  // keep the connection alive
  if (! mqtt.ping()) { //means a dropped packet
    mqtt.disconnect();
  }
}
/********************************************************************
   MQTT_CONNECT
 *******************************************************************/
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      //Serial.println("Could not connect to MQTT");
      // basically die and wait for WDT to reset me
      while (1);
    }

  }
  //Serial.println("...connected to MQTT");
}
/********************************************************
   TIMECALLBACK
   Adafruit.io's MQTT time utility: https://io.adafruit.com/blog/feature/2016/06/01/time-utilities/
 ********************************************************/
void timecallback(uint32_t current) { //current = seconds since Unix epoch: https://en.wikipedia.org/wiki/Unix_time
  //Serial.println("in timecallback...setting time to values got from Adafruit.io time feed.");
  Blink(LED, 400, 4);
  // adjust to local time zone
  current += (timeZone * 60 * 60);
  int sec, min, hour;
  // calculate current time
  sec = current % 60;
  current /= 60;
  min = current % 60;
  current /= 60;
  hour = current % 24;
  // I don't need the day, month, year
  setTime(hour, min, sec, 0, 0, 0);
  isTimeSet = true;
  mqtt.unsubscribe(&timefeed);
}
/********************************************************************
   PRINTTIMEINFO
 *******************************************************************/
void printTimeInfo() {
  //Serial.println("\n**** Time Info*****");
  //Serial.print(timeInfo.values.hour); //Serial.print(":"); //Serial.print(timeInfo.values.minute);
  //Serial.print(":"); //Serial.println(timeInfo.values.second);
  //Serial.print("AM watering hour: "); //Serial.print(timeInfo.values.AM_wateringHour); //Serial.print(" PM watering hour: ");
  //Serial.println(timeInfo.values.PM_wateringHour);
}
/********************************************************
   PUBLISHMOISTUREINFO
 ********************************************************/
void publishMoistureInfo() {
  if (moisture.publish(moistureInfo.values.moistureReading) &
      temperature.publish(moistureInfo.values.temperatureOfRadioChip) &
      batteryLevel.publish(moistureInfo.values.batteryLevel)) {
    //Serial.println("Published!");
    Blink(LED, 200, 10);
  } else {
    //Serial.println("Failed to publish!");
  }
}
/********************************************************
   BLINK
 ********************************************************/
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
