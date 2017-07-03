// Controller2017_v2
// This code is loaded into the ESP826 Feather + FeatherWing RFM69.  It gets moisture readings from the
// Moisture Puck and then updates data streams on Adafruit.io.
// I wrote a blog post on this (http://bitknitting.wordpress.com)
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <TimeLib.h> //used in isTimeOff() to compare controller with moisture puck's hh:mm:ss
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
// RFM69 ID numbers
#define PUCK_ADDRESS           2
#define CONTROLLER_ADDRESS     1
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
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, CONTROLLER_ADDRESS);

/********************************************************
   Data sent from the moisture stick.  There is the higher level
   view seen in moistureInfo_t, and the "on the wire view" seen in
   moistureInfo.b.  Having these two views of the data makes it easy
   to translate from a wire packet to an easily used typedef.
 ********************************************************/
const uint8_t packetMoistureInfo = 2;
struct moistureInfo_t
{
  uint8_t packetType;
  int     moistureReading;
  int8_t  temperatureOfRadioChip;
  float   batteryLevel;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};
union moistureUnion_t
{
  moistureInfo_t values;
  uint8_t b[sizeof(moistureInfo_t)];
} moistureInfo;
//timeInfo
const uint8_t packetTime = 1;
bool isTimeSet = false;
const uint8_t defaultAM_wateringHour = 4;
const uint8_t defaultPM_wateringHour = 21;
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
// Adafruit IO subscriptions and data
#define WLAN_SSID       "NF7VH"
#define WLAN_PASS       "FX9MP5LGC5NHDRQV"

/************************* Adafruit.io Setup *********************************/
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* WiFi Access Point *********************************/

WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883
#define AIO_USERNAME    "sketchy"
#define AIO_KEY         "556280f5b0c3a0587ad072ece406fdbf37d37617"
#include "myNetworkStuff.h"
Adafruit_MQTT_Publish moisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureReadingNode2");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatureNode2");
Adafruit_MQTT_Publish batteryLevel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/batteryLevelNode2");
Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, "time/seconds");
//Adafruit_MQTT_Subscribe water = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/water");
// set timezone offset from UTC
int timeZone = -7; // UTC - 4 eastern daylight time (nyc)
/******************************************************************
   SETUP
 ******************************************************************/
void setup()
{
  Serial.begin(115200);
  while (!Serial) ; // wait until serial console is open, remove if not tethered to computer
  Serial.println("************************** OH YEAH, I'M DAH BOSS **********************");
  initStuff();
  Serial.println("Waiting for packets....");
}
/******************************************************************
   LOOP
 ******************************************************************/
void loop() {
  doMqttLoopStuff();
  if (rf69_manager.available()) {
    getMoistureInfo();
  }
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  pinMode(LED, OUTPUT);
  pinMode(LED, HIGH);
  initRadio();
  initNetwork();
  Blink(LED, 400, 8);
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
    //Serial.println("RF69 manager couldn't init.");
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
  rf69_manager.setRetries(5); // default is 3.
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}
/********************************************************
   INITNETWORK
 ********************************************************/
void initNetwork() {
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" WiFi connected.");
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
/********************************************************
  GETMOISTUREINFO
********************************************************/
void getMoistureInfo() {
  uint8_t len = sizeof(moistureInfo);
  uint8_t from;
  if (rf69_manager.recvfromAckTimeout(moistureInfo.b, &len, 2000, &from))  {
    Serial.print("Got packet. Packet Type: "); Serial.print(moistureInfo.values.packetType); Serial.print("  length: ");
    Serial.println(len);
    Blink(LED, 500, 3);
    if ((moistureInfo.values.packetType == packetMoistureInfo) && (len == sizeof(moistureInfo)) ) {
      printMoistureInfo();
      if (isTimeSet) {
        if (isTimeOff()) {
          sendTime();
        }
      }
      //publishMoistureInfo();
    }
  }
}

/********************************************************
   PRINTMOISTUREINFO
 ********************************************************/
void printMoistureInfo() {
  Serial.println(" ---> Moisture Info <---");
  Serial.print("Moisture Reading: ");
  Serial.print(moistureInfo.values.moistureReading);
  Serial.print("| Battery level: ");
  Serial.print(moistureInfo.values.batteryLevel);
  Serial.print("| Radio Temperature: ");
  Serial.println(moistureInfo.values.temperatureOfRadioChip);
}
/********************************************************
   PUBLISHMOISTUREINFO
 ********************************************************/
void publishMoistureInfo() {
  if (moisture.publish(moistureInfo.values.moistureReading) &
      temperature.publish(moistureInfo.values.temperatureOfRadioChip) &
      batteryLevel.publish(moistureInfo.values.batteryLevel)) {
    //   Serial.println("Published!");
    Blink(LED, 200, 10);
  } else {
    //   Serial.println("Failed to publish!");
  }
}
/********************************************************
   TIMECALLBACK
   Adafruit.io's MQTT time utility: https://io.adafruit.com/blog/feature/2016/06/01/time-utilities/
 ********************************************************/
void timecallback(uint32_t current) { //current = seconds since Unix epoch: https://en.wikipedia.org/wiki/Unix_time
  Serial.println("in timecallback...setting time to values got from Adafruit.io time feed.");
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
/********************************************************
   ISTIMEOFF - Compare the hh:mm:ss from the moisture info
   packet received to the controller's time.  If they are
   different by a minute or more, isTimeOff = true.
 ********************************************************/
bool isTimeOff() {
  tmElements_t tm;  //see TimeLib.h
  Serial.print("time I have: "); Serial.print(hour()); Serial.print(":"); Serial.print(minute()); Serial.print(":"); Serial.println(second());
  Serial.print("time moisture puck has: "); Serial.print(moistureInfo.values.hour); Serial.print(":"); Serial.print(moistureInfo.values.minute);
  Serial.print(":"); Serial.println(moistureInfo.values.second);
  tm.Day = 0;
  tm.Month = 0;
  tm.Year = 0;
  tm.Hour = moistureInfo.values.hour;
  tm.Minute = moistureInfo.values.minute;
  tm.Second = moistureInfo.values.second;
  time_t moisturePuckTime = makeTime(tm);
  time_t difference = abs(now() - moisturePuckTime);
  Serial.print("Time difference:" ); Serial.println(difference);
  if (difference > SECS_PER_MIN) return true;
  return false;
}
/********************************************************
   SENDTIME..so the Moisture Puck and the Controller will have the same values.
 ********************************************************/
void sendTime() {
  Serial.println("...sending timeInfo packet to moisture puck...");
     if (! rf69_manager.sendtoWait(timeInfo.b, sizeof(timeInfo), PUCK_ADDRESS) ) {
      Serial.println("Failed to send timeInfo packet");
     }
//  timeInfo.values.packetType = packetTime;
//  timeInfo.values.hour = hour();
//  timeInfo.values.minute = minute();
//  timeInfo.values.second = second();
//  timeInfo.values.AM_wateringHour = defaultAM_wateringHour;
//  timeInfo.values.PM_wateringHour = defaultPM_wateringHour;
//  // Send the timeInfo to the Moisture Puck...not doing anything if fails.
//  // Try several times...
//  for (int i=0;i<3;i++) {
//    Serial.println("..sending time info packet...");
//
//  }
  rf69.sleep();
}
/*
   BLINK
*/
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}

