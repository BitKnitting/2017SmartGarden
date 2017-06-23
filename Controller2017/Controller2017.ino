/*
   The Controller fits the client examples in the RH library.
*/
#define DEBUG
#include <DebugLib.h>

//***********************************************************************
/*
   Stuff for the RFM69 See Adafruit's fork of RadioHead Library: https://github.com/adafruit/RadioHead
*/
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
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
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
// Class to manage message delivery and receipt, using the driver declared above
#define MY_ADDRESS            1
#define MOISTURESTICK_ADDRESS 2
// Using the reliable datagram library to send/receive.
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);
//*********************************************************************
// stuff for irrigation management
// Dont put this on the stack:
struct moistureInfo_t
{
  int     moistureReading;
  int8_t  temperatureOfRadioChip;
  float   batteryLevel;
};
union moistureUnion_t
{
  moistureInfo_t values;
  uint8_t b[sizeof(moistureInfo_t)];
} moistureInfo;
#define WATERING_THRESHOLD 300
int replyNumber = 0;
//****************************************************************
// Adafruit IO subscriptions and data
#include "myNetworkStuff.h"
Adafruit_MQTT_Publish moisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureReadingNode2");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatureNode2");
Adafruit_MQTT_Publish batteryLevel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/batteryLevelNode2");
Adafruit_MQTT_Subscribe water = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/water");
// set timezone offset from UTC
int AIO_timeZone = -7; // UTC - 4 eastern daylight time (nyc)
//****************************************************************
// FLAGS
//const uint8_t initialStateFlag = 0x0;
//const uint8_t timeSetFlag = 0x01;
//const uint8_t AdafruitIoConnectedFlag = 0x02;
//const uint8_t wateringFlag = 0x04;
//uint8_t currentState = initialStateFlag;
//****************************************************************
// GPIO FEATHER HUZZAH PINS USED FOR VALVES
//#define Node2Valve  14
//#define Node3Valve  12
//#define Node4Valve  13
//***********************************************************************
void setup() {
  initStuff();
}

/********************************************************
   LOOP
 ********************************************************/
void loop() {
  doMqttLoopStuff();
}

/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  DEBUG_PRINTLNF("****Garden Controller***");
  pinMode(LED, OUTPUT); 
  blink(3,300);
  //we've only just begun...
  //currentState = initialStateFlag;
  initRadio();
  initNetwork();

}
/********************************************************
   INITRADIO
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
    DEBUG_PRINTLNF("RFM69 radio init failed");
    while (1);
  }
  DEBUG_PRINTLNF("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    DEBUG_PRINTLNF("setFrequency failed");
  }
  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);
  // The controller is trying to contact a Moisture puck.  I'm finding that perhaps since
  // the Moisture puck has to be woken up, there are times when the sent message doesn't get
  // an ACK....so I'm bumping up the retries and timeout.
  rf69_manager.setRetries(20);
  rf69_manager.setTimeout(500);
  DEBUG_PRINTF("RFM69 radio @");  DEBUG_PRINT((int)RF69_FREQ);  DEBUG_PRINTLNF(" MHz");
}
/********************************************************
   INITNETWORK
 ********************************************************/
void initNetwork() {
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLNF(" WiFi connected.");
  // get updates when the water feed has input..
  water.setCallback(handleWater);
  mqtt.subscribe(&water);
}
/********************************************************
   HANDLEWATER
 ********************************************************/
void handleWater(char *data, uint16_t len) {
  blink(3,200);
  DEBUG_PRINTF("...in handleWater callback.  I received this message: ");
  Serial.println(data);
  DEBUG_PRINTLNF("...now I'll ask the moisture puck for a reading and publish results to Adafruit.io");
  getMoistureInfo();
  publishMoistureInfo();
}
/********************************************************
   GETMOISTUREREADING
 ********************************************************/
void getMoistureInfo() {
  byte command = 0;
  if (rf69_manager.sendtoWait(&command, 1, MOISTURESTICK_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(moistureInfo);
    uint8_t from;
    // from = node == 2 -> node 2 is in Strawberry Swirl.
    if (rf69_manager.recvfromAckTimeout(moistureInfo.b, &len, 2000, &from))
    {
      replyNumber++;
      DEBUG_PRINTF("\n...got reply ");
      Serial.print(replyNumber);
      DEBUG_PRINTF(" from: ");
      Serial.print(from);
      DEBUG_PRINTF(" | RSSI: ");
      Serial.println(rf69.lastRssi());
      printMoistureInfo();
      blink(2,500);
      blink(2,300);
      blink(1,500);
    }
    else
    {
      DEBUG_PRINTF("...No reply from moisture stick ");
      DEBUG_PRINTLN(from);
    }
  }
  else
    DEBUG_PRINTLNF("...sendtoWait failed");
}
/********************************************************
   PUBLISHMOISTUREINFO
 ********************************************************/
void publishMoistureInfo() {
  if (moisture.publish(moistureInfo.values.moistureReading) &
      temperature.publish(moistureInfo.values.temperatureOfRadioChip) &
      batteryLevel.publish(moistureInfo.values.batteryLevel)) {
    DEBUG_PRINTLNF("Published!");
    blink(5,300);
  } else {
    DEBUG_PRINTLNF("Failed to publish!");
  }
}
/********************************************************
   PRINTMOISTUREINFO
 ********************************************************/
void printMoistureInfo() {
  DEBUG_PRINTLNF(" ---> Moisture Info <---");
  DEBUG_PRINTF("Moisture Reading: ");
  Serial.print(moistureInfo.values.moistureReading);
  DEBUG_PRINTF("| Battery level: ");
  Serial.print(moistureInfo.values.batteryLevel);
  DEBUG_PRINTF("| Radio Temperature: ");
  Serial.println(moistureInfo.values.temperatureOfRadioChip);
}
/********************************************************
   DOMQTTLOOPSTUFF
 ********************************************************/
void doMqttLoopStuff() {
  MQTT_connect();
  mqtt.processPackets(2000);
  // keep the connection alive
  if (! mqtt.ping()) { //means a dropped packet
    mqtt.disconnect();
  }
}

