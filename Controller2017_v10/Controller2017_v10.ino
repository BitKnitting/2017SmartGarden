// Controller2017
// This code is loaded into the Huzzah Feather that has a FeatherWing RFM69.  It communicates with the Moisture Puck
// out in the Garden as well as Adafruit.io  I wrote a blog post on this (http://bitknitting.wordpress.com)
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Be Kind. Please give this work credit if you evolve and use.
//
// V5 - added letting the watering puck know which zone (yellow, blue, green) to water through
//      Adafruit.io.
//    - added logging feed to Adafruit.io so the controller can comment on activity...
// V6 - changed watering to be once a day in the early AM.
//    - improved logging messages.
// V7 - added Adafruit.io command to tell the watering puck to start watering.  Command = start.
// V8 - attempt to clean up code by moving actions to states.
// V9 - Decoupled sending a timeInfo every time a moisture info is sent.  This makes sending the moisture info have
//      less overhead, so more moisture info's can be sent as wanted.
// V10- Added states for RF69 messaging.  It is important to keep sending/receiving RF69 messages in a tight loop while also giving
//      the buffers time to do their thing...hmmm...
//
//#define DEBUG
#include <DebugLib.h>
#include "ControlInputStuff.h"
//****************Stuff for TimeAlarm library********************************
// I like how easy it is to use this timer library: https://github.com/PaulStoffregen/TimeAlarms
#include <TimeLib.h>
#include <TimeAlarms.h>
AlarmId alarmId;
//---------------------------------------------------------------------------
//****************ESP826's EEPROM library...********************************
// when the Board is set to an ESP826 which brings in the esp8266 Arduino library:
// http://esp8266.github.io/Arduino/versions/2.0.0/doc/libraries.html#eeprom
// there are EEPROM examples.
// EEPROM is used to store the watering threshold since the watering threshold can be changed
// through an mqtt feed.  This allows me to change the watering threshold from Adafruit.io's feeds.
// For now, I'm only changing the watering threshold since each moisture sensor probe will send different
// readings...it's subjective what watering threshold value should trigger watering (and I'm still learning).
#include <EEPROM.h>
//---------------------------------------------------------------------------
//****************Stuff for RadioHead library********************************
// http://www.airspayce.com/mikem/arduino/RadioHead/index.html
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
// RFM69 ID numbers
#define WATERINGPUCK_ADDRESS   55
#define PUCK_ADDRESS           25
#define CONTROLLER_ADDRESS     10
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
// I first get code running on a Feather M0 RFM69.  Once the RFM69 code works,
// I move on to the ESP826 and a FeatherWing RFM69.

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio - I tested with this
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#else //using a Featherwing RFM69 w/ Huzzah running Arduino IDE
// From: https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/pinouts
// GPIO #0, which does not have an internal pullup, and is also connected a red LED.
// This pin is used by the ESP8266 to determine when to boot into the bootloader.
// If the pin is held low during power-up it will start bootloading! That said, you
// can always use it as an output, and blink the red LED.
#define LED           0
/*
   From https://learn.adafruit.com/radio-featherwing/wiring#esp8266-wiring
*/
#define RFM69_CS  2    // "E"
#define RFM69_RST 16   // "D"
#define RFM69_INT 15   // "B"
#endif
#include "Blinkings.h"
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, CONTROLLER_ADDRESS);
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
// The challenge is MQTT needs more processing time - which blocks other stuff - like receiving
// RF69.  So this is what I do:
// - set the processPacketMs to an amount that is good for MQTT.
// - set an alarm to go off 10 minutes before the moisture puck is to send a reading.
// - at the alarm callback, set the processPacketMs to an amount that is good for RF69 listening.
// - when the moisture packet comes in and other RF69 traffic finishes, set the processPacketMs (back) to an
//   amount of time that is good for MQTT traffic
#define MQTT_PROCESS_TIME 2000
#define RFM69_PROCESS_TIME 100
uint16_t processPacketMs = MQTT_PROCESS_TIME;
//---------------------------------------------------------------------------
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
const uint8_t packetTimeInfo = 4;
struct timeInfo_t
{
  uint8_t packetType;  // packetTime
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t wateringHour;
};
union timeUnion_t
{
  timeInfo_t values;
  uint8_t b[sizeof(timeInfo_t)];
} timeInfo;
/*************************************************************************/
/* Controller / Watering Puck stuff
*/
const uint8_t packetWateringInfo = 3;
struct wateringInfo_t
{
  uint8_t packetType;
  uint8_t valvePin;  //if valvePin == 0xFF, stop watering.
  uint8_t numWateringMinutes;
};
union wateringUnion_t
{
  wateringInfo_t values;
  uint8_t b[sizeof(wateringInfo_t)];
} wateringInfo;
// gValvePinCounter keeps track of the solenoid that is on for watering.  It is reset in the water if needed function
// and incremented in the function that starts watering.  It is set back to 0 after all solenoids that indicate they should
// be turned on/off have been serviced.
uint8_t gValvePinCounter = 0;
const uint16_t gDefaultWateringThreshold = 215; //Should be calibrated w/ Moisture Pucks
const uint8_t gDefaultWateringMinutes = 15;
bool bWatering = false;
char controlString[MAX_BYTES] = {0};
char logStr[80] = {0};

//---------------------------------------------------------------------------

/*************************************************************************/
#include <ESP8266WiFi.h>

// Using this library so SSID and password remains private.
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
//****************Stuff for Adafruit's MQTT ********************************
//#define MQTT_DEBUG
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883
// AIO_USERNAME AND AIO_KEY ARE PRIVATE TO THE PERSON SETTING THIS UP...
// I PUT VALUES FOR THESE IN AdafruitIOLogin.h and then .gitignore on GitHub
#include "AdafruitIOLogin.h"
//#define AIO_USERNAME    "xxxxxxxx"
//#define AIO_KEY         "xxxxxxxx"
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish moisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureReadingNode2");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatureNode2");
Adafruit_MQTT_Publish batteryLevel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/batteryLevelNode2");
Adafruit_MQTT_Publish logText = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Log");
Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, "time/seconds");
Adafruit_MQTT_Subscribe water = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/water");
Adafruit_MQTT_Subscribe controlInput = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ControlInput");
// set timezone offset from UTC
int timeZone = -7; // UTC - 4 eastern daylight time (nyc)
char feedbackString[80] = {0}; // Used to provide feedback to Adafruit.io log feed.
commands ioCommand;


//---------------------------------------------------------------------------
enum states_t {
  STATE_INIT,
  STATE_TIME_SET,
  STATE_LISTENING_TO_RF69,
  STATE_RECEIVED_MOISTURE_INFO,
  STATE_THRESHOLD_CHECK,
  STATE_WATER_FIRST_ZONE,
  STATE_WATER_NEXT_ZONE,
  STATE_STOP_WATERING,
  STATE_DONE_WATERING,
  STATE_IO_COMMAND,
  STATE_UNKNOWN
};
states_t state = STATE_LISTENING_TO_RF69;
/********************************************************
   SETUP
 ********************************************************/
void setup()
{
  initStuff();
  //----------------------------------
  //TBD: This code assumes nothing can happen unless we get the time from Adafruit.io.  The good news is
  //during testing this hasn't been a problem.  The reason to do it this way is to let the Moisture Puck
  //know the realistic time to water.  There are other "off net" ways to do this that aren't reliant on
  //so many layers of networking and mqttisms....
  //----------------------------------
  while (state == STATE_INIT) {
    doMqttLoopStuff(); // Give Adafruit.io a chance to process any incoming/outgoing messages.
  }
}
/********************************************************
   LOOP
 ********************************************************/
void loop()
{
  // I was putting handling RF69 messages within the state handling
  // function...but RF69 traffic needs to be tightly integrated with the
  // loop.
  handleRF69Messages();
  //----------------------------------
  // Keep the timer alive and service mqtt messages.
  //----------------------------------
  Alarm.delay(1); //using a timer so need to allow time for processing.
  doMqttLoopStuff(); // Give Adafruit.io a chance to process any incoming/outgoing messages.
  //----------------------------------
  // Do the activity based on the state.
  //----------------------------------
  if (state != STATE_LISTENING_TO_RF69) {  // handleRF69Messages takes care...
    stateEntryActionRun();
  }
}
/********************************************************
  stateChange
********************************************************/
void stateChange(states_t newState) {
  state = newState;
  logState(state);
}
/********************************************************
  stateEntryActionRun
********************************************************/
void stateEntryActionRun() {
#ifdef DEBUG
  Serial.print(F("...in stateEntryActionRun.  state: ")); Serial.println(state);
#endif
  switch (state) {
    case STATE_TIME_SET:
      {
        stateChange(STATE_LISTENING_TO_RF69);
      }
      break;
    case STATE_RECEIVED_MOISTURE_INFO:
      handleReceivedMoistureInfo();
      break;
    case STATE_THRESHOLD_CHECK:
      handleThresholdCheck();
      break;

    case STATE_WATER_NEXT_ZONE:
      {
        logWateringZone("Stop");
        handleWateringNextZone();
      }
      break;
    case STATE_STOP_WATERING:
      {
        logWateringZone("Stop");
        sendStopWateringCommand();
        stopWatering();
      }
      break;
    case STATE_DONE_WATERING:  // All zones (that are signed up for watering) have been watered.
      stopWatering();
      break;
    case STATE_IO_COMMAND:
      BlinkIOCommand;
      handleIoCommand();
      break;
  }
}

/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  stateChange(STATE_INIT);
  DEBUG_BEGIN;
  DEBUG_WAIT;
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  DEBUG_PRINTLNF("...initializing stuff for the controller...");
  // read in the cached parameters (at this time it's the watering threshold)
  EEPROM.begin(sizeof(cachedParams));
  readCachedParamsInEEPROM();
  if (cachedParams.values.writeCheck != WRITE_CHECK) { // won't be first time read.
    resetCachedParams();
    //TEST
    DEBUG_PRINTLNF("\n...testing if the params were reset in EEPROM");
    readCachedParamsInEEPROM();
  }
  // Set up the number of minutes to water.
  wateringInfo.values.numWateringMinutes = gDefaultWateringMinutes;
  wateringInfo.values.packetType = packetWateringInfo;
  // Set up hour to water
  timeInfo.values.wateringHour = 4;
  // on to initializing RFM69, WiFi, and Adafruit.io = lots-oh-communications....
  initRadio();
  initNetwork();
  bWatering = false;
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
    //Serial.println(F("Could not initialize RH manager."));
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
#ifdef DEBUG
  DEBUG_PRINTF("\nRFM69 radio @");  Serial.print((int)RF69_FREQ);  DEBUG_PRINTLNF(" MHz");
#endif
}
/********************************************************
   INITNETWORK
 ********************************************************/
void initNetwork() {
  // Using WiFiManager library so the SSID and password can be kept private (https://github.com/tzapu/WiFiManager)
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("AutoConnectAP");
  //if you get here you have connected to the WiFi
  DEBUG_PRINTLN(F("connected...yeey :)"));
  timefeed.setCallback(timecallback);
  mqtt.subscribe(&timefeed);
  // get updates when the control Input feed has an entry.  It means I want to give a command to the Controller..
  controlInput.setCallback(controlInputcallback);
  mqtt.subscribe(&controlInput);
}
/********************************************************
   DOMQTTLOOPSTUFF
   ...from testing, I'm finding that in order for subscribed feeds
   to (fairly) reliably get to the callbacks, processPackets should
   be (at least?) 1 second.  The challenge with this is when packets are
   coming in from RFM69...
 ********************************************************/
void doMqttLoopStuff() {
  MQTT_connect();
  mqtt.processPackets(processPacketMs); // checks readSubscription()...
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
  DEBUG_PRINTLNF("... in MQTT_CONNECT MQTT is not connected");
  //uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    mqtt.disconnect();
    Alarm.delay(1000);
    //    retries--;
    //    if (retries == 0) {
    //      DEBUG_PRINTLNF("Could not connect to MQTT");
    //
    //      while (1);
    //    }
  }
  DEBUG_PRINTLNF("...connected to MQTT");
}

/********************************************************
   handleRF69Messages
   - See if we have received one of the messages we are expecting:
    - moistureInfo. If receive this message, the next step is to determine
    if the reading is low enough to require watering.
    - wateringInfo.  This is received from the watering puck to let us know
    watering is done (and completed successfully).
    NOTE: From testing - any receive/send interaction needs to be done within the same "lump of code"
    This is mostly because there are two radios here - the MQTT - S-L-O-W and RF69 - Fast / tight back/forth.
 ********************************************************/
void handleRF69Messages() {
  if (rf69_manager.available()) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from))// Got a message.  Now determine if from the moisture or watering puck.
    {
      if (from == PUCK_ADDRESS) { // this would be a moisture or time info message from the moisture puck.
        if (buf[0] == packetMoistureInfo) {
          stateChange(STATE_RECEIVED_MOISTURE_INFO);
        } if (buf[0] == packetTimeInfo) { // the moisture puck has requested a time info packet.
          //stateChange(STATE_RECEIVED_TIME_INFO_REQUEST);
          setUpATimeInfoPacket();
          if (rf69_manager.sendtoWait(timeInfo.b, sizeof(timeInfo), PUCK_ADDRESS)) {
            DEBUG_PRINTLNF("Time info was received.");
            logText.publish("Time info was received.");
          }
          stateChange(STATE_LISTENING_TO_RF69);
        }
      }
      else if (from == WATERINGPUCK_ADDRESS) {
        if (buf[0] == packetWateringInfo) {
          memcpy(wateringInfo.b, buf, sizeof(wateringInfo));
          if (wateringInfo.values.valvePin == 0xFF) {
            stateChange(STATE_DONE_WATERING);
          } else {
            stateChange(STATE_WATER_NEXT_ZONE);
          }
        }
      }
    }
  }
  if (state == STATE_WATER_FIRST_ZONE) { // Handling this state here because need to send a message to the watering puck...RF69 messages should be within the
    // loop().
    gValvePinCounter = 0;  // There are up to three zones that can be watered. Here we reset to the first zone within valvePins[].
    handleWateringZone();   // gValvePinCounter is a global varable used by the watering routines to track which zone is being watered.  Transition state to wateringZone.
  }
}
/********************************************************
   SETUPTIMEINFOPACKET
 ********************************************************/
void setUpATimeInfoPacket() {
  timeInfo.values.packetType = packetTimeInfo;
  timeInfo.values.hour = hour();
  timeInfo.values.minute = minute();
  timeInfo.values.second = second();
  printTimeInfo();
}

/********************************************************
   WATERIFBELOWTHRESHOLD
 ********************************************************/
void handleThresholdCheck() {
#ifdef DEBUG
  DEBUG_PRINTF(" Threshold: "); Serial.println(cachedParams.values.wateringThreshold);
#endif
  // The watering threshold is unique to the moisture puck in that the moisture puck should be calibrated
  // based on the readings of that particular moisture puck.
  if (moistureInfo.values.moistureReading <= cachedParams.values.wateringThreshold) {
    logBelowThreshold();
    stateChange(STATE_WATER_FIRST_ZONE);
  } else {
    stateChange(STATE_LISTENING_TO_RF69);
  }
}
/********************************************************
  handleReceivedMoistureInfo
********************************************************/
void handleReceivedMoistureInfo() {
  memcpy(moistureInfo.b, buf, sizeof(moistureInfo));
  publishMoistureInfo();
  if (isWateringCheckTime()) {
    stateChange(STATE_THRESHOLD_CHECK);
  } else {
    stateChange(STATE_LISTENING_TO_RF69);
  }
}
/********************************************************
  isWateringCheckTime
  - a check to determine if this moisture info is the one set up
  to do watering.  I'm giving it a few minutes either way...
  - the time could be a few seconds before the watering hour - so using seconds to check and not the hour...
********************************************************/
bool isWateringCheckTime() {
  const time_t aFewMinutesInSeconds = 5 * 60;
  time_t wateringTimeInSeconds = timeInfo.values.wateringHour * 3600; // We don't use minutes / seconds - just the hour to water.
  time_t currentTimeInSeconds = hour() * 3600 + minute() * 60 + second();
  if (abs(currentTimeInSeconds - wateringTimeInSeconds) <= aFewMinutesInSeconds) { // is the current time close enough to the watering time to say this packet coming
    // from the moisture puck was sent because the moisture puck woke up at the watering hour and sent a packet?
    return true;
  } else {
    return false;
  }
}
/********************************************************
  handleWateringZone
  - As I use the rf69 radio and library, I recognize getting a message to
  a device is BEST suited to be within the tight loop(), because it may take a
  few times.  Just looping sending packets won't work because there is no time to
  check for receives.  Also, during this time i want to minimize the mqtt process time.
********************************************************/
void handleWateringZone() {
  if (!bWatering) { // Might get multiple message from the moisture puck???
    DEBUG_PRINTLNF("..time to water...");
    setUpWateringInfo();
    if   ( rf69_manager.sendtoWait(wateringInfo.b, sizeof(wateringInfo), WATERINGPUCK_ADDRESS) ) {
      logWateringZone("Start");
      // wait a little longer than the gDefaultWateringMinutes to determine if something's wrong
      int numSeconds = wateringInfo.values.numWateringMinutes * 60 + 15;
      alarmId = Alarm.timerOnce(numSeconds, shouldBeFinishedWateringZone);
      bWatering = true;
#ifdef DEBUG
      DEBUG_PRINTF("...will check if get response from watering puck in "); Serial.print(numSeconds); DEBUG_PRINTLNF(" seconds.");
#endif
    } else {
      bWatering = false;
      DEBUG_PRINTLNF("ERROR: could not send message to the watering puck.");
      logText.publish("ERROR: could not send message to the watering puck.");
    }
  }
  stateChange(STATE_LISTENING_TO_RF69); // Prevent repeatedly falling in to this case.
}
/********************************************************
  handleWateringNextZone
********************************************************/
void handleWateringNextZone() {
  Alarm.free(alarmId);
  alarmId = dtINVALID_ALARM_ID;
  if (aZoneNeedsWatering()) {
    bWatering = false; // this is to get through the check in handleWateringZone()...not sure this is the best way because not as intuitive as I would like...
    handleWateringZone(); // other zones might need watering.  waterZone() will change state to rf69 listening.
  } else {
    stateChange(STATE_DONE_WATERING);
    DEBUG_PRINTLN(F("Done watering."));
  }
}
/********************************************************
  aZoneNeedsWatering
********************************************************/
bool aZoneNeedsWatering() {
  gValvePinCounter++; // Move to the next if a zone has been watered.
  while ( (cachedParams.values.valvePins[gValvePinCounter] == 0xFF) && (gValvePinCounter < numZones) ) {
    gValvePinCounter++;
  }
  DEBUG_PRINT(F("...in aZoneNeedsWatering.  Pin counter: ")); DEBUG_PRINTLN(gValvePinCounter);
  if (gValvePinCounter < numZones) {
    return true;
  }
  return false;
}
/********************************************************************
  setUpWateringInfo
  This is the stuff needed by the Watering Puck - how many minutes to water
  and what solenoid to turn on/off.
*******************************************************************/
void setUpWateringInfo() {
  wateringInfo.values.packetType = packetWateringInfo;
  wateringInfo.values.numWateringMinutes = gDefaultWateringMinutes; //TBD: The watering time could be adjustable like the watering threshold is...
  wateringInfo.values.valvePin = cachedParams.values.valvePins[gValvePinCounter];
  printWateringInfo();
}
/********************************************************
  SHOULDBEFINISHEDWATERING
  - this is the callback the Alarm timer set in startWatering() will invoke when the timer goes off.
  If the timer goes off, then we haven't received a packet from the watering puck telling us it is finished watering.
  We've given enough time for the watering puck to get back to us, so we really don't want to have the timer go off.
********************************************************/
void shouldBeFinishedWateringZone() {
  logText.publish("Error: Should have received a finished watering message from the Watering Puck.");
  stateChange(STATE_WATER_NEXT_ZONE);
}
/********************************************************
   TIMECALLBACK
   Adafruit.io's MQTT time utility: https://io.adafruit.com/blog/feature/2016/06/01/time-utilities/
 ********************************************************/
void timecallback(uint32_t current) { //current = seconds since Unix epoch: https://en.wikipedia.org/wiki/Unix_time
  if (state == STATE_INIT) { // only want this called once...but gets called several times even though I unsubscribe...
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
    mqtt.unsubscribe(&timefeed);
    stateChange(STATE_TIME_SET);
    BlinkSuccessInit;
  }
}
/********************************************************************
   CONTROLINPUTCALLLBACK
   Adafruit.io callback we subscribed to in initNetwork().
   I figure it is best to get out of a callback quickly and handle what needs to happen by telling the
   loop() we are in a different state.  We use this
   as the control line to send input to the Controller.  So far the
   input we can send include:
   - threshold...what is the threshold value to use to determine whether to water?
      e.g.: threshold = 200
   - zones...we have three solenoids labeled green, blue, yellow.  They turn the water
     on / off for three zones.
      e.g.: zones = yellow, blue
 *******************************************************************/
void controlInputcallback(char *data, uint16_t len)  {
  uint16_t strlen;
  len > MAX_BYTES ? strlen = MAX_BYTES : len;
  strncpy(controlString, data, strlen);
  stateChange(STATE_IO_COMMAND);
}
/********************************************************************
  handleIoCommand
  Commands:
  Threshold=<value> where value is between 0 and 1000 - most likely around 200.  Parsing AND updating
  the Threshold is handled within the doCommand.

  zone = blue, green, yellow Note: the point is to let the Controller know which zones to water.  Could be zone=blue to
  only water the blue zone, etc.  As with Threshold, the Zones are parsed AND updated within doCommand

  stop tell the Controller to stop all watering.
  start tell the Controller to start watering the zones it should water.

*******************************************************************/
void handleIoCommand() {
  if (!doCommand(controlString, &cachedParams, &ioCommand, feedbackString)) { // string parsing to go from chars to values...might not type in stuff right.
    logText.publish(feedbackString);
    stateChange(STATE_LISTENING_TO_RF69);
  } else {
    logText.publish(controlString);
    if (ioCommand == commandStopWater)  {
      stateChange(STATE_STOP_WATERING);
    } else if (ioCommand == commandStartWater) {
      stateChange(STATE_WATER_FIRST_ZONE);
    } else {
      writeCachedParamsToEEPROM(); // the updated values are cached to EEPROM using the ESP826 EEP  ROM APIs.
      stateChange(STATE_LISTENING_TO_RF69);
    }
  }
}
/********************************************************************
  sendStopWateringCommand
*******************************************************************/
void sendStopWateringCommand() {
  if (bWatering) {
    wateringInfo.values.packetType = packetWateringInfo;
    wateringInfo.values.valvePin =  0xFF;
    rf69_manager.sendtoWait(wateringInfo.b, sizeof(wateringInfo), WATERINGPUCK_ADDRESS);
    DEBUG_PRINTLN(F("...sent a stop watering packet to the watering puck."));
  } else {
    DEBUG_PRINTLNF("...did not send a stop watering packet to the watering puck - currently not watering.");
  }
}
/********************************************************************
   stopWatering
 *******************************************************************/
void stopWatering() {
  bWatering = false;
  Alarm.free(alarmId);
  alarmId = dtINVALID_ALARM_ID;
  DEBUG_PRINTLN(F("Stopped watering."));
  stateChange(STATE_LISTENING_TO_RF69);
}
/********************************************************************
   PRINTTIMEINFO
 *******************************************************************/
void printTimeInfo() {
  Serial.println(F("\n**** Time Info*****"));
#ifdef DEBUG
  Serial.print("...packet type: "); Serial.println(timeInfo.values.packetType);
  Serial.print(timeInfo.values.hour); Serial.print(F(":")); Serial.print(timeInfo.values.minute);
  Serial.print(F(":")); Serial.println(timeInfo.values.second);
  Serial.print(F("Watering hour: ")); Serial.println(timeInfo.values.wateringHour);
#endif
}
/********************************************************************
   printWateringInfo
 *******************************************************************/
void printWateringInfo() {
  DEBUG_PRINTLN(F("\n****** Watering Info ******"));
  DEBUG_PRINT(F("Packet type: ")); DEBUG_PRINTLN(wateringInfo.values.packetType);
  DEBUG_PRINT(F("Number of watering minutes: ")); DEBUG_PRINTLN(wateringInfo.values.numWateringMinutes);
  DEBUG_PRINT(F("Valve Pin number: ")); DEBUG_PRINTLN(wateringInfo.values.valvePin);
}
/********************************************************
   PRINTMOISTUREINFO
 ********************************************************/
void printMoistureInfo() {
#ifdef DEBUG
  DEBUG_PRINTLNF("\n ---> Moisture Info <---");
  DEBUG_PRINTF("Moisture Reading: ");
  Serial.print(moistureInfo.values.moistureReading);
  DEBUG_PRINTF("| Battery level: ");
  Serial.print(moistureInfo.values.batteryLevel);
  DEBUG_PRINTF("| Radio Temperature: ");
  Serial.println(moistureInfo.values.temperatureOfRadioChip);
#endif
}
/********************************************************
   PUBLISHMOISTUREINFO
 ********************************************************/
void publishMoistureInfo() {
  if (moisture.publish(moistureInfo.values.moistureReading) &
      temperature.publish(moistureInfo.values.temperatureOfRadioChip) &
      batteryLevel.publish(moistureInfo.values.batteryLevel)) {
    DEBUG_PRINTLNF("Published!");
    printMoistureInfo();
  } else {
    DEBUG_PRINTLNF("Failed to publish!");
  }
}
/********************************************************************
   resetCachedParams
 *******************************************************************/
void resetCachedParams() {
  cachedParams.values.writeCheck = WRITE_CHECK;
  cachedParams.values.wateringThreshold = gDefaultWateringThreshold;
  cachedParams.values.valvePins[0] = blueValvePin;
  cachedParams.values.valvePins[1] = yellowValvePin;
  cachedParams.values.valvePins[2] = 0xFF;
  writeCachedParamsToEEPROM();
}
/********************************************************
   READCACHEDPARAMSINEEPROM
 ********************************************************/
void readCachedParamsInEEPROM() {
  for (int i = 0; i < sizeof(cachedParams); i++) {
    cachedParams.b[i] = EEPROM.read(i);
  }
#ifdef DEBUG //I shouldn't have to do this but the ESP826 isn't honoring println'ing the variables via macro substitution...
  DEBUG_PRINTF("cached params write check: 0x"); Serial.println(cachedParams.values.writeCheck, HEX);
  DEBUG_PRINTF("cached params watering threshold: "); Serial.println(cachedParams.values.wateringThreshold);
  for (int i = 0; i < 3; i++) {
    Serial.print(F("cached params valvePins[ ")); Serial.print(i); Serial.print(F("]: ")); Serial.println(cachedParams.values.valvePins[i]);
  }
#endif
}
/********************************************************
   WRITECACHEDPARAMSTOEEPROM
 ********************************************************/
void writeCachedParamsToEEPROM() {
  for (int i = 0; i < sizeof(cachedParams); i++) {
    EEPROM.write(i, cachedParams.b[i]);
  }
}
/********************************************************
   logProcessPacketMs
 ********************************************************/
void logProcessPacketMs() {
  char processPacketMsStr[10] = {0};
  strcpy(logStr, "Setting ProcessPacket ms to ");
  strcat(logStr, itoa(processPacketMs, processPacketMsStr, 10));
  strcat(logStr, " ms. ");
#ifdef DEBUG
  Serial.println(logStr);
#else
  logText.publish(logStr);
#endif
}
/********************************************************
   logBelowThreshold
 ********************************************************/
void logBelowThreshold() {
  char moistureReadingStr[10] = {0};
  char wateringThresholdStr[10] = {0};
  strcpy(logStr, "The moisture reading (");
  strcat(logStr, itoa(moistureInfo.values.moistureReading, moistureReadingStr, 10));
  strcat(logStr, ") is below the threshold(");
  strcat(logStr, itoa(cachedParams.values.wateringThreshold, wateringThresholdStr, 10));
  strcat(logStr, ")");
#ifdef DEBUG
  Serial.println(logStr);
#else
  logText.publish(logStr);
#endif

}
/********************************************************
   logWateringZone
 ********************************************************/
void logWateringZone(char *action) {
  strcpy(logStr, action);
  strcat(logStr, " watering the ");
  switch (cachedParams.values.valvePins[gValvePinCounter] ) {
    case greenValvePin:
      strcat(logStr, "Green water zone.");
      break;
    case blueValvePin:
      strcat(logStr, "Blue water zone.");
      break;
    case yellowValvePin:
      strcat(logStr, "Yellow water zone.");
      break;
    default:
      strcat(logStr, "Unknown...");
      break;
  }
#ifdef DEBUG
  Serial.println(logStr);
#else
  logText.publish(logStr);
#endif
}
/********************************************************
   logState
 ********************************************************/
void logState(states_t state) {
  static states_t prevState = STATE_UNKNOWN;
  if (state != prevState) {
    strcpy(logStr, "State changing to: ");
    switch (state) {
      case STATE_TIME_SET:
        strcat(logStr, "STATE_TIME_SET");
        break;
      case STATE_LISTENING_TO_RF69:
        strcat(logStr, "STATE_LISTENING_TO_RF69");
        break;
      case STATE_RECEIVED_MOISTURE_INFO:
        strcat(logStr, "STATE_RECEIVED_MOISTURE_INFO");
        break;
      case STATE_THRESHOLD_CHECK:
        strcat(logStr, "STATE_THRESHOLD_CHECK");
        break;
      case STATE_WATER_FIRST_ZONE:
        strcat(logStr, "STATE_WATER_FIRST_ZONE");
        break;
      case STATE_WATER_NEXT_ZONE:
        strcat(logStr, "STATE_WATER_NEXT_ZONE");
        break;
      case STATE_STOP_WATERING:
        strcat(logStr, "STATE_STOP_WATERING");
        break;
      case STATE_DONE_WATERING:  // All zones (that are signed up for watering) have been watered.
        strcat(logStr, "STATE_DONE_WATERING");
        break;
      case STATE_IO_COMMAND:
        strcat(logStr, "STATE_IO_COMMAND");
        break;
    }
  }
#ifdef DEBUG
  Serial.println(logStr);
#else
  logText.publish(logStr);
#endif
}
/********************************************************
   logIntValue
 ********************************************************/
void logIntValue(int value) {
  char valueStr[10] = {0};
  strcpy(logStr, itoa(value, valueStr, 10));
#ifdef DEBUG
  Serial.println(logStr);
#else
  logText.publish(logStr);
#endif
}

