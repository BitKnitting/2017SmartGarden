// MoisturePuck2017_v4
// This code is loaded into the Feather M0 RFM69 that has a moisture sensor attached to it
// and will sit in some soil getting moisture readings.  The moisture readings are sent to
// the Controller - another piece of hw/sw.  I wrote a blog post on this (http://bitknitting.wordpress.com)
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.
// V4 - changed to one watering time.
//#define LED_DEBUG // Using the LED for debugging lets me evaluate the exact code path that will happen in the field.
//#define DEBUG  // Either LED_DEBUG OR DEBUG OR neither...
#include <DebugLib.h>
#ifdef DEBUG
bool bAwake = true;
#endif
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
// We use the RTC library to put the m0 into deep sleep.  I couldn't get the RFM69 interrupt to wake up the m0 from deep sleep,
// but the RTC library worked.  Unfortunately, only one alarm can be active.  I was hoping on setting multiple alarms and taking
// readings several times a day.  For now, I take one reading a day at the watering time.  There is also a reading happening pretty
// soon after the script is loaded to check if the Controller does indeed talk to the moisture puck.
#include <RTCZero.h>
RTCZero rtc;

// RFM69 ID numbers
#define WATERINGPUCK_ADDRESS   55
#define PUCK_ADDRESS           25
#define CONTROLLER_ADDRESS     10
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           LED_BUILTIN
#endif
#include "Blinkings.h"
const int POWER = 12;
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, PUCK_ADDRESS);
uint8_t gCounterMessageSending = 0;

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

bool bHaveTimeInfo = false;
/********************************************************
   SETUP
 ********************************************************/
void setup()
{
  initStuff();
  //Adding initial delay to allow for time to get Moisture Puck assembled and into garden before first reading
#if !defined(DEBUG) && !defined(LED_DEBUG)
  delay( 300000); //# ms in 5 minutes = 5 min * 60 sec/min * 1000 ms/sec
#endif

}
/********************************************************
   LOOP
 ********************************************************/
void loop()
{
#ifdef DEBUG
  if (bAwake) {
#endif
    // I could be awake because I was just reloaded or I woke up.  If I was just reloaded, I won't have time info.
    // OR... the RTC alarm could have fired (which is set to go off at the watering time).  At this point, I should have
    // the time info and I'm awake to read the sensors and send the results to the controller.
    if (bHaveTimeInfo) {
      DEBUG_PRINTLN(F("Sending moisture info packet to the Controller"));
      setUpAMoistureInfoPacket();
      printMoistureInfo();
      if (rf69_manager.sendtoWait(moistureInfo.b, sizeof(moistureInfo), CONTROLLER_ADDRESS)) {
        DEBUG_PRINTLNF("...moisture info packet successfully delivered....");
        goToSleep();
      }
      // I'm using a global counter so that we don't endlessly loop sending a moisture info packet.  We'll know if
      // packets aren't received by looking at the Adafruit.io log file.
      // The globalCounter is set back to 0 in the wake up call back.
      if (++gCounterMessageSending > 10) {  // give up sending a moisture info packet.  Next thing to do is go to sleep until the next wake up time.
        goToSleep();
      }
    } else { // Ask the Controller to send a timeInfo packet.  This happens as soon as the code starts running.
      timeInfo.values.packetType = packetTimeInfo;
      if (rf69_manager.sendtoWait(timeInfo.b, sizeof(timeInfo), CONTROLLER_ADDRESS)) {
        uint8_t len = sizeof(timeInfo);
        uint8_t from;
        delay(500);
        if (rf69_manager.recvfromAck(timeInfo.b, &len, &from)) {
          DEBUG_PRINTLN(F("...received a Time Info message..."));
          setRTC();
          bHaveTimeInfo = true;
          printTimeInfo();
        }
      }
    }
#ifdef DEBUG  // for bAwake 
  }
#endif
}
/********************************************************
  READMOISTURE
********************************************************/
int readMoisture() {
  //read the moisture sensor...take a bunch of readings and calculate an average
  // turn the sensor on and wait a moment...
  digitalWrite(POWER, HIGH);
  delay(10);
  const int nReadings = 40;
  float sumOfReadings = 0.;
  for (int i = 0; i < nReadings; i++) {
    sumOfReadings += analogRead(A0);  // !!!The moisture sensor must be on this analog pin.!!!
  }
  // Turn off power to the moisture sensor.
  digitalWrite(POWER, LOW);
  return ( round(sumOfReadings / nReadings));
}
/********************************************************
  READBATTERYLEVEL
  // Code from Adafruit's learn section:https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/overview?view=all#measuring-battery
********************************************************/
#define VBATPIN A7
float readBatteryLevel() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return (measuredvbat);
}
/********************************************************
  GETTEMPERATUREFROMRADIO
  The code is my attempt to follow the recommendation in
  the RH documentation : http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF69.html#af66bfbf2057f1faeba0ad75007d623ec
  "The RF69 must be in Idle mode ( = RF69 Standby) to measure temperature."
********************************************************/
int8_t getTemperatureFromRadio() {
  rf69.setModeIdle();
  int8_t temp = rf69.temperatureRead() * 1.8 + 32;
  rf69.sleep();
  return (temp);
}

/********************************************************
  SETUPAMOISTUREINFOPACKET
********************************************************/
void setUpAMoistureInfoPacket () {
  moistureInfo.values.packetType = packetMoistureInfo;
  moistureInfo.values.moistureReading = readMoisture();
  moistureInfo.values.batteryLevel = readBatteryLevel();
  moistureInfo.values.temperatureOfRadioChip = getTemperatureFromRadio();
}
/********************************************************
   PRINTMOISTUREINFO
 ********************************************************/
void printMoistureInfo() {
  DEBUG_PRINTLNF("\n ---> Moisture Info <---");
  DEBUG_PRINTF("Moisture Reading: ");
#ifdef DEBUG
  Serial.print(moistureInfo.values.moistureReading);
  DEBUG_PRINTF("| Battery level: ");
  Serial.print(moistureInfo.values.batteryLevel);
  DEBUG_PRINTF("| Radio Temperature: ");
  Serial.println(moistureInfo.values.temperatureOfRadioChip);
#endif
}
/********************************************************************
   PRINTTIMEINFO
 *******************************************************************/
void printTimeInfo() {
  DEBUG_PRINTLN(F("\n**** Time Info*****"));
#ifdef DEBUG
  Serial.print(timeInfo.values.hour); Serial.print(F(":")); Serial.print(timeInfo.values.minute);
  Serial.print(F(":")); Serial.println(timeInfo.values.second);
  Serial.print(F("Watering hour: ")); Serial.println(timeInfo.values.wateringHour);
#endif
}
/********************************************************
  WAKEUP - called back by rtc when an alarm goes off
  haven't sent any moisture readings, so reset the counters.
********************************************************/
void wakeUp() {
  DEBUG_PRINTLN(F("\nI'm Awake!"));
  gCounterMessageSending = 0;
}
/********************************************************
  goToSleep
********************************************************/
void goToSleep() {
  rf69.sleep();
#ifndef DEBUG
  BlinkGoToSleep;
  rtc.standbyMode();
#endif
}
/********************************************************
  setRTC
********************************************************/
void setRTC() {
  // set the rtc library's time so the watering alarm time is correct.
  rtc.setTime(timeInfo.values.hour, timeInfo.values.minute, timeInfo.values.second);
  // Now that we have a time packet, we can set an alarm to fire when we should send a moisture info packet to the Controller.
  rtc.setAlarmTime(timeInfo.values.wateringHour, 00, 00);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
}
/********************************************************
  INITSTUFF
********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  // The Moisture sensor's v + is connected to the POWER GPIO pin.
  pinMode(POWER, OUTPUT);
  initRadio();
  initRtc();
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
    DEBUG_PRINTLN(F("Could not initialize RH manager."));
    while (1);
  }
  //Serial.print("RFM69 radio @");  //Serial.print((int)RF69_FREQ);  //Serial.println(" MHz");
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
}
/********************************************************
  INITRTC

********************************************************/
void initRtc() {
  rtc.begin();
  rtc.attachInterrupt(wakeUp);
}


