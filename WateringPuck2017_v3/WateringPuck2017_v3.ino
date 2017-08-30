// WateringPuck2017
// in v1, it was up to the watering puck to water all zones.  But what if a zone doesn't need watering?
// for this reason I moved the logic of what zones need watering to the Controller.
// This code is loaded into the Watering Puck.  The Watering Puck uses Mains power, so
// power management code is not used.
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.
// Use Radio Head's reliable RFM69 packet library:
// http://www.airspayce.com/mikem/arduino/RadioHead/index.html
// - V3 Updated to follow state model of Controller2017_v8.
//********************DEBUG *********************************
//#define DEBUG
#include <DebugLib.h>
//---------------------------------------------------------------------------
// Use Paul's TimeAlarms library to set a timer that goes off when the watering
// valve should go off: https://github.com/PaulStoffregen/TimeAlarms
#include <TimeAlarms.h>
AlarmId alarmId;
//---------------------------------------------------------------------------
//********************RFM69 stuff*********************************
#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
// RFM69 ID numbers
#define WATERINGPUCK_ADDRESS   55
#define CONTROLLER_ADDRESS     10
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           LED_BUILTIN
#endif
#include "Blinkings.h"
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, WATERINGPUCK_ADDRESS);
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
//---------------------------------------------------------------------------
/*************************************************************************/
/* Controller / Watering Puck stuff
*/
const uint8_t packetWateringInfo = 3;
struct wateringInfo_t // not sending battery info because plugged into 24VAC.
{
  uint8_t packetType;
  uint8_t valvePin;
  uint8_t numWateringMinutes;
};
union wateringUnion_t
{
  wateringInfo_t values;
  uint8_t b[sizeof(wateringInfo_t)];
} wateringInfo;

const uint8_t packetStopWatering = 0x04;
enum states {
  unknown,
  startWatering,
  stopWatering
};
states state = unknown;
//---------------------------------------------------------------------------
/********************************************************
   SETUP
 ********************************************************/
void setup() {
  initStuff();
  BlinkSuccessInit;
}
/********************************************************
   LOOP
 ********************************************************/
void loop() {
  Alarm.delay(1); //I'm using a Timer Alarm.  This requires Alarm.delay()
  handleRF69Messages();
  switch (state) {
    case startWatering:
      {
        BlinkStartWatering;
        waterZone();
        state = unknown;
      }
      break;
    case stopWatering:
      {
        BlinkStopWatering;
        stopWateringZone();
        state = unknown;
      }
      break;
    default:
      state = unknown;
      break;
  }
}
/********************************************************
   handleRF69Messages
 ********************************************************/
void handleRF69Messages() {
  if (rf69_manager.available()) {
    BlinkReceivedMessage;
    delay(100);
    uint8_t len = sizeof(buf);
    uint8_t from;
    DEBUG_PRINTLN(F("Received a message."));
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      if (buf[0] == packetWateringInfo) {
        memcpy(wateringInfo.b, buf, sizeof(wateringInfo));
        if (wateringInfo.values.valvePin == 0xFF) {
          state = stopWatering;
        } else {
          state = startWatering;
        }
      }
    }
  }
}
/********************************************************
   WATERZone
   - water each area/zone for wateringInfo.values.numWateringMinutes.
   a timer is set for numSeconds.  When the timer goes off, the stopWatering() function is called.

 ********************************************************/
void waterZone() {
  pinMode(wateringInfo.values.valvePin, OUTPUT);
  time_t numSeconds = wateringInfo.values.numWateringMinutes * 60;
  DEBUG_PRINTF("Turning valve "); DEBUG_PRINT(wateringInfo.values.valvePin); DEBUG_PRINTF(" on for ");
  DEBUG_PRINT(numSeconds); DEBUG_PRINTLNF(" seconds.");
  digitalWrite(wateringInfo.values.valvePin, HIGH);
  alarmId = Alarm.timerOnce(numSeconds, stopWateringZone);
}
/********************************************************
   STOPWATERING
   - turn the current valve off
   - increase the zones watered countered since this one is done.
   - call waterZone() and let waterZone() decide if there is more to water or we're done.
 ********************************************************/
void stopWateringZone() {
  Alarm.free(alarmId);
  alarmId = dtINVALID_ALARM_ID;
  // turn water off
  DEBUG_PRINTLNF("Turning valve off");
  digitalWrite(wateringInfo.values.valvePin, LOW);
  DEBUG_PRINTLN(F("Let the controller know we have stopped watering."));
  rf69_manager.sendtoWait((uint8_t *)&packetStopWatering, 1, CONTROLLER_ADDRESS) ;
}

/********************************************************
   INITSTUFF
   - init the GPIO pins to send on/off to the valves.
   - init the RFM69 radio.
 ********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  DEBUG_PRINTLNF("...initializing stuff for the watering puck...");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  initRadio();

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
    DEBUG_PRINTLNF(F("Could not initialize RH manager."));
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
  DEBUG_PRINTF("RFM69 radio @");  DEBUG_PRINT((int)RF69_FREQ);  DEBUG_PRINTLNF(" MHz");
}
/********************************************************
  printWaterInfoPacket
  Used while #DEBUG.
********************************************************/
void printWaterInfoPacket(uint8_t from) {
  DEBUG_PRINTF("...got watering Info packet from node: "); DEBUG_PRINTLN(from);
  DEBUG_PRINTF("...valve pin: "); DEBUG_PRINTLN(wateringInfo.values.valvePin);
  DEBUG_PRINTF("...water for "); DEBUG_PRINT(wateringInfo.values.numWateringMinutes); DEBUG_PRINTLNF(" minutes.");
}

