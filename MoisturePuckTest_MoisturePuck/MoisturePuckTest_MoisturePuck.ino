/******************************************************
 * This script is used to test if the moisture puck can
 * send packets and test the values in the moisture info
 * structure.
 * Load this script on the moisture puck.
 * 
 * Run MoisturePuckTest_Tester.ino on another Feather M0 RFM69.
 */ 
#define DEBUG
#include <DebugLib.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// RFM69 ID numbers
#define PUCK_ADDRESS           2
#define CONTROLLER_ADDRESS     10

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           LED_BUILTIN
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, PUCK_ADDRESS);
/******************************** Moisture Puck Stuff *******************************/
const int POWER = 12;
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
/*************************************
   SETUP
 *************************************/
void setup() {
  initStuff();
}
/*************************************
   LOOP
 *************************************/
void loop() {
  DEBUG_PRINTLN(F("In loop"));
  // Take readings and put into the moisture info structure.
  makeMoistureInfoPacket();
  // Send to the tester.
  if (rf69_manager.sendtoWait(moistureInfo.b, sizeof(moistureInfo), CONTROLLER_ADDRESS)) {
    DEBUG_PRINTLNF("...moisture info packet successfully delivered....");
  }
  // Wait a bit before sending another packet..
  delay(5000);
}
/*
   MAKEMOISTUREINFOPACKET
*/
void makeMoistureInfoPacket() {
  moistureInfo.values.packetType = packetMoistureInfo;
  moistureInfo.values.moistureReading = readMoisture();
  moistureInfo.values.batteryLevel = readBatteryLevel();
  moistureInfo.values.temperatureOfRadioChip = getTemperatureFromRadio();
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
