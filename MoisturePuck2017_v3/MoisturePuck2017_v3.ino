// MoisturePuck2017_v3
// This code is loaded into the Feather M0 RFM69 that has a moisture sensor attached to it
// and will sit in some soil getting moisture readings.  The moisture readings are sent to
// the Controller - another piece of hw/sw.  I wrote a blog post on this (http://bitknitting.wordpress.com)
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>
#include <RTCZero.h>
RTCZero rtc;

// RFM69 ID numbers
#define PUCK_ADDRESS           4
#define CONTROLLER_ADDRESS     1
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           LED_BUILTIN
#endif
const int POWER = 12;
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, PUCK_ADDRESS);
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
/********************************************************
   SETUP
 ********************************************************/
void setup()
{
  //Serial.begin(115200);
  //while (!Serial);                                                                                                                                                                     
  initStuff();
  //Adding initial delay to allow for time to get Moisture Puck assembled and into garden before first reading
  delay( 900000); //15 minutes = 15 min * 60 sec/min * 1000 ms/sec 
  wakeUp();
}
/********************************************************
   LOOP
 ********************************************************/
void loop()
{ 
  //Serial.println("Sending moisture info packet to the Controller");
  setUpAMoistureInfoPacket();
  if (rf69_manager.sendtoWait(moistureInfo.b, sizeof(moistureInfo), CONTROLLER_ADDRESS))
  {
    // Now wait for a timeInfo packet from the Controller
    uint8_t len = sizeof(timeInfo);
    uint8_t from;
    if (rf69_manager.recvfromAckTimeout(timeInfo.b, &len, 2000, &from))
    {
      //Serial.print("got Time Info packet from from : 0x");//Serial.println(from, HEX);
      rtc.setTime(timeInfo.values.hour, timeInfo.values.minute, timeInfo.values.second);
      printTimeInfo();
      rf69.sleep();
      goToSleep();
    }
    else
    {
      //Serial.println("No reply from the Controller");
    }
  }
  else
  {
    //Serial.println("sendtoWait failed");
  }
}
/********************************************************
   READMOISTURE
 ********************************************************/
int readMoisture() {
  //read the moisture sensor...take a bunch of readings and calculate an average
  // turn the sensor on and wait a moment...
  delay(10);
  const int nReadings = 40;
  float sumOfReadings = 0.;
  for (int i = 0; i < nReadings; i++) {
    sumOfReadings += analogRead(A0);  // !!!The moisture sensor must be on this analog pin.!!!
  }
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
   "The RF69 must be in Idle mode (= RF69 Standby) to measure temperature."
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
void printTimeInfo() {
  //Serial.println("\n**** Time Info*****");
  //Serial.print(timeInfo.values.hour); //Serial.print(":"); //Serial.print(timeInfo.values.minute);
  //Serial.print(":"); //Serial.println(timeInfo.values.second);
  //Serial.print("AM watering hour: "); //Serial.print(timeInfo.values.AM_wateringHour); //Serial.print(" PM watering hour: ");
  //Serial.println(timeInfo.values.PM_wateringHour);
}
/********************************************************
  WAKEUP - called back by rtc when an alarm goes off
  haven't sent any moisture readings, so reset the counters.
********************************************************/
void wakeUp() {
  //Serial.println("\nI'm Awake!");
  digitalWrite(POWER,HIGH);
}
/********************************************************
   GOTOSLEEP
 ********************************************************/
void goToSleep() {
  digitalWrite(POWER,LOW);
  int hour = rtc.getHours();
  int alarmHour = (hour >= timeInfo.values.AM_wateringHour && hour < timeInfo.values.PM_wateringHour) ? timeInfo.values.PM_wateringHour : timeInfo.values.AM_wateringHour;
  //Serial.print("Setting the alarm for: "); //Serial.print(alarmHour); //Serial.println("...talk to you then.");
  rtc.setAlarmTime(alarmHour, 00, 00);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  //Serial.println("\nGoing to sleep");
  rtc.standbyMode();
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  // The Moisture sensor's v+ is connected to the POWER GPIO pin.
  pinMode(POWER,OUTPUT);
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
    //Serial.println(F("Could not initialize RH manager."));
    while (1) {
      Blink(LED, 400, 10);
    }
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
  INITRTC

********************************************************/
void initRtc() {
  rtc.begin();
  rtc.attachInterrupt(wakeUp);
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
