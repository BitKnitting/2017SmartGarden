// MoisturePuck2017_v2
// This code is loaded into the Feather M0 RFM69 that has a moisture sensor attached to it
// and will sit in some soil getting moisture readings.  The moisture readings are sent to
// the Controller - another piece of hw/sw.  I wrote a blog post on this (http://bitknitting.wordpress.com)
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <RTCZero.h>
RTCZero rtc;
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// RFM69 ID numbers
#define PUCK_ADDRESS           2
#define CONTROLLER_ADDRESS     1

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
/********************************************************
   Data sent from the moisture stick.  There is the higher level
   view seen in moistureInfo_t, and the "on the wire view" seen in
   moistureInfo.b.  Having these two views of the data makes it easy
   to translate from a wire packet to an easily used typedef.
 ********************************************************/
uint8_t numMoistureInfoSends = 0;  //I need to count how many times I repeat sending moisture readings before I go to sleep.
uint8_t maxMoistureInfoSends = 2;  //I've been told to send two moisture readings before I go to sleep.
uint8_t numMoistureInfoFails = 0;
uint8_t maxMoistureInfoFails = 20;  //I've been told to try to send with a guaranteed delivery.  If the delivery doesn't happen after repeated attempts, I stop trying this time.
uint8_t numTimesCheckedForMessages = 0;
uint8_t MaxTimesCheckForMessages = 10;
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
//*************************************************************************
//timeInfo
const uint8_t packetTime = 1;
const uint8_t defaultAM_wateringHour = 4;
const uint8_t defaultPM_wateringHour = 21;
struct timeInfo_t
{
  uint8_t packetType;  //should be packetTime to identify if this is indeed a timeInfo packet.
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

void setup()
{
  Serial.begin(115200);
  while (!Serial) ; // wait until serial console is open, remove if not tethered to computer
  initStuff();
  // variable for delay is an unsigned long ->  0 to 4,294,967,295 (2^32 - 1).
  //delay(1200000); //delay 20 minutes to allow for putting box together and getting into garden 20 * 60 * 1000 =
}

/********************************************************
   LOOP
   Most of the time I sleep.  Sleep gives me the strength for
   a longer life.
   The main job is to get a moisture reading to the Controller
   I have only been trained to handle time messages.
   I will go to sleep after a couple tries sending moisture readings.
   So there is a bit of time to get messages from the Controller.
 *********************************************************/
void loop() {
  Serial.println("in loop");
  sendMoistureInfo();
  checkMessages();
  if ( ((numMoistureInfoSends == maxMoistureInfoSends) ||  (numMoistureInfoFails == maxMoistureInfoFails) ) &&
       (numTimesCheckedForMessages == MaxTimesCheckForMessages) ) {
    goToSleep();
  }
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  pinMode(LED, OUTPUT);
  pinMode(LED, LOW);
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
  //  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}
/********************************************************
  INITRTC

********************************************************/
void initRtc() {
  rtc.begin();
  rtc.attachInterrupt(wakeUp);
  setTimeInfoTo__TIME__();
}
/********************************************************
  WAKEUP - called back by rtc when an alarm goes off
  haven't sent any moisture readings, so reset the counters.
********************************************************/
void wakeUp() {
  numMoistureInfoSends = 0;
  numMoistureInfoFails = 0;
  numTimesCheckedForMessages = 0;
  //  Serial.println("I'm Awake!");
}
/********************************************************
   SETTIMEINFOTO__TIME__
   Set all the timeInfo fields.
   I start off only knowing the time at which I was compiled.
   Hopefully, I'll get a message from my boss telling me what
   the real time is.
 ********************************************************/
void setTimeInfoTo__TIME__() {
  //HH:MM:SS
  char time[10];
  strcpy(time, __TIME__);
  // Replace ":" with 0 so that there are three zero terminated strings.
  time[2] = 0;
  time[5] = 0;
  time[8] = 0;
  // Then go through the string and convert hour/minute/seconds to ints.
  timeInfo.values.hour = atoi(time);
  char *iptr = time + 3;
  timeInfo.values.minute = atoi(iptr);
  iptr = iptr + 3;
  timeInfo.values.second = atoi(iptr);
  rtc.setTime(timeInfo.values.hour, timeInfo.values.minute, timeInfo.values.second);
  timeInfo.values.AM_wateringHour = defaultAM_wateringHour;
  timeInfo.values.PM_wateringHour = defaultPM_wateringHour;
}
/********************************************************
   SENDMOISTUREINFO
 ********************************************************/
void sendMoistureInfo() {
  Serial.println(".....Send moisture reading");
  moistureInfo.values.packetType = packetMoistureInfo;
  moistureInfo.values.moistureReading = readMoisture();
  moistureInfo.values.batteryLevel = readBatteryLevel();
  moistureInfo.values.temperatureOfRadioChip = getTemperatureFromRadio();
  moistureInfo.values.hour = rtc.getHours();
  moistureInfo.values.minute = rtc.getMinutes();
  moistureInfo.values.second = rtc.getSeconds();;
  // Send a message to the Controller
  if (rf69_manager.sendtoWait(moistureInfo.b, sizeof(moistureInfo), CONTROLLER_ADDRESS)) {
    rf69.sleep();
    numMoistureInfoSends++;
    Serial.print("Sending moistureInfo try #: "); Serial.println(numMoistureInfoSends);
    // Go to sleep after the number of moisture readings the boss wants me to send have been sent.
  } else {
    numMoistureInfoFails++;
  }
}
/********************************************************
   GOTOSLEEP
 ********************************************************/
void goToSleep() {
  // first set an alarm when to wake up
  int hour = rtc.getHours();
  int alarmHour = (hour >= timeInfo.values.AM_wateringHour && hour < timeInfo.values.PM_wateringHour) ? timeInfo.values.PM_wateringHour : timeInfo.values.AM_wateringHour;
  //Serial.print("Setting the alarm for: "); Serial.print(alarmHour); Serial.println("...talk to you then.");
  rtc.setAlarmTime(alarmHour, 00, 00);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  Serial.println("Going to sleep");
  rtc.standbyMode();
}
/********************************************************
   CHECKMESSAGES
   The Controller might have messages for me.  Right now,
   I have only been trained to handle time messages.
 ********************************************************/
void checkMessages() {
  numTimesCheckedForMessages++;
  Serial.print("...check messages.  This is message check # "); Serial.println(numTimesCheckedForMessages);
  if (rf69_manager.available()) {
    //handle if time info
    uint8_t len = sizeof(timeInfo);
    uint8_t from;
    if (rf69_manager.recvfromAck(timeInfo.b, &len, &from)) {
      Serial.println("...OOOH! got a message...");
      if ((timeInfo.values.packetType == packetTime) && (len == sizeof(timeInfo)) ) {
        Serial.println("Setting the time to:");
        Serial.print(timeInfo.values.hour); Serial.print(":"); Serial.print(timeInfo.values.minute); Serial.print(":"), Serial.println(timeInfo.values.second);
        rtc.setTime(timeInfo.values.hour, timeInfo.values.minute, timeInfo.values.second);
      }
    }
  } else {
    Serial.println("...no messages available at this time.  Carry on...");
  }
}
/********************************************************
   READMOISTURE
 ********************************************************/
int readMoisture() {
  //read the moisture sensor...take a bunch of readings and calculate an average
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
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
