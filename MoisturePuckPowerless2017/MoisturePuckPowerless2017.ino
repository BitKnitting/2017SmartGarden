/*
  MoisturePuckPowerless.ino

  // Copyright (c) 2017 Margaret Johnson

  // Permission is hereby granted, free of charge, to any person obtaining a copy
  // of this software and associated documentation files (the "Software"), to deal
  // in the Software without restriction, including without limitation the rights
  // to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  // copies of the Software, and to permit persons to whom the Software is
  // furnished to do so, subject to the following conditions:

  // The above copyright notice and this permission notice shall be included in all
  // copies or substantial portions of the Software.

  // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  // IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  // FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  // AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  // LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  // OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  // SOFTWARE.
  //
  // This is the firmware for the Feather M0/RFM69 915MHz Moisture Puck.  It uses the
  // RadioHead library to communicate with a Controller.  The Moisture Puck measures
  // the moisture content of the soil in which it's probe is inserted.
  //
  // This version adds in power management code to get more time out of the LiPo battery.
  // I was getting about 44 hours before I had to recharge the LiPo without power management
  // code.  The power management code is specific to the M0.
*/

//***********************************************************************
/*
   Stuff for the RFM69
*/
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
// Add real time clock power management
#include <RTCZero.h>
RTCZero rtc;
// It's 'nice' to keep in the if def to make sure the right board is being used.
#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
// change addresses for each client board, any number :)
#define MY_ADDRESS          2
#define CONTROLLER_ADDRESS  1
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);
// Dont put this on the stack...packet received from the Controller...using a buffer
// that is the max size that the Controller can send.:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
//***********************************************************************
/********************************************************
   Data sent from the moisture stick.  There is the higher level
   view seen in moistureInfo_t, and the "on the wire view" seen in
   moistureInfo.b.  Having these two views of the data makes it easy
   to translate from a wire packet to an easily used typedef.
 ********************************************************/
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
/********************************************************
   The current time and AM/PM watering hours are set by
   the Controller.
 ********************************************************/
struct timeInfo_t
{
  uint8_t packetType;  //to identify if this is indeed a timeInfo packet.
  time_t currentTime;
  uint8_t AM_wateringHour;
  uint8_t PM_wateringHour;
};
union timeUnion_t
{
  timeInfo_t values;
  uint8_t b[sizeof(timeInfo_t)];
} timeInfo;
const byte packet_time = 1;
const byte packet_moisture_received = 2;
//*************************************************************************
//states
enum states {
  state_waitingForTimeInfo,
  state_okToSendMoistureInfo,
  state_okToSleep
} currentState;

//*************************************************************************
// Not using the serial port because once the M0 goes to sleep and then wakes back up,
// the USB serial port is reattached.  At least for me, even after opening a new serial monitor
// i can't get to Serial.println results...so I am using the built in LED of the feather for debug info.
#define BLINK
#ifdef BLINK
const int errorBlinkRate = 1000;
const int okBlinkRate = 400;
#endif
/********************************************************
   SETUP
 ********************************************************/
void setup() {
  initStuff();
}
/********************************************************
   LOOP
 ********************************************************/
void loop() {
  idle();
#ifdef BLINK
  digitalWrite(LED, HIGH);
#endif
  if (rf69_manager.available() && (currentState == state_waitingForTimeInfo) ) { //This *should* be a timeInfo packet
    getTimeInfo();
  }
  if (currentState == state_okToSendMoistureInfo) {
    sendMoistureInfo();
  }
  if (currentState == state_okToSleep) {
    goToSleep();
  }
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  currentState = state_waitingForTimeInfo;
  rtc.begin();
  rtc.attachInterrupt(wakeUp);
  digitalWrite(LED, HIGH);
  initRadio();
#ifdef BLINK
  pinMode(LED, OUTPUT);
#endif
  //Start delay of 10s to allow for new upload after reset
  //When I didn't have this in, I couldn't get back to the bootloader when
  //I put the m0 in standby mode. Since then I'm cautious.
  delay(10000);
#ifdef BLINK
  blink(5, okBlinkRate);
#endif
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
    while (1) {  //if BLINK is defined it means the board is in front of us..
#ifdef BLINK
      blink(5, errorBlinkRate);
      delay(3000);
#endif
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
}
/*
   getTimeInfo - we're currently in state_waitingForTimeInfo and a packet has come in from the Controller
   hopefully, the packet contains a timeInfo structure.  We rely on this to set when to listen for requests
   from the Controller for moisture readings.  So the first thing is to check if indeed the packet contains timeInfo
   if it does, we can change state as well as set the rtc time which the alarm will then be based from.
*/
#include <Time.h> //RTCZero.cpp includes time.h, so use it here to avoid conflict with TimeLib.h
void getTimeInfo() {
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (rf69_manager.recvfromAck(buf, &len, &from)) {
    memcpy(&timeInfo, buf, sizeof(timeInfo_t));
    rtc.setTime( hour(timeInfo.values.currentTime), minute(timeInfo.values.currentTime), second(timeInfo.values.currentTime));
    currentState = state_okToSendMoistureInfo;
  }
}

/********************************************************
   SENDMOISTUREINFO
 ********************************************************/
void sendMoistureInfo() {
#ifdef BLINK
  blink(1, okBlinkRate);
#endif
  //Send Moisture Info back to node that requested it.
  moistureInfo.values.moistureReading = readMoisture();
  moistureInfo.values.batteryLevel = readBatteryLevel();
  moistureInfo.values.temperatureOfRadioChip = getTemperatureFromRadio();
  // Send a message to the Controller
  if (rf69_manager.sendtoWait(moistureInfo.b, sizeof(moistureInfo), CONTROLLER_ADDRESS)) {
    //Now wait for a reply that lets us know the Controller got the moistureInfo
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      if (buf[0] == packet_moisture_received) {
        currentState = state_okToSleep;
      }
    }
  }
  rf69.sleep();
}
/*
   goToSleep() is a request made by the Controller
*/
void goToSleep() {
  byte alarmHour = setWateringHour();
  rtc.setAlarmTime(alarmHour, 0, 0);
  rf69.sleep();
  rtc.standbyMode();    // Sleep until next alarm match
}
byte setWateringHour() {
  byte currentHour = rtc.getHours();
  return (currentHour > timeInfo.values.AM_wateringHour && currentHour < timeInfo.values.PM_wateringHour) ?
         timeInfo.values.PM_wateringHour : timeInfo.values.AM_wateringHour;
}
void wakeUp() {
  currentState = state_okToSendMoistureInfo;
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
/*
   IDLESLEEP - thanks to cmpxchg8b: https://forums.adafruit.com/viewtopic.php?f=57&t=104548&p=523829&sid=22b8d0735f65f3f53a0f1e8781c886e6#p523829
*/
void idle()
{
  // Select IDLE, not STANDBY, by turning off the SLEEPDEEP bit
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

  // Select IDLE mode 2 (asynchronous wakeup only)
  PM->SLEEP.bit.IDLE = 2;

  // ARM WFI (Wait For Interrupt) instruction enters sleep mode
  __WFI();
}
/*
   BLINK
*/
void blink(int nTimesToBlink, int rate) {
  int i = 1;
  while (i <= nTimesToBlink) {
    digitalWrite(LED, HIGH);
    nonBlockDelay(rate);
    digitalWrite(LED, LOW);
    nonBlockDelay(rate);
    i++;
  }
}
/*
   NONBLOCKDELAY
*/
void nonBlockDelay(int rate) {
  int startingMillis = millis();
  while (millis() - startingMillis < rate);
}

