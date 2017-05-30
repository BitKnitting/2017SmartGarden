/*
   MoisturePuck.ino

   The RadioHead library examples (for the most part) have a client/server
   paradigm.  In this case, the Moisture Puck(s) are servers and the Controller
   is the client requesting packets from the server.
*/
//#define DEBUG
#include <DebugLib.h>
//***********************************************************************
/*
   Stuff for the RFM69
*/
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
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
#define MY_ADDRESS     2
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);
// Dont put this on the stack:
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
   SETUP
 ********************************************************/
void setup() {
  initStuff();
}
/********************************************************
   LOOP
 ********************************************************/
void loop() {
  sendMoistureInfoOnRequest();
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  initRadio();
}
/********************************************************
   INITRADIO
 ********************************************************/
void initRadio() {
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  DEBUG_PRINTLNF("***MOISTURE STICK***\n");

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

  pinMode(LED, OUTPUT);

  DEBUG_PRINTF("RFM69 radio @");  DEBUG_PRINT((int)RF69_FREQ);  DEBUG_PRINTLNF(" MHz");
}
/********************************************************
   SENDMOISTUREINFO
 ********************************************************/
void sendMoistureInfoOnRequest() {
  if (rf69_manager.available() ) {  //RH server examples use this.
    DEBUG_PRINTLNF("...RF69 manager is available");
    // Wait for a message addressed to us from the client (i.e.: the Controller)
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      DEBUG_PRINTF("got request from node ");
      DEBUG_PRINT(from);
      DEBUG_PRINTF(" | RSSI: ");
      DEBUG_PRINTLN(rf69.lastRssi());
      //Send Moisture Info back to node that requested it.
      moistureInfo.values.moistureReading = readMoisture();
      moistureInfo.values.batteryLevel = readBatteryLevel();
      moistureInfo.values.temperatureOfRadioChip = getTemperatureFromRadio();
      if (!rf69_manager.sendtoWait(moistureInfo.b, sizeof(moistureInfo), from)) {
        DEBUG_PRINTLNF("sendtoWait failed");
      }
      rf69.sleep();
    }
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
    sumOfReadings += analogRead(A0);  // The moisture sensor must be on this analog pin.
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

