/*
   The Controller fits the client examples in the RH library.
*/
#define DEBUG
#include <DebugLib.h>
/*
   Using Paul Stroffregen's TimeAlarms library.  GitHub location: https://github.com/PaulStoffregen/TimeAlarms
   Time alarms are used to set when to ask moisture sticks for dry/wet moisture info.
*/
#include <TimeAlarms.h>
//***********************************************************************
/*
   Stuff for the RFM69 See Adafruit's fork of RadioHead Library: https://github.com/adafruit/RadioHead
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
// Class to manage message delivery and receipt, using the driver declared above
#define MY_ADDRESS            1
#define MOISTURESTICK_ADDRESS 2
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);
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
//***********************************************************************
void setup() {
  initStuff();
  getMoistureInfo();
}

void loop() {
  //The Time Alarm callbacks were not called unless Alarm.Delay() was in the loop.
  Alarm.delay(1);

}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  initRadio();
  setWateringTimes();

}
/********************************************************
   INITRADIO
 ********************************************************/
void initRadio() {
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  DEBUG_PRINTLNF("***Garden Controller***\n");

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
   SETWATERINGTIMES
 ********************************************************/
const int AMwateringHour = 5;
const int PMwateringHour = 17;
void setWateringTimes() {
#ifdef DEBUG
  Alarm.timerRepeat(3600, getMoistureInfo);  // 1 hour * 60 min / hour * 60 sec / min
#else
  //set the AM watering window
  Alarm.alarmRepeat(AMwateringHour, 0, 0, getMoistureInfo); // Fire timer in early morning and find out if watering is needed.
  //set the PM watering window
  Alarm.alarmRepeat(PMwateringHour, 0, 0, getMoistureInfo);
#endif
}
/********************************************************
   GETMOISTUREREADING
 ********************************************************/
void getMoistureInfo() {
  // Send a message to manager_server
  byte command = 0;
  if (rf69_manager.sendtoWait(&command, 1, MOISTURESTICK_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(moistureInfo);
    uint8_t from;
    // from = node == 2 -> node 2 is in Strawberry Swirl.
    if (rf69_manager.recvfromAckTimeout(moistureInfo.b, &len, 2000, &from))
    {
      DEBUG_PRINTF("...got reply from node ");
      DEBUG_PRINT(from);
      DEBUG_PRINTF(" | RSSI: ");
      DEBUG_PRINTLN(rf69.lastRssi());
      printMoistureInfo();
    }
    else
    {
      DEBUG_PRINTLNF("No reply from a moisture stick...");
    }
  }
  else
    DEBUG_PRINTLNF("sendtoWait failed");
}
/********************************************************
   PRINTMOISTUREINFO
 ********************************************************/
void printMoistureInfo() {
  DEBUG_PRINTLNF(" ---> Moisture Info <---");
  DEBUG_PRINTF("Moisture Reading: ");
  DEBUG_PRINT(moistureInfo.values.moistureReading);
  DEBUG_PRINTF("| Battery level: ");
  DEBUG_PRINT(moistureInfo.values.batteryLevel);
  DEBUG_PRINTF("| Radio Temperature: ");
  DEBUG_PRINTLN(moistureInfo.values.temperatureOfRadioChip);
}

