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
RHReliableDatagram rf69_manager(rf69, CONTROLLER_ADDRESS);
/******************************** Moisture Puck Stuff *******************************/
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
  // The Moisture puck repeatedly sends moisture info packets.
  if (rf69_manager.available()) {
    // Packet came in from moisture puck.
    uint8_t from;
    uint8_t len = sizeof(moistureInfo_t);
    if (rf69_manager.recvfromAck(moistureInfo.b, &len,&from)) {
      printMoistureInfo();
      
    }
  }
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
  INITSTUFF
********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
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
