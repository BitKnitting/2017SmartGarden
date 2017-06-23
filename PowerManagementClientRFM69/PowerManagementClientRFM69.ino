/*
   The Controller fits the client examples in the RH library.
*/
#define DEBUG
#include <DebugLib.h>
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
#define DEST_ADDRESS          2
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission
//*********************************************************************
void setup() {
  initStuff();
}
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

  char radiopacket[20] = "Hello World #";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  
  // Send a message to the DESTINATION!
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string
      
      Serial.print("Got reply from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);     
    } else {
      Serial.println("No reply, is anyone listening?");
    }
  } else {
    Serial.println("Sending failed (no ack)");
  }
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

  DEBUG_PRINTF("RFM69 radio @");  DEBUG_PRINT((int)RF69_FREQ);  DEBUG_PRINTLNF(" MHz");
}

