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
//*********************************************************************
/********************************************************
   SETUP
 ********************************************************/
void setup()
{
  DEBUG_PRINTLNF("*** Power Management ***");
  initStuff();
}
/********************************************************
   LOOP
 ********************************************************/
void loop()
{
  __WFI();
  DEBUG_PRINTLNF("Got an interrupt, continuing within loop();");
  if (rf69_manager.available() ) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      DEBUG_PRINTF("got request from node ");
      DEBUG_PRINT(from);
      DEBUG_PRINTF(" | RSSI: ");
      DEBUG_PRINTLN(rf69.lastRssi());
    }
    rf69.sleep();
  }
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  initRadio();
  setM0SleepMode();
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

void setM0SleepMode() {
  // Select IDLE, not STANDBY, by turning off the SLEEPDEEP bit
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

  // Select IDLE mode 2 (asynchronous wakeup only)
  PM->SLEEP.bit.IDLE = 2;

}
