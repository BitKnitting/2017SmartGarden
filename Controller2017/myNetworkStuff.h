#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* WiFi Access Point *********************************/
#define WLAN_SSID       "NF7VH"
#define WLAN_PASS       "FX9MP5LGC5NHDRQV"

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883
#define AIO_USERNAME    "sketchy"
#define AIO_KEY         "556280f5b0c3a0587ad072ece406fdbf37d37617"
WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe timefeed = Adafruit_MQTT_Subscribe(&mqtt, "time/seconds");
/********************************************************
   BLINK
 ********************************************************/
void blink(int nTimesToBlink, int rate) {
  int i = 1;
  while (i <= nTimesToBlink) {
    digitalWrite(LED, HIGH);
    delay(rate);
    digitalWrite(LED, LOW);
    delay(rate);
    i++;
  }
}
/********************************************************
   NONBLOCKDELAY
 ********************************************************/
void nonBlockDelay(int rate) {
  int startingMillis = millis();
  while (millis() - startingMillis < rate);
}
/********************************************************************
 * MQTT_CONNECT
 *******************************************************************/
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  DEBUG_PRINTF("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    DEBUG_PRINTLN(mqtt.connectErrorString(ret));
    DEBUG_PRINTLNF("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  DEBUG_PRINTLNF("MQTT Connected!");
  blink(5,300);
}

