#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
/************************* WiFi Access Point *********************************/

WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);
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
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      Serial.println("Could not connect to MQTT");
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
}

