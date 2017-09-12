/*
 * I use blinking of the feather's red LED like smoke signals to give me a bit of an idea of what is going
 * on in the code when not being debugged.
 */
 #define BlinkSuccessInit {Blink(400,10);digitalWrite(LED,LOW);}
 #define BlinkStartWatering {Blink(300,5);Blink(500,2);Blink(300,5);digitalWrite(LED,LOW);}
 #define BlinkStopWatering BlinkStartWatering
 #define BlinkFinishedWatering BlinkStartWatering
 #define BlinkReceivedMessage {Blink(400,6);digitalWrite(LED,LOW);}
 #define BlinkReceivedControlInput BlinkStartWatering
 #define BlinkStopWatering {Blink(300,5);digitalWrite(LED,LOW);}
 #define BlinkDidntHearFromPuck {Blink(300,10);Blink(500,4);Blink(300,5);digitalWrite(LED,LOW);}
 #define BlinkIOCommand {Blink(200,10);digitalWrite(LED,LOW);}
 /********************************************************
   BLINK
 ********************************************************/
 
void Blink(byte DELAY_MS, byte loops) {
#ifndef DEBUG
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(LED, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED, LOW);
    delay(DELAY_MS);
  }
#endif
}

