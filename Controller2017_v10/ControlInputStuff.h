const char THRESHOLD[] = "THRESHOLD"; //Upper case to identify the threshold = command.
const char ZONES[] = "ZONES"; //Upper case to identify the zones = command.
const char STOP[] = "STOP"; // command sent to stop watering.
const char START[] = "START"; //command sent to start watering.a
#define MAX_BYTES  50
#define GREEN "GREEN"
#define BLUE "BLUE"
#define YELLOW "YELLOW"
const uint8_t greenValvePin = 6;
const uint8_t blueValvePin = 11;
const uint8_t yellowValvePin = 12;
#define numZones 3
// cached parameters are written to EEPROM using ESP826's routines.  Right now the only
// parameter cached is the watering threshold because I can change this through Adafruit.io.
#define WRITE_CHECK 0x1234
struct cachedParams_t {
  uint16_t writeCheck;
  uint16_t wateringThreshold;
  uint8_t  valvePins[3]; // These are the three color coded solenoids.  The tell the Controller
  // which solenoids to tell the Water Puck to turn on / off.  By default, the green and blue solenoids
  // are used for watering.  Which solenoids can be turned on /off can be changed within the Adafruit.io's
  // ControlInput feed using the zones= command string (e.g.: zones=yellow,blue chooses the yellow and blue
  // solenoids.
  // The valvePins are set in the function convertStringToZones().  They align with the GPIO pins on the Watering PuckL
  // green:  pin 10
  // blue:   pin 11
  // yellow: pin 12
  // if one or more of the solenoids are not to be turned on/off, a valvePins[] element will have 0xFF (i.e: not a gree, blue, or yellow
  // pin value.
};
union cachedParamsUnion_t
{
  cachedParams_t values;
  uint8_t b[sizeof(cachedParams_t)];
} cachedParams;
// ************************* command requests *************************
enum commands {
  commandUnknown = 0,
  commandStopWater = 1,
  commandStartWater=2
};
/********************************************************************
   STRTOUPR
 *******************************************************************/
void strToUpr(char *aStr) {
  char *ch;
  ch = aStr;
  while (*ch) // exits on terminating NULL
  {
    //Ah..the "ol Ascii trick" -> upper case ascii is toggling of the 5th bit.
    // 0xDF = b1101 1111 -> the 6th bit is 0.  eg.:  a = 0110 0001  A = 0100 0001 .. the difference is bit 6 is 0 when
    // upper case A. so *ch & 0xDF will set the 6th bit to 0.  Also, a check is made to see if once converted to uppercase
    // if the character is indeed within the ASCII character values.
    if (( (*ch & 0xDF) >= 'A' ) && ( (*ch & 0xDF) <= 'Z' )) {  //Ascii upper case characters are between these two
      *ch &= 0xDF;
    }
    ch++;
  }
}
/********************************************************************
   getValvePin
 *******************************************************************/
uint8_t getValvePin(char * buf) {
  strToUpr(buf);
  if (strncmp(buf, GREEN, strlen(GREEN)) == 0) {
    return greenValvePin;
  } else if (strncmp(buf, BLUE, strlen(BLUE)) == 0) {
    return blueValvePin;
  } else if (strncmp(buf, YELLOW, strlen(YELLOW)) == 0) {
    return yellowValvePin;
  }
  return 0xFF;
}
/********************************************************************
   convertStringToZones -> This routine is called by getCommandAndSetVariableValue
   to figure out what watering zones to water.  Blue? Green? Blue and green?
 *******************************************************************/
uint8_t convertStringToZones(char *str, cachedParamsUnion_t *cachedParams_p) {
  char *token;
  char *rest = str;
  uint8_t i = 0;
  uint8_t numErrors = 0;
  while ( (token = strtok_r(rest, ",", &rest)) && (i < numZones) ) {
    strToUpr(token);
    cachedParams_p->values.valvePins[i] = getValvePin(token);
    DEBUG_PRINTF("valve pin: "); DEBUG_PRINT(cachedParams_p->values.valvePins[i]); DEBUG_PRINTF(" | Zone color: ");
    switch (cachedParams_p->values.valvePins[i]) {
      case greenValvePin:
        DEBUG_PRINTLNF("green");
        break;
      case blueValvePin:
        DEBUG_PRINTLNF("blue");
        break;
      case yellowValvePin:
        DEBUG_PRINTLNF("yellow");
        break;
      default:
        DEBUG_PRINTLNF("unknown");
        numErrors++;
        break;
    }
    i++;
  }
  if (i < numZones - 1) { //not using all solenoids
    for (int j = i; j < numZones; j++) {
      cachedParams_p->values.valvePins[j] = 0xFF;
    }
  }
  //DEBUG_PRINTF("Num errors: ");DEBUG_PRINTLN(numErrors);
  return numErrors;
}

/**********************************************************
   doCommand - Commands come through an Adafruit.io
   feed.  
 **********************************************************/
bool doCommand(char *controlString, cachedParamsUnion_t *cachedParams_p, commands *commandRequestMask_p, char *errorStr) {
  char *saveptr;
  char *nameIdStr;
  char buf[MAX_BYTES] = {0};
  strcpy(buf, controlString);
  // check if this is a one word command
  strToUpr(buf);
  // check if the one word command is "STOP"
  *commandRequestMask_p = commandUnknown;
  
  if (strncmp(buf, STOP, strlen(STOP)) == 0 ) {
    DEBUG_PRINTLN(F("-> got command to stop watering."));
    *commandRequestMask_p = commandStopWater;
    return true;
  }
   if (strncmp(buf, START, strlen(START)) == 0 ) {
    DEBUG_PRINTLN(F("-> got command to start watering."));
    *commandRequestMask_p = commandStartWater;
    return true;
  }
  if (strchr(buf, '=') == NULL) {
    strcpy(errorStr, "Error: User input did not include an =");
    DEBUG_PRINTLNF("User input did not include an =");
    return false;
  }
  nameIdStr = strtok_r(buf, "=", &saveptr);
  DEBUG_PRINTF("Command string: "); DEBUG_PRINTLN(nameIdStr);
  DEBUG_PRINTF("value string: "); DEBUG_PRINTLN(saveptr);
  if (strncmp(nameIdStr, THRESHOLD, strlen(THRESHOLD)) == 0) {
    uint16_t thresholdValue = atoi(saveptr);
    DEBUG_PRINTF("Threshold value: "); DEBUG_PRINTLN(thresholdValue);
    if (thresholdValue < 1 || thresholdValue > 1000) { //threshold value should be between 1 and 1000...usually between 200-350...
      strcpy(errorStr, "Error: The threshold value was not between 1-1000");
      return false;
    } else {
      cachedParams_p->values.wateringThreshold = thresholdValue;
      return true;
    }
  } else if (strncmp(nameIdStr, ZONES, strlen(ZONES)) == 0) {
    uint8_t numErrors = convertStringToZones(saveptr, cachedParams_p);
    if (numErrors > 0) {
      strcpy(errorStr, "Error: Check zone names. Some could not be converted into valve pins.");
      DEBUG_PRINTF("The number of strings that could not be converted into zones is: "); DEBUG_PRINTLN(numErrors);
      return false;
    }
  } else {
    strcpy(errorStr, "Error: The command was not recognized");
    DEBUG_PRINTLNF("The command was not recognized");
    return false;
  }
  return true;
}


