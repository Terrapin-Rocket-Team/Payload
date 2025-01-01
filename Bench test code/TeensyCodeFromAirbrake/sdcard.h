#ifndef AERO_SDCARD
#define AERO_SDCARD

#include "SdFat.h"
#include "timing.h"

#define CSV_HEADER F("Program Uptime,Time Since Launch,Desired Actuation,Current Actuation,Last Received Data")

extern SdFs sd;
extern FsFile logFile;
extern String logFileName;

bool setupSDCard();                         // Initializes the sensor
bool isSDReady();                           // Returns whether the sensor is initialized
void outputSDData(float curTime, int curActuation, int desiredActuation, char receivedChars[]);
void switchToNewFile();


#endif
