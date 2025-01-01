#include "sdcard.h"

//SdFs sd;
//FsFile logFile;                                                                                   // File object to use for logging
String logFileName;                                                                             // Name of the log file
bool sdReady = false;                                                                           // Whether the SD card has been initialized
int lastFoundNum = 1;

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
File logFile;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 logFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile logFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile logFile;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

// Initializes the sensor
bool setupSDCard(){
    logFileName.reserve(30);
    if (sd.begin(SD_CONFIG)) {
        // Find file name
        int fileNo = 1;
        bool exists = true;
        while (exists) {
            logFileName = "motor_datalog_" + String(fileNo) + ".csv";
            exists = sd.exists(logFileName);
            fileNo++;
        }

        // Setup file with CSV header
        logFile = sd.open(logFileName, FILE_WRITE);
        if (logFile) {
            logFile.println(CSV_HEADER);
            logFile.close(); // close the file
            Serial.println("Log file created: " + logFileName);
            lastFoundNum = fileNo;
            sdReady = true;
        }else{
            Serial.println(F("SD Card reader found, but file was unable to be created"));
            return false;
        }
    }else{
        Serial.println(F("SD Card Reader NOT found! Data will not be logged!"));
        return false;
    }

    return true;
}

void switchToNewFile(){
  int fileNo = lastFoundNum;
  bool exists = true;
  while (exists) {
      logFileName = "motor_datalog_" + String(fileNo) + ".csv";
      exists = sd.exists(logFileName);
      fileNo++;
  }

  // Setup file with CSV header
  logFile = sd.open(logFileName, FILE_WRITE);
  if (logFile) {
      logFile.println(CSV_HEADER);
      logFile.close(); // close the file
      Serial.println("Log file created: " + logFileName);
      lastFoundNum = fileNo;
  }else{
      Serial.println(F("SD Card reader found, but file was unable to be created"));
  }
}

// Returns whether the sensor is initialized
bool isSDReady(){
  return sdReady;
}

void outputSDData(float curTime, int curActuation, int desiredActuation, char recivedChars[]){
  if(isSDReady()){
    logFile = sd.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.print(curTime - getStartTime());
      logFile.print(",");
      if(getLaunchTime() < 0.1f){
        logFile.print("0.00");
      }else{
        logFile.print(curTime - getLaunchTime());
      }
      logFile.print(",");
      logFile.print(desiredActuation);
      logFile.print(",");
      logFile.print(curActuation);
      logFile.print(",");
      logFile.print(recivedChars);
      logFile.println();
      logFile.close(); // close the file
    }
  }
}
