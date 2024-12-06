#include <Arduino.h>

#include "mule_kf.h"
#include "mule_state.h"
#include "bmp280_breakout.h"
#include "bno055_breakout.h"

const int BUZZER_PIN = 33; // TODO update this to whats actually on the board
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Sensors
BMP280_Breakout barometer;
mmfs::BNO055_Breakout mule_imu;
mmfs::Sensor* mule_sensors[2] = {&barometer, &mule_imu};

// Initialize state
MuleKF kf;
MuleState MULE(mule_sensors, 2, &kf);

// MMFS Stuff
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

int timeOfLastUpdate = 0;

void setup() {
    Serial.begin(115200);

    // MMFS Stuff
    SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
    SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
    psram = new mmfs::PSRAM();
    logger.init(&MULE);

    logger.recordLogData(mmfs::INFO_, "Entering Setup");

    // Check the sd card
    if (!(logger.isSdCardReady())){
        logger.recordLogData(mmfs::INFO_, "SD Card Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{
        bb.onoff(BUZZER_PIN, 1000, 1);
    }

    // Check the psram
    if (!(logger.isPsramReady())){
        logger.recordLogData(mmfs::INFO_, "PSRAM Failed to Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else {
        bb.onoff(BUZZER_PIN, 1000, 1);
    } 
    
    // Initialize State (runs Begin/Init for each sensor)
    if(!MULE.init()){
        logger.recordLogData(mmfs::INFO_, "State Failed to Completely Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{ 
        bb.onoff(BUZZER_PIN, 1000, 1);
    }
    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

static double last = 0; // for better timing than "delay(100)"
void loop() {
    double time = millis();
    bb.update();
    // Update the state of the rocket
    if (time - last < 100)
        return;
    last = time;
    
    MULE.updateState();
    logger.recordFlightData();
}