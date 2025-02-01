#include <Arduino.h>
#include <MMFS.h>
#include "tail_rotor_kf.h"
#include "tail_rotor_state.h"

// Buzzer
const int BUZZER_PIN = 33; // TODO update this to whats actually on the board
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Sensors
BMP280_Breakout barometer;
BNO055_Breakout vehicle_imu;
MAX_M10S_Breakout gps;
mmfs::Sensor* tail_rotor_sensors[3] = {&barometer, &vehicle_imu, &gps};

// Initialize Tail Rotor State
//TailRotorKF kf;
TailRotorState TailRotor(tail_rotor_sensors, 3, nullptr);

// MMFS Stuff
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {

    Serial.begin(115200);

    if (CrashReport) Serial.println(CrashReport);

    // MMFS Stuff
    SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
    SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
    psram = new mmfs::PSRAM();
    logger.init(&TailRotor);

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
    if(!TailRotor.init()){
        logger.recordLogData(mmfs::INFO_, "State Failed to Completely Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{ 
        bb.onoff(BUZZER_PIN, 1000, 1);
        barometer.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    }

    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

static unsigned long lastUpdateTime = 0;
void loop() {
    //Do this as often as possible for best results
    bb.update();

    if (millis() - lastUpdateTime < UPDATE_INTERVAL) {
        return;
    }
    lastUpdateTime = millis();

    // Update state and log data
    TailRotor.updateState();
    logger.recordFlightData();

    // Turn off bias correction during flight
    if (TailRotor.stage == BOOST) {
        barometer.setBiasCorrectionMode(false);
        gps.setBiasCorrectionMode(false);
    } else if (TailRotor.stage == PRELAUNCH) {
        barometer.setBiasCorrectionMode(true);
        gps.setBiasCorrectionMode(true);
    }
}