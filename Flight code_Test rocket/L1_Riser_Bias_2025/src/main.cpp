#include <Arduino.h>
#include <MMFS.h>
#include "l1_riser_bias_kf.h"
#include "l1_riser_bias_state.h"


const int BUZZER_PIN = 9;
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Sensors
//mmfs::BMP390 barometer;
mmfs::BNO055 l1_riser_bias_imu; 
mmfs::MAX_M10S gps;
mmfs::Sensor* l1_riser_bias_sensors[2] = {&l1_riser_bias_imu, &gps};

// Initialize the state
L1RiserBiasKF kf;
L1RiserBiasState L1RiserBias(l1_riser_bias_sensors, 2, &kf);

// MMFS Stuff
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {

    // MMFS Stuff
    SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
    SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
    psram = new mmfs::PSRAM();
    logger.init(&L1RiserBias);

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
    if(!L1RiserBias.init()){
        logger.recordLogData(mmfs::INFO_, "State Failed to Completely Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{ 
        bb.onoff(BUZZER_PIN, 1000, 1);
        gps.setBiasCorrectionMode(true);
    }
    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

static double last = 0; // for better timing than "delay(100)"
void loop() {
    bb.update();

    if (millis() - last < UPDATE_INTERVAL)
        return;
    last = millis();
    
    L1RiserBias.updateState();
    logger.recordFlightData();
}