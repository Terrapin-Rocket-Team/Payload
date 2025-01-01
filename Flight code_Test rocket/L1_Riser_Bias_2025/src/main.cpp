#include <Arduino.h>
#include <MMFS.h>
#include "l1_riser_bias_kf.h"
#include "l1_riser_bias_state.h"


const int BUZZER_PIN = 9; // TODO update this to whats actually on the board
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

//mmfs::BMP390 barometer;
mmfs::BNO055 l1_riser_bias_imu; 
mmfs::MAX_M10S gps;
mmfs::Sensor* l1_riser_bias_sensors[2] = {&l1_riser_bias_imu, &gps};
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
L1RiserBiasKF kf;
L1RiserBiasState L1RiserBias(l1_riser_bias_sensors, 2, &kf);

const int SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
const int SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

int timeOfLastUpdate = 0;

void setup() {

    logger.init();

    logger.recordLogData(mmfs::INFO_, "Entering Setup");

     if (!(logger.isSdCardReady())){
        //bb.onoff(BUZZER_PIN, 200, 3);
        logger.recordLogData(mmfs::ERROR_, "SD card did not initialize");
     }
        

    if (!(logger.isPsramReady())){
        bb.onoff(BUZZER_PIN, 200, 3);
        logger.recordLogData(mmfs::ERROR_, "PSRAM did not initialize");
    }
        
    
    if(!L1RiserBias.init()){
        bb.onoff(BUZZER_PIN, 200, 3);
        logger.recordLogData(mmfs::ERROR_, "State did not initialize");
    }


    //bb.onoff(BUZZER_PIN, 1000, 1);
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

void loop() {
    //Do this as often as possible for best results
    bb.update();
    
    //Do this at a fixed rate
    int currentTime = millis();
    if (currentTime - timeOfLastUpdate < UPDATE_INTERVAL)
        return;
    timeOfLastUpdate = currentTime;
    
    L1RiserBias.updateState();
    GPS *gps = reinterpret_cast<GPS *>(L1RiserBias.getSensor(GPS_));
    
    if (gps != nullptr) {
        // Log the z-axis acceleration from the IMU
        int fixQual = gps->getFixQual();  // Access the z component
        logger.recordLogData(mmfs::INFO_, String(fixQual).c_str());
    } else {
        // Handle error if IMU is not found
        logger.recordLogData(mmfs::ERROR_, "GPS sensor not found!");
    }
}