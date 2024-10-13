#include <Arduino.h>
#include <MMFS.h>
#include "mule_kf.h"
#include "mule_state.h"

const int BUZZER_PIN = 33; // TODO update this to whats actually on the board
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

mmfs::BMP390 barometer;
mmfs::BNO055 mule_imu; 
mmfs::Sensor* mule_sensors[2] = {&barometer, &mule_imu};
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
MuleKF kf;
MuleState MULE(mule_sensors, 2, &kf);

const int SENSOR_BIAS_CORRECTION_DATA_LENGTH = 2;
const int SENSOR_BIAS_CORRECTION_DATA_IGNORE = 1;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

int timeOfLastUpdate = 0;

void setup() {

    logger.init();

    logger.recordLogData(mmfs::INFO_, "Entering Setup");

     if (!(logger.isSdCardReady()))
        bb.onoff(BUZZER_PIN, 200, 3);

    if (!(logger.isPsramReady()))
        bb.onoff(BUZZER_PIN, 200, 3);
    
    // if(!avionicsState.init())
    //     bb.onoff(BUZZER_PIN, 200, 3);

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
    
    //avionicsState.updateState();
}