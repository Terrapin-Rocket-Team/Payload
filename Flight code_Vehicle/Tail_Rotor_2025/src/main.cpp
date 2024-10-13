#include <Arduino.h>
#include <MMFS.h>
#include "tail_rotor_kf.h"
#include "tail_rotor_state.h"


const int BUZZER_PIN = 33; // TODO update this to whats actually on the board
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

mmfs::BMP390 barometer;
mmfs::BNO055 tail_rotor_imu; 
mmfs::MAX_M10S gps;
mmfs::Sensor* tail_rotor_sensors[3] = {&barometer, &tail_rotor_imu, &gps};
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
TailRotorKF kf;
TailRotorState TailRotor(tail_rotor_sensors, 3, &kf);

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
    
    if(!TailRotor.init())
        bb.onoff(BUZZER_PIN, 200, 3);

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
    
    TailRotor.updateState();
}