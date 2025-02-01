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
TailRotorState TailRotor(tail_rotor_sensors, 3, nullptr, BUZZER_PIN);

// MMFS Stuff
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

// Navigation Stuff
double DEFUALT_GOAL = 0; //defined between [0,360] going ccw from north

// Tail Rotor Stuff
int SERVO_PIN = 3;

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

    // Setup Fan
    TailRotor.fanSetup(SERVO_PIN);

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


    double goalAngle = DEFUALT_GOAL;
    Point targetCoords;
    Point windCorrCoords;

    /// Ground Test Code ///
    // currentCoords = Point(-75.87514167, 39.08471667);
    // targetCoords = Point(-75.77514167, 39.18471667);
    // double goal = getGoalAngle(targetCoords, currentCoords);
    // int pwm = TailRotor.findPWM(goalAngle, (millis() - lastUpdateTime)/1000);
    // TailRotor.runFan(pwm);
    // Serial.printf("Goal: %.2f\n", goal)
    // Serial.printf("PWM: %d\n\n", pwm)
    //////////////////////

    /// Flight Code ///
    if (TailRotor.stage == RELEASED) {
        if (gps.getFixQual() > 3) {
            targetCoords = TailRotor.getTargetCoordinates();
            windCorrCoords = TailRotor.getWindCorrectionCoordinates(targetCoords);
            goalAngle = TailRotor.getGoalAngle(windCorrCoords);
            int pwm = TailRotor.findPWM(goalAngle, (millis() - lastUpdateTime)/1000);
            TailRotor.runFan(pwm);
        } else {
            int pwm = TailRotor.findPWM(DEFUALT_GOAL, (millis() - lastUpdateTime)/1000);
            TailRotor.runFan(pwm);
        }
    } else if (TailRotor.stage == LANDED) {
        // Turn off the motor TODO check with cooper
        motor.writeMicroseconds(1500);
    }
    //////////////////////

}