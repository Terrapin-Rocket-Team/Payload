#include <Arduino.h>
#include <MMFS.h>
#include "tail_rotor_kf.h"
#include "tail_rotor_state.h"
#include "bmp280_breakout.h"
#include "bno055_breakout.h"
#include "max_m10s_breakout.h"
#include <Servo.h>

// Buzzer
const int BUZZER_PIN = 9; // TODO update this to whats actually on the board
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Sensors
mmfs::BMP280_Breakout barometer;
mmfs::BNO055_Breakout vehicle_imu;
mmfs::MAX_M10S_Breakout gps;
mmfs::Sensor* tail_rotor_sensors[3] = {&barometer, &vehicle_imu, &gps};

// Initialize Tail Rotor State
//TailRotorKF kf;
TailRotorState TailRotor(tail_rotor_sensors, 3, nullptr, BUZZER_PIN);

// MMFS Stuff
mmfs::Logger logger(120, 5);
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 25;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

// Navigation Stuff
double DEFUALT_GOAL = 0; //defined [-180:180] off the y-axis (CCW +)

// Tail Rotor Stuff
int SERVO_PIN = 22;
Servo motor;

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
    motor.attach(SERVO_PIN, 1000, 2000);
    motor.writeMicroseconds(1500);
    delay(2000); 
    motor.writeMicroseconds(1550);
    delay(1000);
    motor.writeMicroseconds(1450);
    delay(1000);
    motor.writeMicroseconds(1500);
    delay(1000);

    logger.writeCsvHeader();
    logger.recordLogData(mmfs::INFO_, "Leaving Setup");
}

static unsigned long lastUpdateTime = 0;
float goalAngle = 0;
Point targetCoords;
Point windCorrCoords;
int pwm;
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

    

    /// Ground Test Code ///
    // currentCoords = Point(-75.87514167, 39.08471667);
    // targetCoords = Point(-75.77514167, 39.18471667);
    // float goalAngle = getGoalAngle(targetCoords, currentCoords);
    // float goalAngle = 0;
    // int pwm = TailRotor.findPWM(goalAngle, (millis() - lastUpdateTime)/1000);
    // motor.writeMicroseconds(pwm);
    // Serial.printf("Goal: %.2f\n", goalAngle);
    // Serial.printf("PWM: %d\n\n", pwm);
    //////////////////////

    /// Flight Code ///
    if (TailRotor.stage == RELEASED) {
        if (gps.getFixQual() > 3) {

            // Running but not using
            targetCoords = TailRotor.getTargetCoordinates();
            windCorrCoords = TailRotor.getWindCorrectionCoordinates(targetCoords);
            goalAngle = TailRotor.getGoalAngle(windCorrCoords);
            TailRotor.goalAngleCalculated = goalAngle;


            if (barometer.getAGLAltFt() < 500){
                goalAngle = 180; // go south
            } else if (barometer.getAGLAltFt() < 1000){
                goalAngle = 0; // go north
            } else if (barometer.getAGLAltFt() < 1500){
                goalAngle = 180; // go south
            } else if (barometer.getAGLAltFt() < 2000){
                goalAngle = 0; // go north
            }
            pwm = TailRotor.findPWM(goalAngle, (millis() - lastUpdateTime)/1000);
            motor.writeMicroseconds(pwm);
        } else {
            pwm = TailRotor.findPWM(DEFUALT_GOAL, (millis() - lastUpdateTime)/1000);
            motor.writeMicroseconds(pwm);
        }
    } else if (TailRotor.stage == LANDED) {
        pwm = 1500;
        motor.writeMicroseconds(pwm);
    }
    TailRotor.goalAngleUsed = goalAngle;
    TailRotor.pwm = pwm;
    //////////////////////

}