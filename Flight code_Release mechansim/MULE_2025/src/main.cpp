#include <Arduino.h>

#include "mule_kf.h"
#include "mule_state.h"
#include "bmp280_breakout.h"
#include "bno055_breakout.h"
#include "bh1750_breakout.h"

int LIGHT_THRESHOLD = 1500;  // in lx

int VEHICLE_POWER_PIN = 41;

int NICHROME_1_PIN = 37;
int NICHROME_2_PIN = 36;

const int BUZZER_PIN = 17;
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Sensors
BMP280_Breakout barometer;
BNO055_Breakout mule_imu;
BH1750_Breakout light_sensor;
mmfs::Sensor* mule_sensors[3] = {&barometer, &mule_imu, &light_sensor};

// Initialize state
MuleKF kf;
MuleState MULE(mule_sensors, 3, &kf);

// MMFS Stuff
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

int timeOfLastUpdate = 0;

uint32_t LIGHT_THRESHOLD_TIME = 0; // in millis
bool VEHICLE_RELEASED = false;

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

    logger.recordLogData(mmfs::INFO_, "Powering on Vehicle");
    powerOnVehicle(VEHICLE_POWER_PIN);

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

    // Release the vehicle
    if (light_sensor.getLux() > LIGHT_THRESHOLD){
        if (LIGHT_THRESHOLD_TIME == 0){
            LIGHT_THRESHOLD_TIME = millis();
        }
        if (!VEHICLE_RELEASED){
            if (MULE.stage == DROUGE && ((barometer.getAGLAltFt() < 1500) || (millis() - LIGHT_THRESHOLD_TIME > 3000))){ // If we are under 1500 ft or its been open for 3 seconds
                // TODO check this condition
                releaseVehicle();
            }
        }
    }
}

void powerOnVehicle(int power_on_pin) {
  pinMode(power_on_pin, OUTPUT);
  digitalWrite(power_on_pin, HIGH);
  delay(10000);
  digitalWrite(power_on_pin, LOW);
}

void releaseVehicle(){
  // TODO is this function still correct?
  pinMode(NICHROME_1_PIN, OUTPUT);
  pinMode(NICHROME_2_PIN, OUTPUT);

  digitalWrite(NICHROME_1_PIN, HIGH);
  delay(250);
  digitalWrite(NICHROME_1_PIN, LOW);
  digitalWrite(NICHROME_2_PIN, HIGH);
  delay(250);
  digitalWrite(NICHROME_2_PIN, LOW);

  VEHICLE_RELEASED = true;
}