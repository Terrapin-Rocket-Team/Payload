#include <Arduino.h>

#include "servo_vehicle_kf.h"
#include "servo_vehicle_state.h"
#include "bmp280_breakout.h"
#include "bno055_breakout.h"
#include "max_m10s_breakout.h"

// Function Declarations
double getGoalAngle(Point target, Point current);

// Buzzer
const int BUZZER_PIN = 17;
int allowedPins[] = {BUZZER_PIN};
BlinkBuzz bb(allowedPins, 1, true);

// Sensors
BMP280_Breakout barometer;
BNO055_Breakout vehicle_imu;
MAX_M10S_Breakout gps;
mmfs::Sensor* mule_sensors[2] = {&barometer, &vehicle_imu};

double DEFUALT_GOAL = 0; //defined between [0,360] going ccw from north

// Initialize state
ServoVehicleKF kf;
ServoVehicleState SERVO_VEHICLE(mule_sensors, 2, &kf);

// Servos
PWMServo ServoVehicleState::leftServo; //allows for PMWServo objects to be accessed by servo functions in file
PWMServo ServoVehicleState::rightServo; //allows for PMWServo objects to be accessed by servo functions in file

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
    logger.init(&SERVO_VEHICLE);

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
    if(!SERVO_VEHICLE.init()){
        logger.recordLogData(mmfs::INFO_, "State Failed to Completely Initialize");
        bb.onoff(BUZZER_PIN, 200, 3);
    } else{ 
        bb.onoff(BUZZER_PIN, 1000, 1);
    }
    logger.writeCsvHeader();

    logger.recordLogData(mmfs::INFO_, "Setting Up Servos");
    SERVO_VEHICLE.servoSetup(2, 3, 90, 90);

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
    
    SERVO_VEHICLE.updateState();
    logger.recordFlightData();

  // Test code for finding the correct goal angle without active gps onsite
  //  Point targetCoords = TADPOLSTATE.getTargetCoordinates();
  //  double goal = getGoalAngle(targetCoords, Point(-75.87600, 39.07978));
  //  TADPOLSTATE.goDirection(goal);
  //  Serial.println(targetCoords.x, 8);
  //  Serial.println(targetCoords.y, 8);
  //  Serial.println(goal);
  Point targetCoords;
  double goalAngle = DEFUALT_GOAL;
  if (SERVO_VEHICLE.stage == MAIN) {
    if (gps.getFixQual() > 3) {
      targetCoords = SERVO_VEHICLE.getTargetCoordinates();
      goalAngle = getGoalAngle(targetCoords, Point(gps.getPos().y(), gps.getPos().x())); // longitude, latitude
      SERVO_VEHICLE.goDirection(goalAngle);
    }
    else {
      SERVO_VEHICLE.goDirection(DEFUALT_GOAL);
    }
  }
  else if (SERVO_VEHICLE.stage == LANDED) {
    SERVO_VEHICLE.servoSetup(2, 3, 90, 90);
  }
}

double getGoalAngle(Point target, Point current) {
  //Returns an angle from [0:360] from north to get to a target from a current point
  double theta = atan2(target.y - current.y, target.x - current.x);
  theta = theta * 180 / 3.14;
  double goal = 270 + theta;
  if (goal > 360) {
    goal -= 360;
  }
  return goal;
}