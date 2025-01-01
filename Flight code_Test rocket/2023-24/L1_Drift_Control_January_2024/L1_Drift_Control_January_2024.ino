#include <Arduino.h>
#include "TADPOLState.h"

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

PWMServo TADPOLState::leftServo; //allows for PMWServo objects to be accessed by servo functions in file
PWMServo TADPOLState::rightServo;

double DEFUALT_GOAL = 0; //defined between [0,360] going ccw from north
int BUZZER_PIN = 9;

void setup() {
  buzz(BUZZER_PIN, 1000);
  Serial.begin(115200);
  delay(2000);
  //add sensors
  TADPOLSTATE.addBarometer(TADPOLBarometer);
  TADPOLSTATE.addIMU(TADPOLIMU);
  TADPOLSTATE.addGPS(TADPOLGPS);

  //setup steps
  TADPOLSTATE.stateBarometer.setupBarometer();
  TADPOLSTATE.stateIMU.setupIMU();
  TADPOLSTATE.stateGPS.setupGPS();

  TADPOLSTATE.setcsvHeader();
  String csvHeader = TADPOLSTATE.csvHeader + "Left Servo PWM," + "Right Servo PWM," + "Target Lat," + "Target Long," + "Goal Angle," + "BNO Calibration Sys," + "BNO Calibration Accel," + "BNO Calibration Gryo," + "BNO Calibration Mag";
  setupPSRAM(csvHeader);
  bool sdSuccess = setupSDCard(csvHeader);

  TADPOLSTATE.servoSetup(2, 3, 90, 90);
  //pinMode(6, OUTPUT);
  //digitalWrite(6, HIGH); //Set the servos to be on

  if (sdSuccess) {
    buzz(BUZZER_PIN, 3000);
  }
  else {
    buzz(BUZZER_PIN, 250);
    delay(250);
    buzz(BUZZER_PIN, 250);
    delay(250);
    buzz(BUZZER_PIN, 250);
  }
}

void loop() {
  TADPOLSTATE.settimeAbsolute();
  TADPOLSTATE.stateBarometer.readBarometer();
  TADPOLSTATE.stateIMU.readIMU();
  TADPOLSTATE.stateGPS.readGPS();

  TADPOLSTATE.determineTADPOLStage();

  // Test code for finding the correct goal angle without active gps onsite
  //  Point targetCoords = TADPOLSTATE.getTargetCoordinates();
  //  double goal = getGoalAngle(targetCoords, Point(-75.87600, 39.07978));
  //  TADPOLSTATE.goDirection(goal);
  //  Serial.println(targetCoords.x, 8);
  //  Serial.println(targetCoords.y, 8);
  //  Serial.println(goal);
  Point targetCoords;
  double goalAngle = DEFUALT_GOAL;
  if (TADPOLSTATE.stage == "Main") {
    if (TADPOLSTATE.stateGPS.satellites > 3) {
      targetCoords = TADPOLSTATE.getTargetCoordinates();
      goalAngle = getGoalAngle(targetCoords, Point(TADPOLSTATE.stateGPS.longitude, TADPOLSTATE.stateGPS.latitude));
      TADPOLSTATE.goDirection(goalAngle);
    }
    else {
      TADPOLSTATE.goDirection(DEFUALT_GOAL);
    }
  }
  else if (TADPOLSTATE.stage == "Landed") {
    TADPOLSTATE.servoSetup(2, 3, 90, 90);
  }

  //Get bno calibration
  uint8_t bno_cal_system, bno_cal_gyro, bno_cal_accel, bno_cal_mag = 0;
  bno.getCalibration(&bno_cal_system, &bno_cal_gyro, &bno_cal_accel, &bno_cal_mag);

  Serial.println(bno_cal_mag);

  TADPOLSTATE.setdataString();
  String dataString = TADPOLSTATE.getdataString() + String(TADPOLSTATE.left_servo_value) + "," + String(TADPOLSTATE.right_servo_value) + "," + String(targetCoords.x * 10000000) + "," + String(targetCoords.y * 10000000) + "," + String(goalAngle)
                      + "," + String(bno_cal_system) + "," + String(bno_cal_accel) + "," + String(bno_cal_gyro) + "," + String(bno_cal_mag);
  recordData(dataString, TADPOLSTATE.getrecordDataStage());
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
