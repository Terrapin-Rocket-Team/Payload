#include <Arduino.h>
#include "TADPOLState.h"

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

PWMServo TADPOLState::leftServo; //allows for PMWServo objects to be accessed by servo functions in file
PWMServo TADPOLState::rightServo;

double DEFUALT_GOAL = 0; //defined between [0,360] going ccw from north
int FAN_FOWARD_PIN = 3;
int FAN_BACKWARD_PIN = 2;

double PDLASTTIME = millis(); // in ms

void setup() {
  TADPOLSTATE.buzzerPin = 9;
  buzz(TADPOLSTATE.buzzerPin, 200, 1);
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
  String csvHeader = TADPOLSTATE.csvHeader + "PWM," + "Target Lat," + "Target Long," + "Goal Angle," + "BNO Calibration Sys," + "BNO Calibration Accel," + "BNO Calibration Gryo," + "BNO Calibration Mag";
  setupPSRAM(csvHeader);
  bool sdSuccess = setupSDCard(csvHeader);

  TADPOLSTATE.fanSetup(FAN_FOWARD_PIN, FAN_BACKWARD_PIN);

  if (sdSuccess && TADPOLSTATE.successfulSetup()) {buzz(TADPOLSTATE.buzzerPin, 500, 1);}
  else {buzz(TADPOLSTATE.buzzerPin, 250, 5);}

  TADPOLSTATE.previousPwm = 0;
}

void loop() {
  TADPOLSTATE.settimeAbsolute();
  TADPOLSTATE.stateBarometer.readBarometer();
  TADPOLSTATE.stateIMU.readIMU();
  TADPOLSTATE.stateGPS.readGPS();

  TADPOLSTATE.determineTADPOLStage();

  Point targetCoords;
  double goalAngle = DEFUALT_GOAL;
  double pwm = 0;

  pwm = TADPOLSTATE.findPWM(DEFUALT_GOAL, (millis()-PDLASTTIME)/1000);
  pwm = -200;
  if (millis() > 12000){
    pwm = -200;
  }
  TADPOLSTATE.runFan(pwm, FAN_FOWARD_PIN, FAN_BACKWARD_PIN);
  PDLASTTIME = millis();

  //Get bno calibration //TODO test that is working
  uint8_t bno_cal_system, bno_cal_gyro, bno_cal_accel, bno_cal_mag = 0;
  bno.getCalibration(&bno_cal_system, &bno_cal_gyro, &bno_cal_accel, &bno_cal_mag);

  //Serial.println(bno_cal_mag);

  TADPOLSTATE.setdataString();
  String dataString = TADPOLSTATE.getdataString() + String(pwm) + "," + String(targetCoords.x * 10000000) + "," + String(targetCoords.y * 10000000) + "," + String(goalAngle)
                      + "," + String(bno_cal_system) + "," + String(bno_cal_accel) + "," + String(bno_cal_gyro) + "," + String(bno_cal_mag);
  recordData(dataString, "Test");
}

double getGoalAngle(Point target, Point current) {
  //Returns an angle from [-180:180] from north to get to a target from a current point
  //CCW is positive angles
  double theta = atan2(target.y - current.y, target.x - current.x);
  theta = theta * 180 / 3.14;

  //We now have theta which is position [-180:180] off the x-axis (CCW +)
  //These lines move that goal angle to position as [-180:180] off the y-axis (CCW +)
  double goal = theta - 90;
  if (goal < -180){goal = goal + 360;}

  return goal;
}
