#include <Arduino.h>
#include "TADPOLState.h"
#include <Kalman_Filter.h>

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

PWMServo TADPOLState::leftServo; //allows for PMWServo objects to be accessed by servo functions in file
PWMServo TADPOLState::rightServo;

double DEFUALT_GOAL = 0; //defined between [0,360] going ccw from north
int FAN_FOWARD_PIN = 2;
int FAN_BACKWARD_PIN = 3;
int FAN_ENABLE_PIN = 23;

double PDLASTTIME = millis(); // in ms

KFState KFSTATE;
long KF_PREVIOUS_ITER_TIME; //in milliseconds
bool HAS_GPS = false;

void setup() {
  TADPOLSTATE.buzzerPin = 9;
  buzz(TADPOLSTATE.buzzerPin, 1000, 1);
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

  TADPOLSTATE.fanSetup(FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);

  //setup KF
  //Initialize kf state
  int KFStateSize = 6;
  int KFMeasSize = 3;
  int KFControlSize = 3;
  double* initialState = new double[6] {0, 0, 0, 0, 0, 0};
  double* initialControl = new double[3] {0, 0, 0};
  KFSTATE = initialize(KFStateSize, KFMeasSize, KFControlSize, initialState, initialControl);

  if (sdSuccess && TADPOLSTATE.successfulSetup()) {buzz(TADPOLSTATE.buzzerPin, 3000, 1);}
  else {buzz(TADPOLSTATE.buzzerPin, 250, 5);}
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
  if (TADPOLSTATE.stage == "Main") {
    if (TADPOLSTATE.stateGPS.satellites > 3) {
      targetCoords = TADPOLSTATE.getTargetCoordinates();
      goalAngle = getGoalAngle(targetCoords, Point(TADPOLSTATE.stateGPS.longitude, TADPOLSTATE.stateGPS.latitude));
      pwm = TADPOLSTATE.findPWM(goalAngle, (millis()-PDLASTTIME)/1000);
      TADPOLSTATE.runFan(pwm, FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);
      PDLASTTIME = millis();
    }
    else {
      pwm = TADPOLSTATE.findPWM(DEFUALT_GOAL, (millis()-PDLASTTIME)/1000);
      TADPOLSTATE.runFan(pwm, FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);
      PDLASTTIME = millis();
    }
  }
  else if (TADPOLSTATE.stage == "Landed") {
    TADPOLSTATE.runFan(0, FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);
  }

  //Run the KF
  double timeStep = (TADPOLSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME)/1000;
  double* meas = new double[3] {TADPOLSTATE.stateGPS.gpsPosition.x(), TADPOLSTATE.stateGPS.gpsPosition.y(), TADPOLSTATE.stateBarometer.relativeAltitude};
  double* control = new double[3] {TADPOLSTATE.stateIMU.acceleration.x(), TADPOLSTATE.stateIMU.acceleration.y(), TADPOLSTATE.stateIMU.acceleration.z() - 9.8};

  if (TADPOLSTATE.stateGPS.satellites > 5){HAS_GPS = true;}
  else {HAS_GPS = false;}

  KFSTATE = iterate(KFSTATE, timeStep, meas, control, HAS_GPS);
  delete[] meas;
  delete[] control;
  double* KFOutputState = KFSTATE.X;
  KF_PREVIOUS_ITER_TIME = millis();

  //Set the kf output data in the TERPSTATE
  TADPOLSTATE.position.x() = KFOutputState[0]; TADPOLSTATE.position.y() = KFOutputState[1]; TADPOLSTATE.position.z() = KFOutputState[2];
  TADPOLSTATE.velocity.x() = KFOutputState[3]; TADPOLSTATE.velocity.y() = KFOutputState[4]; TADPOLSTATE.velocity.z() = KFOutputState[5];
  TADPOLSTATE.stateIMU.acceleration.x() = TADPOLSTATE.stateIMU.acceleration.x(); TADPOLSTATE.acceleration.y() = TADPOLSTATE.stateIMU.acceleration.y(); TADPOLSTATE.acceleration.z() = TADPOLSTATE.stateIMU.acceleration.z();

  uint8_t bno_cal_system, bno_cal_gyro, bno_cal_accel, bno_cal_mag = 0;
  bno.getCalibration(&bno_cal_system, &bno_cal_gyro, &bno_cal_accel, &bno_cal_mag);

  TADPOLSTATE.setdataString();
  String dataString = TADPOLSTATE.getdataString() + String(pwm) + "," + String(targetCoords.x * 10000000) + "," + String(targetCoords.y * 10000000) + "," + String(goalAngle)
                      + "," + String(bno_cal_system) + "," + String(bno_cal_accel) + "," + String(bno_cal_gyro) + "," + String(bno_cal_mag);
  recordData(dataString, TADPOLSTATE.getrecordDataStage());
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
