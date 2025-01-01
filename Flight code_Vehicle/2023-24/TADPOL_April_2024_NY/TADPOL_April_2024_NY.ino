#include <Arduino.h>
#include "TADPOLState.h"
#include <Kalman_Filter.h>

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

double KF_PREVIOUS_ITER_TIME; //in seconds
double dt = 0.01;
Config LinearKFConfig = initialize_filter(dt);
LinearKalmanFilter kf(LinearKFConfig);

double DEFUALT_GOAL = -45; //defined between [-180,180] going ccw from north
int FAN_FOWARD_PIN = 2;
int FAN_BACKWARD_PIN = 3;
int FAN_ENABLE_PIN = 23;

double PDLASTTIME = millis(); // in ms

void setup() {
    TADPOLSTATE.buzzerPin = 9;
    buzz(TADPOLSTATE.buzzerPin, 1000, 1);
    Serial.begin(115200);

    //add sensors
    TADPOLSTATE.addBarometer(TADPOLBarometer);
    TADPOLSTATE.addIMU(TADPOLIMU);
    TADPOLSTATE.addGPS(TADPOLGPS);

    //setup steps
    TADPOLSTATE.stateBarometer.setupBarometer();
    TADPOLSTATE.stateIMU.setupIMU();
    TADPOLSTATE.stateGPS.setupGPS();

    //setup datalogging
    TADPOLSTATE.setcsvHeader();
    String csvHeader = TADPOLSTATE.csvHeader + "PWM," + "Target Lat," + "Target Long," + "Goal Angle," + "BNO Calibration Sys," + "BNO Calibration Accel," + "BNO Calibration Gryo," + "BNO Calibration Mag";
    setupPSRAM(csvHeader);
    bool sdSuccess = setupSDCard(csvHeader);

    TADPOLSTATE.fanSetup(FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);

    KF_PREVIOUS_ITER_TIME = millis()/1000.0;

    if(sdSuccess && TADPOLSTATE.successfulSetup()){buzz(TADPOLSTATE.buzzerPin, 3000, 1);}
    else{buzz(TADPOLSTATE.buzzerPin, 250, 5);}

    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);
}

void loop() {
    TADPOLSTATE.settimeAbsolute();
    TADPOLSTATE.stateBarometer.readBarometer();
    TADPOLSTATE.stateIMU.readIMU();
    TADPOLSTATE.stateGPS.readGPS();

    TADPOLSTATE.determineTADPOLStage();

    //Run Target Point and Movement
    Point targetCoords;
    double goalAngle = DEFUALT_GOAL;
    double pwm = 0;

    if (TADPOLSTATE.stage == "Powered Ascent"){
      digitalWrite(22, LOW);
    }

    if (TADPOLSTATE.stage == "Main") {
      if (TADPOLSTATE.stateBarometer.relativeAltitude > 300){
          // Go South West (135 degrees)
          goalAngle = 135;
      }
      else if (TADPOLSTATE.stateBarometer.relativeAltitude > 150){
          // Go North East (-45 degrees)
          goalAngle = -45;
      }
      else if (TADPOLSTATE.stateGPS.satellites > 3) {
        // Go to a point
        targetCoords = TADPOLSTATE.getTargetCoordinates();
        goalAngle = getGoalAngle(targetCoords, Point(TADPOLSTATE.stateGPS.longitude, TADPOLSTATE.stateGPS.latitude));
      }
      else {
        goalAngle = DEFUALT_GOAL;
      }
      pwm = TADPOLSTATE.findPWM(goalAngle, (millis()-PDLASTTIME)/1000);
      //TADPOLSTATE.runFan(pwm, FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);
      PDLASTTIME = millis();
    }
    else if (TADPOLSTATE.stage == "Landed") {
      //TADPOLSTATE.runFan(0, FAN_FOWARD_PIN, FAN_BACKWARD_PIN, FAN_ENABLE_PIN);
    }

    //Run the KF
    dt = TADPOLSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME;
    double* meas_arr = new double[3] {TADPOLSTATE.stateGPS.gpsPosition.x(), TADPOLSTATE.stateGPS.gpsPosition.y(), TADPOLSTATE.stateBarometer.relativeAltitude};
    double* control_arr = new double[3] {TADPOLSTATE.stateIMU.acceleration.x(), TADPOLSTATE.stateIMU.acceleration.y(), TADPOLSTATE.stateIMU.acceleration.z() - 9.8};
    LinearKFConfig = iterate_filter(dt, meas_arr, control_arr, TADPOLSTATE.stateGPS.satellites);
    Matrix X = kf.iterate(LinearKFConfig);

    double* KFOutputState = X.getArr();
    // //Set the kf output data in the TADPOLSTATE 
    TADPOLSTATE.position.x() = KFOutputState[0]; TADPOLSTATE.position.y() = KFOutputState[1]; TADPOLSTATE.position.z() = KFOutputState[2];
    TADPOLSTATE.velocity.x() = KFOutputState[3]; TADPOLSTATE.velocity.y() = KFOutputState[4]; TADPOLSTATE.velocity.z() = KFOutputState[5];
    TADPOLSTATE.stateIMU.acceleration.x() = TADPOLSTATE.stateIMU.acceleration.x(); TADPOLSTATE.acceleration.y() = TADPOLSTATE.stateIMU.acceleration.y(); TADPOLSTATE.acceleration.z() = TADPOLSTATE.stateIMU.acceleration.z();

    uint8_t bno_cal_system, bno_cal_gyro, bno_cal_accel, bno_cal_mag = 0;
    bno.getCalibration(&bno_cal_system, &bno_cal_gyro, &bno_cal_accel, &bno_cal_mag);

    TADPOLSTATE.setdataString();
    String dataString = TADPOLSTATE.getdataString() + String(pwm) + "," + String(targetCoords.x * 10000000) + "," + String(targetCoords.y * 10000000) + "," + String(goalAngle)
                        + "," + String(bno_cal_system) + "," + String(bno_cal_accel) + "," + String(bno_cal_gyro) + "," + String(bno_cal_mag);
    recordData(dataString, TADPOLSTATE.getrecordDataStage());
    
    KF_PREVIOUS_ITER_TIME = TADPOLSTATE.timeAbsolute;
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