#include <Arduino.h>
#include "TERPState.h"
#include <Kalman_Filter.h>


int LIGHT_THRESHOLD = 30;  // TODO set at launch // in lx

int TADPOL_POWER_PIN = 41;

int CAMERA_ON_PIN = 4;

int CAMERA_PIN_DURATION = 100; // in milliseconds

Barometer TERPBarometer("BMP280");
IMU TERPIMU("BNO055");
LightSensor TERPLightSensor("BH1750");
TERPState TERPSTATE;

KFState KFSTATE;
long KF_PREVIOUS_ITER_TIME; //in milliseconds

void setup() {
  TERPSTATE.buzzerPin = 17;
  buzz(TERPSTATE.buzzerPin, 1000, 1);
  Serial.begin(115200);

  //add sensors
  TERPSTATE.addBarometer(TERPBarometer);
  TERPSTATE.addIMU(TERPIMU);
  TERPSTATE.addLightSensor(TERPLightSensor);

  //setup steps
  TERPSTATE.stateBarometer.setupBarometer();
  TERPSTATE.stateIMU.setupIMU();
  TERPSTATE.stateLightSensor.setupLightSensor();

  //setup datalogging
  TERPSTATE.setcsvHeader();
  setupPSRAM(TERPSTATE.csvHeader);
  bool sdSuccess = setupSDCard(TERPSTATE.csvHeader);

  pinMode(CAMERA_ON_PIN, OUTPUT);

  //setup KF
  //Initialize kf state
  int KFStateSize = 6;
  int KFMeasSize = 3;
  int KFControlSize = 3;
  double* initialState = new double[6] {0, 0, 0, 0, 0, 0};
  double* initialControl = new double[3] {0, 0, 0};
  KFSTATE = initialize(KFStateSize, KFMeasSize, KFControlSize, initialState, initialControl);

  if (sdSuccess) {buzz(TERPSTATE.buzzerPin, 3000, 1);}
  else {buzz(TERPSTATE.buzzerPin, 250, 4);}

  powerOnTADPOL(TADPOL_POWER_PIN);
  KF_PREVIOUS_ITER_TIME = millis();
}

void loop() {
  TERPSTATE.settimeAbsolute();
  TERPSTATE.stateBarometer.readBarometer();
  TERPSTATE.stateIMU.readIMU();
  TERPSTATE.stateLightSensor.readLightSensor();

  //Run the KF
  double timeStep = (TERPSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME)/1000;
  double* meas = new double[3] {0.0, 0.0, TERPSTATE.stateBarometer.relativeAltitude};
  double* control = new double[3] {TERPSTATE.stateIMU.acceleration.x(), TERPSTATE.stateIMU.acceleration.y(), TERPSTATE.stateIMU.acceleration.z() - 9.8};

  KFSTATE = iterate(KFSTATE, timeStep, meas, control, false);
  delete[] meas;
  delete[] control;
  double* KFOutputState = KFSTATE.X;
  KF_PREVIOUS_ITER_TIME = millis();

  //Set the kf output data in the TERPSTATE
  TERPSTATE.position.x() = KFOutputState[0]; TERPSTATE.position.y() = KFOutputState[1]; TERPSTATE.position.z() = KFOutputState[2];
  TERPSTATE.velocity.x() = KFOutputState[3]; TERPSTATE.velocity.y() = KFOutputState[4]; TERPSTATE.velocity.z() = KFOutputState[5];
  TERPSTATE.stateIMU.acceleration.x() = TERPSTATE.stateIMU.acceleration.x(); TERPSTATE.acceleration.y() = TERPSTATE.stateIMU.acceleration.y(); TERPSTATE.acceleration.z() = TERPSTATE.stateIMU.acceleration.z();
  TERPSTATE.determineTERPStage();

  if (TERPSTATE.stage == "Drogue") {
    startCameraRecording(CAMERA_ON_PIN, CAMERA_PIN_DURATION);
  }

  TERPSTATE.setdataString();
  recordData(TERPSTATE.getdataString(), TERPSTATE.getrecordDataStage());

}

void powerOnTADPOL(int power_on_pin) {
  Serial.println("Turning on TADPOL");
  pinMode(power_on_pin, OUTPUT);
  digitalWrite(power_on_pin, HIGH);
  delay(10000);
  digitalWrite(power_on_pin, LOW);
  Serial.println("Done turning on TADPOL");
}

void startCameraRecording(int cameraOnPin, int cameraPinDuration){
    // cameraPinDuration in ms
    digitalWrite(cameraOnPin, HIGH);
    delay(cameraPinDuration);
    digitalWrite(cameraOnPin, LOW);
}
