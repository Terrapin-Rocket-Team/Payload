#include <Arduino.h>
#include "TERPState.h"
#include <Kalman_Filter.h>


int LIGHT_THRESHOLD = 30;  // in lx

int BUZZER_PIN = 17;

int TADPOL_POWER_PIN = 23;

int CAMERA_ON_PIN = 4;

int CAMERA_PIN_DURATION = 100; // in milliseconds

long KF_PREVIOUS_ITER_TIME; //in milliseconds

Barometer TERPBarometer("BMP280");
IMU TERPIMU("BNO055");
LightSensor TERPLightSensor("BH1750");
TERPState TERPSTATE;

KFState KFSTATE;

void setup() {
  buzz(BUZZER_PIN, 1000);
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

  if (sdSuccess) {
    buzz(BUZZER_PIN, 3000);
  }
  else {
    buzz(BUZZER_PIN, 250);
    delay(250);
    buzz(BUZZER_PIN, 250);
    delay(250);
    buzz(BUZZER_PIN, 250);
    delay(250);
    buzz(BUZZER_PIN, 250);
    delay(250);
  }

  powerOnTADPOL(TADPOL_POWER_PIN);
  KF_PREVIOUS_ITER_TIME = millis();
}

void loop() {
  TERPSTATE.settimeAbsolute();
  TERPSTATE.stateBarometer.readBarometer();
  TERPSTATE.stateIMU.readIMU();
  TERPSTATE.stateLightSensor.readLightSensor();
  
//   if(Serial.available() > 0){
//     Serial.println("KFState[0] = " + String(KFSTATE.X[0]));
//     Serial.println("KFState[1] = " + String(KFSTATE.X[1]));
//     Serial.println("KFState[2] = " + String(KFSTATE.X[2]));
//     Serial.println("KFState[3] = " + String(KFSTATE.X[3]));
//     Serial.println("KFState[4] = " + String(KFSTATE.X[4]));
//     Serial.println("KFState[5] = " + String(KFSTATE.X[5]));
//   }

  //Run the KF
  double timeStep = TERPSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME;
  double* meas = new double[3] {0.0, 0.0, TERPSTATE.stateBarometer.relativeAltitude};
  double* control = new double[3] {TERPSTATE.stateIMU.acceleration.x(), TERPSTATE.stateIMU.acceleration.y(), TERPSTATE.stateIMU.acceleration.z() - 9.8};
//   if(Serial.available() > 0){
//     Serial.println("Meas[0] = " + String(meas[0]));
//     Serial.println("Meas[1] = " + String(meas[1]));
//     Serial.println("Meas[2] = " + String(meas[2]));

//     Serial.println("Control[0] = " + String(control[0]));
//     Serial.println("Control[1] = " + String(control[1]));
//     Serial.println("Control[2] = " + String(control[2]));

//   }
  KFSTATE = iterate(KFSTATE, timeStep, meas, control, false);
  delete[] meas;
  delete[] control;
  double* KFOutputState = KFSTATE.X;
  KF_PREVIOUS_ITER_TIME = millis();

  //Set the kf output data in the TERPSTATE
  TERPSTATE.position.x() = KFOutputState[0]; TERPSTATE.position.y() = KFOutputState[1]; TERPSTATE.position.z() = KFOutputState[2];
  TERPSTATE.velocity.x() = KFOutputState[3]; TERPSTATE.velocity.y() = KFOutputState[4]; TERPSTATE.velocity.z() = KFOutputState[5];
  TERPSTATE.acceleration.x() = TERPSTATE.stateIMU.acceleration.x(); TERPSTATE.acceleration.y() = TERPSTATE.stateIMU.acceleration.y(); TERPSTATE.acceleration.z() = TERPSTATE.stateIMU.acceleration.z();
  TERPSTATE.determineTERPStage();

  if (TERPSTATE.stage == "Drogue") {
    digitalWrite(CAMERA_ON_PIN, HIGH);
    delay(CAMERA_PIN_DURATION);
    digitalWrite(CAMERA_ON_PIN, LOW);
  }

  TERPSTATE.setdataString();

  recordData(TERPSTATE.getdataString(), TERPSTATE.getrecordDataStage());

}

void powerOnTADPOL(int power_on_pin) {
  pinMode(power_on_pin, OUTPUT);
  digitalWrite(power_on_pin, HIGH);
  delay(10000);
  digitalWrite(power_on_pin, LOW);
}
