#include <Arduino.h>
#include <Multi_Mission_Flight_Software.h>

State STATE;
IMU TESTIMU("BNO055"); //Input IMU type here

float pwm = 255; // [-255, 255]
float f = 100;
String recordDataMode = "Flight";
float pwmtimer=millis();
int fwdPin=3;//10 for micro;
int bckPin=2;//9 for micro;

void setup() {

  Serial.begin(115200);

  Serial.println("Adding IMU");
  STATE.addIMU(TESTIMU);

  Serial.print("Setting up IMU...");
  STATE.stateIMU.setupIMU();
  if(STATE.successfulSetup()){
      Serial.println("Success!");
  }

  //setup steps
  STATE.setcsvHeader();
  setupPSRAM(STATE.csvHeader);
  setupSDCard(STATE.csvHeader);

  analogWriteFrequency(fwdPin, f);
  analogWriteFrequency(bckPin, f);

  pinMode(fwdPin,OUTPUT);
  pinMode(bckPin,OUTPUT);

}

void loop() {

  analogWrite(fwdPin, 0);
  analogWrite(bckPin, 0);

  if (millis() > 10000){
    recordDataMode = "Flight";
    analogWrite(fwdPin, 0);
    analogWrite(bckPin, 0);
  } else if (millis() > 5000) {
    recordDataMode = "Test";
    if(pwm > 0){
      analogWrite(fwdPin, pwm);
      analogWrite(bckPin, 0);
    } else if (pwm < 0){
      analogWrite(bckPin, -pwm);
      analogWrite(fwdPin, 0);
    }
  } else {
    recordDataMode = "Flight";
    analogWrite(fwdPin, 0);
    analogWrite(bckPin, 0);
  }

  STATE.settimeAbsolute();

  STATE.stateIMU.readIMU();
  Serial.print("Angualar Velo X: "); Serial.println(STATE.stateIMU.angularVelocity.x());
  Serial.print("Angualar Velo Y: "); Serial.println(STATE.stateIMU.angularVelocity.y());
  Serial.print("Angualar Velo Z: "); Serial.println(STATE.stateIMU.angularVelocity.z());

  STATE.setdataString();
  String dataString = STATE.getdataString();
  recordData(dataString, recordDataMode);
  

}
