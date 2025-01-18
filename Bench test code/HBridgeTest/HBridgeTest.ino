#include <Arduino.h>
#include <Multi_Mission_Flight_Software.h>

State STATE;
IMU TESTIMU("BNO055"); //Input IMU type here

float dc=.5;
float dc_actual;
String recordDataMode = "Flight";
int hz=20;
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
  setupPSRAM(STATE.csvHeader + "Duty Cycle,");
  setupSDCard(STATE.csvHeader);

  pinMode(fwdPin,OUTPUT);
  pinMode(bckPin,OUTPUT);

}

void loop() {

  if (millis() > 15000){
    dc_actual = 0;
    recordDataMode = "Flight";
  } else if (millis() > 5000) {
    dc_actual = dc;
    recordDataMode = "Test";
  } else {
    dc_actual = 0;
    recordDataMode = "Flight";
  }

  STATE.settimeAbsolute();

  STATE.stateIMU.readIMU();
  Serial.print("Angualar Velo X: "); Serial.println(STATE.stateIMU.angularVelocity.x());
  Serial.print("Angualar Velo Y: "); Serial.println(STATE.stateIMU.angularVelocity.y());
  Serial.print("Angualar Velo Z: "); Serial.println(STATE.stateIMU.angularVelocity.z());
    Serial.print("dc_actual: "); Serial.println(dc_actual);

  if((millis()-pwmtimer)<(1000*abs(dc_actual)/hz)){
    if(dc_actual>0){
      digitalWrite(fwdPin,HIGH);
      digitalWrite(bckPin,LOW);
    }
    else if(dc_actual<0){
      digitalWrite(fwdPin,LOW);
      digitalWrite(bckPin,HIGH);
    }
    else{
      digitalWrite(fwdPin,LOW);
      digitalWrite(bckPin,LOW);    
    }
  }
  else if(millis()-pwmtimer<(1000*1/hz)){
    digitalWrite(fwdPin,LOW);
    digitalWrite(bckPin,LOW); 
  }
  else{
    pwmtimer=millis();
  }

  STATE.setdataString();
  String dataString = STATE.getdataString() + String(dc) + ",";
  Serial.println(dataString);
  recordData(dataString, recordDataMode);
  

}
