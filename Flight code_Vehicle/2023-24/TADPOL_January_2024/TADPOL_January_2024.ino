#include <Arduino.h>
#include "TADPOLState.h"
#include <Kalman_Filter.h>

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

int BUZZER_PIN = 9;

//int TADPOL_I2C_ADDRESS = 1;

//String TERPData = 0;

// long KF_PREVIOUS_ITER_TIME; //in milliseconds

// bool HAS_GPS = false;

// //Initialize kf state
// int KFStateSize = 6;
// int KFMeasSize = 3;
// int KFControlSize = 3;
// double initialState[] = {0,0,0,0,0,0};
// double initialControl[] = {0,0,0};
// KFState KFSTATE = initialize(KFStateSize, KFMeasSize, KFControlSize, initialState, initialControl);

void setup() {
    buzz(BUZZER_PIN, 1000);
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
    setupPSRAM(TADPOLSTATE.csvHeader);
    bool sdSuccess = setupSDCard(TADPOLSTATE.csvHeader);

    // Wire1.begin(TADPOL_I2C_ADDRESS);
    // //calls function when data is requested from master
    // Wire1.onRequest(requestEvent);
    // //calls function when receiving data from master
    // Wire1.onReceive(receiveEvent);

    if(sdSuccess){
        buzz(BUZZER_PIN, 3000);
    }
    else{
        buzz(BUZZER_PIN, 250);
        delay(250);
        buzz(BUZZER_PIN, 250);
        delay(250);
        buzz(BUZZER_PIN, 250);
        delay(250);
        buzz(BUZZER_PIN, 250);
        delay(250);
    }

    // KF_PREVIOUS_ITER_TIME = millis();
}

void loop() {
    TADPOLSTATE.settimeAbsolute();
    TADPOLSTATE.stateBarometer.readBarometer();
    TADPOLSTATE.stateIMU.readIMU();
    TADPOLSTATE.stateGPS.readGPS();

    // //Run the KF here
    // double timeStep = TADPOLSTATE.timeAbsolute - KF_PREVIOUS_ITER_TIME;
    // //Check if there is a new gps reading
    // if ((TADPOLSTATE.stateGPS.satellites > 3) && (millis() - TADPOLSTATE.stateGPS.gpsTimer > 1000)){HAS_GPS = true;}
    // else{HAS_GPS = false;}
    // double meas[3] = {TADPOLSTATE.stateGPS.gpsPosition.x(), TADPOLSTATE.stateGPS.gpsPosition.y(), TADPOLSTATE.stateBarometer.relativeAltitude};
    // double control[3] = {TADPOLSTATE.stateIMU.acceleration.x(), TADPOLSTATE.stateIMU.acceleration.y(), TADPOLSTATE.stateIMU.acceleration.z()};
    // double *KFOutputState;
    // KFOutputState = iterate(&KFSTATE, timeStep, meas, control, HAS_GPS);
    // KF_PREVIOUS_ITER_TIME = millis();
    
    // //Set the kf output data in the TADPOLSTATE 
    // TADPOLSTATE.position.x() = KFOutputState[0]; TADPOLSTATE.position.y() = KFOutputState[1]; TADPOLSTATE.position.z() = KFOutputState[2];
    // TADPOLSTATE.velocity.x() = KFOutputState[3]; TADPOLSTATE.velocity.y() = KFOutputState[4]; TADPOLSTATE.velocity.z() = KFOutputState[5];
    // TADPOLSTATE.acceleration.x() = TADPOLSTATE.stateIMU.acceleration.x(); TADPOLSTATE.acceleration.y() = TADPOLSTATE.stateIMU.acceleration.y(); TADPOLSTATE.acceleration.z() = TADPOLSTATE.stateIMU.acceleration.z();
    

    TADPOLSTATE.determineTADPOLStage();

    TADPOLSTATE.setdataString();
    recordData(TADPOLSTATE.getdataString(), TADPOLSTATE.getrecordDataStage());
}

// void receiveEvent(int byteNum) {
//   String receivedMessage;
  
//   //Receive bytes and append them to the received message
//   while (byteNum > 0 && Wire1.available()) {
//     char c = Wire1.read();    //Receive byte as a character
    
//     if (byteNum == byteNum) { //Check if it's the first character
//       receivedMessage += c;  //Append the first character to the received message
//     } else {
//       receivedMessage += c;  //Append the remaining characters to the received message
//     }
    
//     byteNum--;
//   }

//   //return the received message (print necessary for demonstartion)
//   TERPData = receivedMessage;
// }

// // function that executes whenever data is requested by master
// void requestEvent()
// {
//   Wire1.write(TADPOLSTATE.stageNumber);
// }
