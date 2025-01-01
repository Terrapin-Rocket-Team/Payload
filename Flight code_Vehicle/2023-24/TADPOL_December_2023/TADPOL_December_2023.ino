#include <Arduino.h>
#include "TADPOLState.h"

Barometer TADPOLBarometer("BMP280");
IMU TADPOLIMU("BNO055");
GPS TADPOLGPS("NEO-M9N");
TADPOLState TADPOLSTATE;

int BUZZER_PIN = 9;

//int TADPOL_I2C_ADDRESS = 1;

//String TERPData = 0;

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
    setupSDCard(TADPOLSTATE.csvHeader);

    // Wire1.begin(TADPOL_I2C_ADDRESS);
    // //calls function when data is requested from master
    // Wire1.onRequest(requestEvent);
    // //calls function when receiving data from master
    // Wire1.onReceive(receiveEvent);

    buzz(BUZZER_PIN, 3000);
}

void loop() {
    TADPOLSTATE.settimeAbsolute();
    TADPOLSTATE.stateBarometer.readBarometer();
    TADPOLSTATE.stateIMU.readIMU();
    TADPOLSTATE.stateGPS.readGPS();

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
