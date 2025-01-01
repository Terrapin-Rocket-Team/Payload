#include <Arduino.h>
#include "TERPState.h"


int LIGHT_THRESHOLD = 30;  // in lx

int BUZZER_PIN = 17;

int TADPOL_POWER_PIN = 23;

//int TADPOL_I2C_ADDRESS = 1;

Barometer TERPBarometer("BMP280");
IMU TERPIMU("BNO055");
LightSensor TERPLightSensor("BH1750");
TERPState TERPSTATE;

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
    setupSDCard(TERPSTATE.csvHeader);

    //Wire1.begin();

    buzz(BUZZER_PIN, 3000);

    powerOnTADPOL(TADPOL_POWER_PIN);
}

void loop() {
    TERPSTATE.settimeAbsolute();
    TERPSTATE.stateBarometer.readBarometer();
    TERPSTATE.stateIMU.readIMU();
    TERPSTATE.stateLightSensor.readLightSensor();

    TERPSTATE.determineTERPStage();

    // if(TERPSTATE.stage == "Drogue" && TERPSTATE.stateLightSensor.lux > LIGHT_THRESHOLD){
    //     buzz(BUZZER_PIN, 1000);
    //     //releaseUDM();  // TOD_O need to create this function
    // }

    TERPSTATE.setdataString();

    // I2CSend(TERPSTATE.stage, TADPOL_I2C_ADDRESS);
    // String TADPOLData = I2CReceive(1, TADPOL_I2C_ADDRESS);
    recordData(TERPSTATE.getdataString(), TERPSTATE.getrecordDataStage());

}

void powerOnTADPOL(int power_on_pin) {
    pinMode(power_on_pin, OUTPUT);
    digitalWrite(power_on_pin, HIGH);
    delay(10000);
    digitalWrite(power_on_pin, LOW);
}

// void I2CSend(String data, int address){
//     // Master to Slave
//     int dataStringLength = data.length();
//     char dataChar[dataStringLength];
//     data.toCharArray(dataChar, dataStringLength);

//     Wire1.beginTransmission(address);
//     Wire1.write(dataChar, strlen(dataChar));
//     Wire1.endTransmission();

// }

// String I2CReceive(int size, int address) {
//   //Slave to Master
//   Wire1.requestFrom(address, size);

//   String receivedMessage;
//   //receive bytes and append to the received message
//   while (Wire1.available()) {
//     char c = Wire1.read();  //receive byte as a character
//     receivedMessage += c;  //append the character to the received message
//   }

//   return receivedMessage;
// }
