#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Utils/Astra.h>
#include "CncState.h"

using namespace astra;

CncState cncState;


BMI088 imu("BMI088", &Wire);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    AstraConfig config = AstraConfig()
        .with6DoFIMU(&imu)  
        .withState(&cncState);

    Serial.println("Setup complete");
}

void loop() {
    
    cncState.updateCncState();

    // Read accelerometer
    Vector<3> accel = imu.getAccelSensor()->getAccel();
    Serial.print(accel.x());
    Serial.print(accel.y());
    Serial.print(accel.z());

    delay(100);
}
