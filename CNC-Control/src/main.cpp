#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Utils/Astra.h>
#include "CncState.h"

using namespace astra;

CncState cncState;


BMI088 myImu("BMI088", &Wire);

AstraConfig config = AstraConfig()
        .with6DoFIMU(&myImu)  
        .withState(&cncState);

Astra sys(&config);


void setup() {
    Serial.begin(115200);
    Wire.begin();
     int err = sys.init();
    if (err != 0) {
        LOGE("Astra init failed with %d error(s)", err);
    }
    Serial.println("Setup complete");
}

void loop() {
    
    sys.update();
    cncState.updateCncState();

    // Read accelerometer
    Vector<3> accel = myImu.getAccelSensor()->getAccel();
    Serial.print(accel.x());
    Serial.print(accel.y());
    Serial.print(accel.z());

    float zAccel = accel.z();

    if (zAccel > 40) {
        delay(100);

    Serial1.print("$J=G91 X100 F500\n");

    // Keep moving for the specified time
    delay(1000);

    // 0x85 is the Real-time "Jog Cancel" command for GRBL
    // This stops the motion immediately with a controlled deceleration
    Serial1.write(0x85);
    }
   
}
