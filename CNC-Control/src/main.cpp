#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Utils/Astra.h>
#include <AstraRocket.h>
#include <Sensors/MountingTransform.h>
using namespace astra;
using namespace astra_rocket;

BMI088 imu;
DPS368 baro;

AstraRocketConfig config; 

AstraRocket cnc(config);


void setup() {
    Serial.begin(115200);

    imu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);

    config.with6DoFIMU(&imu);
    config.withBaro(&baro);
    
    Serial.println("Initializing AstraRocket...");
    if (!cnc.init())
    {
        Serial.println("ERROR: AstraRocket initialization failed!");
        while (1)
        {
            delay(1000);
        }
    }
}

void loop() {
    
    cnc.update();

    // Read accelerometer
    Vector<3> accel = imu.getAccelSensor()->getAccel();
    Serial.print("accel x: ");
    Serial.print(accel.x());
    Serial.print("  accel y: ");
    Serial.print(accel.y());
    Serial.print("  accel z: ");
    Serial.println(accel.z());

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
