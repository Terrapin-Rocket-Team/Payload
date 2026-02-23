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
    Serial8.begin(115200);

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
    // Vector<3> accel = imu.getAccelSensor()->getAccel();
    // Serial.print("accel x: ");
    // Serial.print(accel.x());
    // Serial.print("  accel y: ");
    // Serial.print(accel.y());
    // Serial.print("  accel z: ");
    // Serial.println(accel.z());

    // Check if the user typed something in the Serial Monitor
    if (Serial.available() > 0) {
        char incomingByte = Serial.read();

        if (incomingByte == '1') {
            Serial.println("Action: Jogging X+100");
            // $J= is a Jogging command. G91 is incremental mode.
            Serial8.print("$J=G91 X100 F500\n");
        } 
        else if (incomingByte == '2') {
            Serial.println("Action: Jogging X-100");
            Serial8.print("$J=G91 X-100 F500\n");
        } 
        else if (incomingByte == 's') {
            Serial.println("Action: Emergency Stop (Jog Cancel)");
            Serial8.write(0x85); // GRBL Real-time Jog Cancel
        }
    }
   
}
