#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Utils/Astra.h>
#include <AstraRocket.h>
#include <Sensors/MountingTransform.h>
#include <Servo.h>
using namespace astra;
using namespace astra_rocket;

BMI088 imu;
DPS368 baro;

AstraRocketConfig config; 

AstraRocket cnc(config);

Servo esc22;
Servo esc23;

const int ESC_STOP_US = 1500; // 1000µs is 0% throttle (also arms the ESC)
const int ESC_RUN_US = 2000;  // 1150µs is low throttle (adjust safely!)


void setup() {
    Serial.begin(115200);
    Serial8.begin(115200);

    esc22.attach(22);
    esc23.attach(23);

    esc22.writeMicroseconds(ESC_STOP_US);
    esc23.writeMicroseconds(ESC_STOP_US);

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
            Serial8.print("$J=G91 X150 F100\n");
        } 
        else if (incomingByte == '2') {
            Serial.println("Action: Jogging X-100");
            Serial8.print("$J=G91 X-100 F100\n");
        } 

        // -- Y-Axis Jogging --
        else if (incomingByte == '3') {
            Serial.println("Action: Jogging Y+100");
            Serial8.print("$J=G91 Y100 F25\n");
        } 
        else if (incomingByte == '4') {
            Serial.println("Action: Jogging Y-100");
            Serial8.print("$J=G91 Y-100 F25\n");
        }

        else if (incomingByte == 's') {
            Serial.println("Action: Emergency Stop (Jog Cancel)");
            Serial8.write(0x85); // GRBL Real-time Jog Cancel
            esc22.writeMicroseconds(ESC_STOP_US);
            esc23.writeMicroseconds(ESC_STOP_US);
        }

        // Pin 22 Control
        else if (incomingByte == 'a') {
            Serial.println("Action: Starting ESC on Pin 22 (Low Throttle)");
            esc22.writeMicroseconds(ESC_RUN_US);
        }
        else if (incomingByte == 'q') {
            Serial.println("Action: Stopping ESC on Pin 22");
            esc22.writeMicroseconds(ESC_STOP_US);
        }
        
        // Pin 23 Control
        else if (incomingByte == 'b') {
            Serial.println("Action: Starting ESC on Pin 23 (Low Throttle)");
            esc23.writeMicroseconds(ESC_RUN_US);
        }
        else if (incomingByte == 'w') {
            Serial.println("Action: Stopping ESC on Pin 23");
            esc23.writeMicroseconds(ESC_STOP_US);
        }
    }
   
}
