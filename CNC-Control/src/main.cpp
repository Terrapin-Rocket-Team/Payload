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

// Variables we want to track in the CSV log
float totalAccel = 0;
bool commandSent = false;


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

    // astra::Logger::addTelemetryVar("Total_Accel", &totalAccel);
    // astra::Logger::addTelemetryVar("CNC_Active", &commandSent);
}

void loop() {
    
    cnc.update();

    // Read accelerometer
    Vector<3> accel = imu.getAccelSensor()->getAccel();
    totalAccel = abs(accel.x()) + abs(accel.y()) + abs(accel.z());
    Serial.print("Total Accleration: ");
    Serial.println(totalAccel);

    if (totalAccel > 40 && !commandSent) {
        delay(100);

        // Log the event to the .log file
        // cnc.getLogger().log("LAUNCH_DETECTED: Acceleration threshold exceeded.", LogLevel::INFO);
        
        // // Record the exact command being sent to Serial8 in the log
        // cnc.getLogger().log("ACTION: Sending Command $J=G91 X20.8 F500 to GRBL", LogLevel::INFO);

        Serial8.print("$J=G91 X150 F500\n");
        commandSent = true;

        // cnc.getLogger().log("STATE: CNC is moving...", LogLevel::INFO);
        
        // // Ensure logs are written to SD before stopping
        // cnc.getLogger().flush();

        while(1) {
            cnc.update();
            delay(100);
        }
    }
   
}
