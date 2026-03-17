#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Utils/Astra.h>
#include <AstraRocket.h>
#include <Sensors/MountingTransform.h>
#include <Servo.h>
#include <CncState.h>
using namespace astra;
using namespace astra_rocket;

BMI088 myimu;
DPS368 baro;
Servo esc23;

AstraRocketConfig config; 
AstraRocket cnc(config);

CncState cncState;

const int ESC_STOP_US = 1500; // 1000µs is 0% throttle (also arms the ESC)
const int ESC_RUN_US = 2000;  // 1150µs is low throttle (adjust safely!)

// Variables we want to track in the CSV log
float totalAccel = 0;
bool commandSent = false;
bool commandStopped = false;
bool detect;
unsigned long detectTime = 0;
unsigned long start = 0;


void setup() {
    Serial.begin(115200);
    Serial8.begin(115200);

    Serial.println("Initializing CNC...");
    cncState.begin(Serial8, esc23);

    myimu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    config.with6DoFIMU(&myimu);
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
    Vector<3> accel = myimu.getAccelSensor()->getAccel();

    totalAccel = sqrt(
        accel.x()*accel.x() +
        accel.y()*accel.y() +
        accel.z()*accel.z()
    );

    Serial.print("Total Accleration: ");
    Serial.println(totalAccel);

    if (totalAccel > 40 and !detect) {
        detectTime = millis();
        detect = true;
    }

    if (totalAccel < 40 and detect) {
        detect = false;
    }

    if (totalAccel > 40 && !commandSent && ((millis() - detectTime) > 500)) {

        // Log the event to the .log file
        // cnc.getLogger().log("LAUNCH_DETECTED: Acceleration threshold exceeded.", LogLevel::INFO);

        // // Record the exact command being sent to Serial8 in the log
        cncState.start = millis();
        cncState.spindleStart();
        cncState.send("$J=G91 X150 F100\n");
        cncState.send("$J=G91 Y100 F25\n");
        commandSent = true;

        

        // cnc.getLogger().log("STATE: CNC is moving...", LogLevel::INFO);
        
        // // Ensure logs are written to SD before stopping
        // cnc.getLogger().flush();

    }
   
    if (commandSent && (millis() - cncState.start > 5000) && !commandStopped) {
        cncState.cancelJog();
        cncState.spindleStop();
        commandStopped = true;

        // cnc.getAstraSystem()->getLogger().log("STATE: Sequence complete. System Idle.", LogLevel::INFO);
        // cnc.getAstraSystem()->getLogger().flush();

        Serial.println("Sequence Complete.");
    }

}
