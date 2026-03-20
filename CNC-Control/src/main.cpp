#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Utils/Astra.h>
#include <AstraRocket.h>
#include <Sensors/MountingTransform.h>
#include <Servo.h>
#include <CncState.h>
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>
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

// Sink for human-readable events (e.g., LAUNCH_DETECTED)
FileLogSink eventFile("EVENTS.log", StorageBackend::SD_CARD, true);
ILogSink* eventSinks[] = { &eventFile };

// Sink for telemetry data (e.g., IMU data, custom variables)
FileLogSink telemFile("TELEM.csv", StorageBackend::SD_CARD, true);
ILogSink* telemSinks[] = { &telemFile };

void setup() {
    Serial.begin(115200);
    Serial8.begin(115200);

    config.withEventLogs(eventSinks, 1);
    config.withDataLogs(telemSinks, 1);

    Serial.println("Initializing CNC...");
    cncState.begin(Serial8, esc23);

    cncState.commandSent = false;
    cncState.commandStopped = false;
    cncState.detectTime = 0;
    cncState.start = 0;
    cncState.prevAccel = 0;
    cncState.step = 0;

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

}

void loop() {
    
    cnc.update();

    // Read accelerometer
    cncState.accel = myimu.getAccelSensor()->getAccel();

    cncState.totalAccel = 0.8*sqrt(
        cncState.accel.x()*cncState.accel.x() +
        cncState.accel.y()*cncState.accel.y() +
        cncState.accel.z()*cncState.accel.z()
    ) + 0.2*cncState.prevAccel;

    Serial.print("Total Accleration: ");
    Serial.println(cncState.totalAccel);

    if (cncState.totalAccel > 40 and !cncState.detect) {
        cncState.detectTime = millis();
        cncState.detect = true;
    }

    if (cncState.totalAccel < 40 and cncState.detect) {
        cncState.detect = false;
        cncState.detectTime = 0;
    }

    if ((cncState.totalAccel > 40) && !cncState.commandSent && ((millis() - cncState.detectTime) > 500) && (cncState.step == 0)) {

        // Log the event to the .log file
        LOGI("LAUNCH_DETECTED: Acceleration threshold exceeded.");

        // // Record the exact command being sent to Serial8 in the log
        cncState.start = millis();
        //cncState.spindleStart();
        cncState.send("$J=G91 X150 F100\n");
        cncState.commandSent = true;
        LOGI("STATE: CNC is moving into stock");

        cncState.step++;
    }
   
    if (cncState.commandSent && (millis() - cncState.start > 5000) && !cncState.commandStopped && (cncState.step == 1)) {
        cncState.cancelJog();
        cncState.send("$J=G91 Y-100 F25\n");
        LOGI("STATE: CNC is moving in +Y direction");

        cncState.step++;
    }

    if (cncState.commandSent && (millis() - cncState.start > 10000) && !cncState.commandStopped && (cncState.step == 2)) {
        cncState.cancelJog();
        cncState.send("$J=G91 X100 F100\n");
        LOGI("STATE: CNC is moving in +X direction");

        cncState.step++;
    }

    if (cncState.commandSent && (millis() - cncState.start > 50000) && !cncState.commandStopped && (cncState.step == 3)) {
        cncState.cancelJog();
        cncState.send("$J=G91 Y100 F25\n");
        LOGI("STATE: CNC is moving in -Y direction");

        cncState.step++;
    }

    if (cncState.commandSent && (millis() - cncState.start > 55000) && !cncState.commandStopped && (cncState.step == 4)) {
        cncState.cancelJog();
        cncState.send("$J=G91 X-100 F100\n");
        LOGI("STATE: CNC is moving in -X direction");

        cncState.step++;
    }

    if (cncState.commandSent && (millis() - cncState.start > 90000) && !cncState.commandStopped && (cncState.step == 5)) {
        cncState.cancelJog();
        //cncState.spindleStop();
        cncState.commandStopped = true;
        LOGI("STATE: Sequence complete. System Idle.");

        cncState.step++;
    }    

    cncState.prevAccel = cncState.totalAccel;

}
