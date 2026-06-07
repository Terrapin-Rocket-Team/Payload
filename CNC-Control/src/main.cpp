#include <Arduino.h>
#include <Wire.h>
#include <Sensors/HW/IMU/BMI088.h>
#include <Sensors/HW/Baro/DPS368.h>
#include <Utils/Astra.h>
#include <AstraRocket.h>
#include <Sensors/MountingTransform.h>
#include <Servo.h>
#include <CncState.h>
#include "FileLoader.h"
#include "LaunchSequencer.h"
#include <RecordData/Logging/EventLogger.h>
#include <RecordData/Logging/LoggingBackend/ILogSink.h>
#include <RecordData/Storage/StorageFactory.h>

using namespace astra;
using namespace astra_rocket;

// Hardware Drivers
BMI088 myimu;
DPS368 baro;
Servo esc23; // Spindle ESC

// Rocket Avionics Framework Configuration
AstraRocketConfig config;
AstraRocket cnc(config);

// CNC State and File Utilities
CncState cncState;
FileLoader fileLoader;

#define ESC_PIN 23

const char* GCODE_SEQUENCE_FILE = "sequence.txt";

const int ESC_STOP_US = 1500; // 0% Throttle / Arming signal
const int ESC_RUN_US = 1750;  // Operating spindle speed

// Flight Logging Configuration
FileLogSink eventFile("EVENTS.log", StorageBackend::SD_CARD, true);
ILogSink* eventSinks[] = { &eventFile };

FileLogSink telemFile("TELEM.csv", StorageBackend::SD_CARD, true);
ILogSink* telemSinks[] = { &telemFile };

// Launch Sequencer state machine
LaunchSequencer sequencer(cncState, fileLoader, myimu);

void setup() {
    // 1. Initialize Communications Bus
    Wire.begin(); // Crucial for I2C communication with IMU/Baro
    Serial.begin(115200);
    Serial8.begin(115200); // UART Link to Protoneer Shield (GRBL)

    // 2. Attach and safely Arm the Spindle ESC
    esc23.attach(ESC_PIN);
    esc23.writeMicroseconds(ESC_STOP_US); // Send arming pulse immediately

    // 3. Handle GRBL Boot and Alarm Lock
    Serial.println("Waiting for GRBL to complete bootloader sequence...");
    delay(2500);

    Serial8.print("\n"); // Clear junk characters in hardware RX buffer
    delay(100);

    Serial.println("Unlocking GRBL from boot-up ALARM state...");
    Serial8.print("$X\n"); // Send explicit unlock sequence with newline
    delay(500);

    // Clear any residual feedback sent back by GRBL during boot
    while (Serial8.available()) Serial8.read();

    // 4. Initialize Storage and Load Sequence G-code
    config.withEventLogs(eventSinks, 1);
    config.withDataLogs(telemSinks, 1);

    IStorage* gcodeStorage = StorageFactory::create(StorageBackend::SD_CARD);
    if (!gcodeStorage || !gcodeStorage->begin()) {
        Serial.println("ERROR: SD card init failed!");
    }

    if (!fileLoader.load(*gcodeStorage, GCODE_SEQUENCE_FILE)) {
        Serial.print("ERROR: Could not load ");
        Serial.print(GCODE_SEQUENCE_FILE);
        Serial.println("!");
        while (1);
    }
    gcodeStorage->end();
    delete gcodeStorage;

    Serial.print("Loaded ");
    Serial.print(fileLoader.countLine());
    Serial.print(" lines from ");
    Serial.println(GCODE_SEQUENCE_FILE);

    // 5. Initialize internal CNC Tracking States
    Serial.println("Initializing CNC State Trackers...");
    cncState.begin(Serial8, esc23);

    cncState.commandSent = false;
    cncState.commandStopped = false;
    cncState.detectTime = 0;
    cncState.start = 0;
    cncState.prevAccel = 0;
    cncState.step = 0;
    cncState.detect = false;

    // Send configuration parameters to GRBL (Ensure terminating \n is present)
    Serial8.print("$1=25\n");
    delay(50);
    while (Serial8.available()) Serial8.read();

    // 6. Initialize Astra Rocket Avionics State Machine
    myimu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    config.with6DoFIMU(&myimu);
    config.withBaro(&baro);

    Serial.println("Initializing AstraRocket Flight Core...");
    if (!cnc.init()) {
        Serial.println("ERROR: AstraRocket initialization failed!");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("System Ready for Flight Loop.");
}

void loop() {
    // Keeps flight filters, sensor polling, and logger sinks running
    cnc.update();

    // Evaluates flight state changes and handles streaming lines to GRBL
    sequencer.update();
}
