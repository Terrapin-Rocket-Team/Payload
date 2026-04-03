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
#include <SPI.h>
#include <SD.h>

using namespace astra;
using namespace astra_rocket;

BMI088 myimu;
DPS368 baro;
Servo esc23;

AstraRocketConfig config; 
AstraRocket cnc(config);

CncState cncState;
FileLoader fileLoader;

#define SD_CS_PIN 10

const int ESC_STOP_US = 1500;
const int ESC_RUN_US = 2000;

FileLogSink eventFile("EVENTS.log", StorageBackend::SD_CARD, true);
ILogSink* eventSinks[] = { &eventFile };

FileLogSink telemFile("TELEM.csv", StorageBackend::SD_CARD, true);
ILogSink* telemSinks[] = { &telemFile };

LaunchSequencer sequencer(cncState, fileLoader, myimu); // task

void setup() {
    Serial.begin(115200);
    Serial8.begin(115200);

    Serial8.write("\r\n\r\n");
    delay(2000);
    while (Serial8.available()) Serial8.read();

    config.withEventLogs(eventSinks, 1);
    config.withDataLogs(telemSinks, 1);

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("ERROR: SD card init failed!");
        while (1);
    }

    if (!fileLoader.load("sequence.gcode")) {
        Serial.println("ERROR: Could not load sequence.gcode!");
        while (1);
    }

    Serial.print("Loaded ");
    Serial.print(fileLoader.countLine());
    Serial.println(" lines from sequence.gcode");

    Serial.println("Initializing CNC...");
    cncState.begin(Serial8, esc23);

    cncState.commandSent = false;
    cncState.commandStopped = false;
    cncState.detectTime = 0;
    cncState.start = 0;
    cncState.prevAccel = 0;
    cncState.step = 0;
    cncState.detect = false;

    Serial8.print("$1=25");

    myimu.setMountingOrientation(MountingOrientation::ROTATE_90_Z);
    config.with6DoFIMU(&myimu);
    config.withBaro(&baro);

    Serial.println("Initializing AstraRocket...");
    if (!cnc.init()) {
        Serial.println("ERROR: AstraRocket initialization failed!");
        while (true) { 
            delay(1000); 
        }
    }
}

void loop() {

    cnc.update();
    sequencer.update();

}

