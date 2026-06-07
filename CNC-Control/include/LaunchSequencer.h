#pragma once

#include <Arduino.h>
#include <CncState.h>
#include "FileLoader.h"
#include <Sensors/HW/IMU/BMI088.h>

class LaunchSequencer {
public:
    // Constructor matching the initializer list inside your LaunchSequencer.cpp
    LaunchSequencer(CncState& inState, FileLoader& inLoader, BMI088& inImu);

    // Main periodic execution pass running in the main loop
    void update();

private:
    // Hardware and Core Data Structure References
    CncState& state;
    FileLoader& loader;
    BMI088& imu;

    // Asynchronous State Machine tracking for streaming G-code to GRBL
    enum class StreamState {
        IDLE,
        SEND_NEXT,
        WAITING_FOR_OK,
        WAITING_FOR_IDLE
    };

    StreamState streamState;
    String rxBuffer;
    unsigned long lastQueryTime;
    unsigned long lastLogTime;

    // Internal Asynchronous Flight & Sequencing Worker Methods
    void updateAcceleration();
    void checkLaunchDetection();
    void runSequence();
    void parseGrblResponse();
};
