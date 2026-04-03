#pragma once

#include <Arduino.h>
#include <CncState.h>
#include "FileLoader.h"
#include <Sensors/HW/IMU/BMI088.h>

class LaunchSequencer {
public:
    CncState& state;
    FileLoader& loader;
    BMI088& imu;

    LaunchSequencer(CncState& inState, FileLoader& inLoader, BMI088& inImu);

    void update();

private:
    void updateAcceleration();
    void checkLaunchDetection();
    void runSequence();
    bool waitForOk(unsigned long timeoutMs = 5000);
    bool waitForIdle();
};