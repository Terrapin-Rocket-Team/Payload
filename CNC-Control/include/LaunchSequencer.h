#pragma once

#include <Arduino.h>
#include <CncState.h>
#include "FileLoader.h"
#include <Sensors/HW/IMU/BMI088.h>

class LaunchSequencer {
public:
    LaunchSequencer(CncState& state, FileLoader& loader, BMI088& imu);
    void update();

private:
    void updateAcceleration();
    void checkLaunchDetection();
    void runSequence();
    bool waitForOk(unsigned long timeoutMs = 5000);
    bool waitForIdle();

    CncState& _state;
    FileLoader& _loader;
    BMI088& _imu;
};
