#pragma once

#include <Arduino.h>
#include <CncState.h>
#include "FileLoader.h"
#include <Sensors/HW/IMU/BMI088.h>

class LaunchSequencer {
public:
<<<<<<< HEAD
    CncState& state;
    FileLoader& loader;
    BMI088& imu;

    LaunchSequencer(CncState& inState, FileLoader& inLoader, BMI088& inImu);

=======
    LaunchSequencer(CncState& state, FileLoader& loader, BMI088& imu);
>>>>>>> 1765d448d2c1068f4e5883d4d1b52a3980737428
    void update();

private:
    void updateAcceleration();
    void checkLaunchDetection();
    void runSequence();
    bool waitForOk(unsigned long timeoutMs = 5000);
    bool waitForIdle();
<<<<<<< HEAD
};
=======

    CncState& _state;
    FileLoader& _loader;
    BMI088& _imu;
};
>>>>>>> 1765d448d2c1068f4e5883d4d1b52a3980737428
