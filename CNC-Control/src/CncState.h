#pragma once

#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <Sensors/HW/Accel/ADXL375.h>
#include <Vector.h>

using namespace astra;

class CncState : public State  
{

public:
    CncState();

    void updateCncState();
    bool isCncRunning();
    

    // Pointer to accelerometer from library
    Accel* accelSensor;

    //Getter
    void get_Acceleration();

private:
    bool cncActive;
    bool cncDone;
    const float accelerationThreshold = 40;

    void startCNC();
    void stopCNC();
};
