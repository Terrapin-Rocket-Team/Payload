#ifndef MULE_STATE_H
#define MULE_STATE_H

#include "MMFS.h"

using namespace mmfs;

enum MuleStages {
    PRELAUNCH,
    BOOST,
    COAST,
    DROUGE,
    MAIN,
    LANDED,
    DUMPED
};

class MuleState : public State
{
public:
    MuleState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, int buzzerPin);
    void updateState(double newTime = -1) override;
    int stage;
    int buzzerPin;

private:
    void determineStage();
    double timeOfLaunch;
    double timeOfLastStage;
    double timeOfDay;
};
#endif