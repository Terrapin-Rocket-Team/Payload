#ifndef TAIL_ROTOR_STATE_H
#define TAIL_ROTOR_STATE_H

#include "MMFS.h"
#include "Target.h"

enum TailRotorStages {
    PRELAUNCH,
    BOOST,
    COAST,
    DROUGE,
    RELEASED,
    LANDED
};

/// Navigation Stuff ///
// Higgs Farm
const int numTarg = 3;
const int numObs = 3;

extern const Line line1;
extern Obstacle* obstacles[numTarg];
extern const Point targetPoints[numObs];
/////////////////

class TailRotorState : public mmfs::State
{
    struct PackedData
    {
        float t;
        float px;
        float py;
        float pz;
        float vx;
        float vy;
        float vz;
        float ax;
        float ay;
        float az;
        // Payload specific data
        int pwm;
        float goalAngleUsed;
        float goalAngleCalculated;
        float gx;
        float gy;
        float wx;
        float wy;
        float vehicleSpeed;
        float averageWindCorrectionCoords_X;
        float averageWindCorrectionCoords_Y;
        float targetCoords_X;
        float targetCoords_Y;
    } __attribute__((packed));

public:
    TailRotorState(mmfs::Sensor **sensors, int numSensors, mmfs::LinearKalmanFilter *kfilter, int BuzzerPin);
    void updateState(double newTime = -1) override;
    int buzzerPin;
    int stage;

    bool topParachuteFlag;
    bool releasedFlag;

    float kp = .7; // Proportional gain -- MANUAL INPUT
    float kd = 2*kp; // Derivative gain -- MANUAL INPUT TODO
    float ki = 0.2; // Integral gain -- MANUAL INPUT TODO
    float directionalCorrection = 0.8; // Correction for prop inefficiency
    float previousError = 0.0;
    float integral = 0.0;

    imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
    imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
    float v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
    Point averageWindCorrectionCoords;

    void determineTADPOLStage();
    int findPWM(float goal, float deltaTime);
    Point getTargetCoordinates();
    Point getWindCorrectionCoordinates(Point r);
    float getGoalAngle(Point target);

    // DataReporter functions (State override)
    virtual const mmfs::PackedType *getPackedOrder() const override;
    virtual const int getNumPackedDataPoints() const override;
    virtual const char **getPackedDataLabels() const override;
    virtual void packData() override;

    // For pakcing data
    int pwm;
    float goalAngleUsed;
    float goalAngleCalculated;
    Point targetCoords;

private:
    void determineStage();
    float timeOfLaunch;
    float timeOfLastStage;
    float timeOfDay;
};
#endif