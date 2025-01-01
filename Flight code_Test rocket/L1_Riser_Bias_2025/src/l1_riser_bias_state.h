#ifndef L1_RISER_BIAS_H
#define L1_RISER_BIAS_H

#include "MMFS.h"
#include "Target.h"

using namespace mmfs;

enum L1RiserBiasStages { // TODO update this
    PRELAUNCH,
    BOOST,
    COAST,
    DROUGE,
    MAIN,
    LANDED
};

// SOD Farm
const int numTarg = 2;
const int numObs = 1;

extern const Line line1;
extern Obstacle* obstacles[];
extern const Point targetPoints[];

class L1RiserBiasState : public State
{
public:
    L1RiserBiasState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, bool stateRecordsOwnData = true);
    void updateState(double newTime = -1) override;
    int buzzerPin;

    bool topParachuteFlag;
    bool releasedFlag;

    double Kp = .17; //Proportional error constant -- MANUAL INPUT TODO
    double Kd = .33; //Derivative error constant -- MANUAL INPUT TODO
    int pwmMax=150;
    float pwmScale=1.3; // TOOD
    int pwmChangeSignDelay = 5; //in ms
    float pwmZeroAngleCone = 5; //in degrees, the +/- angle which creates a cone around Ep setting it to 0 if within the range
    double pwmTimer = 0.0; //in ms
    double pwmFrequency = 250; //in ms TODO set this

    imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
    imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
    double v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
    Point averageWindCorrectionCoords;

    void determineTADPOLStage();
    void fanSetup(int fowardFanPin,int backwardFanPin);
    void runFan(double pwm, int forwardFanPin, int backwardFanPin);
    double findPWM(double direction, double timeSinceLastIteration);
    Point getTargetCoordinates();
    Point getWindCorrectionCoordinates(Point r);
    //imu::Vector<3> getInertialAngularVelocity();

private:
    void determineStage();
    int stage;
    double timeOfLaunch;
    double timeOfLastStage;
    double timeOfDay;
};
#endif