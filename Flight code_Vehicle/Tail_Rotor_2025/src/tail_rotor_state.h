#ifndef TAIL_ROTOR_STATE_H
#define TAIL_ROTOR_STATE_H

#include "MMFS.h"
#include "bmp280_breakout.h"
#include "bno055_breakout.h"
#include "max_m10s_breakout.h"
#include "Target.h"
#include <Servo.h>

enum TailRotorStages {
    PRELAUNCH,
    BOOST,
    COAST,
    DROUGE,
    RELEASED,
    LANDED
};

// Tail Rotor Stuff
Servo motor;

// SOD Farm
const int numTarg = 2;
const int numObs = 1;

extern const Line line1;
extern Obstacle* obstacles[];
extern const Point targetPoints[];

class TailRotorState : public mmfs::State
{
public:
    TailRotorState(mmfs::Sensor **sensors, int numSensors, mmfs::LinearKalmanFilter *kfilter, int BuzzerPin, bool stateRecordsOwnData = true);
    void updateState(double newTime = -1) override;
    int buzzerPin;
    int stage;

    bool topParachuteFlag;
    bool releasedFlag;

    float kp = .5; //Proportional error constant -- MANUAL INPUT TODO
    float kd = .4; //Derivative error constant -- MANUAL INPUT TODO
    float directionalCorrection = 0.8; // Correction for prop inefficiency
    float previousError = 0.0;
    // float pwmScale=1.3; // TOOD
    // int pwmChangeSignDelay = 5; //in ms
    // float pwmZeroAngleCone = 5; //in degrees, the +/- angle which creates a cone around Ep setting it to 0 if within the range
    // double pwmTimer = 0.0; //in ms
    // double pwmFrequency = 250; //in ms TODO set this

    imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
    imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
    float v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
    Point averageWindCorrectionCoords;

    void determineTADPOLStage();
    void fanSetup(int servoPin);
    void runFan(int pwm);
    int findPWM(float goal, float deltaTime);
    Point getTargetCoordinates();
    Point getWindCorrectionCoordinates(Point r);
    float getGoalAngle(Point target);

private:
    void determineStage();
    float timeOfLaunch;
    float timeOfLastStage;
    float timeOfDay;
};
#endif