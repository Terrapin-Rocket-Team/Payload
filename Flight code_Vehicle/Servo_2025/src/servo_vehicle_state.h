#ifndef SERVO_VEHICLE_STATE_H
#define SERVO_VEHICLE_STATE_H

#include "MMFS.h"
#include "Target.h"
#include <PWMServo.h>

using namespace mmfs;

enum ServoVehcileStages { // TODO update this
    PRELAUNCH,
    BOOST,
    COAST,
    MAIN,
    LANDED,
    DUMPED
};

// SOD Farm
const int numTarg = 2;
const int numObs = 1;

extern const Line line1;
extern Obstacle* obstacles[];
extern Point targetPoints[];

class ServoVehicleState : public State
{

static PWMServo leftServo;
static PWMServo rightServo;

public:
    ServoVehicleState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, int buzz_pin);
    void updateState(double newTime = -1) override;

    int stage;
    int buzzer_pin;

    bool topParachuteFlag;
    bool releasedFlag;

    double left_servo_value;
    double right_servo_value; 

    imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
    imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
    double v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
    Point averageWindCorrectionCoords;

    void determineTADPOLStage();
    Point getTargetCoordinates();
    Point getWindCorrectionCoordinates(Point r);

    // Servo Functions
    void servoSetup(int leftServoPin,int rightServoPin,double leftSetNeutral,double rightSetNeutral);
    void moveServo(double delta);
    double findDelta(double phi, double gamma);
    void goDirection(double direction);

    // DataReporter functions (State override)
    virtual const PackedType *getPackedOrder() const override;
    virtual const int getNumPackedDataPoints() const override;
    virtual const char **getPackedDataLabels() const override;

private:
    void determineStage();

    double timeOfLaunch;
    double timeOfLastStage;
    double timeOfDay;

    // DataReporter functions
    virtual void packData() override;
};
#endif