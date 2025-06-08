#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <State/State.h>
#include "target.h"
#include "Servo.h"


enum VehcileStages {
    PRELAUNCH,
    BOOST, //Rocket accelerating upwards
    COAST, //Rocket coasting
    DROGUE,
    EJECTED, //Vehicle Ejected
    GLIDING, //Vehicle Gliding
    LANDED, //Vehicle Landed
    DUMPED
};

// SOD Farm
const int numTarg = 1;
const int numObs = 0;

extern const Line line1;
extern Obstacle* obstacles[];
extern Point targetPoints[];

using namespace mmfs;

class VehicleState : public State {


    public:
        VehicleState(Sensor **sensors, int numSensors, Filter *filter);
        void updateState(double newTime = -1) override;
        
        int stage;
        int buzzer_pin;

        bool topParachuteFlag;
        bool releasedFlag;

        double left_servo_value;
        double right_servo_value; 

        double rocketx;
        double rockety;

        imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
        imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
        double v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
        Point averageWindCorrectionCoords;

        Point targetCoords;

        Point getTargetCoordinates();
        Point getWindCorrectionCoordinates(Point r);

        void servoSetup(int leftServoPin,int rightServoPin,int camServoPin,double leftSetNeutral,double rightSetNeutral,double camSetNeutral);
        void moveServo(double delta);
        void moveCam();
        double findDelta(double phi, double gamma);
        void goDirection(double direction);
        double goalOrbit(double rocketX, double rocketY, double X, double Y, double R);

        Servo pitch;
        Servo left;
        Servo right;

        float leftServoValue;
        float rightServoValue;
        float servoOutput;
        float vehicleX;
        float vehicleY;
        float currentAngle;
        float targetAngle;
        float currentAngleY;
        float currentAngleZ;
        float vehicleSpeedX;
        float vehicleSpeedY;
        float vehicleSpeedZ;
        float altitude;
    
        
    private:

    void determineStage() override;

    double timeOfLaunch;
    double timeOfLastStage;
    double timeOfDay;
};

#endif