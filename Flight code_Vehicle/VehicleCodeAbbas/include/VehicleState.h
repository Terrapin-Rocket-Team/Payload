#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <State/State.h>
#include "target.h"


enum VehcileStages { // TODO update this
    PRELAUNCH,
    BOOST, //Rocket accelerating upwards
    COAST, //Rocket coasting
    ACTUATION, //Vehicle Ejected
    MAIN, //Vehicle Gliding
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
        float leftServoVal;
        float rightServoVal;
        float gx;
        float gy;
        float wx;
        float wy;
        float vehicleSpeed;
        float averageWindCorrectionCoords_X;
        float averageWindCorrectionCoords_Y;
        float targetCoords_X;
        float targetCoords_Y;
    };

    Servo pitch;
    Servo left;
    Servo right;

    public:
        VehicleState(Sensor **sensors, int numSensors, Filter *filter);
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

        Point targetCoords;

        Point getTargetCoordinates();
        Point getWindCorrectionCoordinates(Point r);

        void servoSetup(int leftServoPin,int rightServoPin,int camServoPin,double leftSetNeutral,double rightSetNeutral,double camSetNeutral);
        void moveServo(double delta);
        void moveCam();
        double findDelta(double phi, double gamma);
        void goDirection(double direction);

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