#ifndef VEHICLESTATE_H
#define VEHICLESTATE_H

#include <State/State.h>
#include "PWMServo.h"

enum VehicleStages { // TODO update this
    PRELAUNCH,
    BOOST, //Rocket accelerating upwards
    COAST, //Rocket coasting
    DROGUE, //Rocket under Drogue
    ACTUATION, //Vehicle Ejected
    MAIN, //Vehicle Gliding
    LANDED, //Vehicle Landed
    DUMPED
};

// SOD Farm
const int numTarg = 1;
const int numObs = 0;



using namespace mmfs;

class VehicleState : public State {


    public:
        VehicleState(Sensor **sensors, int numSensors, Filter *filter);
        virtual ~VehicleState();

        void updateState(double newTime = -1) override;
        
        int stage;
        int buzzer_pin;

        bool topParachuteFlag;
        bool releasedFlag;

        double servo_value;

      

      

        void servoSetup(int ServoPin,int camServoPin,double SetNeutral,double camSetNeutral);
        void moveCam();
        double goalOrbit(double rocketX, double rocketY, double X, double Y, double R);

        PWMServo pitch;
        PWMServo servo;

        float ServoValue;
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