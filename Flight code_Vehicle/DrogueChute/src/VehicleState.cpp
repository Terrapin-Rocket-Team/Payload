#include "VehicleState.h"
#include "MMFS.h"

using namespace mmfs;

VehicleState::~VehicleState() {
    // No special cleanup needed? Leave empty.
}

VehicleState::VehicleState(Sensor **sensors, int numSensors, Filter *filter) : State(sensors, numSensors, filter) {
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;

    addColumn(DOUBLE, &servo_value, "servo value");
    addColumn(DOUBLE, &servoOutput, "combined servo output");
    addColumn(DOUBLE, &vehicleX, "GPS X");
    addColumn(DOUBLE, &vehicleY, "GPS Y");
    addColumn(DOUBLE, &currentAngle, "current angle");
    addColumn(DOUBLE, &targetAngle, "target angle");
    addColumn(DOUBLE, &currentAngleY, "current angle pitch");
    addColumn(DOUBLE, &currentAngleZ, "current angle roll");
    addColumn(DOUBLE, &vehicleSpeedX, "Vehicle Speed X");
    addColumn(DOUBLE, &vehicleSpeedY, "Vehicle Speed Y");
    addColumn(DOUBLE, &vehicleSpeedZ, "Vehicle Speed Z");
    addColumn(DOUBLE, &altitude, "Altitude");

}

void VehicleState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    determineStage(); // determine the stage of the flight
}

void VehicleState::determineStage(){
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor(mmfs::BAROMETER_));
    
    if(stage == PRELAUNCH && acceleration.z() > 40){
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Launch detected.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
    else if(stage == BOOST && acceleration.z() < 0){
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Coasting detected.");
    }
    else if(stage == COAST &&  baro->getAGLAltFt() < 1500){ // logic for detecting vehicle ejection
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Ejection detected at %.2f m.", position.z());
        mmfs::getLogger().recordLogData(mmfs::INFO_, logData);
        stage = ACTUATION;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Ejection Detected - Actuating.");
    }
    else if(stage == ACTUATION && baro->getAGLAltFt() < 2000 && currentTime - timeOfLastStage > 2){ 
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = MAIN;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Main detected.");
    }
    else if(stage == MAIN && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 60))){
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = LANDED;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Landing detected.");
        mmfs::getLogger().setRecordMode(mmfs::GROUND);
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Dumped data after landing.");
    }
    else if (stage == LANDED && (currentTime - timeOfLastStage) > 60)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        stage = PRELAUNCH;
    }
    else if((stage == PRELAUNCH || stage == BOOST) && (baro->getAGLAltM() > 1500) && (millis() > 60000)){
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Launch detected. Using Backup Condition.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
}

// Servo Functions

void VehicleState::servoSetup(int ServoPin,int camServoPin,double SetNeutral,double camSetNeutral){ //input the servo pins, and the value for the servo to be up
    servo.attach(ServoPin);
    pitch.attach(camServoPin);
    servo.write(SetNeutral);
    pitch.write(camSetNeutral);
}

void VehicleState::moveCam(){

    double cam_servo_value = 0;

    //implement cam control code

    pitch.write(cam_servo_value);
}

double VehicleState::goalOrbit(double rocketX, double rocketY, double X, double Y, double R){
    // FIX THE RADIANS STUFF TO BE CONSISTENT CW vs CCW, WHERE 0 DEGREES IS DEFINED ETC
    GPS *gps = reinterpret_cast<GPS *>(getSensor(GPS_));
    double ke= 1; // determine
    double targetRadius=R; // meters? determine value as well-->we could consider dynamically changing target radius based on height difference between rocket and payload vehicle
    double radius=sqrt((rocketY-Y)*(rocketY-Y)+(rocketX-X)*(rocketX-X));
    double goal = 180/3.14*atan2((rocketY-Y),(rocketX-X))-90; // Goal is 90 degrees CW from vector between
    double adjusted_goal = goal - constrain(ke*(targetRadius - radius), -40, 40); // Adjust goal based on distance from target
    return adjusted_goal;
}
