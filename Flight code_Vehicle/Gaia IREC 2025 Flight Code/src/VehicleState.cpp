#include "VehicleState.h"
#include "MMFS.h"

using namespace mmfs;


VehicleState::VehicleState(Sensor **sensors, int numSensors, Filter *filter) : State(sensors, numSensors, filter) {
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
    
    addColumn(DOUBLE, &servoOutput, "combined servo output");
    addColumn(DOUBLE, &vehicleX, "GPS X");
    addColumn(DOUBLE, &vehicleY, "GPS Y");
    addColumn(DOUBLE, &currentAngle, "current angle");
    addColumn(DOUBLE, &targetAngle, "target angle");
    addColumn(DOUBLE, &currentAngleY, "current angle pitch");
    addColumn(DOUBLE, &currentAngleZ, "current angle roll");
}

void VehicleState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    // determineStage(); // determine the stage of the flight
}

void VehicleState::determineStage(){
    mmfs::Barometer *baro = reinterpret_cast<mmfs::Barometer *>(getSensor("Barometer"_i));
    mmfs::GPS *gps = reinterpret_cast<mmfs::GPS *>(getSensor("GPS"_i));
    
    if(stage == PRELAUNCH && acceleration.magnitude() > 40 && baro->getAGLAltM() > 100) {
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
    else if(stage == BOOST && (acceleration.z() < 0 || (currentTime - timeOfLastStage) > 6)){
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Coasting detected.");
    }
    else if (stage == COAST && ((baro->getAGLAltFt() > 10000 && (currentTime - timeOfLastStage) > 5) || (currentTime - timeOfLastStage) > 45)) {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        mmfs::getLogger().recordLogData(mmfs::INFO_, logData);
        stage = DROGUE;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Drogue detected.");
    }
    else if(stage == DROGUE &&  baro->getAGLAltFt() < 1500 && (currentTime - timeOfLastStage) > 5){ // logic for detecting vehicle ejection
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        snprintf(logData, 100, "Ejection detected at %.2f m.", position.z());
        mmfs::getLogger().recordLogData(mmfs::INFO_, logData);
        stage = EJECTED;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Ejection Detected - Actuating.");

        rocketx = gps->getPos()[0]; // set point of ejection as origin to orbit around
        rockety = gps->getPos()[1];
    }
    else if(stage == EJECTED && currentTime - timeOfLastStage > 2){ 
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = GLIDING;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "GLIDING detected.");
    }
    else if(stage == GLIDING && (((currentTime - timeOfLastStage) > 60) && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 120)))){
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = LANDED;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Landing detected.");
        mmfs::getLogger().setRecordMode(mmfs::GROUND);
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Dumped data after landing.");
    }
    else if (stage == LANDED && (currentTime - timeOfLastStage) > 30)
    {
        bb.aonoff(mmfs::BUZZER, 200, 2);
        stage = PRELAUNCH;
    }
    else if((stage == PRELAUNCH || stage == BOOST) && (baro->getAGLAltM() > 1500) && (millis() > 60000)){
        mmfs::getLogger().setRecordMode(mmfs::FLIGHT);
        bb.aonoff(mmfs::BUZZER, 200, 2);
        timeOfLastStage = currentTime;
        stage = DROGUE;
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

/*
Point VehicleState::getTargetCoordinates(){

    GPS *gps = reinterpret_cast<GPS *>(getSensor(GPS_));
    double x = gps->getPos().y(); // longitude 
    double y = gps->getPos().x(); // latitude
    Point current(x, y);
  
    // copies targets into a valids array and a safes array
    Point valids[numTarg];
    for(int i = 0; i < numTarg; i++){
        valids[i] = targetPoints[i];
    }
  
    Point safes[numTarg];
    for(int i = 0; i < numTarg; i++){
        safes[i] = targetPoints[i];
    }
  
    // loops through all targets
    
    for (int i = 0; i < numTarg; i++) {
        // checks if a target point from the valid list interacts with an obstacle
        for (Obstacle* obs : obstacles) {
            // if the target point intersects an obstacle, remove it from both lists
            if (obs -> intersect(current, targetPoints[i])) {
                valids[i] = Point();
                safes[i] = Point();
                break;
            }
  
            // if the target point is within error of an obstacle, remove it from the "safe" list
            if (inError(current, valids[i], *obs)) {
                safes[i] = Point();
            }
        }
    }

    // determines the best point to go to
    Point closestSafePoint = closest(current, safes, numTarg);
    if (closestSafePoint != Point(0.0, 0.0)){
        targetCoords = closestSafePoint;
        return closestSafePoint;
    }
    
    Point closestValidPoint = closest(current, valids, numTarg);
    if (closestValidPoint != Point(0.0, 0.0)){
        targetCoords = closestValidPoint;
        return closestValidPoint;
    } 

    Point closestPoint = closest(current, targetPoints, numTarg);
    targetCoords = closestPoint;
    return closestPoint;
}
    */

Point VehicleState::getWindCorrectionCoordinates(Point r){
    // Design and logic in this doc (https://docs.google.com/document/d/1soUME8JDSpf028hsgl010TmuEHOHm2ZJCv7ecYDvrWE/edit)
    // Input r is the desired heading point w/o wind

    // update wind speed
    imu::Vector<2> v(velocity.x(), velocity.y());
    w = v-g;

    //double norm_r = sqrt(r.x*r.x + r.y*r.y);
    imu::Vector<2> h(r.x, r.y); h.normalize(); // unit vector in the direction of the actual velocity
    imu::Vector<2> w_h = h.scale(w.dot(h)); // wind vector in direction of desired heading
    imu::Vector<2> w_c = w-w_h; // cross wind vector
    imu::Vector<2> h_prime = sqrt((v_s*v_s) + (w_c.magnitude()*w_c.magnitude())); // resultant velocity vector of this wind correction
    imu::Vector<2> g = h_prime-w_c; // heading vector to go in to account for velocity
    Point g_point = Point(g.x(), g.y()); //turn g from a 2D imu vector object to a point this doesn't work, velo not pos

    // 90/10 Weighted Average split
    averageWindCorrectionCoords.x = .9*averageWindCorrectionCoords.x + .1*g_point.x;
    averageWindCorrectionCoords.y = .9*averageWindCorrectionCoords.y + .1*g_point.y;

    return averageWindCorrectionCoords;
}

// Servo Functions

void VehicleState::servoSetup(int leftServoPin,int rightServoPin,int camServoPin, double leftSetNeutral,double rightSetNeutral,double camSetNeutral){ //input the servo pins, and the value for the servo to be up
    //left.attach(leftServoPin);
    // right.attach(rightServoPin);
    // pitch.attach(camServoPin);
    // left.write(leftSetNeutral);
    // right.write(rightSetNeutral);
    // pitch.write(camSetNeutral);
}

void VehicleState::moveCam(){

    double cam_servo_value = 0;

    //implement cam control code

    //pitch.write(cam_servo_value);
}

double VehicleState::goalOrbit(double rocketX, double rocketY, double X, double Y, double R){
    // FIX THE RADIANS STUFF TO BE CONSISTENT CW vs CCW, WHERE 0 DEGREES IS DEFINED ETC
    double ke= 1; // determine
    double targetRadius=R; // meters? determine value as well-->we could consider dynamically changing target radius based on height difference between rocket and payload vehicle
    double radius=sqrt((rocketY-Y)*(rocketY-Y)+(rocketX-X)*(rocketX-X));
    double goal = 180/3.14*atan2((rocketY-Y),(rocketX-X))-90; // Goal is 90 degrees CW from vector between
    double adjusted_goal = goal - constrain(ke*(targetRadius - radius), -40, 40); // Adjust goal based on distance from target
    return adjusted_goal;
}


void VehicleState::goDirection(double goal){

    //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
    //the pseudocode
    goal += 180;
    if(goal>360){goal -= 360;}

    IMU *imu = reinterpret_cast<IMU *>(getSensor("IMU"_i));
    mmfs::Vector<3> ori = imu->getOrientation().toEuler321(); //function from BNO55.cpp
    double yaw = ori.z(); //body frame from Inertial frame angle
    double delta = findDelta(yaw, goal);
     moveServo(delta);

}