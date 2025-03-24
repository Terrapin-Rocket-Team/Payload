#include "VehicleState.h"
#include "MMFS.h"

using namespace mmfs;

/*
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
        */

VehicleState::VehicleState(Sensor **sensors, int numSensors, Filter *filter) : State(sensors, numSensors, filter) {
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
    //buzzer_pin = buzz_pin;

    addColumn(DOUBLE, &leftServoVal, "left servo value");
    addColumn(DOUBLE, &rightServoVal, "right servo value");
    addColumn(DOUBLE, &gx, "gx");
    addColumn(DOUBLE, &gy, "gy");
    addColumn(DOUBLE, &wx, "wx");
    addColumn(DOUBLE, &wy, "wy");
    addColumn(DOUBLE, &vehicleSpeed, "Vehicle Speed");
    addColumn(DOUBLE, &targetCoords_X, "Vehicle Speed");
    addColumn(DOUBLE, &targetCoords_Y, "Vehicle Speed");

}

void VehicleState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    determineStage(); // determine the stage of the flight
}

void VehicleState::determineStage()
{
  int timeSinceLaunch = currentTime - timeOfLaunch;
    IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));
    Barometer *baro = reinterpret_cast<Barometer *>(getSensor(BAROMETER_));
    //Serial.println(imu->getAccelerationGlobal().z());
    if(stage == PRELAUNCH && imu->getAccelerationGlobal().z() > 10){
        getLogger().setRecordMode(FLIGHT);;
        bb.aonoff(buzzer_pin, 200);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        getLogger().recordLogData(INFO_, "Launch detected.");
        getLogger().recordLogData(INFO_, "Printing static data.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
    else if(stage == BOOST && imu->getAccelerationGlobal().z() < 0){
        bb.aonoff(buzzer_pin, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        getLogger().recordLogData(INFO_, "Coasting detected.");
    }
    else if(stage == COAST && baroVelocity <= 0 && timeSinceLaunch > 3){ // TODO fix this, this baroVelocity is not resistant to noise
        bb.aonoff(buzzer_pin, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        getLogger().recordLogData(INFO_, logData);
        stage = ACTUATION;
        getLogger().recordLogData(INFO_, "Ejection detected.");
    }
    else if(stage == ACTUATION && baroVelocity <= 0 && timeSinceLaunch > 3){ // TODO fix this, this baroVelocity is not resistant to noise
        bb.aonoff(buzzer_pin, 200, 2);
        timeOfLastStage = currentTime;
        char logData[100];
        getLogger().recordLogData(INFO_, logData);
        stage = MAIN;
        getLogger().recordLogData(INFO_, "Flight detected.");
    }
    else if(stage == MAIN && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 600000))){
        bb.aonoff(buzzer_pin, 200, 2);
        timeOfLastStage = currentTime;
        stage = LANDED;
        getLogger().setRecordMode(GROUND);
        getLogger().recordLogData(INFO_, "Landing detected.");
    }
    else if (stage == LANDED && currentTime - timeOfLastStage > 60) // TODO check if it can dump data in 60 seconds
    {
        stage = DUMPED;
        getLogger().recordLogData(INFO_, "Dumped data after landing.");
    }
    else if((stage == PRELAUNCH || stage == BOOST) && baro->getAGLAltFt() > 250){
        getLogger().setRecordMode(FLIGHT);
        bb.aonoff(buzzer_pin, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        getLogger().recordLogData(INFO_, "Launch detected. Using Backup Condition.");
        getLogger().recordLogData(INFO_, "Printing static data.");
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
    left.attach(leftServoPin);
    right.attach(rightServoPin);
    pitch.attach(camServoPin);
    left.write(leftSetNeutral);
    right.write(rightSetNeutral);
    pitch.write(camSetNeutral);
}

void VehicleState::moveServo(double delta){

    double left_servo_value;
    double right_servo_value;


    if(delta < 0) {
        left_servo_value = 180; //this should be whatever angle is straight up
        right_servo_value = -1*delta;
    }

    if(delta > 0) {
        right_servo_value = 0; //this should be whatever angle is straight up
        left_servo_value = 180-delta;
    }
    //Serial.print("Left Servo Value: "); Serial.print(left_servo_value); Serial.print(", Right Servo Value: "); Serial.println(right_servo_value);
    
    //constrain to range
    if (left_servo_value <= 90){left_servo_value = 90;}
    if (right_servo_value >= 90){right_servo_value = 90;}

    left.write(left_servo_value);
    right.write(right_servo_value);
}

void VehicleState::moveCam(){

    double cam_servo_value = 0;

    //implement cam control code

    pitch.write(cam_servo_value);
}


void VehicleState::goDirection(double goal){
    //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
    //the pseudocode
    goal += 180;
    if(goal>360){goal -= 360;}

    IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));
    mmfs::Vector<3> ori = imu->getOrientation().toEuler(); //function from BNO55.cpp
    double yaw = ori.z(); //body frame from Inertial frame angle
    double delta = findDelta(yaw, goal);
     moveServo(delta);

}