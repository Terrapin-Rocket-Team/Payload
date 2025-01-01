#ifndef TADPOL_STATE_H
#define TADPOL_STATE_H

#include <Multi_Mission_Flight_Software.h>
#include <PWMServo.h>
#include "Target.h"

const int numTarg = 3;
const int numObs = 4;

// Obstacle* obstacles[] = {
// };

Point targetPoints[] = {
        Point(-76.10574722, 38.99640556)
};

class TADPOLState: public State{

static PWMServo leftServo;
static PWMServo rightServo;


public:
    bool topParachuteFlag;
    bool releasedFlag;

    double left_servo_value;
    double right_servo_value; 

    double goalCommand = 0;
    double rampRate;

    void determineTADPOLStage();
    void servoSetup(int leftServoPin,int rightServoPin,double leftSetNeutral,double rightSetNeutral);
    void moveServo(double delta);
    double findDelta(double phi, double gamma);
    void goDirection(double direction);
    Point getTargetCoordinates();

    void turnRateDriver(double goalAngle, double loopTime);

private:

};
void TADPOLState::servoSetup(int leftServoPin,int rightServoPin,double leftSerNeutral,double rightSerNeutral){ //input the servo pins, and the value for the servo to be up
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  leftServo.write(leftSerNeutral);
  rightServo.write(rightSerNeutral);
}

void TADPOLState::moveServo(double delta){
  //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
  //an explaination of how the values here were derivated
  double pi = 3.14;
  double leftservo_angle_offset_from_body = 45;
  double rightservo_angle_offset_from_body = -45;

  double left_servo_value = 90*(cos((leftservo_angle_offset_from_body-delta)*(pi/180)) + 1); //CHECK THIS FOR ANY NEW ANGLES -> a line is running the wrong way need to add a 180- term -> we cant "rotate" the heading, we can only flip line direction
  double right_servo_value = 180-90*(cos((rightservo_angle_offset_from_body-delta)*(pi/180)) + 1);



  leftServo.write(left_servo_value);
  rightServo.write(right_servo_value);
}

double TADPOLState::findDelta(double phi, double gamma){
  //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
  //an explaination of how the values here were derivated

  //Change the yaw in [-180,180] to [0,360]
  if(phi<0) phi += 360;

  //Find delta
  return phi - gamma;
}


void TADPOLState::goDirection(double goal){
  //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
  //the pseudocode
  goal += 180;
  if(goal>360){goal -= 360;}

  imu::Vector<3> ori = stateIMU.absoluteOrientationEuler; //function from BNO55.cpp
  double roll = ori.x();
  double pitch = ori.y();
  double yaw = ori.z(); //body frame from Inertial frame angle

  if (isnan(yaw)) {
    yaw = 200;
  }
  double delta = findDelta(yaw, goal);
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" degrees, Pitch: ");
  Serial.print(pitch);
  Serial.print(" degrees, Roll: ");
  Serial.print(roll);
  Serial.print(" degrees, Goal: ");
  Serial.print(goal);
  Serial.print(" degrees, delta: ");
  Serial.print(delta);
  Serial.println(" degrees");

  moveServo(delta);

}

void TADPOLState::determineTADPOLStage(){
  //Stages: "Pre Launch", "Powered Ascent", "Coasting", "Main", "Landed"
  //determineaccelerationMagnitude(acceleration);
  determinetimeSincePreviousStage();
  determineapogee(stateBarometer.relativeAltitude);
  //Serial.print("Z Accel: "); Serial.println(stateIMU.acceleration.z());
  //Serial.print("Time Since Prev Stage: "); Serial.println(timeSincePreviousStage);
  //Serial.print("apogeeTime: "); Serial.println(apogeeTime);
  if(stage == "Pre Launch" && stateIMU.acceleration.z() > 30){
      buzz(9, 100);
      delay(100);
      buzz(9, 100);
      timeLaunch = millis()/1000;
      timePreviousStage = millis()/1000;
      stage = "Powered Ascent";
      recordDataStage = "Flight";
  }
  else if(stage == "Powered Ascent" && stateIMU.acceleration.z() < 0){
      timePreviousStage = millis()/1000;
      stage = "Coasting";
  }
  else if(stage == "Coasting" && timeAbsolute > (apogeeTime+10)){
      timePreviousStage = millis()/1000;
      stage = "Main";
  }
  else if(stage == "Main" && stateBarometer.relativeAltitude < 30){
      timePreviousStage = millis()/1000;
      stage = "Landed";
      recordDataStage = "PostFlight";
  }
  else if((stage == "Pre Launch" || stage == "Powered Ascent") && stateBarometer.relativeAltitude > 100){
    buzz(9, 100);
    delay(100);
    buzz(9, 100);
    timePreviousStage = millis()/1000;
    stage = "Coasting";
    recordDataStage = "Flight";
  }
}

Point TADPOLState::getTargetCoordinates(){
  double x = stateGPS.longitude;
  double y = stateGPS.latitude;
  Point current(x, y);

  current = Point(-76.105696, 38.995326);

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
  // for (int i = 0; i < numTarg; i++) {
  //     // checks if a target point from the valid list interacts with an obstacle
  //     for (Obstacle* obs : obstacles) {
  //         // if the target point intersects an obstacle, remove it from both lists
  //         if (obs -> intersect(current, targetPoints[i])) {
  //             valids[i] = Point();
  //             safes[i] = Point();
  //             break;
  //         }

  //         // if the target point is within error of an obstacle, remove it from the "safe" list
  //         if (inError(current, valids[i], *obs)) {
  //             safes[i] = Point();
  //         }
  //     }
  // }

  // determines the best point to go to
  Point closestSafePoint = closest(current, safes, numTarg);
  if (closestSafePoint != Point(0.0, 0.0))
      return closestSafePoint;
  
  Point closestValidPoint = closest(current, valids, numTarg);
  if (closestValidPoint != Point(0.0, 0.0))
      return closestValidPoint;

  Point closestPoint = closest(current, targetPoints, numTarg);
  return closestPoint;
}

void TADPOLState::turnRateDriver(double goalAngle, double loopTime) {
  double delta=goalAngle-goalCommand;
  if (abs(delta) > 180) {
      if (goalAngle < 180 ) {
          goalCommand -= 360;
      }
      else {
          goalCommand += 360;
      }
      delta=goalAngle-goalCommand;
  }
  if (abs(delta) < 1.2*rampRate*loopTime) {
      goalCommand=goalAngle;
  }
  else {
      goalCommand += rampRate*loopTime*abs(delta)/delta;
  }
  if (goalCommand>360){
      goalCommand -= 360;
  }
  else if (goalCommand<0) {
      goalCommand += 360;
  }
}

#endif
