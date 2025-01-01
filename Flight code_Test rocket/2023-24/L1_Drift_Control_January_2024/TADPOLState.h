#ifndef TADPOL_STATE_H
#define TADPOL_STATE_H

#include <Multi_Mission_Flight_Software.h>
#include <PWMServo.h>
#include "Target.h"

const int numTarg = 3;
const int numObs = 4;

Line line1 = Line(Point(-75.87740556, 39.08141944), Point(-75.87596, 39.07918));
Line line2 = Line(Point(-75.87439444, 39.079425), Point(-75.87331111, 39.08091111));
Line line3 = Line(Point(-75.87320833, 39.07993889), Point(-75.871675, 39.07527778));
Line line4 = Line(Point(-75.87404444, 39.08616667), Point(-75.87263056, 39.08143889));

Obstacle* obstacles[] = {
    &line1,
    &line2,
    &line3,
    &line4
};

Point targetPoints[] = {
        Point(-75.87514167, 39.08471667),
        Point(-75.87820833, 39.077525),
        Point(-75.87034444, 39.08389722)
};

class TADPOLState: public State{

static PWMServo leftServo;
static PWMServo rightServo;


public:
    bool topParachuteFlag;
    bool releasedFlag;

    double left_servo_value;
    double right_servo_value; 

    void determineTADPOLStage();
    void servoSetup(int leftServoPin,int rightServoPin,double leftSetNeutral,double rightSetNeutral);
    void moveServo(double delta);
    double findDelta(double phi, double gamma);
    void goDirection(double direction);
    Point getTargetCoordinates();

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
  double leftservo_angle_offset_from_body = 90;
  double rightservo_angle_offset_from_body = 0;

  double left_servo_value = 90*(cos((leftservo_angle_offset_from_body-delta)*(pi/180)) + 1);
  double right_servo_value = 90*(cos((rightservo_angle_offset_from_body-delta)*(pi/180)) + 1);

  //Serial.print("Left Servo Value: "); Serial.print(left_servo_value); Serial.print(", Right Servo Value: "); Serial.println(right_servo_value);
  if (left_servo_value <= 90){left_servo_value = 0;}
  else{left_servo_value = 180;}
  if (right_servo_value <= 90){right_servo_value = 0;}
  else{right_servo_value = 180;}

  leftServo.write(left_servo_value);
  rightServo.write(right_servo_value);
}

double TADPOLState::findDelta(double psi, double gamma){
  //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
  //an explaination of how the values here were derivated

  //Change the yaw in [-180,180] to [0,360]
  if(psi<0) psi += 360;

  //Find delta
  return psi - gamma;
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
  else if(stage == "Coasting" && timeAbsolute > (apogeeTime+5)){
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
  if (closestSafePoint != Point(0.0, 0.0))
      return closestSafePoint;
  
  Point closestValidPoint = closest(current, valids, numTarg);
  if (closestValidPoint != Point(0.0, 0.0))
      return closestValidPoint;

  Point closestPoint = closest(current, targetPoints, numTarg);
  return closestPoint;
}

#endif
