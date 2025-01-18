#ifndef TADPOL_STATE_H
#define TADPOL_STATE_H

#include <Multi_Mission_Flight_Software.h>
#include "Target.h"

// SOD Farm
const int numTarg = 4;
const int numObs = 1;

Line line1 = Line(Point(-76.10572, 39.00226), Point(-76.10513, 38.99972));

Obstacle* obstacles[] = {
    &line1
};

Point targetPoints[] = {
        Point(-76.10574722, 38.99640556),
        Point(-76.10963889, 38.99779167),
        Point(-76.10016389, 39.00023333),
        Point(-76.10376111, 39.00292778)
};

class TADPOLState: public State{

public:
    bool topParachuteFlag;
    bool releasedFlag;

    double previousPwm;

    double Kp = .17; //Proportional error constant -- MANUAL INPUT
    double Kd = .33; //Derivative error constant -- MANUAL INPUT
    int pwmMax=190;
    float pwmScale=1.3;
    int pwmChangeSignDelay = 5; //in ms
    float pwmZeroAngleCone = 5; //in degrees, the +/- angle which creates a cone around Ep setting it to 0 if within the range

    int buzzerPin;

    void determineTADPOLStage();
    void fanSetup(int fowardFanPin,int backwardFanPin, int enableFanPin);
    void runFan(double pwm, int forwardFanPin, int backwardFanPin, int enableFanPin);
    double findPWM(double direction, double timeSinceLastIteration);
    Point getTargetCoordinates();
    imu::Vector<3> getInertialAngularVelocity();

private:

};

void TADPOLState::fanSetup(int fowardFanPin,int backwardFanPin, int enableFanPin){
  pinMode(fowardFanPin, OUTPUT);
  pinMode(backwardFanPin, OUTPUT);
  pinMode(enableFanPin, OUTPUT);
}

void TADPOLState::runFan(double pwm, int forwardFanPin, int backwardFanPin, int enableFanPin){
  if (pwm > 0){
    digitalWrite(forwardFanPin, HIGH);
    digitalWrite(backwardFanPin, LOW);
    analogWrite(enableFanPin, int(pwm));
  }
  else if (pwm < 0){
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, HIGH);
    analogWrite(enableFanPin, -int(pwm));
  }
  else{
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
    analogWrite(enableFanPin, 0);
  }
  previousPwm = pwm;
}


double TADPOLState::findPWM(double goal, double timeSinceLastIteration){
  //Input goal is angle to position [-180:180] off the y-axis (CCW +)
  //Input timeSinceLastIteration should be in seconds

  imu::Vector<3> ori = stateIMU.absoluteOrientationEuler;
  double roll = ori.x();
  double pitch = ori.y();
  double yaw = ori.z(); //body frame from Inertial frame angle (psi)

  //Find proportional error
  double Ep = goal - yaw;
  if(Ep >= 180){Ep = 360 - Ep;}
  else if(Ep <= -180){Ep = 360 + Ep;}

  // Ed Angular Velocity Method
  imu::Vector<3> ineritalAngularVelocity = getInertialAngularVelocity();
  double Ed = -ineritalAngularVelocity.z();

  //Find PWM
  double pwm = Kp*Ep + Kd*Ed;
  if(pwm<0){pwm=pwm*pwmScale;}
  if(pwm>pwmMax){pwm=pwmMax;}
  else if(pwm<-pwmScale*pwmMax){pwm=-pwmScale*pwmMax;}

  if(Ep>-pwmZeroAngleCone && Ep<pwmZeroAngleCone){pwm=0;}

  if (isnan(pwm)){
    pwm = 0;
  }

  return pwm;
}

void TADPOLState::determineTADPOLStage(){
    //Stages: "Pre Launch", "Powered Ascent", "Coasting", "Drogue", "Main", "Landed"
    //determineaccelerationMagnitude(acceleration);
    determinetimeSincePreviousStage();
    determineapogee(stateBarometer.relativeAltitude);
    if(stage == "Pre Launch" && stateIMU.acceleration.z() > 30){
        buzz(buzzerPin, 100, 2);
        timeLaunch = millis();
        timePreviousStage = millis();
        stage = "Powered Ascent";
        stageNumber = 1;
        recordDataStage = "Flight";

        //turn on camera
        pinMode(22, OUTPUT);
        digitalWrite(22, HIGH);
        delay(5);
        digitalWrite(22, LOW);
    }
    else if(stage == "Powered Ascent" && stateIMU.acceleration.z() < 0){
        timePreviousStage = millis();
        stage = "Coasting";
        stageNumber = 2;
    }
    else if(stage == "Coasting" && timeAbsolute > (apogeeTime+5)){
        timePreviousStage = millis();
        stage = "Drogue";
        stageNumber = 3;
    }
    else if(stage == "Drogue" && stateBarometer.relativeAltitude < 450){
        timePreviousStage = millis();
        stage = "Main";
        stageNumber = 4;
    }
    else if(stage == "Main" && stateBarometer.relativeAltitude < 20){
        timePreviousStage = millis();
        stage = "Landed";
        recordDataStage = "PostFlight";
        stageNumber = 5;
    }
    else if((stage == "Pre Launch" || stage == "Powered Ascent") && stateBarometer.relativeAltitude > 100){
        buzz(buzzerPin, 100, 2);
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

imu::Vector<3> TADPOLState::getInertialAngularVelocity(){
  imu::Quaternion orientation = stateIMU.absoluteOrientation;
  orientation.normalize();
  imu::Quaternion orientationConj = orientation.conjugate();

  imu::Vector<3> rocketFrameAngularVelocity = stateIMU.angularVelocity;
  imu::Quaternion rocketFrameAngularVelocityQuat;
  rocketFrameAngularVelocityQuat.w() = 0;
  rocketFrameAngularVelocityQuat.x() = rocketFrameAngularVelocity.x();
  rocketFrameAngularVelocityQuat.y() = rocketFrameAngularVelocity.y();
  rocketFrameAngularVelocityQuat.z() = rocketFrameAngularVelocity.z();

  imu::Quaternion absoluteAngularVelocityQuat = quatMultiplication(quatMultiplication(orientation, rocketFrameAngularVelocityQuat), orientationConj);
  imu::Vector<3> absoluteAngularVelocity;
  absoluteAngularVelocity.x() = absoluteAngularVelocityQuat.x();
  absoluteAngularVelocity.y() = absoluteAngularVelocityQuat.y();
  absoluteAngularVelocity.z() = absoluteAngularVelocityQuat.z();

  return absoluteAngularVelocity;
}

#endif