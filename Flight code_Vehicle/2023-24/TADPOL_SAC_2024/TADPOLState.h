#ifndef TADPOL_STATE_H
#define TADPOL_STATE_H

#include <Multi_Mission_Flight_Software.h>
#include "Target.h"

// SOD Farm
const int numTarg = 2;
const int numObs = 1;

Line line1 = Line(Point(-106.922582, 32.939719), Point(-106.909264, 32.943206));

Obstacle* obstacles[] = {
    &line1
};

Point targetPoints[] = {
        Point(-106.914875, 32.937792),
        Point(-106.920284, 32.943033)
};

class TADPOLState: public State{

public:
    bool topParachuteFlag;
    bool releasedFlag;

    double Kp = .17; //Proportional error constant -- MANUAL INPUT TODO
    double Kd = .33; //Derivative error constant -- MANUAL INPUT TODO
    int pwmMax=150;
    float pwmScale=1.3; // TOOD
    int pwmChangeSignDelay = 5; //in ms
    float pwmZeroAngleCone = 5; //in degrees, the +/- angle which creates a cone around Ep setting it to 0 if within the range
    double pwmTimer = 0.0; //in ms
    double pwmFrequency = 250; //in ms TODO set this

    imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
    imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
    double v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
    Point averageWindCorrectionCoords;


    int buzzerPin;

    void determineTADPOLStage();
    void fanSetup(int fowardFanPin,int backwardFanPin);
    void runFan(double pwm, int forwardFanPin, int backwardFanPin);
    double findPWM(double direction, double timeSinceLastIteration);
    Point getTargetCoordinates();
    Point getWindCorrectionCoordinates(Point r);
    imu::Vector<3> getInertialAngularVelocity();

private:

};

void TADPOLState::fanSetup(int fowardFanPin,int backwardFanPin){
  pinMode(fowardFanPin, OUTPUT);
  pinMode(backwardFanPin, OUTPUT);
}

void TADPOLState::runFan(double pwm, int forwardFanPin, int backwardFanPin){

  // if (pwm > pwmMax){
  //   pwm = pwmMax;
  // }
  // else if (pwm < -pwmMax){
  //   pwm = -pwmMax;
  // }

  // Pick the direction that the fan spins
  if (pwm == 0){
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
    return;
  }

  // If the duty cycle is over, reset the timer
  if (millis() - pwmTimer > pwmFrequency){
    pwmTimer = millis();
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
    if (pwm > 0){
      digitalWrite(forwardFanPin, HIGH);
    }
    else if (pwm < 0){
      digitalWrite(backwardFanPin, HIGH);
    }
  }

  // If the timer is past the pwm cycle time turn the pulse to low
  if (millis() < pwmTimer+((abs(pwm)*pwmFrequency)/255)){
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
    if (pwm > 0){
      Serial.println("Going Forward");
      digitalWrite(forwardFanPin, HIGH);
    }
    else if (pwm < 0){
      Serial.println("Going Backward");
      digitalWrite(backwardFanPin, HIGH);
    }
  }
  else{
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
  }
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

Point TADPOLState::getWindCorrectionCoordinates(Point r){
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