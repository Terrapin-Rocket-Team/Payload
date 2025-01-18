#ifndef TADPOL_STATE_H
#define TADPOL_STATE_H

#include <Multi_Mission_Flight_Software.h>
#include <PWMServo.h>
#include "Target.h"

const int numTarg = 3;
const int numObs = 4;

int BUZZER_PIN = 9;

double PREVIOUS_PROPORTIONAL_ERROR = 0;

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

    double yawAvg;
    double previousPwm;

    double Kp = 30; //Proportional error constant -- MANUAL INPUT
    double Kd = 30; //Derivative error constant -- MANUAL INPUT
    int pwmMax=190;
    float pwmScale=1.3;
    int pwmChangeSignDelay = 5; //in ms

    int buzzerPin;

    void determineTADPOLStage();
    void fanSetup(int fowardFanPin,int backwardFanPin);
    void runFan(double pwm, int forwardFanPin, int backwardFanPin);
    double findPWM(double direction, double timeSinceLastIteration);
    Point getTargetCoordinates();

private:

};
void TADPOLState::fanSetup(int fowardFanPin,int backwardFanPin){
  pinMode(fowardFanPin, OUTPUT);
  pinMode(backwardFanPin, OUTPUT);
  //analogWriteFrequency(fowardFanPin, 20);
  //analogWriteFrequency(backwardFanPin, 20);
}

void TADPOLState::runFan(double pwm, int forwardFanPin, int backwardFanPin){
  if (pwm > 0){
    Serial.println("Pwm()" + String(pwm) +" > 0: Teensy Pin " + String(forwardFanPin)+ " = " + String("HIGH"));
    analogWrite(forwardFanPin, int(pwm));
    digitalWrite(backwardFanPin, LOW);
  }
  else if (pwm < 0){
    Serial.println("Pwm()" + String(pwm) +" < 0: Teensy Pin " + String(forwardFanPin) + " = " + String("LOW"));
    digitalWrite(forwardFanPin, LOW);
    analogWrite(backwardFanPin, int(pwm));
  }
  else{
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
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

  if(!isnan(yaw)){
    if (yaw*yawAvg>0){yawAvg=yawAvg*0.8+yaw*0.2;}
    else if(yaw>90){yawAvg=(360+yawAvg)*0.8+yaw*0.2;}
    else if(yaw<-90){yawAvg=(yawAvg-360)*0.8+yaw*0.2;}
    else{yawAvg=yawAvg*0.8+yaw*0.2;}
  }
  
  

  //Find proportional error
  double Ep = goal - yawAvg;
  if(Ep >= 180){Ep = 360 - Ep;}
  else if(Ep <= -180){Ep = 360 + Ep;}

  //Find derivative error
  double Ed = Ep - PREVIOUS_PROPORTIONAL_ERROR;
  if(Ep*PREVIOUS_PROPORTIONAL_ERROR<0){
    if(Ep>0){Ed-=360;}
    else{Ed+=360;}
  }
  Ed=Ed/timeSinceLastIteration;
  PREVIOUS_PROPORTIONAL_ERROR = Ep;

  //Find PWM
  double pwm = Kp*Ep + Kd*Ed;
  if(pwm<0){pwm=pwm*pwmScale;}
  if(pwm>pwmMax){pwm=pwmMax;}
  else if(pwm<-pwmScale*pwmMax){pwm=-pwmScale*pwmMax;}

  if(Ep>-5 && Ep< 5){pwm=0;}

  Serial.print("PWM: "); Serial.println(pwm);
  Serial.print("Ep: "); Serial.println(Ep);
  Serial.print("Ed: "); Serial.println(Ed);
  Serial.print("Yaw: "); Serial.println(yaw);

  return pwm;
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
      buzz(buzzerPin, 100, 2);
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

#endif
