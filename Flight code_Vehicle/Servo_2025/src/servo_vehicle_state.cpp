#include "servo_vehicle_state.h"

using namespace mmfs;

const Line line1 = Line(Point(-106.922582, 32.939719), Point(-106.909264, 32.943206));

Obstacle* obstacles[] = {
    &line1
};

Point targetPoints[] = {
        Point(-106.914875, 32.937792),
        Point(-106.920284, 32.943033)
};

ServoVehicleState::ServoVehicleState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter) : State(sensors, numSensors, kfilter)
{
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
}

void ServoVehicleState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    determineStage(); // determine the stage of the flight
}

void ServoVehicleState::determineStage()
{
  // TODO
}

void ServoVehicleState::fanSetup(int fowardFanPin,int backwardFanPin){
  pinMode(fowardFanPin, OUTPUT);
  pinMode(backwardFanPin, OUTPUT);
}

void ServoVehicleState::runFan(double pwm, int forwardFanPin, int backwardFanPin){

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
      digitalWrite(forwardFanPin, HIGH);
    }
    else if (pwm < 0){
      digitalWrite(backwardFanPin, HIGH);
    }
  }
  else{
    digitalWrite(forwardFanPin, LOW);
    digitalWrite(backwardFanPin, LOW);
  }
}


double ServoVehicleState::findPWM(double goal, double timeSinceLastIteration){
  //Input goal is angle to position [-180:180] off the y-axis (CCW +)
  //Input timeSinceLastIteration should be in seconds

  IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));

  //Vector<3> ori = stateIMU.absoluteOrientationEuler;
  Vector<3> ori = imu->getAngularVelocity(); // TODO this is obivously wrong, see the line above
  double roll = ori.x();
  double pitch = ori.y();
  double yaw = ori.z(); //body frame from Inertial frame angle (psi)

  //Find proportional error
  double Ep = goal - yaw;
  if(Ep >= 180){Ep = 360 - Ep;}
  else if(Ep <= -180){Ep = 360 + Ep;}

  // Ed Angular Velocity Method
  Vector<3> angularVelocity = imu->getAngularVelocity();
  double Ed = -angularVelocity.z(); // TODO does this need to be inertial ang velo?

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

Point ServoVehicleState::getTargetCoordinates(){
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
  if (closestSafePoint != Point(0.0, 0.0))
      return closestSafePoint;
  
  Point closestValidPoint = closest(current, valids, numTarg);
  if (closestValidPoint != Point(0.0, 0.0))
      return closestValidPoint;

  Point closestPoint = closest(current, targetPoints, numTarg);
  return closestPoint;
}

Point ServoVehicleState::getWindCorrectionCoordinates(Point r){
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
