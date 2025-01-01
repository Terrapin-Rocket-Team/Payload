#include "l1_riser_bias_state.h"

using namespace mmfs;

const Line line1 = Line(Point(-106.922582, 32.939719), Point(-106.909264, 32.943206));

Obstacle* obstacles[] = {
    &line1
};

const Point targetPoints[] = {
        Point(-106.914875, 32.937792),
        Point(-106.920284, 32.943033)
};

L1RiserBiasState::L1RiserBiasState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, bool stateRecordsOwnData) : State(sensors, numSensors, kfilter, stateRecordsOwnData)
{
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
}

void L1RiserBiasState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    determineStage(); // determine the stage of the flight
    
}

void L1RiserBiasState::determineStage() // TODO Change this for the tail rotor
{
    int timeSinceLaunch = currentTime - timeOfLaunch;
    IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));
    Barometer *baro = reinterpret_cast<Barometer *>(getSensor(BAROMETER_));
    if (stage == 0 &&
        (sensorOK(imu) || sensorOK(baro)) &&
        (sensorOK(imu) ? abs(imu->getAccelerationGlobal().z()) > 25 : true) &&
        (sensorOK(baro) ? baro->getAGLAltFt() > 60 : true))
    // if we are in preflight AND
    // we have either the IMU OR the barometer AND
    // imu is ok AND the z acceleration is greater than 29 ft/s^2 OR imu is not ok AND
    // barometer is ok AND the relative altitude is greater than 30 ft OR baro is not ok
    // essentially, if we have either sensor and they meet launch threshold, launch. Otherwise, it will never detect a launch.
    {
        bb.aonoff(buzzerPin, 200);
        logger.setRecordMode(FLIGHT);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Launch detected.");
        logger.recordLogData(INFO_, "Printing static data.");
        for (int i = 0; i < maxNumSensors; i++)
        {
            if (sensorOK(sensors[i]))
            {
                char logData[200];
                snprintf(logData, 200, "%s: %s", sensors[i]->getName(), sensors[i]->getStaticDataString());
                logger.recordLogData(INFO_, logData);
                sensors[i]->setBiasCorrectionMode(false);
            }
        }
    }
    else if (stage == BOOST && abs(acceleration.z()) < 10)
    {
        bb.aonoff(buzzerPin, 200, 2);
        timeOfLastStage = currentTime;
        stage = COAST;
        logger.recordLogData(INFO_, "Coasting detected.");
    }
    else if (stage == COAST && baroVelocity <= 0 && timeSinceLaunch > 5)
    {
        bb.aonoff(buzzerPin, 200, 3);
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        logger.recordLogData(INFO_, logData);
        timeOfLastStage = currentTime;
        stage = DROUGE;
        logger.recordLogData(INFO_, "Drogue conditions detected.");
    }
    else if (stage == DROUGE && baro->getAGLAltFt() < 1000 && timeSinceLaunch > 10)
    {
        bb.aonoff(buzzerPin, 200, 4);
        stage = MAIN;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Main parachute conditions detected.");
    }
    else if (stage == MAIN && baroVelocity > -1 && baro->getAGLAltFt() < 66 && timeSinceLaunch > 15)
    {
        bb.aonoff(buzzerPin, 200, 5);
        timeOfLastStage = currentTime;
        stage = LANDED;
        logger.recordLogData(INFO_, "Landing detected. Waiting for 5 seconds to dump data.");
    }
    else if (stage == LANDED && currentTime - timeOfLastStage > 5)
    {
        logger.setRecordMode(GROUND);
        logger.recordLogData(INFO_, "Dumped data after landing.");
    }
}

void L1RiserBiasState::fanSetup(int fowardFanPin,int backwardFanPin){
  pinMode(fowardFanPin, OUTPUT);
  pinMode(backwardFanPin, OUTPUT);
}

void L1RiserBiasState::runFan(double pwm, int forwardFanPin, int backwardFanPin){

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


double L1RiserBiasState::findPWM(double goal, double timeSinceLastIteration){
  //Input goal is angle to position [-180:180] off the y-axis (CCW +)
  //Input timeSinceLastIteration should be in seconds

  IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));

  //Vector<3> ori = stateIMU.absoluteOrientationEuler;
  Vector<3> ori = imu->getGyroReading(); // TODO this is obivously wrong, see the line above
  double roll = ori.x();
  double pitch = ori.y();
  double yaw = ori.z(); //body frame from Inertial frame angle (psi)

  //Find proportional error
  double Ep = goal - yaw;
  if(Ep >= 180){Ep = 360 - Ep;}
  else if(Ep <= -180){Ep = 360 + Ep;}

  // Ed Angular Velocity Method
  Vector<3> angularVelocity = imu->getGyroReading();
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

Point L1RiserBiasState::getTargetCoordinates(){
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

Point L1RiserBiasState::getWindCorrectionCoordinates(Point r){
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

// imu::Vector<3> L1RiserBiasState::getInertialAngularVelocity(){
//   //TODO whats even going on here?
//   imu::Quaternion orientation = stateIMU.absoluteOrientation;
//   orientation.normalize();
//   imu::Quaternion orientationConj = orientation.conjugate();

//   imu::Vector<3> rocketFrameAngularVelocity = stateIMU.angularVelocity;
//   imu::Quaternion rocketFrameAngularVelocityQuat;
//   rocketFrameAngularVelocityQuat.w() = 0;
//   rocketFrameAngularVelocityQuat.x() = rocketFrameAngularVelocity.x();
//   rocketFrameAngularVelocityQuat.y() = rocketFrameAngularVelocity.y();
//   rocketFrameAngularVelocityQuat.z() = rocketFrameAngularVelocity.z();

//   imu::Quaternion absoluteAngularVelocityQuat = quatMultiplication(quatMultiplication(orientation, rocketFrameAngularVelocityQuat), orientationConj);
//   imu::Vector<3> absoluteAngularVelocity;
//   absoluteAngularVelocity.x() = absoluteAngularVelocityQuat.x();
//   absoluteAngularVelocity.y() = absoluteAngularVelocityQuat.y();
//   absoluteAngularVelocity.z() = absoluteAngularVelocityQuat.z();

//   return absoluteAngularVelocity;
// }
