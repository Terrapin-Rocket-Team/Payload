#include "tail_rotor_state.h"

using namespace mmfs;

/// Navigation Stuff ///
// Higgs	
const Line line1 = Line(Point(-75.87439444, 39.079425), Point(-75.87331111, 39.08091111));
const Line line2 = Line(Point(-75.87320833, 39.07993889), Point(-75.871675, 39.07527778));
const Line line3 = Line(Point(-75.87404444, 39.08616667), Point(-75.87263056, 39.08143889));

Obstacle* obstacles[numObs] = {
    &line1,
    &line2,
    &line3
};

const Point targetPoints[numTarg] = {
        Point(-75.87514167, 39.08471667),
        Point(-75.87820833, 39.077525),
        Point(-75.87034444, 39.08389722),
};
/////////////////

TailRotorState::TailRotorState(Sensor **sensors, int numSensors, LinearKalmanFilter *kfilter, int BuzzerPin) : State(sensors, numSensors, kfilter)
{
    stage = PRELAUNCH;
    timeOfLaunch = 0;
    timeOfLastStage = 0;
    timeOfDay = 0;
    buzzerPin = BuzzerPin;
}

void TailRotorState::updateState(double newTime)
{
    State::updateState(newTime); // call base version for sensor updates
    determineStage(); // determine the stage of the flight
}

void TailRotorState::determineStage() // TODO Change this for the tail rotor
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
        bb.aonoff(buzzerPin, 200, 2);
        logger.setRecordMode(FLIGHT);
        stage = BOOST;
        timeOfLaunch = currentTime;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Launch detected.");
    }
    else if (stage == BOOST && imu->getAccelerationGlobal().z() < 0)
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
    else if (stage == DROUGE && baro->getAGLAltFt() < 3000 && timeSinceLaunch > 10)
    {
        bb.aonoff(buzzerPin, 200, 4);
        stage = RELEASED;
        timeOfLastStage = currentTime;
        logger.recordLogData(INFO_, "Main parachute conditions detected.");
    }
    else if (stage == RELEASED && ((baro->getAGLAltFt() < 100) || ((currentTime - timeOfLastStage) > 600000)))
    {
        bb.aonoff(buzzerPin, 200, 5);
        timeOfLastStage = currentTime;
        stage = LANDED;
        logger.recordLogData(INFO_, "Landing detected. Waiting for 30 seconds to dump data.");
    }
    else if (stage == LANDED && currentTime - timeOfLastStage > 30)
    {
        logger.setRecordMode(GROUND);
        logger.recordLogData(INFO_, "Dumped data after landing.");
        stage = PRELAUNCH;
    }
}

int TailRotorState::findPWM(float goal, float deltaTime){
  //Input goal is angle to position [-180:180] off the y-axis (CCW +)
  //Input deltaTime should be in seconds

  IMU *imu = reinterpret_cast<IMU *>(getSensor(IMU_));

  float currentAngle = getOrientation().x();

  // Calculate error
  float error = currentAngle - goal;
  
  // Correct for angle wrap and propellor directionality
  if(error > 180){
    error = error - 360;
    error = error*directionalCorrection;
  }
  
  // Calculate derivative term
  float derivative = 0;
  if (deltaTime > 0) {
    derivative = (error - previousError) / deltaTime;
  }
  
  // PD Controller output
  float output = (kp * error) + (kd * derivative);

  // Map the values of the output to the pwm values
  int pwmOutput = map(output,180,-180,1200,1800);
  // Constrain the inner limits to prevent motor stall
  if((1470 < pwmOutput) && (1530 > pwmOutput)){
    pwmOutput = 1500;
  }
  // Constrain the outer limits to prevent current overdraw
  pwmOutput = constrain(pwmOutput, 1200, 1800);
  
  // Set motor speed to zero if a sign change is detected to prevent motor stall
  if(((error<0) && (previousError>0)) || ((error>0) && (previousError<0))){
    pwmOutput = 1500;
  }

  return pwmOutput;
}

Point TailRotorState::getTargetCoordinates(){
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

Point TailRotorState::getWindCorrectionCoordinates(Point r){
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

float TailRotorState::getGoalAngle(Point target) {
  //Returns an angle from [0:360] from north to get to a target from a current point

  // Get current angle
  GPS *gps = reinterpret_cast<GPS *>(getSensor(GPS_));
  Point current = Point(gps->getPos().y(), gps->getPos().x()); // longitude, latitude

  double theta = atan2(target.y - current.y, target.x - current.x);
  theta = theta * 180 / 3.14;
  double goal = 270 + theta;
  if (goal > 360) {
    goal -= 360;
  }
  return goal;
}

const int TailRotorState::getNumPackedDataPoints() const { return 22; }

const PackedType *TailRotorState::getPackedOrder() const
{
    static const PackedType order[22] = {
        FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, INT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT, FLOAT};
    return order;
}

const char **TailRotorState::getPackedDataLabels() const
{
    static const char *labels[22] = {
        "Time (s)",
        "PX (m)",
        "PY (m)",
        "PZ (m)",
        "VX (m/s)",
        "VY (m/s)",
        "VZ (m/s)",
        "AX (m/s/s)",
        "AY (m/s/s)",
        "AZ (m/s/s)",
        "pwm",
        "goalAngleUsed",
        "goalAngleCalculated",
        "GX",
        "GY",
        "WX",
        "WY",
        "vehicleSpeed",
        "averageWindCorrectionCoords_X",
        "averageWindCorrectionCoords_Y",
        "targetCoords_X",
        "targetCoords_Y"
        };
    return labels;
}

void TailRotorState::packData()
{

    struct PackedData data;
    data.t = currentTime;
    data.px = position.x();
    data.py = position.y();
    data.pz = position.z();
    data.vx = velocity.x();
    data.vy = velocity.y();
    data.vz = velocity.z();
    data.ax = acceleration.x();
    data.ay = acceleration.y();
    data.az = acceleration.z();
    data.pwm = pwm;
    data.goalAngleUsed = goalAngleUsed;
    data.goalAngleCalculated = goalAngleCalculated;
    data.gx = g.x();
    data.gy = g.y();
    data.wx = w.x();
    data.wy = w.y();
    data.vehicleSpeed = v_s;
    data.averageWindCorrectionCoords_X = averageWindCorrectionCoords.x;
    data.averageWindCorrectionCoords_Y = averageWindCorrectionCoords.y;
    data.targetCoords_X = targetCoords.x;
    data.targetCoords_Y = targetCoords.y;
    memcpy(packedData, &data, sizeof(PackedData));
}
