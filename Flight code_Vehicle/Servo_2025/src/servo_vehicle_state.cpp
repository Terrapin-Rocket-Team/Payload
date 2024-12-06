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

// Servo Functions

void ServoVehicleState::servoSetup(int leftServoPin,int rightServoPin,double leftSerNeutral,double rightSerNeutral){ //input the servo pins, and the value for the servo to be up
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  leftServo.write(leftSerNeutral);
  rightServo.write(rightSerNeutral);
}

void ServoVehicleState::moveServo(double delta){
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

double ServoVehicleState::findDelta(double psi, double gamma){
  //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
  //an explaination of how the values here were derivated

  //Change the yaw in [-180,180] to [0,360]
  if(psi<0) psi += 360;

  //Find delta
  return psi - gamma;
}


void ServoVehicleState::goDirection(double goal){
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
