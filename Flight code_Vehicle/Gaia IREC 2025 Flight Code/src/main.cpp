#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <Servo.h>
#include "MMFS.h"
#include "VehicleState.h"


mmfs::MAX_M10S gps;
mmfs::MS5611 baro;
mmfs::BMI088andLIS3MDL vehicle_imu;
Logger logger;
Sensor *sensors[] = {&gps, &baro, &vehicle_imu};

VehicleState vehicle(sensors, 3, nullptr);

MMFSConfig config = MMFSConfig()
                        .withState(&vehicle)
                        .withBuzzerPin(13)
                        .withBBPin(13)
                        .withUpdateRate(10)
                        .withBBAsync(true, 50);

MMFSSystem computer = MMFSSystem(&config);

float targetAngle = 0;
float timeofupdate = 100;
// PD Controller gains
float kp = 1;
float ki = 0.0; 
float kd = 0.05;

float error = 0.0;
float previousError = 0.0;
unsigned long previousTime = 0;
int timeOfLastUpdate = 0;

Vector<3> orientationVector(0, 0, 0);
double heading = 0;

Servo actuator;
float integral = 0;
void setup()
{
  computer.init();
  getLogger().recordLogData(mmfs::INFO_, "Entering Setup");  
  actuator.attach(2);
  getLogger().recordLogData(mmfs::INFO_, "Leaving Setup");
}

void loop() {
  if (computer.update()) {  
    //if (vehicle.stage == GLIDING) {
      
      // Pull yaw from the DCM matrix
      mmfs::Matrix DCM_Matrix = vehicle_imu.getOrientation().toMatrix();
      double yaw = atan2(DCM_Matrix.get(1, 0), DCM_Matrix.get(1, 1));
      if (yaw < 0){
        yaw = yaw + 2 * 3.14159;
      }
      heading = yaw*360/(2*3.14159);
      
      
      // Calculate error
      // targetAngle = vehicle.goalOrbit(vehicle.rocketx, vehicle.rockety, gps.getPos()[0], gps.getPos()[1], 50/111111); //final argument is target radius (converting 50 long/lat to meters - is this right?)
      error =  heading - targetAngle;
      //Serial.println(error);
      if(error > 180){
         error = error - 360;
      }

      // Calculate derivative term
      float derivative = 0;
      derivative = (error - previousError)/(timeofupdate/1000);
      // Serial.println(derivative);
      //Serial.print(UPDATE_INTERVAL);
      //Serial.println(derivative);
      integral = integral + error * (timeofupdate/1000);
      Serial.println(error);
      if ((previousError > 0 && error < 0) || (previousError < 0 && error > 0)){
        integral = 0;
        derivative = 0;
      }
      // Serial.println(integral);

      float output = kp * error + ki * integral + kd * derivative;
      
      double servoAngle = map(output, -180, 180, 900, 1800); // Map output to servo angle range
      servoAngle = constrain(servoAngle, 900, 1800);
      actuator.writeMicroseconds(servoAngle);
      Serial.println(servoAngle);
      previousError = error;
    //}
  }
}