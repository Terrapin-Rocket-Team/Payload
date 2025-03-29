#include <Arduino.h>
#include <Wire.h>
#include <vector>
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
                        .withBBPin(LED_BUILTIN)
                        .withUpdateRate(10);

MMFSSystem computer = MMFSSystem(&config);

float targetAngle;

// PD Controller gains
float kp = 0.5; // Proportional gain
float kd = 0.4; // Derivative gain

float previousError = 0.0;
unsigned long previousTime = 0;
int timeOfLastUpdate = 0;
double servoAngle=60;

void setup()
{
  computer.init();
  getLogger().recordLogData(mmfs::INFO_, "Entering Setup");  

  vehicle.servoSetup(2,3,4,180,0,90);

  // how many nichrome do we need?
  pinMode(4, OUTPUT);
  pinMode(5,OUTPUT);

  Serial.begin(9600);

  getLogger().recordLogData(mmfs::INFO_, "Leaving Setup");

}

double previousAngle = 0;

void loop()
{
  if (computer.update()){
    if (vehicle.stage == GLIDING) {
      //nichrome for bag/parafoil
      digitalWrite(3, HIGH);
      digitalWrite(4, HIGH);
    }
  
    if (vehicle.stage == GLIDING) {
  
      // // Get current orientation
      // Matrix orientation = vehicle_imu.getOrientation().toMatrix();
      // mmfs::Matrix m = vehicle_imu.getOrientation().toMatrix();
      // double C20 = m.get(2,0);
      // C20 = max(-1.0, min(1.0, C20)); // Clamping to valid range
  
      // double currentAngle=0.8*asin(-C20)*180/3.14 + 0.2*previousAngle; //Running into Sin domain error here
      // previousAngle = currentAngle;
  
      // // Calculate error
      // targetAngle = vehicle.goalOrbit(vehicle.rocketx, vehicle.rockety, gps.getPos()[0], gps.getPos()[1], 50/111111); //final argument is target radius (converting 50 long/lat to meters - is this right?)
      // float error = currentAngle - targetAngle;
  
      // Serial.print("Error: ");
      // Serial.print(error);
  
      // if(error > 180){
      //   error = error - 360;
      // }
  
      // // Calculate derivative term
      // float derivative = 0;
  
      // derivative = (error - previousError)/(UPDATE_INTERVAL/1000);
  
      // // Calculate integral term
      // // PD Controller output
  
      // float output = kp*constrain(error,-60,60) + (kd * derivative);
      // double servoAngle = map(constrain(output,-100,100),-100,100,0,180);
  
      // For actuating:
      vehicle.left.write(90 - constrain(servoAngle,0,90));
      vehicle.right.write(180 - constrain(servoAngle,90,180));
  
      Serial.print(", Target: ");
      Serial.print(targetAngle);
      Serial.print(", Current: ");
      //Serial.print(currentAngle);
      Serial.print(", rservoval: ");
      Serial.println(vehicle.rightServoValue);
      Serial.print(", lservoval: ");
      Serial.println(vehicle.leftServoValue);
  
      vehicle.leftServoValue = vehicle.left.read();
      vehicle.rightServoValue = vehicle.right.read();
      //vehicle.servoOutput = output;
      vehicle.vehicleX = gps.getPos()[0];
      vehicle.vehicleY = gps.getPos()[1];
      //vehicle.currentAngle = currentAngle;
      vehicle.targetAngle = targetAngle;
      //vehicle.currentAngleY = orientation.get(1,0);
      //vehicle.currentAngleZ = orientation.get(0,0);  
  
      //previousError = error;
  
    }

}

}