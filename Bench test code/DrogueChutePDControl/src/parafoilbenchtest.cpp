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
double previousAngle=0;

VehicleState vehicle(sensors, 3, nullptr);

MMFSConfig config = MMFSConfig()
                        .withState(&vehicle)
                        .withBuzzerPin(13)
                        .withBBPin(LED_BUILTIN);

MMFSSystem computer = MMFSSystem(&config);

float targetAngle = 60;
float rocketx;
float rockety;

// PD Controller gains
float kp = 0.5; // Proportional gain
float kd = 0.4; // Derivative gain

float previousError = 0.0;
unsigned long previousTime = 0;
int timeOfLastUpdate = 0;

void setup()
{
  computer.init();
  getLogger().recordLogData(mmfs::INFO_, "Entering Setup");  

  vehicle.servoSetup(2,3,90,90);

  Serial.begin(9600);

  getLogger().recordLogData(mmfs::INFO_, "Leaving Setup");

}

void loop()
{
  computer.update();

  int currentTime = millis();
    if (currentTime - timeOfLastUpdate < UPDATE_INTERVAL)
        return;
    timeOfLastUpdate = currentTime;


    // Get current orientation

    Matrix orientation = vehicle_imu.getOrientation().toMatrix();
    mmfs::Matrix m = vehicle_imu.getOrientation().toMatrix();
    //double C01 = m.get(0, 1);
    //double C11 = m.get(1, 1);
    double C20 = m.get(2,0);
    
    
    double currentAngle=0.8*asin(-C20)*180/3.14 + 0.2*previousAngle;

    // Calculate error
    //targetAngle = vehicle.goalOrbit(rocketx, rockety, gps.getPos()[0], gps.getPos()[1], 50/111111); //final argument is target radius (converting 50 long/lat to meters)
    float error = currentAngle - targetAngle;

    Serial.print("Error: ");
    Serial.print(error);

    if(error > 180){
      error = error - 360;
    }

    float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

    // Calculate derivative term
    float derivative = 0;

    derivative = (error - previousError)/deltaTime;

    // Calculate integral term
    // PD Controller output

    float output = kp*constrain(error,-60,60) + (kd * derivative);
    double servoAngle = map(constrain(output,-100,100),-100,100,0,180);

    // For actuating:
    vehicle.servo.write(constrain(servoAngle,30,150));


    Serial.print(", Target: ");
    Serial.print(targetAngle);
    Serial.print(", Current: ");
    Serial.print(currentAngle);
    Serial.print(", servoval: ");
    Serial.println(vehicle.ServoValue);  
    Vector magstuff = vehicle_imu.getMagField();
    Serial.print("X magnetic field:");
    Serial.println(magstuff.x());
    Serial.print("Y magnetic field:");
    Serial.println(magstuff.y());
    Serial.print("Z magnetic field:");
    Serial.println(magstuff.z());
    vehicle.ServoValue = vehicle.servo.read();
    vehicle.servoOutput = output;
    vehicle.vehicleX = gps.getPos()[0];
    vehicle.vehicleY = gps.getPos()[1];
    vehicle.currentAngle = currentAngle;
    vehicle.targetAngle = targetAngle;
    vehicle.currentAngleY = orientation.get(1,0);
    vehicle.currentAngleZ = orientation.get(0,0);
    vehicle.altitude = baro.getAGLAltFt();
    



    previousError = error;
    previousTime = currentTime;
    previousAngle = currentAngle;
    

    // Small delay to avoid saturating the loop
    delay(10);



}