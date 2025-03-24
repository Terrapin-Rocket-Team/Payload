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
                        .withBBPin(LED_BUILTIN);

MMFSSystem computer = MMFSSystem(&config);

float targetAngle = 60;

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
  /*
  if (!(logger.isSdCardReady()))
        bb.onoff(BUZZER_PIN, 200, 3);

    if (!(logger.isPsramReady()))
        bb.onoff(BUZZER_PIN, 200, 3);

  */
  vehicle.servoSetup(1,2,3,180,0,90);


  // how many nichrome do we need?
  pinMode(4, OUTPUT);
  pinMode(5,OUTPUT);

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

  if (vehicle.stage == ACTUATION) {
    //digitalWrite(...) scr?

    delay(2000); //wait for drogue chute to catch completely

    //nichrome for bag/parafoil

    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);

  }

  if (vehicle.stage == MAIN) {

    // Extract the roll angle
    Matrix orientation = vehicle_imu.getOrientation().toMatrix();

    float currentAngle = orientation.get(2,0); //yaw
    float currentAngley = orientation.get(1,0); //pitch
    float currentAnglez = orientation.get(0,0); //roll

    // Calculate error
    float error = currentAngle - targetAngle;

    Serial.print("Error: ");
    Serial.print(error);

    if(error > 180){
      error = error - 360;
    }

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

    /*

    // Calculate derivative term
    float derivative = 0;
    if (deltaTime > 0) {
      derivative = (error - previousError) / deltaTime;
    }
      
    // PD Controller output
    float output = (kp * error) + (kd * derivative);

    */

    vehicle.moveServo(error);


    Serial.print(", Target: ");
    Serial.print(targetAngle);
    Serial.print(", Current: ");
    Serial.print(currentAngle);
    Serial.print(", rservoval: ");
    Serial.println(vehicle.right_servo_value);
    Serial.print(", lservoval: ");
    Serial.println(vehicle.left_servo_value);  


    previousError = error;
    previousTime = currentTime;

    // Small delay to avoid saturating the loop
    delay(10);

  }

}