#include <Arduino.h>
#include <Servo.h>
#include <MMFS.h>

#include "servo_vehicle_state.h"
#include "bmp280_breakout.h"
#include "bno055_breakout.h"
#include "max_m10s_breakout.h"

int ESC_PIN = 3;
Servo MOTOR;

BMP280_Breakout barometer;
BNO055_Breakout vehicle_imu;
MAX_M10S_Breakout gps;
mmfs::Sensor* servo_vehicle_sensors[3] = {&barometer, &vehicle_imu, &gps};
mmfs::Logger logger;
// Target angle (in degrees)
float TARGET_ANGLE = 300.0;

// PD Controller gains
float kp = 0.5; // Proportional gain
float kd = 0.4; // Derivative gain
float directional_correction = 0.8; // Correction for prop inefficiency

float previousError = 0.0;
unsigned long previousTime = 0;

const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

int timeOfLastUpdate = 0;
void setup() {
  logger.init();
  logger.recordLogData(mmfs::INFO_, "Entering Setup");
  
  if (!(logger.isSdCardReady()))
        bb.onoff(BUZZER_PIN, 200, 3);

    if (!(logger.isPsramReady()))
        bb.onoff(BUZZER_PIN, 200, 3);

  logger.recordLogData(mmfs::INFO_, "Leaving Setup");
  
  // Initialize previousTime
  previousTime = millis();
  // Arming sequence
  MOTOR.attach(3, 1000, 2000);   
  MOTOR.writeMicroseconds(1500);
  delay(2000); 
  MOTOR.writeMicroseconds(1550);
  delay(1000);
  MOTOR.writeMicroseconds(1450);
  delay(1000);
  MOTOR.writeMicroseconds(1500);
  delay(1000); 
  Serial.begin(9600);

}

void loop() {
  int currentTime = millis();
    if (currentTime - timeOfLastUpdate < UPDATE_INTERVAL)
        return;
    timeOfLastUpdate = currentTime;
  
  // Extract the roll angle
  float currentAngle = event.orientation.x;
  float currentAngley = event.orientation.y;
  float currentAnglez = event.orientation.z+90;
  
  if(abs(currentAngley) > 45 || abs(currentAnglez) > 45){
    MOTOR.writeMicroseconds(1500);
    return;
  }
  // Calculate error
  float error = currentAngle - TARGET_ANGLE;
  
  // Correct for angle wrap and propellor directionality
  if(error > 180){
    error = error - 360;
    error = error*directional_correction;
  }

  // Get current time
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  
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
  // Write the output to the ESC
  MOTOR.writeMicroseconds(pwmOutput);

  // Update previous error and time
  previousError = error;
  previousTime = currentTime;

}