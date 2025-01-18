#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

int servo_pin = 3;
Servo motor;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Target angle (in degrees)
float targetAngle = 200.0;

// PD Controller gains
float kp = 0.5; // Proportional gain
float kd = 0.4; // Derivative gain
float directional_correction = 0.8; // Correction for prop inefficiency

// Previous error for derivative term
float previousError = 0.0;

// Time tracking for derivative computation
unsigned long previousTime = 0;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  // Initialize the BNO055
  if (!bno.begin()) {
    Serial.println("Error initializing BNO055!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  // Initialize previousTime
  previousTime = millis();
  // Arming sequence
  motor.attach(3, 1000, 2000);   
  motor.writeMicroseconds(1500);
  delay(2000); 
  motor.writeMicroseconds(1550);
  delay(1000);
  motor.writeMicroseconds(1450);
  delay(1000);
  motor.writeMicroseconds(1500);
  delay(1000); 
  Serial.begin(9600);
}

unsigned long lastTime = 0; // Rename time to lastTime

void loop() {
  // Changes the target angle by 12 degrees every 6 seconds
  if(millis() - lastTime > 6000){
    targetAngle = targetAngle + 120;
    if (targetAngle > 360){
      targetAngle = targetAngle - 360;
    }
    lastTime = millis();
  }

  // Read the current orientation
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Extract the roll angle
  float currentAngle = event.orientation.x;

  // Calculate error
  float error = currentAngle - targetAngle;
  
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
  motor.writeMicroseconds(pwmOutput);

  // Debugging information
  Serial.print("Target: ");
  Serial.print(targetAngle);
  Serial.print(", Current: ");
  Serial.print(currentAngle);
  Serial.print(", Error: ");
  Serial.print(error);
  Serial.print(", Output: ");
  Serial.println(pwmOutput);

  // Update previous error and time
  previousError = error;
  previousTime = currentTime;

  // Small delay to avoid saturating the loop
  delay(10);
}