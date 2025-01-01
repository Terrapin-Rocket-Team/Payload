#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

int fwdPin = 3; // 10 for micro;
int bckPin = 2; // 9 for micro;

// BNO055 Sensor Initialization
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Target angle (in degrees)
float targetAngle = 0.0;

// PD Controller gains
float kp = 1.0; // Proportional gain
float kd = 0; // Derivative gain

// Previous error for derivative term
float previousError = 0.0;

// Time tracking for derivative computation
unsigned long previousTime = 0;

// PWM output pin for motor control
const int pwmPin = 9;

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

  // Set PWM pin as output
  pinMode(pwmPin, OUTPUT);
  pinMode(fwdPin, OUTPUT);
  pinMode(bckPin, OUTPUT);

  // Initialize previousTime
  previousTime = millis();

  Serial.println("Setup complete.");
}

void loop() {
  // Read the current orientation
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Extract the roll angle (assume X is roll for simplicity)
  float currentAngle = event.orientation.x;

  // Calculate error
  float error = targetAngle - currentAngle;

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

  // Constrain output to PWM range (-255 to 255)
  int pwmOutput = constrain(output, -255, 255);

  // Output PWM signal to motor
  if (pwmOutput >= 0) {
    analogWrite(fwdPin, pwmOutput);
    analogWrite(bckPin, 0);
  } else {
    analogWrite(fwdPin, 0);
    analogWrite(bckPin, pwmOutput);
  }

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