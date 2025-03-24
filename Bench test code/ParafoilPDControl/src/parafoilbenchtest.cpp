#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <vector>


int buzzer_pin = 9;

int left_servo_pin = 22;
int right_servo_pin = 23; // figure out which pins are being used

Servo leftServo;
Servo rightServo;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Target angle (in degrees)
float targetAngle = -60.0;

// PD Controller gains
float kp = 1; // Proportional gain
float kd = 1*kp; // Derivative gain
float ki = 0.2; // Integral gain


// Previous error for derivative term
float previousError = 0.0;

// Time tracking for derivative computation
unsigned long previousTime = 0;

const int UPDATE_RATE = 25;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

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
  leftServo.attach(left_servo_pin);
  delay(250);
  leftServo.write(90);
  delay(250);
  rightServo.attach(right_servo_pin);
  delay(250);
  rightServo.write(90);
  delay(250);
  Serial.begin(9600);
  //set buzzer pin to output
  pinMode(buzzer_pin, OUTPUT);
  //beep buzzer twice
  for(int i = 0; i < 2; i++){
    digitalWrite(buzzer_pin, HIGH);
    delay(200);
    digitalWrite(buzzer_pin, LOW);
    delay(200);
  }
}

unsigned long lastTime = 0; // Rename time to lastTime
static double last = 0;
float integral = 0;
void loop() {
  double time = millis();
  if (time - last < UPDATE_INTERVAL)
        return;
    last = time;
  // if(millis() - lastTime > 4000){
  //   if(targetAngle == 300.0){
  //     targetAngle = 120.0;
  //   } else {
  //     targetAngle = 300.0;
  //   }
  //   lastTime = millis();
  // }
  // Read the current orientation
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Extract the roll angle
  float currentAngle = event.orientation.x;
  float currentAngley = event.orientation.y;
  float currentAnglez = event.orientation.z-90;
  Serial.print("currenty =");
  Serial.println(currentAngley);
  Serial.print("currentz =");
  Serial.println(currentAnglez);
  if(abs(currentAngley) > 45 || abs(currentAnglez) > 45){
    leftServo.write(90);
    rightServo.write(90);
    return;
  }
  // Calculate error
  float error = currentAngle - targetAngle;
  
  // Correct for angle wrap and propellor directionality
  Serial.print("Error: ");
  Serial.print(error);
  
  if(error > 180){
    error = error - 360;

  }

  Serial.print(", Corrected Error: ");
  Serial.print(error);
  
 if(abs(error)>30){
  digitalWrite(buzzer_pin, LOW);
 }
 else{
  digitalWrite(buzzer_pin, LOW);
 }
  // Get current time
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  
  // Calculate derivative term
  float derivative = 0;
  if (deltaTime > 0) {
    derivative = -bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[1];
  }
  // Calculate integral term
  
  // PD Controller output
  float output = kp*constrain(error,-60,60) + (kd * derivative) + (ki * integral);
  double servoAngle = map(constrain(output,-100,100),-100,100,0,180);
  // For actuating:
  leftServo.write(constrain(servoAngle,0,90));
  rightServo.write(constrain(servoAngle,90,180));


  integral += error * deltaTime;
  // Set motor speed to zero if a sign change is detected to prevent motor stall

  // Debugging information
  Serial.print(", Target: ");
  Serial.print(targetAngle);
  Serial.print(", Current: ");
  Serial.print(currentAngle);
  Serial.print(", Output: ");
  Serial.println(servoAngle);
  //Serial.print("gyroy =");
  //Serial.println(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[1]);
  //Serial.print("gyrox =");
  //Serial.println(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[0]);
  
  //Serial.print("gyroz =");
  //Serial.println(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[2]);

  // Update previous error and time
  previousError = error;
  previousTime = currentTime;

  // Small delay to avoid saturating the loop
  delay(10);
}