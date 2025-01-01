#include <Arduino.h>
#include <Multi_Mission_Flight_Software.h>
#include <PWMServo.h>

State STATE;
IMU TESTIMU("BNO055"); //Input IMU type here

static PWMServo leftServo;
static PWMServo rightServo;

float delta=0;
int goal=90;

String recordDataMode = "Flight";


void setup() {

  Serial.begin(115200);

  Serial.println("Adding IMU");
  STATE.addIMU(TESTIMU);

  Serial.print("Setting up IMU...");
  STATE.stateIMU.setupIMU();
  if(STATE.successfulSetup()){
      Serial.println("Success!");
  }

  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  leftServo.attach(2);
  rightServo.attach(3);
  leftServo.write(90);
  rightServo.write(90);



}

void loop() {

  STATE.settimeAbsolute();

  STATE.stateIMU.readIMU();
  imu::Vector<3> ori = STATE.stateIMU.absoluteOrientationEuler; //function from BNO55.cpp
  double yaw = ori.z(); //body frame from Inertial frame angle

  delta=yaw-goal;
  Serial.println(yaw);

  //See https://github.com/Terrapin-Rocket-Team/SAC-TRT24/blob/main/Code/Payload/Orientation%20Matlab/Orientation.md for
  //an explaination of how the values here were derivated
  double pi = 3.14;
  double leftservo_angle_offset_from_body = 45; //CHECK THIS -> because the servo output ends up offset to the imu axes by 45, should be +-45. Which one is which idk, need to define whats front on the vehicle (what face points to north when yaw=0? Mark it)
  double rightservo_angle_offset_from_body = -45;

  double left_servo_value = 90*(cos((leftservo_angle_offset_from_body-delta)*(pi/180)) + 1); //CHECK THIS FOR ANY NEW ANGLES -> a line is running the wrong way need to add a 180- term -> we cant "rotate" the heading, we can only flip line direction
  double right_servo_value = 180-90*(cos((rightservo_angle_offset_from_body-delta)*(pi/180)) + 1);

  //Serial.print("Left Servo Value: "); Serial.print(left_servo_value); Serial.print(", Right Servo Value: "); Serial.println(right_servo_value);
  //if (left_servo_value <= 90){left_servo_value = 0;} //This section isnt strictly needed but is a good catch, so keep anyway
  //else{left_servo_value = 180;}
  //if (right_servo_value <= 90){right_servo_value = 0;}
  //else{right_servo_value = 180;}

  leftServo.write(left_servo_value);
  rightServo.write(right_servo_value);
  

}
