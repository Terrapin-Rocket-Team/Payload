#include <Arduino.h>
#include <Servo.h>

Servo motor;

void setup() {
    motor.attach(3, 1000, 2000);
    delay(1000);
    motor.writeMicroseconds(1500);  
    delay(1000);
}

void loop() {
    motor.writeMicroseconds(1600);
    delay(1000);
    motor.writeMicroseconds(1400);
    delay(1000);  
}