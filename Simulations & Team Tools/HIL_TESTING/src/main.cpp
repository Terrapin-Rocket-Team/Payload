#include <Arduino.h>
#include <math.h>
float angleToRocket;
float radius;
float error = 0;
float previousError = 0;
void setup() {
  Serial.begin(115200);          // Start serial communication
  while (!Serial) {}             // Wait for serial connection
}

float computeControl(float heading, float x, float y, float angularVelocity) {
  angleToRocket = atan2(y,x);
  radius = sqrt(x*x+y*y);
  dError = previous
  
  
  
  return 
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read until newline
    float heading, lat, lon, ang_vel, ang_accel;

    // Parse the CSV string into variables
    int parsed = sscanf(input.c_str(), "%f,%f,%f,%f,%f", 
                        &heading, &lat, &lon, &ang_vel, &ang_accel);

    if (parsed == 5) {
      float control_angle = computeControl(heading, lat, lon, ang_vel, ang_accel);
      Serial.println(control_angle, 6);  // Send it back with 6 decimal places
    }
  }
}
