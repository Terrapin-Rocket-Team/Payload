#include <Arduino.h>

void setup() {
  // USB baud rate doesn't strictly matter for Teensy, but 115200 is standard
  Serial.begin(115200);
  
  // GRBL's default baud rate is 115200
  Serial8.begin(115200);
  
  while (!Serial) {
    ; // Wait for user to open Serial Monitor
  }
  
  Serial.println("--- Teensy to GRBL Bridge Active ---");
  Serial.println("Type '$' to see GRBL settings or '$J=G91 X10 F500' to jog.");
}


void loop() {
  // 1. Listen for commands from the Computer and send to GRBL

  if (Serial.available()) {
    char c = Serial.read();
    Serial8.write(c);
  }

  // 2. Listen for responses from GRBL and send to Computer
  if (Serial8.available()) {
    char c = Serial8.read();
    Serial.write(c);
  }
}