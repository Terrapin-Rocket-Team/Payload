#include <Arduino.h>

#define GrblSerial Serial8 // Change to Serial1 if using pins 0/1

void setup() {
  // USB baud rate doesn't strictly matter for Teensy, but 115200 is standard
  Serial.begin(115200);
  
  // GRBL's default baud rate is 115200
  GrblSerial.begin(115200);
  
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
    GrblSerial.write(c);
  }

  // 2. Listen for responses from GRBL and send to Computer
  if (GrblSerial.available()) {
    char c = GrblSerial.read();
    Serial.write(c);
  }
}