#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

char GcodeFileName[] ="circles2.gcode";
int sendToSerial();
void setup() {
  //  make sure that the baud rate is set at 9600 (or 115200 for v0.9+).
  Serial8.begin(115200);
  while (!Serial8); // Wait for Serial to be ready
  Serial8.write("\r\n\r\n");
  delay(2000);   // Wait for grbl to initialize
  while (Serial8.available()) {
    Serial8.read(); // Flush startup text from serial input
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  sendToSerial();
  while (true); // Stop the loop after sending the file once
}
int sendToSerial() {
  File dataFile = SD.open(GcodeFileName, FILE_READ);
  if (dataFile) {
    while (dataFile.available()) {
      // Read a line until a newline character ('\n') is found
      String line = dataFile.readStringUntil('\n');
      line.trim(); // Remove leading/trailing whitespace (including carriage return '\r' if present)
      Serial8.println(line);
      
    }
    dataFile.close();
    return 0;
  } else {
    Serial.println("Error opening gcode file");
    return 1;
  }
}
