#include <Arduino.h>
using namespace mmfs;
//sensors

// MMFS Stuff (stolen from airbrake)
mmfs::Logger logger;
mmfs::ErrorHandler errorHandler;
mmfs::PSRAM *psram;
const int UPDATE_RATE = 10;
const int UPDATE_INTERVAL = 1000.0 / UPDATE_RATE;

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}