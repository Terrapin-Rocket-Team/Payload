#include "timing.h"

float startTime = 0.0f;                 // Used for relative timestamps
float launchTime = 0.0f;                // Used for relative launch timestamp

// Set the startup timestamp
void setStartTime(float input){
    startTime = input;
}

// Set the launch timestamp
void setLaunchTime(float input){
    launchTime = input;
}

// Return our startup timestamp
float getStartTime(){
    return startTime;
}

// Return our launch timestamp
float getLaunchTime(){
    return launchTime;
}
