#ifndef AERO_TIMING
#define AERO_TIMING

extern float startTime;                 // Used for relative timestamps
extern float launchTime;                // Used for relative launch timestamp

void setStartTime(float input);         // Set the startup timestamp
void setLaunchTime(float input);        // Set the launch timestamp

float getStartTime();                   // Return our startup timestamp
float getLaunchTime();                  // Return our launch timestamp

#endif
