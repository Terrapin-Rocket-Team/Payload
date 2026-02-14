#include <Arduino.h>
#include <Utils/Astra.h>
#include "CncState.h"

using namespace astra;

// Constructor
CncState::CncState() : DefaultState() {
    cncActive = false;
    cncDone = false;
    
}


void CncState::updateCncState() {
    Vector<3> accel = accelSensor->getAccel();
    float az = accel.z();

    if (!cncActive && az >= accelerationThreshold) {
        startCNC();
        cncDone = true;
        Serial.println("CNC ON");
    }

    if (cncActive && az < accelerationThreshold) {
        stopCNC();
        Serial.println("CNC OFF");
    }
}






// Start CNC
void CncState::startCNC() {
    cncActive = true;
    Serial.println("CNC STARTED!");
}

// Stop CNC
void CncState::stopCNC() {
    cncActive = false;
    Serial.println("CNC STOPPED!");
}



// Check if CNC is running
bool CncState::isCncRunning() {
    return cncActive;
}






void CncState::get_Acceleration(){
    Vector<3> accel = accelSensor->getAccel();
    float z = accel.x();
    Serial.println(z);
}
