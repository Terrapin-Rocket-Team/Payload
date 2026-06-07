#include <Arduino.h>
#include <Utils/Astra.h>
#include <Servo.h>
#include "CncState.h"

using namespace astra;

void CncState::begin(HardwareSerial& serial, Servo& esc_) {
    grbl = &serial;
    esc = &esc_;

    // NOTE: Ensure 'escPin' is populated in your CncState header file
    // or passed into this function to match your hardware layout (Pin 23).
    if (!esc->attached()) {
        esc->attach(escPin);
    }

    esc->writeMicroseconds(1500); // Send neutral/stop to arm the ESC
    start = millis();
}

void CncState::spindleStart() {
    if (esc) esc->writeMicroseconds(1750);
}

void CncState::spindleStop() {
    if (esc) esc->writeMicroseconds(1500);
}

void CncState::send(const char* cmd) {
    if (grbl) {
        grbl->print(cmd);
        grbl->print("\n"); // GRBL requires a newline to execute instructions
    }
}

// REMOVED: sendAndWait()
// Reason: Blocking while-loops drop rocket telemetry frames.
// Handled asynchronously in LaunchSequencer::parseGrblResponse instead.

void CncState::cancelJog() {
    if (grbl) {
        grbl->write(0x85); // Real-time Jog Cancel byte sent straight to GRBL
    }
}
