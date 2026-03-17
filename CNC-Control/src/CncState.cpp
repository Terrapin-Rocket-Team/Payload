#include <Arduino.h>
#include <Utils/Astra.h>
#include <Servo.h>
#include "CncState.h"


using namespace astra;

void CncState::begin(HardwareSerial& serial, Servo& esc_) {
    grbl = &serial;
    esc = &esc_;
    esc->attach(escPin);
    esc->writeMicroseconds(1500); // neutral / stop
    start = millis();
}

void CncState::spindleStart() {
    if (esc) esc->writeMicroseconds(2000);
}

void CncState::spindleStop() {
    if (esc) esc->writeMicroseconds(1500);
}


void CncState::send(const char* cmd) {
    if (grbl) {
        grbl->print(cmd);
        grbl->print("\n");
    }
    
}

// --- Send and wait for "ok" or "error" ---
bool CncState::sendAndWait(const char* cmd, unsigned long timeout) {
    send(cmd);

    unsigned long start = millis();
    String line = "";

    while (millis() - start < timeout) {
        while (grbl->available()) {
            char c = grbl->read();

            if (c == '\n') {
                line.trim();

                if (line == "ok") return true;
                if (line.startsWith("error")) return false;

                line = "";
            } else {
                line += c;
            }
        }
    }

    return(false);
}

void CncState::cancelJog() {
    grbl->write(0x85);
}