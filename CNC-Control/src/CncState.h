#pragma once

#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <Sensors/HW/Accel/ADXL375.h>
#include <Vector.h>

using namespace astra;

class CncState {
public:
    void begin(HardwareSerial& serial, Servo& esc_);
    void trigger();
    void update();
    void spindleStart();
    void spindleStop();
    void send(const char* cmd);                
    bool sendAndWait(const char* cmd, unsigned long timeout = 1000);
    void cancelJog();


    HardwareSerial* grbl = nullptr;
    Servo* esc = nullptr;
    int escPin = 23;
    long start;

    // response buffer
    void processIncoming();
};

