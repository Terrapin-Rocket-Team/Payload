#pragma once

#include <Arduino.h>
#include <Utils/Astra.h>
#include <Servo.h>
#include <Vector.h>

using namespace astra;

class CncState {
public:
    void begin(HardwareSerial& serial, Servo& esc_);
    void spindleStart();
    void spindleStop();
    void send(const char* cmd);
    void cancelJog();


    HardwareSerial* grbl = nullptr;
    Servo* esc = nullptr;
    int escPin = 23;
    unsigned long start = 0;
    Vector<3> accel;
    float totalAccel = 0.0f;
    float prevAccel = 0.0f;
    bool commandSent = false;
    bool commandStopped = false;
    bool detect = false;
    unsigned long detectTime = 0;
    int step = 0;
};

