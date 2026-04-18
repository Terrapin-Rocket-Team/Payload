#ifndef CNC_H
#define CNC_H

#include <Arduino.h>

template<int N>
class Vector {
private:
    float values[N];
public:
    Vector() {
        for (int i = 0; i < N; i++) {
            values[i] = 0.0;
        }
    }
    
    float getX() { return values[0]; }
    float getY() { return values[1]; }
    float getZ() { return values[2]; }
};

class CncState {
private:
    bool cncActive;
    bool cncDone;
    
public:
    CncState();
    
    void updateCncState();
    bool isCncRunning();
    void startCNC();
    void stopCNC();
    
    Vector<3> getAcceleration();
};

#endif
