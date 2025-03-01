#ifndef MAX_M10S_BREAKOUT_H
#define MAX_M10S_BREAKOUT_H

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino.h>
#include <Sensors/GPS/GPS.h>

namespace mmfs
{

    class MAX_M10S_Breakout : public GPS
    {
    private:
        SFE_UBLOX_GNSS m10s;

    public:
        MAX_M10S_Breakout(const char *name = "MAX_M10S_Breakout");
        bool init() override;
        void read() override;
    };
}
#endif // MAX_M10S_BREAKOUT_H