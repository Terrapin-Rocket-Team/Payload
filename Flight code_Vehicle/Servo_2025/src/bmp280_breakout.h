#ifndef BMP280_BREAKOUT_H
#define BMP280_BREAKOUT_H

#include <Adafruit_BMP280.h>
#include <Sensors/Baro/Barometer.h>

namespace mmfs
{
    class BMP280_Breakout : public Barometer
    {
    private:
        Adafruit_BMP280 bmp;

    public:
        BMP280_Breakout(const char *name = "BMP280_BREAKOUT");
        virtual bool init() override;
        virtual void read() override;
    };
} // namespace mmfs

#endif