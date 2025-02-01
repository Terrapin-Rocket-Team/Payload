#include "bmp280_breakout.h"

namespace mmfs
{

    BMP280_Breakout::BMP280_Breakout(const char *name) : bmp()
    {
        setName(name);
    }

    bool BMP280_Breakout::init()
    {
        if (!bmp.begin())
        { // hardware I2C mode, can pass in address & alt Wire
            // Serial.println("Could not find a valid BMP280 sensor, check wiring!");
            return initialized = false;
        }
        return initialized = true;
    }

    void BMP280_Breakout::read()
    {
        pressure = bmp.readPressure() / 100.0;       // hPa
        temp = bmp.readTemperature();                // C
    }
}