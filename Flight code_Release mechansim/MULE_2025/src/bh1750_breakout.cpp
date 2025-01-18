#include "bh1750_breakout.h"

namespace mmfs
{

    BH1750_Breakout::BH1750_Breakout(const char *name) : lightMeter()
    {
        setName(name);
    }

    bool BH1750_Breakout::init()
    {
        if (!lightMeter.begin())
        { // hardware I2C mode, can pass in address & alt Wire
            return initialized = false;
        }
        initialLux = lightMeter.readLightLevel();
        return initialized = true;
    }

    void BH1750_Breakout::read()
    {
        lux = lightMeter.readLightLevel(); // lux
    }
}