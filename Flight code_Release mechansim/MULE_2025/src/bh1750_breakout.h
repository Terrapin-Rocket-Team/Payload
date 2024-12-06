#ifndef BH1750_BREAKOUT_H
#define BH1750_BREAKOUT_H

#include <BH1750.h>
#include <Sensors/LightSensor/LightSensor.h>

namespace mmfs
{
    class BH1750_Breakout : public LightSensor
    {
    private:
        BH1750 lightMeter;

    public:
        BH1750_Breakout(const char *name = "BH1750_Breakout");
        virtual bool init() override;
        virtual void read() override;
    };
} // namespace mmfs

#endif