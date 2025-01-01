#include "bno055_breakout.h"

namespace mmfs
{

    BNO055_Breakout::BNO055_Breakout(const char *name) : bno()
    {
        setName(name);
    }
    bool BNO055_Breakout::init()
    {
        if (!bno.begin())
        {
            return initialized = false;
        }
        bno.setExtCrystalUse(true);

        initialMagField = convertIMUtoMMFS(bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER));
        return initialized = true;
    }

    void BNO055_Breakout::read()
    {
        measuredAcc = convertIMUtoMMFS(bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
        measuredGyro = convertIMUtoMMFS(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));
        measuredMag = convertIMUtoMMFS(bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER));

        orientation = convertIMUtoMMFS(bno.getQuat());
        //check the i2c bus to make sure the BNO didn't misbehave
        // Wire.beginTransmission(0x28); //BNO default address. TODO: Allow users to change addresses of devices
        // byte b = Wire.endTransmission();
        // if (b != 0x00)
        // {
        //     Wire.end();
        //     Wire.begin();
        //     logger.recordLogData(ERROR_, "I2C Error");
        // }
    }

    void BNO055_Breakout::calibrateBno()
    {
        // Instructions on how to calibrate BNO055
        // Hold still to calibrate gyro
        // Move in figure 8 to calibrate mag
        // Steps to calibrate accel
        // 1. lay flat for 3 seconds
        // 2. tilt at 45 degree angle for 3 seconds
        // 3. tilt opposite 45 degree angle for 3 seconds
        // 4. tilt 45 degrees in other plane for 3 seconds
        // 5. tilt oppsite 45 degrees in other plane for 3 seconds
        // 6. turn upside for 3 seconds
        // 7. repeat 2-5 but start with accel upsidedown
        // 8. repeat 1-7 until fully calibrated
        uint8_t system, gyro, accel, mag, i = 0;
        while ((system != 3) || (gyro != 3) || (accel != 3) || (mag != 3))
        {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            i = i + 1;
            if (i == 10)
            {
                i = 0;
            }
            delay(10);
        }
    }
}