#ifndef BNO055_BREAKOUT_H
#define BNO055_BREAKOUT_H

#include <Adafruit_BNO055.h>
#include <Sensors/IMU/IMU.h>

namespace mmfs
{
    class BNO055_Breakout : public IMU
    {
    private:
        Adafruit_BNO055 bno;

        // Helper function to convert Adafruit's imu::Vector to mmfs::Vector
        template <uint8_t N>
        mmfs::Vector<N> convertIMUtoMMFS(imu::Vector<N> src)
        {
            mmfs::Vector<N> dest;
            for (int i = 0; i < N; i++)
            {
                dest[i] = src[i];
            }
            return dest;
        }
        mmfs::Quaternion convertIMUtoMMFS(imu::Quaternion src)
        {
            mmfs::Quaternion dest;
            dest.w() = src.w();
            dest.x() = src.x();
            dest.y() = src.y();
            dest.z() = src.z();
            return dest;
        }

    public:
        // BNO Returns ACC in m/s^2, orientation in quaternion, orientation in euler angles, and magnetometer in uT (microteslas)
        BNO055_Breakout(const char *name = "BNO055_Breakout");
        virtual void calibrateBno();
        virtual bool init() override;
        virtual void read() override;
    };
}

#endif // BNO055_Breakout_H