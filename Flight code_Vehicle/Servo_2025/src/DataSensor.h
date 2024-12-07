#ifndef DATASENSOR_H
#define DATASENSOR_H
#include "MMFS.h"
#include "Target.h"


using namespace mmfs;

class DataSensor: public Sensor {
    public:
        DataSensor() {
            setUpPackedData();
        };
        ~DataSensor() {}
        virtual void update() override {}
        virtual bool begin(bool useBiasCorrection = true) override {}
        virtual const SensorType getType() const override { return SensorType::LIGHT_SENSOR_; }
        virtual const char *getTypeString() const override { return "DataProxy"; }

        virtual const int getNumPackedDataPoints() const override { return 9; }
        virtual const PackedType *getPackedOrder() const override
        {
            static const PackedType order[] = {DOUBLE, DOUBLE, DOUBLE, DOUBLE, DOUBLE, DOUBLE, DOUBLE, DOUBLE, DOUBLE};
            return order;
        }
        virtual const char **getPackedDataLabels() const override
        {
            static const char *labels[] = {
                "left_servo_value",
                "right_servo_value",
                "GX",
                "GY",
                "WX",
                "WY"
                "vehicleSpeed",
                "averageWindCorrectionCoords_X",
                "averageWindCorrectionCoords_Y"
            };
            return labels;
        }
        virtual void packData();

        virtual void updateStuff(double lsv, double rsv, imu::Vector<2> g1, imu::Vector<2> w1, double vs, Point awcc) {
            left_servo_value = lsv;
            right_servo_value = rsv;

            g = g1;
            w = w1;

            v_s = vs;
            averageWindCorrectionCoords = awcc;
        }

        double left_servo_value;
        double right_servo_value; 

        imu::Vector<2> g; // wind speed in m/s (2D velocity vector) bad comment
        imu::Vector<2> w; // wind speed in m/s (2D velocity vector)
        double v_s = 1.6; // vehicle speed in m/s from https://docs.google.com/document/d/1qh7_YLZrvnW2anWGSmRbWwFUWjS0ocPswoAIC7827A4/edit
        Point averageWindCorrectionCoords;
    
    protected:
        virtual bool init() override {};
        virtual void read() {};
};


#endif DATASENSOR_H