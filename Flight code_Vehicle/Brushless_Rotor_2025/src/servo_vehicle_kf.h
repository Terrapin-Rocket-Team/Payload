#ifndef SERVO_VEHICLE_KF_H
#define SERVO_VEHICLE_KF_H

#include "../src/Filters/LinearKalmanFilter.h"

namespace mmfs {

class ServoVehicleKF : public LinearKalmanFilter {
public:
    ServoVehicleKF();
    ~ServoVehicleKF() = default;

    // Override getter methods to provide subteam-specific matrix implementations
    void initialize() override {};
    Matrix getF(double dt) override;
    Matrix getG(double dt) override;
    Matrix getH() override;
    Matrix getR() override;
    Matrix getQ() override;
};

} // namespace mmfs

#endif // SERVO_VEHICLE_KF_H