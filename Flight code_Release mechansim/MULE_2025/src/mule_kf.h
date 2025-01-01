#ifndef MULE_KF_H
#define MULE_KF_H

#include "../src/Filters/LinearKalmanFilter.h"

namespace mmfs {

class MuleKF : public LinearKalmanFilter {
public:
    MuleKF();
    ~MuleKF() = default;

    // Override getter methods to provide subteam-specific matrix implementations
    void initialize() override {};
    Matrix getF(double dt) override;
    Matrix getG(double dt) override;
    Matrix getH() override;
    Matrix getR() override;
    Matrix getQ() override;
};

} // namespace mmfs

#endif // MULE_KF_H