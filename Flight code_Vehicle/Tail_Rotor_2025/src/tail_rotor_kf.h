#ifndef TAIL_ROTOR_KF_H
#define TAIL_ROTOR_KF_H

#include "../src/Filters/LinearKalmanFilter.h"

namespace mmfs {

class TailRotorKF : public LinearKalmanFilter {
public:
    TailRotorKF();
    ~TailRotorKF() = default;

    // Override getter methods to provide subteam-specific matrix implementations
    void initialize() override {};
    Matrix getF(double dt) override;
    Matrix getG(double dt) override;
    Matrix getH() override;
    Matrix getR() override;
    Matrix getQ() override;
};

} // namespace mmfs

#endif // TAIL_ROTOR_KF_H