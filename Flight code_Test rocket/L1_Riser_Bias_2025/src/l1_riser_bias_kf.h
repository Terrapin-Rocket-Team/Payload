#ifndef L1_RISER_BIAS_KF_H
#define L1_RISER_BIAS_KF_H

#include "../src/Filters/LinearKalmanFilter.h"

namespace mmfs {

class L1RiserBiasKF : public LinearKalmanFilter {
public:
    L1RiserBiasKF();
    ~L1RiserBiasKF() = default;

    // Override getter methods to provide subteam-specific matrix implementations
    void initialize() override {};
    Matrix getF(double dt) override;
    Matrix getG(double dt) override;
    Matrix getH() override;
    Matrix getR() override;
    Matrix getQ() override;
};

} // namespace mmfs

#endif // L1_RISER_BIAS_KF_H