#include "tail_rotor_kf.h"

namespace mmfs {

// Define the measurement size, control size, and state size
TailRotorKF::TailRotorKF() : LinearKalmanFilter(3, 3, 6) {}

Matrix TailRotorKF::getF(double dt) {
    double *data = new double[36]{
        1.0, 0, 0, dt, 0, 0,
        0, 1.0, 0, 0, dt, 0,
        0, 0, 1.0, 0, 0, dt,
        0, 0, 0, 1.0, 0, 0,
        0, 0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 0, 1.0
    };
    return Matrix(6, 6, data);
}

Matrix TailRotorKF::getG(double dt) {
    double *data = new double[18]{
        0.5 * dt * dt, 0, 0,
        0, 0.5 * dt * dt, 0,
        0, 0, 0.5 * dt * dt,
        dt, 0, 0,
        0, dt, 0,
        0, 0, dt
    };
    return Matrix(6, 3, data);
}

Matrix TailRotorKF::getH() {
    double *data = new double[18]{
        1.0, 0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 0
    };
    return Matrix(3, 6, data);
}

Matrix TailRotorKF::getR() {
    double *data = new double[9]{
        1.0, 0, 0,
        0, 1.0, 0,
        0, 0, 0.5
    };
    return Matrix(3, 3, data);
}

Matrix TailRotorKF::getQ() {
    double *data = new double[36]{
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
    };
    return Matrix(6, 6, data);
}

} // namespace mmfs