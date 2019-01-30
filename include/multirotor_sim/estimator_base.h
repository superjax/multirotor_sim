#pragma once

#include <Eigen/Core>

#include "multirotor_sim/state.h"
#include "multirotor_sim/gtime.h"
#include "multirotor_sim/satellite.h"

namespace  multirotor_sim
{

class EstimatorBase
{
public:
    // t - current time (seconds)
    // z - imu measurement [acc, gyro]
    // R - imu covariance
    virtual void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    virtual void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) {}
    virtual void mocapCallback(const double& t, const Xformd& z, const Matrix6d& R) {}
    virtual void voCallback(const double& t, const Xformd& z, const Matrix6d& R) {}
    virtual void imageCallback(const double& t, const Image& z, const Matrix2d& R) {}

    // t - current time (seconds)
    // z - gnss measurement [p_{b/ECEF}^ECEF, v_{b/ECEF}^ECEF]
    // R - gnss covariance
    virtual void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    // t - Time of measurement (GPS Time)
    // z - gnss measurement [rho(m), rhodot(m/s), l(cycles)]
    // R - gnss covariance
    // sat - Satellite object related to this measurement
    virtual void rawGnssCallback(const GTime& t, const Vector3d& z, const Matrix3d& R, Satellite& sat) {}
};

}
