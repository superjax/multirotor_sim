#pragma once

#include <Eigen/Core>

#include "multirotor_sim/state.h"

namespace  multirotor_sim
{

class EstimatorBase
{
public:
    // t - current time (seconds)
    // z - imu measurement [acc, gyro]
    // R - imu covariance
    virtual void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) = 0;

    virtual void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) = 0;
    virtual void posCallback(const double& t, const Vector3d& z, const Matrix3d& R) = 0;
    virtual void attCallback(const double& t, const Quatd& z, const Matrix3d& R) = 0;
    virtual void voCallback(const double& t, const Xformd& z, const Matrix6d& R) = 0;
    virtual void featCallback(const double& t, const Vector2d& z, const Matrix2d& R, int id, double depth) = 0;

    // t - current time (seconds)
    // z - gnss measurement [p_{b/ECEF}^ECEF, v_{b/ECEF}^ECEF]
    // R - gnss covariance
    virtual void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) = 0;
};

}
