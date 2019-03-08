#pragma once

#include <Eigen/Core>

#include "multirotor_sim/state.h"
#include "multirotor_sim/gtime.h"
#include "multirotor_sim/satellite.h"
#include "multirotor_sim/state.h"

namespace  multirotor_sim
{

typedef std::vector<Vector3d, aligned_allocator<Vector3d>> VecVec3;
typedef std::vector<Matrix3d, aligned_allocator<Matrix3d>> VecMat3;
class EstimatorBase
{
public:
    // t - current time (seconds)
    // z - imu measurement [acc, gyro]
    // R - imu covariance
    virtual void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    virtual void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) {}
    virtual void baroCallback(const double& t, const Vector1d& z, const Matrix1d& R) {}
    virtual void mocapCallback(const double& t, const Xformd& z, const Matrix6d& R) {}
    virtual void voCallback(const double& t, const Xformd& z, const Matrix6d& R) {}
    virtual void imageCallback(const double& t, const ImageFeat& z, const Matrix2d& R_pix, const Matrix1d& R_depth) {}

    // t - current time (seconds)
    // z - gnss measurement [p_{b/ECEF}^ECEF, v_{b/ECEF}^ECEF]
    // R - gnss covariance
    virtual void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    // t - Time of measurement (GPS Time)
    // z - gnss measurements [[rho(m), rhodot(m/s), l(cycles)], x N]
    // R - gnss covariance xN
    // sat - Satellite objects related to each measurement
    virtual void rawGnssCallback(const GTime& t, const VecVec3& z, const VecMat3& R, std::vector<Satellite>& sat, const std::vector<bool>& slip) {}
};

}
