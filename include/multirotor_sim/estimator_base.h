#pragma once

#include <Eigen/Core>

#include "gnss_utils/gtime.h"
#include "gnss_utils/satellite.h"
#include "multirotor_sim/state.h"

namespace  multirotor_sim
{

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3;
typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> VecMat3;
typedef std::vector<gnss_utils::Satellite, Eigen::aligned_allocator<gnss_utils::Satellite>> SatVec;
class EstimatorBase
{
public:
    // t - current time (seconds)
    // z - imu measurement [acc, gyro]
    // R - imu covariance
    virtual void imuCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    virtual void altCallback(const double& t, const Vector1d& z, const Matrix1d& R) {}
    virtual void baroCallback(const double& t, const Vector1d& z, const Matrix1d& R) {}
    virtual void mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R) {}
    virtual void voCallback(const double& t, const xform::Xformd& z, const Matrix6d& R) {}
    virtual void imageCallback(const double& t, const ImageFeat& z, const Eigen::Matrix2d& R_pix,
                               const Matrix1d& R_depth) {}

    // t - current time (seconds)
    // z - gnss measurement [p_{b/ECEF}^ECEF, v_{b/ECEF}^ECEF]
    // R - gnss covariance
    virtual void gnssCallback(const double& t, const Vector6d& z, const Matrix6d& R) {}

    // t - Time of measurement (GPS Time)
    // z - gnss measurements [[rho(m), rhodot(m/s), l(cycles)], x N]
    // R - gnss covariance xN
    // sat - Satellite objects related to each measurement
    virtual void rawGnssCallback(const gnss_utils::GTime& t, const VecVec3& z, const VecMat3& R,
                                 SatVec& sat, const std::vector<bool>& slip) {}
};

}
