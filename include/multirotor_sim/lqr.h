// LQR Control of a multirotor
#pragma once

#include "multirotor_sim/pid.h"
#include "lin_alg_tools/care.h"
#include "geometry/quat.h"

#include "multirotor_sim/state.h"
#include "multirotor_sim/utils.h"

using namespace Eigen;

namespace multirotor_sim
{

template<typename T>
Matrix<T,3,1> saturateVector(const T& max, Matrix<T,3,1>& vec)
{
  T vec_mag = vec.norm();
  if (vec_mag <= max)
    return vec;
  else
    return vec / vec_mag * max;
}


// wrap angle to +- input bound (typically [0,2*pi] or [-pi,pi])
template<typename T>
T wrapAngle(const T &angle, const T &bound)
{
  if (angle > bound)
    return angle - T(2.0) * T(M_PI);
  if (angle < bound - T(2.0) * T(M_PI))
    return angle + T(2.0) * T(M_PI);
  return angle;
}


template<typename T>
class LQR
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CareSolver<6,4> care_solver;
  Matrix<T,6,6> A_, Q_, P_;
  Matrix<T,6,4> B_;
  Matrix<T,4,6> K_;
  Matrix<T,4,4> R_, R_inv_;
  max_t max_;
  T p_err_max_;
  T v_err_max_;
  T yaw_err_max_;
  T s_prev_;
  int path_type_;

  LQR()
  {
    A_.setZero();
    A_.template block<3,3>(0,3).setIdentity();
    B_.setZero();
    Q_.setIdentity();
    R_.setIdentity();
    s_prev_ = (T)0.001;
  }

  void init(const int& path_type, const max_t& max, const T& p_err_max, const T& v_err_max,
            const T& yaw_err_max, const Matrix<T,6,6>& Q, const Matrix<T,4,4>& R)
  {
    max_ = max;
    path_type_ = path_type;
    p_err_max_ = p_err_max;
    v_err_max_ = v_err_max;
    yaw_err_max_ = yaw_err_max;
    Q_ = Q;
    R_ = R;
    R_inv_ = R_.inverse();
  }

  void computeControl(const State& xhat, State& xc, const T& sh, double& throttle)
  {
    // Function constants
    static Matrix<T,3,1> e3((T)0, (T)0, (T)1); // general unit vector in z-direction
    static T g(9.80665); // gravity, m/s^2

    // Unpack states
    Matrix<T,3,1> vI = xhat.q.rota(xhat.v);

    // Reference states/inputs
    Matrix<T,3,1> p_err = xc.p - xhat.p;
    Matrix<T,3,1> v_ref = saturateVector<T>((T)max_.vel, p_err);
    T s_ref = sh;
    quat::Quat<T> q_ref = quat::Quat<T>::Identity();
    Matrix<T,3,1> v_err = v_ref - vI;

    // Create error state
    Matrix<T,6,1> x_tilde;
    x_tilde.template segment<3>(0) = saturateVector<T>(p_err_max_, p_err);
    x_tilde.template segment<3>(3) = saturateVector<T>(v_err_max_, v_err);

    // Jacobians
    B_.template block<3,1>(3,0) = -g * sh * xhat.q.rota(e3);
    B_.template block<3,3>(3,1) = g * s_prev_ / sh * xhat.q.inverse().R() * quat::Quat<T>::skew(e3);

    // Compute control
    care_solver.solve(P_, A_, B_, Q_, R_);
    K_ = R_inv_ * B_.transpose() * P_;
    Matrix<T,4,1> u_tilde = -K_ * x_tilde;

    // Extract control components
    T s_tilde = u_tilde(0);
    Matrix<T,3,1> q_tilde = u_tilde.template segment<3>(1);

    // Commands
    throttle = sat(s_ref - s_tilde, max_.throttle, 0.001);
    quat::Quat<T> qc = q_ref * quat::Quat<T>::exp(-q_tilde);

    // Get yaw rate in direction that reduces yaw error
    xc.w(2) = sat(wrapAngle(xc.q.yaw() - xhat.q.yaw(), M_PI), max_.yaw_rate, -max_.yaw_rate);

    // Pack up roll, pitch, and yaw rate commands
    T phi = sat(qc.roll(), max_.roll, -max_.roll);
    T theta = sat(qc.pitch(), max_.pitch, -max_.pitch);
    xc.q = quat::Quat<T>::from_euler(phi, theta, 0);

    // Save throttle command for next iteration
    s_prev_ = throttle;
  }
};
}
