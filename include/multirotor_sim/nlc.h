// Nonlinear controller for multirotor
#pragma once

#include <Eigen/Dense>
#include "pid.h"

using namespace Eigen;

template<typename T>
class NLC
{
public:
  typedef Matrix<T,3,3> Mat3;
  typedef Matrix<T,3,1> Vec3;
  Mat3 K_p_; // position
  Mat3 K_v_; // velocity
  Mat3 K_d_; // disturbance acceleration
  Vec3 dhat_; // disturbance acceleration
  int path_type_;
  max_t max_;
  T traj_heading_walk_;
  T traj_heading_straight_gain_;
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> udist_;

  NLC()
  {
    K_p_.setIdentity();
    K_v_.setIdentity();
    K_d_.setIdentity();
    dhat_.setZero();
  }

  void init(const Mat3& Kp, const Mat3& Kv, const Mat3& Kd, const int& path_type, const max_t& max,
            const T& traj_head_walk, const T& traj_head_straight_gain, const std::default_random_engine& rng,
            const std::uniform_real_distribution<T>& udist)
  {
    K_p_ = Kp;
    K_v_ = Kv;
    K_d_ = Kd;
    path_type_ = path_type;
    max_ = max;
    traj_heading_walk_ = traj_head_walk;
    traj_heading_straight_gain_ = traj_head_straight_gain;
    rng_ = rng;
    udist_ = udist;
  }

  void computeControl(const state_t& xhat, state_t& xc, const T& dt, const T& sh)
  {
    // Function constants
    static Matrix<T,3,1> e3((T)0, (T)0, (T)1); // general unit vector in z-direction
    static T g(9.80665); // gravity, m/s^2

    // Different velocity and yaw rate commands depending on path type
    T vmag;
    if (path_type_ < 3)
    {
      // Compute vehicle-1 velocity command
      Vector3d phat(xhat.pn, xhat.pe, xhat.pd); // position estimate
      Vector3d pc(xc.pn, xc.pe, xc.pd); // position command
      Vector3d vc = frame_helper::R_v_to_v1(xhat.psi) * K_p_ * (pc-phat); // velocity command

      // enforce max commanded velocity
      vmag = vc.norm();
      if (vmag > max_.vel)
        vc *= max_.vel/vmag;

      // store velocity command
      xc.u = vc(0);
      xc.v = vc(1);
      xc.w = vc(2);

      // get yaw rate direction and allow it to saturate
      xc.r = xc.psi - xhat.psi;
    }
    else
    {
      // Constant forward velocity and constant altitude
      xc.u += traj_heading_walk_ * udist_(rng_) * dt - ((xc.u - vmag) * traj_heading_straight_gain_);
      xc.v += traj_heading_walk_ * udist_(rng_) * dt - (xc.v * traj_heading_straight_gain_);
      xc.w = K_p_(2,2) * (xc.pd - xhat.pd);

      // Wandering yaw rate
      xc.r += traj_heading_walk_ * udist_(rng_) * dt - (xc.r * traj_heading_straight_gain_);
    }

    // Saturate and prevent wrong direction in yaw rate
    if (xc.r > M_PI)
      xc.r -= 2*M_PI;
    if (xc.r < -M_PI)
      xc.r += 2*M_PI;
    xc.r = sat(xc.r, max_.yaw_rate, -max_.yaw_rate);

    // Compute vehicle-1 thrust vector and update disturbance term
    Vector3d vb(xhat.u,xhat.v,xhat.w);
    Vector3d vc(xc.u, xc.v, xc.w);
    Matrix3d R_v1_to_b = frame_helper::R_v_to_b(xhat.phi, xhat.theta, 0);
    Vector3d vhat = R_v1_to_b.transpose()*vb;
    dhat_ = dhat_ - K_d_*(vc-vhat)*dt; // update disturbance estimate
    Vector3d k_tilde = sh * (e3 - (1.0 / g) * (K_v_ * (vc - vhat) - dhat_));

    // pack up throttle command
    xc.throttle = e3.transpose() * R_v1_to_b * k_tilde;
    xc.throttle = sat(xc.throttle, max_.throttle, 0.001);

    // Compute the desired tilt angle
    Vector3d kd = (1.0 / xc.throttle) * k_tilde; // desired body z direction
    kd = kd / kd.norm(); // need direction only
    double kTkd = e3.transpose() * kd;
    double tilt_angle;
    if (fabs(kTkd - 1.0) > 1.0e-6)
      tilt_angle = acos(kTkd); // desired tilt
    else
      tilt_angle = 0;

    // Shortest rotation to desired tilt
    Quatd q_c;
    if (tilt_angle < 1e-6)
      q_c = Quatd::Identity();
    else
    {
      Vector3d k_cross_kd = e3.cross(kd);
      q_c = Quatd::exp(tilt_angle * k_cross_kd / k_cross_kd.norm());
    }

    // Pack up roll/pitch commands
    xc.phi = sat(q_c.roll(), max_.roll, -max_.roll);
    xc.theta = sat(q_c.pitch(), max_.pitch, -max_.pitch);
  }
};
