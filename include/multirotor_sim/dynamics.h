#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "utils.h"
#include "geometry/quat.h"
#include "geometry/xform.h"
#include "geometry/support.h"

#include "multirotor_sim/state.h"

using namespace quat;
using namespace xform;

namespace multirotor_sim
{

// Input indices
enum {
  THRUST = 0,
  TAUX = 1,
  TAUY = 2,
  TAUZ = 3,
  WX = 1,
  WY = 2,
  WZ = 3,
  INPUT_SIZE = 4
};

// IMU indices
enum {
  ACC = 0,
  GYRO = 3
};

static const double G = 9.80665;

static const Eigen::Vector3d gravity_ = [] {
  Eigen::Vector3d tmp;
  tmp << 0, 0, G;
  return tmp;
}();

static const Eigen::Matrix3d M_ = [] {
  Eigen::Matrix3d tmp;
  tmp << 1, 0, 0, 0, 1, 0, 0, 0, 0;
  return tmp;
}();


class Dynamics
{      
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Dynamics();
  
  void load(std::string filename);
  void run(const double dt, const Eigen::Vector4d& u);
  
  void f(const State& x, const Eigen::Vector4d& ft, ErrorState& dx) const;
  void f(const State& x, const Eigen::Vector4d& u, ErrorState& dx, Vector6d& imu) const;
  
  const State& get_state() const { return x_; }
  State& get_state() { return x_; }
  void set_state(const State& x) { x_ = x; }

  Eigen::Vector4d low_level_control(const State& x, const Eigen::Vector4d& u) const;

  const Xformd& get_global_pose() const { return x_.X; }
  const double& get_drag() const { return drag_constant_; }
  const Eigen::Vector3d& get_wind() const { return vw_; }
  Eigen::Vector3d get_imu_accel() const;
  Eigen::Vector3d get_imu_gyro() const;
  
  // States and RK4 Workspace
  State x_, x2_, x3_, x4_;
  ErrorState dx_, k1_, k2_, k3_, k4_;

  // Parameters
  bool RK4_;
  double mass_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  double drag_constant_;
  double angular_drag_;
  double max_thrust_;
  Vector6d imu_;
  Eigen::Vector3d p_b2u_; // Body to IMU translation
  Quatd q_b2u_; // Body to IMU rotation

  bool wind_enabled_;
  double vw_walk_stdev_;
  Eigen::Vector3d vw_; // Wind velocity
  
  bool noise_enabled_;
  Matrix12d Qsqrt_;

  std::default_random_engine rng_;
  std::normal_distribution<double> standard_normal_dist_;
};
}

