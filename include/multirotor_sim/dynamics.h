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
  THRUST,
  TAUX,
  TAUY,
  TAUZ,
  INPUT_SIZE
};

// IMU indices
enum {
  ACC = 0,
  GYRO = 3
};

static const double G = 9.80665;

static const Vector3d gravity_ = [] {
  Vector3d tmp;
  tmp << 0, 0, G;
  return tmp;
}();

static const Matrix3d M_ = [] {
  Matrix3d tmp;
  tmp << 1, 0, 0, 0, 1, 0, 0, 0, 0;
  return tmp;
}();

class Dynamics
{      
public:
  Dynamics();
  
  void load(std::string filename);
  void run(const double dt, const Vector4d& u);
  
  void f(const State& x, const Vector4d& u, ErrorState& dx);
  void f(const State& x, const Vector4d& u, ErrorState& dx, Vector6d& imu);
  
  const State& get_state() const { return x_; }
  void set_state(const State& x) { x_ = x; }

  const Xformd& get_global_pose() const { return x_.X; }
  const double& get_drag() const { return drag_constant_; }
  const Eigen::Vector3d& get_wind() const { return vw_; }
  Vector3d get_imu_accel() const;
  Vector3d get_imu_gyro() const;
  
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
  Vector3d p_b_u_; // Body to IMU translation
  Quatd q_b_u_; // Body to IMU rotation

  bool wind_enabled_;
  double vw_walk_stdev_;
  Eigen::Vector3d vw_; // Wind velocity
  std::default_random_engine rng_;
  std::normal_distribution<double> standard_normal_dist_;
};
}

