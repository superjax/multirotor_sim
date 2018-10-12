#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "utils.h"
#include "geometry/quat.h"
#include "geometry/xform.h"
#include "geometry/support.h"

using namespace quat;
using namespace xform;

namespace dynamics {


typedef Eigen::Matrix<double, 16, 1> xVector;
typedef Eigen::Matrix<double, 12, 1> dxVector;
typedef Eigen::Matrix<double, 4, 1> commandVector;

// State Indexes
enum {
  PX = 0,
  PY = 1,
  PZ = 2,
  VX = 3,
  VY = 4,
  VZ = 5,
  QW = 6,
  QX = 7,
  QY = 8,
  QZ = 9,
  WX = 10,
  WY = 11,
  WZ = 12,
  AX = 13,
  AY = 14,
  AZ = 15,
  
  DQX = 6, // Attitude derivative indexes
  DWX = 9
};

// Input indexes
enum {
  THRUST,
  TAUX,
  TAUY,
  TAUZ
};


static const Vector3d gravity_ = [] {
  Vector3d tmp;
  tmp << 0, 0, 9.80665;
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
  void run(const double dt, const commandVector& u);
  
  void f(const xVector& x, const commandVector& u, dxVector& dx);
  
  const xVector& get_state() const { return x_; }
  Xformd get_global_pose() const { return Xformd(x_.segment<3>(PX), Quatd(x_.segment<4>(QW))); }
  const double& get_drag() const { return drag_constant_; }
  const Eigen::Vector3d& get_wind() const { return vw_; }
  Vector3d get_imu_accel() const;
  void compute_accel(const commandVector& u);
  
private:
  // States and RK4 Workspace
  xVector x_, x2_, x3_, x4_;
  dxVector dx_, k1_, k2_, k3_, k4_;

  // Parameters
  bool RK4_;
  double mass_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  double drag_constant_;
  double angular_drag_;
  double max_thrust_;

  bool wind_enabled_;
  double vw_walk_stdev_;
  Eigen::Vector3d vw_; // Wind velocity
  std::default_random_engine rng_;
  std::normal_distribution<double> standard_normal_dist_;
};
}

