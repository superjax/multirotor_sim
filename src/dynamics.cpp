#include "dynamics.h"
#include "simulator.h"

namespace dynamics
{
Dynamics::Dynamics() {}


void Dynamics::load(std::string filename)
{
  get_yaml_node("mass", filename, mass_);
  get_yaml_node("drag_constant", filename, drag_constant_);
  get_yaml_node("max_thrust", filename, max_thrust_);
  get_yaml_node("angular_drag_constant", filename, angular_drag_);
  get_yaml_node("RK4", filename, RK4_);

  // Initialize wind and its random walk/noise parameters
  double vw_init_var, vw_walk_stdev;
  get_yaml_node("wind_init_stdev", filename, vw_init_var);
  get_yaml_node("wind_walk_stdev", filename, vw_walk_stdev_);
  get_yaml_node("enable_wind", filename, wind_enabled_);
  vw_ = vw_init_var * Eigen::Vector3d::Random();
  if (!wind_enabled_)
    vw_.setZero();

  int seed;
  get_yaml_node("random_seed", filename, seed);
  if (seed < 0)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  rng_ = std::default_random_engine(seed);

  Vector3d inertia_diag;
  get_yaml_eigen<xVector>("x0", filename, x_);
  if (get_yaml_eigen<Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
}

void Dynamics::f(const xVector &x, const commandVector &u, dxVector &dx)
{
  Quatd q_i2b = Quatd(x.segment<4>(QW));
  Eigen::Vector3d v_rel_ = x.segment<3>(VX) - q_i2b.rotp(vw_); // Vehicle air velocity
  dx.segment<3>(PX) = q_i2b.rota(x.segment<3>(VX));
  dx.segment<3>(VX) = -1.0 * e_z * u(THRUST)*max_thrust_ / mass_ - drag_constant_ * v_rel_ +
                      q_i2b.rotp(gravity_) - x.segment<3>(WX).cross(x.segment<3>(VX));
//  dx.segment<3>(VX) = -1.0 * e_z * u(THRUST)*max_thrust_ / mass_ - drag_constant_ * M_ * v_rel_.cwiseProduct(v_rel_) +
//          q_i2b.rotp(gravity_) - x.segment<3>(WX).cross(x.segment<3>(VX));
  dx.segment<3>(DQX) = x.segment<3>(WX);
  dx.segment<3>(DWX) = inertia_inv_ * (u.segment<3>(TAUX) - x.segment<3>(WX).cross(inertia_matrix_ * x.segment<3>(WX)) -
                       angular_drag_ * x.segment<3>(WX).cwiseProduct(x.segment<3>(WX)));
}

void Dynamics::run(const double dt, const commandVector &u)
{
  if (RK4_)
  {
    // 4th order Runge-Kutta integration
    f(x_, u, k1_);

    x2_ = x_;
    x2_.segment<6>(PX) += k1_.segment<6>(PX) * dt / 2;
    x2_.segment<4>(QW) = (Quatd(x2_.segment<4>(QW)) + (k1_.segment<3>(DQX)) * dt / 2).elements();
    x2_.segment<3>(WX) += k1_.segment<3>(DWX) * dt / 2;
    f(x2_, u, k2_);

    x3_ = x_;
    x3_.segment<6>(PX) += k2_.segment<6>(PX) * dt / 2;
    x3_.segment<4>(QW) = (Quatd(x3_.segment<4>(QW)) + (k2_.segment<3>(DQX)) * dt / 2).elements();
    x3_.segment<3>(WX) += k2_.segment<3>(DWX) * dt / 2;
    f(x3_, u, k3_);

    x4_ = x_;
    x4_.segment<6>(PX) += k3_.segment<6>(PX) * dt;
    x4_.segment<4>(QW) = (Quatd(x4_.segment<4>(QW)) + (k3_.segment<3>(DQX)) * dt).elements();
    x4_.segment<3>(WX) += k3_.segment<3>(DWX) * dt;
    f(x4_, u, k4_);

    dx_ = (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6.0;
  }
  else
  {
    // Euler integration
    f(x_, u, dx_);
    dx_ = dx_ * dt;
  }

  // Copy output
  x_.segment<6>(PX) += dx_.segment<6>(PX);
  x_.segment<4>(QW) = (Quatd(x_.segment<4>(QW)) + dx_.segment<3>(DQX)).elements();
  x_.segment<3>(WX) += dx_.segment<3>(DWX);

  // Update wind velocity for next iteration
  if (wind_enabled_)
  {
    Eigen::Vector3d vw_walk;
    random_normal_vec(vw_walk, vw_walk_stdev_, standard_normal_dist_, rng_);
    vw_ += vw_walk * dt;
  }
}

Vector3d Dynamics::get_imu_accel() const
{
  return x_.segment<3>(AX) - Quatd(x_.segment<4>(QW)).rotp(gravity_) + x_.segment<3>(WX).cross(x_.segment<3>(VX));
}

void Dynamics::compute_accel(const commandVector &u)
{
  f(x_, u, dx_);
  x_.segment<3>(AX) = dx_.segment<3>(VX);
}

}
