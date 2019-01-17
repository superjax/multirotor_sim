#include "dynamics.h"
#include "simulator.h"

namespace multirotor_sim
{
Dynamics::Dynamics() {}


void Dynamics::load(std::string filename)
{
  Vector4d q_b_u;
  get_yaml_node("mass", filename, mass_);
  get_yaml_node("drag_constant", filename, drag_constant_);
  get_yaml_node("max_thrust", filename, max_thrust_);
  get_yaml_node("angular_drag_constant", filename, angular_drag_);
  get_yaml_node("RK4", filename, RK4_);
  get_yaml_eigen("p_b_u", filename, p_b_u_);
  get_yaml_eigen("q_b_u", filename, q_b_u);
  q_b_u_ = Quatd(q_b_u);

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
  get_yaml_eigen("x0", filename, x_.arr);
  if (get_yaml_eigen<Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
}

void Dynamics::f(const State &x, const Vector4d &u, ErrorState &dx)
{
  Eigen::Vector3d v_rel_ = x.v - x.q.rotp(vw_); // Vehicle air velocity
  dx.p = x.q.rota(x.v);
  dx.v = -1.0 * e_z * u(THRUST)*max_thrust_ / mass_ - drag_constant_ * v_rel_ + x.q.rotp(gravity_) - x.w.cross(x.v);
  dx.q = x.w;
  dx.w = inertia_inv_ * (u.segment<3>(TAUX) - x.w.cross(inertia_matrix_ * x.w) - angular_drag_ * x.w.cwiseProduct(x.w));
}

void Dynamics::run(const double dt, const Vector4d &u)
{
  if (RK4_)
  {
    // 4th order Runge-Kutta integration
    f(x_, u, k1_);

    x2_ = x_;
    k1_.arr *= dt*0.5;
    x2_ += k1_;
    f(x2_, u, k2_);

    x3_ = x_;
    k2_.arr *= dt *0.5;
    x3_ += k2_;
    f(x3_, u, k3_);

    x4_ = x_;
    k3_.arr *= dt;
    x4_ += k3_;
    f(x4_, u, k4_);

    dx_.arr = (k1_.arr + 2 * k2_.arr + 2 * k3_.arr + k4_.arr) * dt / 6.0;
  }
  else
  {
    // Euler integration
    f(x_, u, dx_);
    dx_.arr *= dt;
  }

  // Copy output
  x_ += dx_;

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
  return imu_.segment<3>(ACC);
}

Vector3d Dynamics::get_imu_gyro() const
{
  return imu_.segment<3>(GYRO);
}

void Dynamics::compute_imu(const Vector4d &u)
{
  f(x_, u, dx_);
  imu_.segment<3>(ACC) = q_b_u_.rotp(dx_.v + x_.w.cross(x_.v) + x_.w.cross(x_.w.cross(p_b_u_)) + dx_.w.cross(p_b_u_) - x_.q.rotp(gravity_));
  imu_.segment<3>(GYRO) = q_b_u_.rotp(x_.w);
}

}
