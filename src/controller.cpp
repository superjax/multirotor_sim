#include <stdio.h>

#include "controller.h"
#include "simulator.h"



namespace controller 
{

Controller::Controller() :
  prev_time_(0),
  initialized_(false)
{
  dhat_.setZero();
}

void Controller::computeControl(const dynamics::xVector &x, const double t, dynamics::commandVector& u)
{ 
  // Copy the current state
  Vector3d euler = Quatd(x.segment<4>(dynamics::QW)).euler();
  xhat_.pn = x(dynamics::PX);
  xhat_.pe = x(dynamics::PY);
  xhat_.pd = x(dynamics::PZ);
  xhat_.u = x(dynamics::VX);
  xhat_.v = x(dynamics::VY);
  xhat_.w = x(dynamics::VZ);
  xhat_.phi = euler(0);
  xhat_.theta = euler(1);
  xhat_.psi = euler(2);
  xhat_.p = x(dynamics::WX);
  xhat_.q = x(dynamics::WY);
  xhat_.r = x(dynamics::WZ); 
  
  xc_.t = t;
  
  // Refresh the waypoint
  if (path_type_ < 2)
    updateWaypointManager();
  if (path_type_ == 2)
    updateTrajectoryManager();
  
  double dt = t - prev_time_;
  prev_time_ = t;
  
  if (dt < 0.0001)
  {
    u.setZero();
    return;
  }
  
  // get data that applies to both position and velocity control  
  Eigen::Matrix3d R_v_to_v1 = frame_helper::R_v_to_v1(euler(2)); // rotation from vehicle to vehicle-1 frame
  Eigen::Matrix3d R_v1_to_b = frame_helper::R_v_to_b(euler(0), euler(1),0); // rotation from vehicle-1 to body frame
  static Eigen::Vector3d k(0,0,1); // general unit vector in z-direction
  static double gravity = 9.80665; // m/s^2
  
  // Get velocity and yaw rate commands based on waypoints or define them directly
  if (path_type_ < 3)
  {
    Eigen::Vector3d phat(xhat_.pn, xhat_.pe, xhat_.pd); // position estimate
    Eigen::Vector3d pc(xc_.pn, xc_.pe, xc_.pd); // position command
    Vector3d vc = R_v_to_v1 * K_p_ * (pc-phat); // velocity command

    // enforce max velocity
    double vmag = vc.norm();
    if (vmag > max_.vel)
      vc *= max_.vel/vmag;

    // store velocity command
    xc_.u = vc(0);
    xc_.v = vc(1);
    xc_.w = vc(2);

    // get yaw rate direction and allow it to saturate
    xc_.r = xc_.psi - xhat_.psi;
    if (xc_.r > M_PI)
      xc_.r -= 2*M_PI;
    if (xc_.r < -M_PI)
      xc_.r += 2*M_PI;
    xc_.r = sat(xc_.r, max_.yaw_rate, -max_.yaw_rate);
  }
  else
  {
    // Constant forward velocity and constant altitude
    xc_.u += traj_heading_walk_ * udist_(rng_) * dt - ((xc_.u - vmag_) *traj_heading_straight_gain_);
    xc_.v += traj_heading_walk_ * udist_(rng_) * dt - (xc_.v *traj_heading_straight_gain_);
    xc_.w = K_p_(2,2) * (xc_.pd - xhat_.pd);

    // Wandering yaw rate
//    xc_.psi += traj_heading_walk_ * udist_(rng_) * dt;
//    xc_.r = xc_.psi - xhat_.psi;
    xc_.r += traj_heading_walk_ * udist_(rng_) * dt - (xc_.r *traj_heading_straight_gain_);
    if (xc_.r > M_PI)
      xc_.r -= 2*M_PI;
    if (xc_.r < -M_PI)
      xc_.r += 2*M_PI;
    xc_.r = sat(xc_.r, max_.yaw_rate, -max_.yaw_rate);
  }

  Eigen::Vector3d vc(xc_.u, xc_.v, xc_.w);
  Eigen::Vector3d vhat_b(xhat_.u,xhat_.v,xhat_.w); // body velocity estimate
  Eigen::Vector3d vhat = R_v1_to_b.transpose()*vhat_b; // vehicle-1 velocity estimate
  dhat_ = dhat_ - K_d_*(vc-vhat)*dt; // update disturbance estimate
  Eigen::Vector3d k_tilde = throttle_eq_ * (k - (1.0 / gravity) * (K_v_ * (vc - vhat) - dhat_));
  
  // pack up throttle command
  xc_.throttle = k.transpose() * R_v1_to_b * k_tilde;
  xc_.throttle = sat(xc_.throttle, max_.throttle, 0.001);
  
  Eigen::Vector3d kd = (1.0 / xc_.throttle) * k_tilde; // desired body z direction
  kd = kd / kd.norm(); // need direction only
  double kTkd = k.transpose() * kd;
  double tilt_angle;
  if (fabs(kTkd - 1.0) > 1.0e-6)
    tilt_angle = acos(kTkd); // desired tilt
  else
    tilt_angle = 0;
  
  // get shortest rotation to desired tilt
  Quatd q_c;
  if (tilt_angle < 1e-6)
  {
    q_c = Quatd::Identity();
  }
  else
  {
    Eigen::Vector3d k_cross_kd = k.cross(kd);
    q_c = Quatd::exp(tilt_angle * k_cross_kd / k_cross_kd.norm());
  }
  
  // pack up attitude commands
  xc_.phi = sat(q_c.roll(), max_.roll, -max_.roll);
  xc_.theta = sat(q_c.pitch(), max_.pitch, -max_.pitch);
  
  // Calculate the Final Output Torques using PID
  u(dynamics::THRUST) = xc_.throttle;
  u(dynamics::TAUX) = roll_.run(dt, xhat_.phi, xc_.phi, false, xhat_.p);
  u(dynamics::TAUY) = pitch_.run(dt, xhat_.theta, xc_.theta, false, xhat_.q);
  u(dynamics::TAUZ) = yaw_rate_.run(dt, xhat_.r, xc_.r, false);
}

Controller::PID::PID() :
  kp_(0.0f),
  ki_(0.0f),
  kd_(0.0f),
  max_(1.0f),
  integrator_(0.0f),
  differentiator_(0.0f),
  prev_x_(0.0f),
  tau_(0.05)
{}

void Controller::PID::init(float kp, float ki, float kd, float max, float min, float tau)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  max_ = max;
  tau_ = tau;
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator)
{
  float xdot;
  if (dt > 0.0001f)
  {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
    // The dirty derivative is a sort of low-pass filtered version of the derivative.
    //// (Include reference to Dr. Beard's notes here)
    differentiator_ = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
        + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
    xdot = differentiator_;
  }
  else
  {
    xdot = 0.0f;
  }
  prev_x_ = x;

  return run(dt, x, x_c, update_integrator, xdot);
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator, float xdot)
{
  // Calculate Error
  float error = x_c - x;

  // Initialize Terms
  float p_term = error * kp_;
  float i_term = 0.0f;
  float d_term = 0.0f;

  // If there is a derivative term
  if (kd_ > 0.0f)
  {
    d_term = kd_ * xdot;
  }

  //If there is an integrator term and we are updating integrators
  if ((ki_ > 0.0f) && update_integrator)
  {
    // integrate
    integrator_ += error * dt;
    // calculate I term
    i_term = ki_ * integrator_;
  }

  // sum three terms
  float u = p_term - d_term + i_term;

  // Integrator anti-windup
  float u_sat = (u > max_) ? max_ : (u < -1.0 * max_) ? -1.0 * max_ : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > 0.0f)
    integrator_ = (u_sat - p_term + d_term)/ki_;

  // Set output
  return u_sat;
}

void Controller::load(const std::string filename)
{
  if(file_exists(filename))
  {
    // Random number generation
    int seed;
    get_yaml_node("random_seed", filename, seed);
    if (seed < 0)
      seed = std::chrono::system_clock::now().time_since_epoch().count();
    rng_ = std::default_random_engine(seed);
    udist_ = std::uniform_real_distribution<double>(-1.0, 1.0);

    get_yaml_node("path_type", filename, path_type_);
    int num_waypoints;
    if (path_type_ == 0)
    {
      std::vector<double> loaded_wps;
      if (get_yaml_node("waypoints", filename, loaded_wps))
      {
        num_waypoints = std::floor(loaded_wps.size()/4.0);
        waypoints_ = Map<MatrixXd>(loaded_wps.data(), 4, num_waypoints);
      }
    }
    else if (path_type_ == 1)
    {
      // Load random waypoint parameters
      double random_heading_bound, altitude, alt_var, wp_sep, wp_var;
      dynamics::xVector x0;
      get_yaml_node("heading_walk", filename, random_heading_bound);
      get_yaml_node("altitude", filename, altitude);
      get_yaml_node("altitude_variance", filename, alt_var);
      get_yaml_node("waypoint_separation", filename, wp_sep);
      get_yaml_node("waypoint_sep_variance", filename, wp_var);
      get_yaml_eigen("x0", filename, x0); // need initial horizontal position

      // Get number of waypoints to create and initialize array and heading
      get_yaml_node("num_random_waypoints", filename, num_waypoints);
      waypoints_.setZero(4, num_waypoints);

      // For each waypoint, compute horizontal position components based on previous heading
      for (int i = 0; i < waypoints_.cols(); ++i)
      {
        // Get heading and position of previous waypoint
        double pn, pe, psi;
        if (i == 0)
        {
          pn = x0(PX);
          pe = x0(PY);
          psi = 0;
        }
        else
        {
          pn = waypoints_(0,i-1);
          pe = waypoints_(1,i-1);
          psi = waypoints_(3,i-1);
        }

        // Step position forward from previous position along heading direction of previous waypoint
        double step_size = wp_sep + wp_var * (udist_(rng_) + 1.0) / 2.0;
        waypoints_(0,i) = pn + step_size * cos(psi);
        waypoints_(1,i) = pe + step_size * sin(psi);
        waypoints_(2,i) = altitude + alt_var * udist_(rng_);
        waypoints_(3,i) = psi + random_heading_bound * udist_(rng_);
      }
    }
    else if (path_type_ == 2)
    {
      double traj_north_period, traj_east_period, traj_alt_period, traj_yaw_period;
      get_yaml_node("traj_delta_north", filename, traj_delta_north_);
      get_yaml_node("traj_delta_east", filename, traj_delta_east_);
      get_yaml_node("traj_delta_alt", filename, traj_delta_alt_);
      get_yaml_node("traj_delta_yaw", filename, traj_delta_yaw_);
      get_yaml_node("traj_nom_north", filename, traj_nom_north_);
      get_yaml_node("traj_nom_east", filename, traj_nom_east_);
      get_yaml_node("traj_nom_alt", filename, traj_nom_alt_);
      get_yaml_node("traj_nom_yaw", filename, traj_nom_yaw_);
      get_yaml_node("traj_north_period", filename, traj_north_period);
      get_yaml_node("traj_east_period", filename, traj_east_period);
      get_yaml_node("traj_alt_period", filename, traj_alt_period);
      get_yaml_node("traj_yaw_period", filename, traj_yaw_period);
      traj_north_freq_ = 2.0 * M_PI / traj_north_period;
      traj_east_freq_ = 2.0 * M_PI / traj_east_period;
      traj_alt_freq_ = 2.0 * M_PI / traj_alt_period;
      traj_yaw_freq_ = 2.0 * M_PI / traj_yaw_period;
    }
    else if (path_type_ == 3)
    {
      // Load constant velocity magnitude and yaw rate walk parameters
      get_yaml_node("traj_altitude", filename, xc_.pd);
      get_yaml_node("velocity_magnitude", filename, vmag_);
      get_yaml_node("traj_heading_walk", filename, traj_heading_walk_);
      get_yaml_node("traj_heading_straight_gain", filename, traj_heading_straight_gain_);
      xc_.psi = 0;
    }
    else
    {
      std::stringstream err;
      err << "\n\tFile: " << __FILE__ << "\n\tLine: " << __LINE__;
      err << "\n\tMessage: You specified an invalid path type";
      throw std::runtime_error(err.str());
    }
    
    get_yaml_eigen("Kp", filename, K_p_);
    get_yaml_eigen("Kd", filename, K_d_);
    get_yaml_eigen("Kv", filename, K_v_);
    
    get_yaml_node("throttle_eq", filename, throttle_eq_);
    get_yaml_node("mass", filename, mass_);
    get_yaml_node("max_thrust", filename, max_thrust_);
    get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
    get_yaml_node("waypoint_velocity_threshold", filename, waypoint_velocity_threshold_);
    get_yaml_node("drag_constant", filename, drag_constant_);
    
    get_yaml_node("roll_kp", filename, roll_.kp_);
    get_yaml_node("roll_ki", filename, roll_.ki_);
    get_yaml_node("roll_kd", filename, roll_.kd_);
    get_yaml_node("pitch_kp", filename, pitch_.kp_);
    get_yaml_node("pitch_ki", filename, pitch_.ki_);
    get_yaml_node("pitch_kd", filename, pitch_.kd_);
    get_yaml_node("yaw_rate_kp", filename, yaw_rate_.kp_);
    get_yaml_node("yaw_rate_ki", filename, yaw_rate_.ki_);
    get_yaml_node("yaw_rate_kd", filename, yaw_rate_.kd_);
    get_yaml_node("max_tau_x", filename, roll_.max_);
    get_yaml_node("max_tau_y", filename, pitch_.max_);
    get_yaml_node("max_tau_z", filename, yaw_rate_.max_);
    get_yaml_node("max_roll", filename, max_.roll);
    get_yaml_node("max_pitch", filename, max_.pitch);
    get_yaml_node("max_yaw_rate", filename, max_.yaw_rate);
    get_yaml_node("max_throttle", filename, max_.throttle);
    get_yaml_node("max_vel", filename, max_.vel);
  }
  else
    printf("Unable to find file %s\n", (current_working_dir() + filename).c_str());
  
}

void Controller::updateWaypointManager()
{
  if (!initialized_)
  {
    initialized_ = true;
    Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, 0).data());
    xc_.pn = new_waypoint(PX);
    xc_.pe = new_waypoint(PY);
    xc_.pd = new_waypoint(PZ);
    xc_.psi = new_waypoint(PSI);
    current_waypoint_id_ = 0;
  }
    
  // Find the distance to the desired waypoint
  Vector4d current_waypoint = waypoints_.block<4,1>(0, current_waypoint_id_);
  Vector4d error;
  error(PX) = current_waypoint(PX) - xhat_.pn;
  error(PY) = current_waypoint(PY) - xhat_.pe;
  error(PZ) = current_waypoint(PZ) - xhat_.pd;
  error(PSI) = current_waypoint(PSI) - xhat_.psi;
  
  // Angle wrapping on heading
  if (error(PSI) > M_PI)
    error(PSI) -= 2.0 * M_PI;
  else if (error(PSI) < -M_PI)
    error(PSI) += 2.0 * M_PI;
  
  Vector3d current_velocity(xhat_.u, xhat_.v, xhat_.w);
  
  if (error.norm() < waypoint_threshold_ && current_velocity.norm() < waypoint_velocity_threshold_)
  {    
    // increment waypoint
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();   
    
    // Update The commanded State
    Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, current_waypoint_id_).data());
    xc_.pn = new_waypoint(PX);
    xc_.pe = new_waypoint(PY);
    xc_.pd = new_waypoint(PZ);
    xc_.psi = new_waypoint(PSI);
  }  
}

void Controller::updateTrajectoryManager()
{
  xc_.pn = traj_nom_north_ + traj_delta_north_ / 2.0 * cos(traj_north_freq_ * xc_.t);
  xc_.pe = traj_nom_east_ + traj_delta_east_ / 2.0 * sin(traj_east_freq_ * xc_.t);
  xc_.pd = -(traj_nom_alt_ + traj_delta_alt_ / 2.0 * sin(traj_alt_freq_ * xc_.t));
  xc_.psi = traj_nom_yaw_ + traj_delta_yaw_ / 2.0 * sin(traj_yaw_freq_ * xc_.t);
}

}
