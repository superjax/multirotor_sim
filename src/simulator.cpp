#include <chrono>
#include <iomanip>

#include <Eigen/StdVector>

#include "multirotor_sim/simulator.h"
#include "multirotor_sim/estimator_base.h"
#include "multirotor_sim/controller.h"

using namespace std;
using namespace gnss_utils;
using namespace Eigen;

namespace  multirotor_sim
{


Simulator::Simulator(bool prog_indicator, uint64_t seed) :
  seed_(seed == 0 ? std::chrono::system_clock::now().time_since_epoch().count() : seed),
  env_(seed_),
  rng_(seed_),
  uniform_(0.0, 1.0),
  prog_indicator_(prog_indicator),
  t_round_off_(1e7)
{
  cont_ = static_cast<ControllerBase*>(&ref_con_);
  traj_ = static_cast<TrajectoryBase*>(&ref_con_);
  srand(seed_);
}

Simulator::~Simulator()
{
  if (prog_indicator_)
    cout << endl;

  if (log_.is_open())
    log_.close();
}


void Simulator::load(string filename)
{
  param_filename_ = filename;
  t_ = 0;
  get_yaml_node("tmax", filename, tmax_);
  get_yaml_node("dt", filename, dt_);
  get_yaml_node("seed", filename, seed_);
  if (seed_ == 0)
    seed_ = std::chrono::system_clock::now().time_since_epoch().count();
  rng_ = default_random_engine(seed_);
  srand(seed_);

  // Log
  get_yaml_node("log_filename", filename, log_filename_);
  if (!log_filename_.empty())
  {
    log_.open(log_filename_);
  }

  // Initialize Desired sensors
  get_yaml_node("imu_enabled", filename, imu_enabled_);
  get_yaml_node("alt_enabled", filename, alt_enabled_);
  get_yaml_node("baro_enabled", filename, baro_enabled_);
  get_yaml_node("mocap_enabled", filename, mocap_enabled_);
  get_yaml_node("vo_enabled", filename, vo_enabled_);
  get_yaml_node("camera_enabled", filename, camera_enabled_);
  get_yaml_node("gnss_enabled", filename, gnss_enabled_);
  get_yaml_node("raw_gnss_enabled", filename, raw_gnss_enabled_);

  init_platform();

  if (imu_enabled_)
    init_imu();
  if (camera_enabled_)
    init_camera();
  if (alt_enabled_)
    init_altimeter();
  if (baro_enabled_)
    init_baro();
  if (vo_enabled_)
    init_vo();
  if (mocap_enabled_)
    init_mocap();
  if (gnss_enabled_)
    init_gnss();
  if (raw_gnss_enabled_)
    init_raw_gnss();

  // Load sub-class parameters
  if (camera_enabled_)
    env_.load(filename);
  dyn_.load(filename);
  ref_con_.load(filename);

  // Start Progress Bar
  if (prog_indicator_)
    prog_.init(std::round(tmax_/dt_), 40);

  // start at hover throttle
  u_(multirotor_sim::THRUST) = dyn_.mass_ / dyn_.max_thrust_ * multirotor_sim::G;
}


bool Simulator::run()
{
  if (t_ < tmax_ - dt_ / 2.0) // Subtract half time step to prevent occasional extra iteration
  {
    // Propagate forward in time and get new control input and true acceleration
    t_ += dt_;
    traj_->getCommandedState(t_, xc_, ur_);
    cont_->computeControl(t_, dyn_.get_state(), xc_, ur_, u_);
    dyn_.run(dt_, compute_low_level_control(u_));
    if (prog_indicator_)
      prog_.print(t_/dt_, t_);
    update_measurements();
    return true;
  }
  else
  {
    if (prog_indicator_)
      prog_.finished();
    return false;
  }
}

void Simulator::init_platform()
{
    get_yaml_node("max_thrust", param_filename_, max_thrust_);
    get_yaml_eigen("max_torque", param_filename_, max_torque_);
    get_yaml_eigen("kp_w", param_filename_, kp_w_);
    get_yaml_eigen("kd_w", param_filename_, kd_w_);
    w_err_prev_ = Vector3d::Zero();
}

void Simulator::init_imu()
{
  // Load IMU parameters
  Vector4d q_b_u;
  get_yaml_node("imu_update_rate", param_filename_, imu_update_rate_);
  get_yaml_eigen("p_b_u", param_filename_, p_b2u_);
  get_yaml_eigen("q_b_u", param_filename_, q_b_u);
  q_b2u_ = Quatd(q_b_u);

  // Accelerometer
  bool use_accel_truth;
  double accel_noise, accel_walk, accel_init;
  get_yaml_node("use_accel_truth", param_filename_, use_accel_truth);
  get_yaml_node("accel_init_stdev", param_filename_, accel_init);
  get_yaml_node("accel_noise_stdev", param_filename_, accel_noise);
  get_yaml_node("accel_bias_walk", param_filename_, accel_walk);
  accel_bias_ =  accel_init * Vector3d::Random() * !use_accel_truth; // Uniformly random init within +-accel_walk
  accel_noise_stdev_ = !use_accel_truth * accel_noise;
  accel_walk_stdev_ = !use_accel_truth * accel_walk;

  // Gyro
  bool use_gyro_truth;
  double gyro_noise, gyro_walk, gyro_init;
  get_yaml_node("use_gyro_truth", param_filename_, use_gyro_truth);
  get_yaml_node("gyro_noise_stdev", param_filename_, gyro_noise);
  get_yaml_node("gyro_init_stdev", param_filename_, gyro_init);
  get_yaml_node("gyro_bias_walk", param_filename_, gyro_walk);
  gyro_bias_ = gyro_init * Vector3d::Random()  * !use_gyro_truth; // Uniformly random init within +-gyro_walk
  gyro_noise_stdev_ = gyro_noise * !use_gyro_truth;
  gyro_walk_stdev_ = gyro_walk * !use_gyro_truth;

  imu_R_.setZero();
  imu_R_.topLeftCorner<3,3>() = accel_noise * accel_noise * I_3x3;
  imu_R_.bottomRightCorner<3,3>() = gyro_noise * gyro_noise * I_3x3;
  last_imu_update_ = 0.0;
}


void Simulator::init_camera()
{
  // Camera
  bool use_camera_truth;
  double pixel_noise;
  Vector2d focal_len;
  get_yaml_node("num_features", param_filename_, num_features_);
  get_yaml_node("camera_time_offset", param_filename_, camera_time_offset_);
  get_yaml_node("camera_transmission_time", param_filename_, camera_transmission_time_);
  get_yaml_node("camera_transmission_noise", param_filename_, camera_transmission_noise_);
  get_yaml_node("use_camera_truth", param_filename_, use_camera_truth);
  get_yaml_node("camera_update_rate", param_filename_, camera_update_rate_);
  get_yaml_eigen("cam_center", param_filename_, cam_.cam_center_);
  get_yaml_eigen("image_size", param_filename_, cam_.image_size_);
  get_yaml_eigen("q_b_c", param_filename_, x_b2c_.q_.arr_);
  get_yaml_eigen("p_b_c", param_filename_, x_b2c_.t_);
  get_yaml_eigen("focal_len", param_filename_, cam_.focal_len_);
  get_yaml_node("pixel_noise_stdev", param_filename_, pixel_noise);
  get_yaml_node("loop_closure", param_filename_, loop_closure_);
  pixel_noise_stdev_ = !use_camera_truth * pixel_noise;

  // Depth
  double depth_noise;
  bool use_depth_truth;
  get_yaml_node("use_depth_truth", param_filename_, use_depth_truth);
  get_yaml_node("depth_update_rate", param_filename_, depth_update_rate_);
  get_yaml_node("depth_noise_stdev", param_filename_, depth_noise);
  depth_noise_stdev_ = depth_noise * !use_depth_truth;

  image_id_ = 0;
  next_feature_id_ = 0;
  last_camera_update_ = 0.0;
  feat_R_ = pixel_noise * pixel_noise * I_2x2;
  depth_R_ << depth_noise * depth_noise;

  tracked_points_.reserve(num_features_);
  img_.reserve(num_features_);
}


void Simulator::init_altimeter()
{
  bool use_altimeter_truth;
  double altimeter_noise;
  get_yaml_node("use_altimeter_truth", param_filename_, use_altimeter_truth);
  get_yaml_node("altimeter_update_rate", param_filename_, altimeter_update_rate_);
  get_yaml_node("altimeter_noise_stdev", param_filename_, altimeter_noise);
  altimeter_noise_stdev_ = altimeter_noise * !use_altimeter_truth;
  alt_R_ << altimeter_noise * altimeter_noise;
  last_altimeter_update_ = 0.0;
}

void Simulator::init_baro()
{
  bool use_baro_truth;
  double baro_noise, baro_walk, baro_init;
  Vector3d refLla;
  get_yaml_eigen("ref_LLA", param_filename_, refLla);
  get_yaml_node("use_baro_truth", param_filename_, use_baro_truth);
  get_yaml_node("baro_update_rate", param_filename_, baro_update_rate_);
  get_yaml_node("baro_noise_stdev", param_filename_, baro_noise);
  get_yaml_node("baro_init_stdev", param_filename_, baro_init);
  get_yaml_node("baro_bias_walk", param_filename_, baro_walk);
  alt0_ = refLla(2);
  baro_noise_stdev_ = baro_noise * !use_baro_truth;
  baro_bias_walk_stdev_ = baro_walk * !use_baro_truth;
  baro_bias_ = normal_(rng_) * baro_bias_walk_stdev_;
  baro_R_ << baro_noise * baro_noise;
  last_baro_update_ = 0.0;
}


void Simulator::init_vo()
{
  double vo_translation_noise, vo_rotation_noise;
  X_I2bk_ = dyn_.get_global_pose();
  bool use_vo_truth;
  get_yaml_node("use_vo_truth", param_filename_, use_vo_truth);
  get_yaml_node("vo_delta_position", param_filename_, vo_delta_position_);
  get_yaml_node("vo_delta_attitude", param_filename_, vo_delta_attitude_);
  get_yaml_node("vo_translation_noise_stdev", param_filename_, vo_translation_noise);
  get_yaml_node("vo_rotation_noise_stdev", param_filename_, vo_rotation_noise);
  vo_translation_noise_stdev_ = vo_translation_noise * !use_vo_truth;
  vo_rotation_noise_stdev_ = vo_rotation_noise * !use_vo_truth;

  vo_R_.setIdentity();
  vo_R_.block<3,3>(0,0) *= vo_translation_noise * vo_translation_noise;
  vo_R_.block<3,3>(3,3) *= vo_rotation_noise * vo_rotation_noise;
}


void Simulator::init_mocap()
{
  // Truth
  double att_noise, pos_noise;
  bool use_mocap_truth;
  get_yaml_node("mocap_update_rate", param_filename_, mocap_update_rate_);
  get_yaml_node("use_mocap_truth", param_filename_, use_mocap_truth);
  get_yaml_node("attitude_noise_stdev", param_filename_, att_noise);
  get_yaml_node("position_noise_stdev", param_filename_, pos_noise);
  get_yaml_node("mocap_time_offset", param_filename_, mocap_time_offset_);
  get_yaml_node("mocap_transmission_noise", param_filename_, mocap_transmission_noise_);
  get_yaml_node("mocap_transmission_time", param_filename_, mocap_transmission_time_);
  get_yaml_eigen("p_b_m", param_filename_, p_b2m_);
  get_yaml_eigen("q_b_m", param_filename_, q_b2m_.arr_);
  attitude_noise_stdev_ = att_noise * !use_mocap_truth;
  position_noise_stdev_ = pos_noise * !use_mocap_truth;

  mocap_R_ << pos_noise * pos_noise * I_3x3,
              Matrix3d::Zero(),
              Matrix3d::Zero(),
              att_noise * att_noise * I_3x3;

  last_mocap_update_ = 0.0;
  next_mocap_measurement_ = 0.0;
}


void Simulator::init_gnss()
{
  // gnss
  Vector3d refLla;
  bool use_gnss_truth;
  double gnss_pos_noise_h, gnss_pos_noise_v, gnss_vel_noise;
  get_yaml_eigen("ref_LLA", param_filename_, refLla);
  X_e2n_ = WGS84::x_ecef2ned(WGS84::lla2ecef(refLla));
  get_yaml_eigen("p_b2g", param_filename_, p_b2g_);
  get_yaml_node("gnss_update_rate", param_filename_, gnss_update_rate_);
  get_yaml_node("use_gnss_truth", param_filename_, use_gnss_truth);
  get_yaml_node("gnss_horizontal_position_stdev", param_filename_, gnss_pos_noise_h);
  get_yaml_node("gnss_vertical_position_stdev", param_filename_, gnss_pos_noise_v);
  get_yaml_node("gnss_velocity_stdev", param_filename_, gnss_vel_noise);
  get_yaml_eigen("p_b2g", param_filename_, p_b2g_);
  gnss_horizontal_position_stdev_ = gnss_pos_noise_h * !use_gnss_truth;
  gnss_vertical_position_stdev_ = gnss_pos_noise_v * !use_gnss_truth;
  gnss_velocity_stdev_ = gnss_vel_noise * !use_gnss_truth;

  gnss_R_.setIdentity();
  gnss_R_.block<2,2>(0,0) *= gnss_pos_noise_h*gnss_pos_noise_h;
  gnss_R_(2,2) *= gnss_pos_noise_v*gnss_pos_noise_v;
  gnss_R_.block<3,3>(3,3) *= gnss_vel_noise*gnss_vel_noise;
  auto gnss_pos_block = gnss_R_.block<3,3>(0,0);
  auto gnss_vel_block = gnss_R_.block<3,3>(3,3);
  gnss_pos_block = X_e2n_.q().R().transpose() * gnss_pos_block * X_e2n_.q().R();
  gnss_vel_block = X_e2n_.q().R().transpose() * gnss_vel_block * X_e2n_.q().R();

  last_gnss_update_ = 0.0;
}

void Simulator::init_raw_gnss()
{
  Vector3d refLla;
  bool use_raw_gnss_truth;
  std::vector<double> area;
  double pseudorange_noise, p_rate_noise, cp_noise, clock_walk;
  get_yaml_eigen("ref_LLA", param_filename_, refLla);
  get_yaml_eigen("p_b2g", param_filename_, p_b2g_);
  get_yaml_node("gnss_update_rate", param_filename_, gnss_update_rate_);
  get_yaml_node("use_raw_gnss_truth", param_filename_, use_raw_gnss_truth);
  get_yaml_node("use_raw_gnss_truth", param_filename_, use_raw_gnss_truth);
  get_yaml_node("pseudorange_stdev", param_filename_, pseudorange_noise);
  get_yaml_node("pseudorange_rate_stdev", param_filename_, p_rate_noise);
  get_yaml_node("carrier_phase_stdev", param_filename_, cp_noise);
  get_yaml_node("ephemeris_filename", param_filename_, ephemeris_filename_);
  get_yaml_node("clock_init_stdev", param_filename_, clock_init_stdev_);
  get_yaml_node("clock_walk_stdev", param_filename_, clock_walk);
  get_yaml_node("start_time_week", param_filename_, start_time_.week);
  get_yaml_node("start_time_tow_sec", param_filename_, start_time_.tow_sec);
  get_yaml_node("multipath_prob", param_filename_, multipath_prob_);
  get_yaml_node("multipath_area", param_filename_, area);
  multipath_area_.x.min = area[0];
  multipath_area_.x.max = area[1];
  multipath_area_.y.min = area[2];
  multipath_area_.y.max = area[3];
  get_yaml_node("gps_denied_area", param_filename_, area);
  gps_denied_area_.x.min = area[0];
  gps_denied_area_.x.max = area[1];
  gps_denied_area_.y.min = area[2];
  gps_denied_area_.y.max = area[3];
  get_yaml_node("cycle_slip_prob", param_filename_, cycle_slip_prob_);
  get_yaml_node("multipath_error_range", param_filename_, multipath_error_range_);
  X_e2n_ = WGS84::x_ecef2ned(WGS84::lla2ecef(refLla));
  pseudorange_stdev_ = pseudorange_noise * !use_raw_gnss_truth;
  pseudorange_rate_stdev_ = p_rate_noise * !use_raw_gnss_truth;
  carrier_phase_stdev_ = cp_noise * !use_raw_gnss_truth;
  clock_walk_stdev_ = clock_walk * !use_raw_gnss_truth;

  for (int i = 0; i < 100; i++)
  {
    Satellite sat(i, satellites_.size());
    sat.readFromRawFile(ephemeris_filename_);
    if (sat.eph_.A > 0)
    {
      satellites_.push_back(sat);
      carrier_phase_integer_offsets_.push_back(use_raw_gnss_truth ? 0 : round(uniform_(rng_) * 100) - 50);
    }
  }
  multipath_offset_.resize(satellites_.size());
  for (int i = 0; i < multipath_offset_.size(); i++)
  {
      multipath_offset_[i] = 0.0;
  }

  raw_gnss_R_ = Vector3d{pseudorange_noise*pseudorange_noise,
                         p_rate_noise*p_rate_noise,
                         cp_noise*cp_noise}.asDiagonal();

  clock_bias_ = uniform_(rng_) * clock_init_stdev_;
  clock_bias_rate_ = uniform_(rng_)*clock_walk_stdev_;
  last_raw_gnss_update_ = 0.0;
}

void Simulator::register_estimator(EstimatorBase *est)
{
  est_.push_back(est);
}

void Simulator::use_custom_controller(ControllerBase *cont)
{
  cont_ = cont;
}

void Simulator::use_custom_trajectory(TrajectoryBase *traj)
{
  traj_ = traj;
}

void Simulator::update_camera_pose()
{
  x_I2c_ = state().X * x_b2c_;
}


void Simulator::update_imu_meas()
{
  double dt = t_ - last_imu_update_;
  if (std::round(dt * t_round_off_) / t_round_off_ >= 1.0/imu_update_rate_)
  {
    last_imu_update_ = t_;

    // Bias random walks and IMU noise
    accel_bias_ += randomNormal<Vector3d>(accel_walk_stdev_, normal_, rng_) * dt;
    gyro_bias_ += randomNormal<Vector3d>(gyro_walk_stdev_, normal_, rng_) * dt;

    // Populate accelerometer and gyro measurements
    Vector6d imu;
    imu.segment<3>(0) = dyn_.get_imu_accel() + accel_bias_ + randomNormal<Vector3d>(accel_noise_stdev_, normal_,  rng_);
    imu.segment<3>(3) = dyn_.get_imu_gyro() + gyro_bias_ + randomNormal<Vector3d>(gyro_noise_stdev_, normal_,  rng_);;

    for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
      (*it)->imuCallback(t_, imu, imu_R_);
  }
}


void Simulator::update_camera_meas()
{
  // If it's time to capture new measurements, then do it
  if (std::round((t_ - last_camera_update_) * t_round_off_) / t_round_off_ >= 1.0/camera_update_rate_)
  {
    last_camera_update_ = t_;
    update_camera_pose();

    double pub_time = t_ + std::max(camera_transmission_time_ + normal_(rng_) * camera_transmission_noise_, 0.0);

    // Update feature measurements for currently tracked features
    for(auto it = tracked_points_.begin(); it != tracked_points_.end();)
    {
      if (update_feature(*it))
      {
        measurement_t meas;
        meas.t = t_ + camera_time_offset_;
        meas.z = it->pixel + randomNormal<Vector2d>(pixel_noise_stdev_, normal_, rng_);
        meas.R = feat_R_;
        meas.feature_id = (*it).id;
        meas.depth = it->depth + depth_noise_stdev_ * normal_(rng_);
        camera_measurements_buffer_.push_back(std::pair<double, measurement_t>{pub_time, meas});
        DBG("update feature - ID = %d\n", it->id);
        it++;
      }
      else
      {
        if (it->zeta(2,0) < 0)
        {
          DBG("clearing feature - ID = %d because went negative [%f, %f, %f]\n",
              it->id, it->zeta(0,0), it->zeta(1,0), it->zeta(2,0));
        }
        else if ((it->pixel.array() < 0).any() || (it->pixel.array() > cam_.image_size_.array()).any())
        {
          DBG("clearing feature - ID = %d because went out of frame [%f, %f]\n",
              it->id, it->pixel(0,0), it->pixel(1,0));
        }
        tracked_points_.erase(it);
      }
    }

    while (tracked_points_.size() < num_features_)
    {
      // Add the new feature to our "tracker"
      feature_t new_feature;
      if (!get_feature_in_frame(new_feature, loop_closure_))
        break;
      tracked_points_.push_back(new_feature);
      DBG("new feature - ID = %d [%f, %f, %f], [%f, %f]\n",
          new_feature.id, new_feature.zeta(0,0), new_feature.zeta(1,0),
          new_feature.zeta(2,0), new_feature.pixel(0,0), new_feature.pixel(1,0));

      // Create a measurement for this new feature
      measurement_t meas;
      meas.t = t_ + camera_time_offset_;
      meas.z = new_feature.pixel + randomNormal<Vector2d>(pixel_noise_stdev_, normal_, rng_);
      meas.R = feat_R_;
      meas.feature_id = new_feature.id;
      meas.depth = new_feature.depth + depth_noise_stdev_ * normal_(rng_);
      camera_measurements_buffer_.push_back(std::pair<double, measurement_t>{pub_time, meas});
    }
  }

  // Push out the measurement if it is time to send it
  if (camera_measurements_buffer_.size() > 0 && camera_measurements_buffer_[0].first >= t_)
  {
    // Populate the Image class with all feature measurements
    img_.clear();
    img_.t = camera_measurements_buffer_[0].second.t;
    img_.id = image_id_;
    for (auto zit = camera_measurements_buffer_.begin(); zit != camera_measurements_buffer_.end(); zit++)
    {
      img_.pixs.push_back(zit->second.z);
      img_.feat_ids.push_back(zit->second.feature_id);
      img_.depths.push_back(zit->second.depth);
    }

    for (estVec::iterator eit = est_.begin(); eit != est_.end(); eit++)
        (*eit)->imageCallback(t_, img_, feat_R_, depth_R_);

    camera_measurements_buffer_.clear();
    ++image_id_;
  }
}


void Simulator::update_alt_meas()
{
  if (std::round((t_ - last_altimeter_update_) * t_round_off_) / t_round_off_ >= 1.0/altimeter_update_rate_)
  {
    Vector1d z_alt;
    z_alt << -1.0 * state().p.z() + altimeter_noise_stdev_ * normal_(rng_);

    last_altimeter_update_ = t_;
    for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
      (*it)->altCallback(t_, z_alt, alt_R_);
  }
}

void Simulator::update_baro_meas()
{
  if (std::round((t_ - last_baro_update_) * t_round_off_) / t_round_off_ >= 1.0/baro_update_rate_)
  {
    double dt = t_ - last_baro_update_;
    baro_bias_ += dt * normal_(rng_)*baro_bias_walk_stdev_;

    double alt = -1.0 * state().p.z() + alt0_;
    double pa = 101325.0f*(float)pow((1-2.25694e-5 * alt), 5.2553);

    Vector1d z_baro;
    z_baro << pa + baro_noise_stdev_ * normal_(rng_) + baro_bias_;

    last_baro_update_ = t_;
    for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
        (*it)->baroCallback(t_, z_baro, baro_R_);
  }
}


void Simulator::update_mocap_meas()
{
  if (std::round((t_ - last_mocap_update_) * t_round_off_) / t_round_off_ >= 1.0/mocap_update_rate_)
  {
    measurement_t meas;
    meas.t = t_ + mocap_time_offset_;
    meas.z.resize(7,1);

    // Add noise to mocap measurements and transform into mocap coordinate frame
    Vector3d noise = randomNormal<Vector3d>(position_noise_stdev_, normal_, rng_);
    Vector3d I_p_b_I = state().p; // p_{b/I}^I
    Vector3d I_p_m_I = I_p_b_I + state().q.rota(p_b2m_); // p_{m/I}^I = p_{b/I}^I + R(q_I^b)^T (p_{m/b}^b)
    meas.z.topRows<3>() = I_p_m_I + noise;

    noise = randomNormal<Vector3d>(attitude_noise_stdev_, normal_, rng_);
    Quatd q_I_m = state().q * q_b2m_; //  q_I^m = q_I^b * q_b^m
    meas.z.bottomRows<4>() = (q_I_m + noise).elements();

    meas.R = mocap_R_;

    double pub_time = std::max(mocap_transmission_time_ + normal_(rng_) * mocap_transmission_noise_, 0.0) + t_;

    mocap_measurement_buffer_.push_back(std::pair<double, measurement_t>{pub_time, meas});
    last_mocap_update_ = t_;
  }

  while (mocap_measurement_buffer_.size() > 0 && mocap_measurement_buffer_[0].first >= t_)
  {
    measurement_t* m = &(mocap_measurement_buffer_[0].second);
    if (mocap_enabled_)
    {
      for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
        (*it)->mocapCallback(m->t, Xformd(m->z), m->R);
    }
    mocap_measurement_buffer_.erase(mocap_measurement_buffer_.begin());
  }
}


void Simulator::update_vo_meas()
{
  Xformd T_i2b = dyn_.get_global_pose();
  Vector6d delta = T_i2b - X_I2bk_;
  if (delta.segment<3>(0).norm() >= vo_delta_position_ || delta.segment<3>(3).norm() >= vo_delta_attitude_)
  {
    // Compute position and attitude relative to the keyframe
    Xformd T_c2ck;
    T_c2ck.t_ = x_b2c_.rotp(T_i2b.q().rotp(X_I2bk_.t() + X_I2bk_.q().inverse().rotp(x_b2c_.t()) -
                                           (T_i2b.t() + T_i2b.q().inverse().rotp(x_b2c_.t()))));
    T_c2ck.q_ = x_b2c_.q_.inverse() * T_i2b.q().inverse() * X_I2bk_.q().inverse() * x_b2c_.q_;

    for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
      (*it)->voCallback(t_, T_c2ck, vo_R_);

    // Set new keyframe to current pose
    X_I2bk_ = dyn_.get_global_pose();
  }
}

void Simulator::update_gnss_meas()
{
  /// TODO: Simulate gnss sensor delay
  if (std::round((t_ - last_gnss_update_) * t_round_off_) / t_round_off_ >= 1.0/gnss_update_rate_)
  {
    last_gnss_update_ = t_;
    /// TODO: Simulate the random walk associated with gnss position
    Vector3d p_NED = dyn_.get_global_pose().t();
    p_NED.segment<2>(0) += gnss_horizontal_position_stdev_ * randomNormal<Vector2d>(normal_, rng_);
    p_NED(2) += gnss_vertical_position_stdev_ * normal_(rng_);
    Vector3d p_ECEF = WGS84::ned2ecef(X_e2n_, p_NED);

    Vector3d v_NED = dyn_.get_global_pose().q().rota(dyn_.get_state().v);
    Vector3d v_ECEF = X_e2n_.q().rota(v_NED);
    v_ECEF += gnss_velocity_stdev_ * randomNormal<Vector3d>(normal_, rng_);

    Vector6d z;
    z << p_ECEF, v_ECEF;

    for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
      (*it)->gnssCallback(t_, z, gnss_R_);
  }
}

void Simulator::update_raw_gnss_meas()
{
  /// TODO: Simulator gnss sensor delay
  double dt = t_ - last_raw_gnss_update_;
  if (std::round(dt * t_round_off_) / t_round_off_ >= 1.0/gnss_update_rate_)
  {
    last_raw_gnss_update_ = t_;
    clock_bias_rate_ += normal_(rng_) * clock_walk_stdev_ * dt;
    clock_bias_ += clock_bias_rate_ * dt;

    GTime t_now = t_ + start_time_;
    Vector3d p_ECEF = get_gps_position_ecef();
    Vector3d v_ECEF = get_gps_velocity_ecef();
    Vector3d p_ned = get_gps_position_ned();

    if (gps_denied_area_.inside(p_ned))
        return;

    VecVec3 z;
    VecMat3 R;
    int i;
    vector<Satellite, aligned_allocator<Satellite>>::iterator sat;
    vector<bool> slip(satellites_.size(), false);
    for (i = 0, sat = satellites_.begin(); sat != satellites_.end(); sat++, i++)
    {
      if (normal_(rng_) * dt_ < cycle_slip_prob_)
      {
        slip[i] = true;
        carrier_phase_integer_offsets_[i] = round(uniform_(rng_) * 100) - 50;
      }

      if (multipath_area_.inside(p_ned))
      {
          if (multipath_offset_[i] > 0)
          {
              if (uniform_(rng_) < (multipath_prob_ + (0.7 * (1.0 - multipath_prob_)))* 1.0/gnss_update_rate_ )
              {
                  multipath_offset_[i] = 0;
              }
          }
          else if ((uniform_(rng_)  < multipath_prob_ * 1.0/gnss_update_rate_))
          {
              multipath_offset_[i] = uniform_(rng_) * multipath_error_range_;
          }
      }
      else
      {
          multipath_offset_[i] = 0;
      }

      Vector3d z_i;
      sat->computeMeasurement(t_now, p_ECEF, v_ECEF, Vector2d{clock_bias_, clock_bias_rate_}, z_i);
      z_i(0) += normal_(rng_) * pseudorange_stdev_+ multipath_offset_[i];
      z_i(1) += normal_(rng_) * pseudorange_rate_stdev_;
      z_i(2) += normal_(rng_) * carrier_phase_stdev_ + carrier_phase_integer_offsets_[i];
      z.push_back(z_i);
      R.push_back(raw_gnss_R_);
    }

    for (estVec::iterator it = est_.begin(); it != est_.end(); it++)
      (*it)->rawGnssCallback(t_now, z, R, satellites_, slip);
  }
}


void Simulator::update_measurements()
{
  if (imu_enabled_)
    update_imu_meas();
  if (camera_enabled_)
    update_camera_meas();
  if (alt_enabled_)
    update_alt_meas();
  if (mocap_enabled_)
    update_mocap_meas();
  if (vo_enabled_)
    update_vo_meas();
  if (gnss_enabled_)
    update_gnss_meas();
  if (raw_gnss_enabled_)
    update_raw_gnss_meas();
}


bool Simulator::update_feature(feature_t &feature) const
{
  if (feature.id > env_.get_points().size() || feature.id < 0)
    return false;

  // Calculate the bearing vector to the feature
  Vector3d pt = env_.get_points()[feature.id];
  feature.zeta = x_I2c_.transformp(pt);

  // we can reject anything behind the camera
  if (feature.zeta(2) < 0.0)
    return false;

  feature.depth = feature.zeta.norm();
  feature.zeta /= feature.depth;

  // See if the pixel is in the camera frame
  cam_.proj(feature.zeta, feature.pixel);
  if ((feature.pixel.array() < 0).any() || (feature.pixel.array() > cam_.image_size_.array()).any())
    return false;
  else
    return true;
}

bool Simulator::get_previously_tracked_feature_in_frame(feature_t &feature)
{

  Vector3d ground_pt;
  env_.get_center_img_center_on_ground_plane(x_I2c_, ground_pt);
  vector<Vector3d, aligned_allocator<Vector3d>> pts;
  vector<size_t> ids;
  if (env_.get_closest_points(ground_pt, num_features_, 2.0, pts, ids))
  {
    for (int i = 0; i < pts.size(); i++)
    {
      if (is_feature_tracked(ids[i]))
        continue;
      // Calculate the bearing vector to the feature
      Vector3d pt = env_.get_points()[ids[i]];
      feature.zeta = x_I2c_.transformp(pt);
      if (feature.zeta(2) < 0.0)
        continue;

      feature.depth = feature.zeta.norm();
      feature.zeta /= feature.depth;
      cam_.proj(feature.zeta, feature.pixel);
      if ((feature.pixel.array() < 0).any() || (feature.pixel.array() > cam_.image_size_.array()).any())
        continue;
      else
      {
        feature.id = ids[i];
        return true;
      }
    }
  }
  return false;
}

bool Simulator::get_feature_in_frame(feature_t &feature, bool retrack)
{
  if (retrack && get_previously_tracked_feature_in_frame(feature))
  {
    return true;
  }
  else
  {
    return create_new_feature_in_frame(feature);
  }
}

bool Simulator::create_new_feature_in_frame(feature_t &feature)
{
  // First, look for features in frame that are not currently being tracked
  feature.id = env_.add_point(x_I2c_.t_, x_I2c_.q_, feature.zeta, feature.pixel, feature.depth);
  if (feature.id != -1)
  {
    feature.id = next_feature_id_++;
    return true;
  }
  else
  {
    //    cout << "\nGround Plane is not in camera frame " << endl;
    return false;
  }
}

bool Simulator::is_feature_tracked(int id) const
{
  auto it = tracked_points_.begin();
  while (it < tracked_points_.end() && it->id != id)
  {
    it++;
  }
  return it != tracked_points_.end();
}

Vector3d Simulator::get_position_ecef() const
{
  return WGS84::ned2ecef(X_e2n_, dyn_.get_state().p);
}

Vector3d Simulator::get_velocity_ecef() const
{
  Vector3d v_NED = dyn_.get_global_pose().q().rota(dyn_.get_state().v);
  return X_e2n_.q().rota(v_NED);
}

Vector3d Simulator::get_gps_position_ned() const
{
    return state().p + state().q.rota(p_b2g_);
}

Vector3d Simulator::get_gps_position_ecef() const
{
    return WGS84::ned2ecef(X_e2n_, get_gps_position_ned());
}

Vector3d Simulator::get_gps_velocity_ecef() const
{
    Vector3d v_NED = dyn_.get_global_pose().q().rota(state().v + state().w.cross(p_b2g_));
    return X_e2n_.q().rota(v_NED);
}

inline static double sat(double x, double max, double min)
{
    return x > max ? max : x < min ? min : x;
}

inline static Vector3d sat(const Vector3d& x, const Vector3d& max, const Vector3d& min)
{
    Vector3d out;
    for (int i = 0; i < 3; i++)
        out(i) = x(i) > max(i) ? max(i) : x(i) < min(i) ? min(i) : x(i);
    return out;
}


// u = {F[0-1], Wx(rad/s), Wy, Wz}
Vector4d Simulator::compute_low_level_control(const Vector4d& u)
{
    Vector4d forces_and_torques;
    forces_and_torques(THRUST) = sat(u(THRUST)*max_thrust_, max_thrust_, 0.0);
    Vector3d w_err = u.segment<3>(WX) - state().w;
    Vector3d p_term = kp_w_.cwiseProduct(w_err);
    Vector3d d_term = kd_w_.cwiseProduct(w_err - w_err_prev_)/dt_;
    w_err_prev_ = w_err;

    forces_and_torques.segment<3>(TAUX) = sat(p_term - d_term, max_torque_, -1.0*max_torque_);
    return forces_and_torques;
}

}

