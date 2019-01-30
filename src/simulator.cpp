#include "simulator.h"
#include <Eigen/StdVector>
#include <chrono>

#include "multirotor_sim/estimator_base.h"

using namespace std;

namespace  multirotor_sim
{


Simulator::Simulator(ControllerBase &_cont, TrajectoryBase& _traj) :
  cont_(_cont),
  traj_(_traj),
  seed_(std::chrono::system_clock::now().time_since_epoch().count()),
  env_(seed_),
  rng_(seed_),
  uniform_(0.0, 1.0)
{
  srand(seed_);
}

Simulator::Simulator(ControllerBase &_cont, TrajectoryBase& _traj, bool prog_indicator) :
  cont_(_cont),
  traj_(_traj),
  seed_(std::chrono::system_clock::now().time_since_epoch().count()),
  env_(seed_),
  rng_(seed_),
  uniform_(0.0, 1.0),
  prog_indicator_(prog_indicator)
{
  uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  rng_ = std::default_random_engine(seed);
  srand(seed);
}


Simulator::Simulator(ControllerBase &_cont, TrajectoryBase& _traj, bool prog_indicator, uint64_t seed):
    cont_(_cont),
    traj_(_traj),
    seed_(seed),
    env_(seed_),
    rng_(seed_),
    uniform_(0.0, 1.0),
    prog_indicator_(prog_indicator)
{
  srand(seed);
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
  if (seed_ > 1)
  {
    rng_ = default_random_engine(seed_);
    srand(seed_);
  }

  // Log
  get_yaml_node("log_filename", filename, log_filename_);
  if (!log_filename_.empty())
  {
    log_.open(log_filename_);
  }

  // Initialize Desired sensors
  get_yaml_node("imu_enabled", filename, imu_enabled_);
  get_yaml_node("alt_enabled", filename, alt_enabled_);
  get_yaml_node("att_enabled", filename, att_enabled_);
  get_yaml_node("pos_enabled", filename, pos_enabled_);
  get_yaml_node("vo_enabled", filename, vo_enabled_);
  get_yaml_node("features_enabled", filename, features_enabled_);
  get_yaml_node("gnss_enabled", filename, gnss_enabled_);
  get_yaml_node("raw_gnss_enabled", filename, raw_gnss_enabled_);

  if (imu_enabled_)
    init_imu();
  if (features_enabled_)
    init_camera();
  if (alt_enabled_)
    init_altimeter();
  if (vo_enabled_)
    init_vo();
  if (pos_enabled_ || att_enabled_)
    init_truth();
  if (gnss_enabled_)
    init_gnss();
  if (raw_gnss_enabled_)
      init_raw_gnss();

  // Load sub-class parameters
  if (features_enabled_)
    env_.load(filename);
  dyn_.load(filename);

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
    cont_.computeControl(t_, dyn_.get_state(), traj_.getCommandedState(t_), u_);
    dyn_.run(dt_, u_);
    if (prog_indicator_)
        prog_.print(t_/dt_);
    update_measurements();

    log_state();
    return true;
  }
  else
  {
    if (prog_indicator_)
        prog_.finished();
    return false;
  }
}


void Simulator::init_imu()
{
    // Load IMU parameters
    Vector4d q_b_u;
    get_yaml_node("imu_update_rate", param_filename_, imu_update_rate_);
    get_yaml_eigen("p_b_u", param_filename_, p_b_u_);
    get_yaml_eigen("q_b_u", param_filename_, q_b_u);
    q_b_u_ = Quatd(q_b_u);

    // Accelerometer
    double accel_noise, accel_walk, accel_init;
    get_yaml_node("use_accel_truth", param_filename_, use_accel_truth_);
    get_yaml_node("accel_init_stdev", param_filename_, accel_init);
    get_yaml_node("accel_noise_stdev", param_filename_, accel_noise);
    get_yaml_node("accel_bias_walk", param_filename_, accel_walk);
    accel_bias_ =  !use_accel_truth_ * accel_init * Vector3d::Random(); // Uniformly random init within +-accel_walk
    accel_noise_stdev_ = !use_accel_truth_ * accel_noise;
    accel_walk_stdev_ = !use_accel_truth_ * accel_walk;
    accel_noise_.setZero();

    // Gyro
    double gyro_noise, gyro_walk, gyro_init;
    get_yaml_node("use_gyro_truth", param_filename_, use_gyro_truth_);
    get_yaml_node("gyro_noise_stdev", param_filename_, gyro_noise);
    get_yaml_node("gyro_init_stdev", param_filename_, gyro_init);
    get_yaml_node("gyro_bias_walk", param_filename_, gyro_walk);
    gyro_bias_ = gyro_init * Vector3d::Random()  * !use_gyro_truth_; // Uniformly random init within +-gyro_walk
    gyro_noise_stdev_ = gyro_noise * !use_gyro_truth_;
    gyro_walk_stdev_ = gyro_walk * !use_gyro_truth_;
    gyro_noise_.setZero();

    imu_R_.topLeftCorner<3,3>() = accel_noise * accel_noise * I_3x3;
    imu_R_.bottomRightCorner<3,3>() = gyro_noise * gyro_noise * I_3x3;
    imu_R_.bottomLeftCorner<3,3>().setZero();
    imu_R_.topRightCorner<3,3>().setZero();
    last_imu_update_ = 0.0;
}


void Simulator::init_camera()
{
    // Camera
    double pixel_noise;
    Vector2d focal_len;
    get_yaml_node("camera_time_delay", param_filename_, camera_time_delay_);
    get_yaml_node("use_camera_truth", param_filename_, use_camera_truth_);
    get_yaml_node("camera_update_rate", param_filename_, camera_update_rate_);
    get_yaml_eigen("cam_center", param_filename_, cam_center_);
    get_yaml_eigen("image_size", param_filename_, image_size_);
    get_yaml_eigen("q_b_c", param_filename_, q_b_c_.arr_);
    get_yaml_eigen("p_b_c", param_filename_, p_b_c_);
    get_yaml_eigen("focal_len", param_filename_, focal_len);
    get_yaml_node("pixel_noise_stdev", param_filename_, pixel_noise);
    get_yaml_node("loop_closure", param_filename_, loop_closure_);
    pixel_noise_stdev_ = !use_camera_truth_ * pixel_noise;
    pixel_noise_.setZero();
    cam_F_ << focal_len(0,0), 0, 0, 0, focal_len(1,0), 0; // Copy focal length into 2x3 matrix for future use

    // Depth
    double depth_noise;
    get_yaml_node("use_depth_truth", param_filename_, use_depth_truth_);
    get_yaml_node("depth_update_rate", param_filename_, depth_update_rate_);
    get_yaml_node("depth_noise_stdev", param_filename_, depth_noise);
    get_yaml_node("feat_move_prob", param_filename_, feat_move_prob_);
    depth_noise_stdev_ = depth_noise * !use_depth_truth_;
    depth_noise_ = 0;

    next_feature_id_ = 0;
    last_camera_update_ = 0.0;
    feat_R_ = pixel_noise * pixel_noise * I_2x2;
    depth_R_ << depth_noise * depth_noise;
}


void Simulator::init_altimeter()
{
    // Altimeter
    double altimeter_noise;
    get_yaml_node("use_altimeter_truth", param_filename_, use_altimeter_truth_);
    get_yaml_node("altimeter_update_rate", param_filename_, altimeter_update_rate_);
    get_yaml_node("altimeter_noise_stdev", param_filename_, altimeter_noise);
    altimeter_noise_stdev_ = altimeter_noise * !use_altimeter_truth_;
    altimeter_noise_ = 0;
    alt_R_ << altimeter_noise * altimeter_noise;
    last_altimeter_update_ = 0.0;
}


void Simulator::init_vo()
{
    // Visual Odometry
    double vo_translation_noise, vo_rotation_noise;
    T_i2bk_ = dyn_.get_global_pose();
    get_yaml_node("use_vo_truth", param_filename_, use_vo_truth_);
    get_yaml_node("vo_delta_position", param_filename_, vo_delta_position_);
    get_yaml_node("vo_delta_attitude", param_filename_, vo_delta_attitude_);
    get_yaml_node("vo_translation_noise_stdev", param_filename_, vo_translation_noise);
    get_yaml_node("vo_rotation_noise_stdev", param_filename_, vo_rotation_noise);
    vo_translation_noise_stdev_ = vo_translation_noise * !use_vo_truth_;
    vo_rotation_noise_stdev_ = vo_rotation_noise * !use_vo_truth_;

    vo_R_.setIdentity();
    vo_R_.block<3,3>(0,0) *= vo_translation_noise * vo_translation_noise;
    vo_R_.block<3,3>(3,3) *= vo_rotation_noise * vo_rotation_noise;
}


void Simulator::init_truth()
{
    // Truth
    double att_noise, pos_noise;
    get_yaml_node("truth_update_rate", param_filename_, mocap_update_rate_);
    get_yaml_node("use_attitude_truth", param_filename_, use_attitude_truth_);
    get_yaml_node("use_position_truth", param_filename_, use_position_truth_);
    get_yaml_node("attitude_noise_stdev", param_filename_, att_noise);
    get_yaml_node("position_noise_stdev", param_filename_, pos_noise);
    get_yaml_node("truth_time_offset", param_filename_, mocap_time_offset_);
    get_yaml_node("truth_transmission_noise", param_filename_, mocap_transmission_noise_);
    get_yaml_node("truth_transmission_time", param_filename_, mocap_transmission_time_);
    get_yaml_eigen("p_b_m", param_filename_, p_b_m_);
    get_yaml_eigen("q_b_m", param_filename_, q_b_m_.arr_);
    attitude_noise_stdev_ = att_noise * !use_attitude_truth_;
    position_noise_stdev_ = pos_noise * !use_position_truth_;

    att_R_ = att_noise * att_noise * I_3x3;
    pos_R_ = pos_noise * pos_noise * I_3x3;

    last_truth_update_ = 0.0;
    next_truth_measurement_ = 0.0;
}


void Simulator::init_gnss()
{
    // gnss
    Vector3d refLla;
    double gnss_pos_noise_h, gnss_pos_noise_v, gnss_vel_noise;
    get_yaml_eigen("ref_LLA", param_filename_, refLla);
    x_e2n_ = WSG84::x_ecef2ned(WSG84::lla2ecef(refLla));
    get_yaml_node("gnss_update_rate", param_filename_, gnss_update_rate_);
    get_yaml_node("use_gnss_truth", param_filename_, use_gnss_truth_);
    get_yaml_node("gnss_horizontal_position_stdev", param_filename_, gnss_pos_noise_h);
    get_yaml_node("gnss_vertical_position_stdev", param_filename_, gnss_pos_noise_v);
    get_yaml_node("gnss_velocity_stdev", param_filename_, gnss_vel_noise);
    gnss_horizontal_position_stdev_ = gnss_pos_noise_h * !use_gnss_truth_;
    gnss_vertical_position_stdev_ = gnss_pos_noise_v * !use_gnss_truth_;
    gnss_velocity_stdev_ = gnss_vel_noise * !use_gnss_truth_;

    gnss_R_.setIdentity();
    gnss_R_.block<2,2>(0,0) *= gnss_pos_noise_h;
    gnss_R_(2,2) *= gnss_pos_noise_v;
    gnss_R_.block<3,3>(3,3) *= gnss_vel_noise;
    auto gnss_pos_block = gnss_R_.block<3,3>(0,0);
    auto gnss_vel_block = gnss_R_.block<3,3>(3,3);
    gnss_pos_block = x_e2n_.q().R().transpose() * gnss_pos_block * x_e2n_.q().R();
    gnss_vel_block = x_e2n_.q().R().transpose() * gnss_vel_block * x_e2n_.q().R();

    last_gnss_update_ = 0.0;
}

void Simulator::init_raw_gnss()
{
    Vector3d refLla;
    get_yaml_eigen("ref_LLA", param_filename_, refLla);
    x_e2n_ = WSG84::x_ecef2ned(WSG84::lla2ecef(refLla));
    double pseudorange_noise, p_rate_noise, cp_noise, clock_walk;
    get_yaml_node("gnss_update_rate", param_filename_, gnss_update_rate_);
    get_yaml_node("use_raw_gnss_truth", param_filename_, use_raw_gnss_truth_);
    get_yaml_node("use_raw_gnss_truth", param_filename_, use_raw_gnss_truth_);
    get_yaml_node("pseudorange_stdev", param_filename_, pseudorange_noise);
    get_yaml_node("pseudorange_rate_stdev", param_filename_, p_rate_noise);
    get_yaml_node("carrier_phase_stdev", param_filename_, cp_noise);
    get_yaml_node("ephemeris_filename", param_filename_, ephemeris_filename_);
    get_yaml_node("clock_init_stdev", param_filename_, clock_init_stdev_);
    get_yaml_node("clock_walk_stdev", param_filename_, clock_walk);
    get_yaml_node("start_time_week", param_filename_, start_time_.week);
    get_yaml_node("start_time_tow_sec", param_filename_, start_time_.tow_sec);
    pseudorange_stdev_ = pseudorange_noise * !use_raw_gnss_truth_;
    pseudorange_rate_stdev_ = p_rate_noise * !use_raw_gnss_truth_;
    carrier_phase_stdev_ = cp_noise * !use_raw_gnss_truth_;
    clock_walk_stdev_ = clock_walk * !use_gnss_truth_;

    for (int i = 0; i < 100; i++)
    {
        Satellite sat(i, satellites_.size());
        sat.readFromRawFile(ephemeris_filename_);
        if (sat.eph_.A > 0)
        {
            satellites_.push_back(sat);
            carrier_phase_integer_offsets_.push_back(use_raw_gnss_truth_ ? 0 : round(uniform_(rng_) * 100) - 50);
        }
    }

    raw_gnss_R_ = Vector3d{pseudorange_stdev_*pseudorange_stdev_,
                           pseudorange_rate_stdev_*pseudorange_rate_stdev_,
                           carrier_phase_stdev_*carrier_phase_stdev_}.asDiagonal();

    clock_bias_ = uniform_(rng_) * clock_init_stdev_;
    last_raw_gnss_update_ = 0.0;
}

void Simulator::register_estimator(EstimatorBase *est)
{
    est_.push_back(est);
}


void Simulator::log_state()
{
  if (log_.is_open())
  {
    log_.write((char*)&t_, sizeof(double));
    log_.write((char*)dyn_.get_state().arr.data(), sizeof(double)*State::SIZE);
  }
}


void Simulator::update_camera_pose()
{
  t_I_c_ = dyn_.get_state().p + dyn_.get_state().q.rota(p_b_c_);
  q_I_c_ = dyn_.get_state().q * q_b_c_;
}


void Simulator::update_imu_meas()
{
    if (std::abs(t_ - last_imu_update_) > 1.0/imu_update_rate_)
    {
      double dt = t_ - last_imu_update_;
      last_imu_update_ = t_;

      // Bias random walks and IMU noise
      if (!use_accel_truth_)
      {
        Vector3d accel_walk;
        random_normal_vec(accel_walk, accel_walk_stdev_, normal_, rng_);
        accel_bias_ += accel_walk * accel_walk_stdev_ * dt;
        random_normal_vec(accel_noise_, accel_noise_stdev_, normal_,  rng_);
      }
      if (!use_gyro_truth_)
      {
        Vector3d gyro_walk;
        random_normal_vec(gyro_walk, gyro_walk_stdev_, normal_,  rng_);
        gyro_bias_ += gyro_walk * dt;
        random_normal_vec(gyro_noise_, gyro_noise_stdev_, normal_,  rng_);
      }

      // Populate accelerometer and gyro measurements
      Vector6d imu;
      imu.segment<3>(0) = dyn_.get_imu_accel() + accel_bias_ + accel_noise_;
      imu.segment<3>(3) = dyn_.get_imu_gyro() + gyro_bias_ + gyro_noise_;

      for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
          (*it)->imuCallback(t_, imu, imu_R_);
    }
}


void Simulator::update_feature_meas()
{
    // If it's time to capture new measurements, then do it
    if (std::abs(t_ - last_camera_update_ - 1.0/camera_update_rate_) < 0.0005)
    {
      last_camera_update_ = t_;
      update_camera_pose();

      // Update feature measurements for currently tracked features
      for(auto it = tracked_points_.begin(); it != tracked_points_.end();)
      {
        if (update_feature(*it))
        {
          measurement_t meas;
          meas.t = t_;
          meas.type = FEAT;
          meas.z = get_pixel((*it));
          meas.R = feat_R_;
          meas.feature_id = (*it).id;
          meas.depth = get_depth((*it));
          camera_measurements_buffer_.push_back(meas);
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
          else if ((it->pixel.array() < 0).any() || (it->pixel.array() > image_size_.array()).any())
          {
            DBG("clearing feature - ID = %d because went out of frame [%f, %f]\n",
                it->id, it->pixel(0,0), it->pixel(1,0));
          }
          tracked_points_.erase(it);
        }
      }

      while (tracked_points_.size() < NUM_FEATURES)
      {
        // Add the new feature to our "tracker"
        feature_t new_feature;
        if (!get_feature_in_frame(new_feature, loop_closure_))
          break;
        tracked_points_.push_back(new_feature);
        DBG("new feature - ID = %d [%f, %f, %f], [%f, %f]\n",
            new_feature.id, new_feature.zeta(0,0), new_feature.zeta(1,0),
            new_feature.zeta(2,0), new_feature.pixel(0,0), new_feature.pixel(1,0));

        // Pixel noise
        if (!use_camera_truth_)
          random_normal_vec(pixel_noise_, pixel_noise_stdev_, normal_, rng_);

        // Create a measurement for this new feature
        measurement_t meas;
        meas.t = t_;
        meas.type = FEAT;
        meas.z = get_pixel(new_feature) + pixel_noise_;
        meas.R = feat_R_;
        meas.feature_id = new_feature.id;
        meas.depth = get_depth(new_feature);
        camera_measurements_buffer_.push_back(meas);
      }
    }

    // Push out the measurement if it is time to send it
    if ((t_ > last_camera_update_ + camera_time_delay_) && (camera_measurements_buffer_.size() > 0));
    {
      for (auto zit = camera_measurements_buffer_.begin(); zit != camera_measurements_buffer_.end(); zit++)
      {
          for (std::vector<EstimatorBase*>::iterator eit = est_.begin(); eit != est_.end(); eit++)
              (*eit)->featCallback(t_, zit->z, zit->R, zit->feature_id, zit->depth);
      }
      camera_measurements_buffer_.clear();
    }
}


void Simulator::update_alt_meas()
{
    if (std::abs(t_ - last_altimeter_update_ - 1.0/altimeter_update_rate_) < 0.0005)
    {
      // Altimeter noise
      if (!use_altimeter_truth_)
        altimeter_noise_ = altimeter_noise_stdev_ * normal_(rng_);
      Matrix<double, 1, 1> noise(altimeter_noise_);

      last_altimeter_update_ = t_;
      for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
          (*it)->altCallback(t_, get_altitude() + noise, alt_R_);
    }
}


void Simulator::update_mocap_meas()
{
    if (std::abs(t_ - last_truth_update_ - 1.0/mocap_update_rate_) < 0.0005)
    {
      measurement_t att_meas;
      att_meas.t = t_ - mocap_time_offset_;
      att_meas.type = ATT;
      att_meas.z = get_attitude();
      att_meas.R = att_R_;

      measurement_t pos_meas;
      pos_meas.t = t_ - mocap_time_offset_;
      pos_meas.type = POS;
      pos_meas.z = get_position();
      pos_meas.R = pos_R_;

      double pub_time = std::max(mocap_transmission_time_ + normal_(rng_) * mocap_transmission_noise_, 0.0) + t_;

      mocap_measurement_buffer_.push_back(std::pair<double, measurement_t>{pub_time, pos_meas});
      mocap_measurement_buffer_.push_back(std::pair<double, measurement_t>{pub_time, att_meas});
      last_truth_update_ = t_;
    }

    while (mocap_measurement_buffer_.size() > 0 && mocap_measurement_buffer_[0].first >= t_)
    {
      measurement_t* m = &(mocap_measurement_buffer_[0].second);
      if (pos_enabled_ && m->type == POS)
      {
          for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
              (*it)->posCallback(t_, m->z, m->R);
      }
      else if (att_enabled_ && m->type == ATT)
      {
          for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
              (*it)->attCallback(t_, Quatd(m->z), m->R);
      }
      mocap_measurement_buffer_.erase(mocap_measurement_buffer_.begin());
    }
}


void Simulator::update_vo_meas()
{
  Xformd T_i2b = dyn_.get_global_pose();
  Vector6d delta = T_i2b - T_i2bk_;
  if (delta.segment<3>(0).norm() >= vo_delta_position_ || delta.segment<3>(3).norm() >= vo_delta_attitude_)
  {
    // Compute position and attitude relative to the keyframe
    Xformd T_c2ck;
    T_c2ck.t_ = q_b_c_.rotp(T_i2b.q().rotp(T_i2bk_.t() + T_i2bk_.q().inverse().rotp(p_b_c_) -
                                          (T_i2b.t() + T_i2b.q().inverse().rotp(p_b_c_))));
    T_c2ck.q_ = q_b_c_.inverse() * T_i2b.q().inverse() * T_i2bk_.q().inverse() * q_b_c_;

    for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
        (*it)->voCallback(t_, T_c2ck, vo_R_);

    // Set new keyframe to current pose
    T_i2bk_ = dyn_.get_global_pose();
  }
}

void Simulator::update_gnss_meas()
{
    /// TODO: Simulate gnss sensor delay
    if (std::abs(t_ - last_gnss_update_) > 1.0/gnss_update_rate_)
    {
        last_gnss_update_ = t_;
        /// TODO: Simulate the random walk associated with gnss position
        Vector3d p_NED = dyn_.get_global_pose().t();
        p_NED.segment<2>(0) += gnss_horizontal_position_stdev_ * randomNormal<double, 2, 1>(normal_, rng_);
        p_NED(2) += gnss_vertical_position_stdev_ * normal_(rng_);
        Vector3d p_ECEF = WSG84::ned2ecef(x_e2n_, p_NED);

        Vector3d v_NED = dyn_.get_global_pose().q().rota(dyn_.get_state().v);
        Vector3d v_ECEF = x_e2n_.q().rota(v_NED);
        v_ECEF += gnss_velocity_stdev_ * randomNormal<double, 3, 1>(normal_, rng_);

        Vector6d z;
        z << p_ECEF, v_ECEF;

        for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
            (*it)->gnssCallback(t_, z, gnss_R_);
    }
}

void Simulator::update_raw_gnss_meas()
{
    /// TODO: Simulator gnss sensor delay
    if (std::abs(t_ - last_raw_gnss_update_) > 1.0/gnss_update_rate_)
    {
        double dt = t_ - last_raw_gnss_update_;
        last_raw_gnss_update_ = t_;
        clock_bias_rate_ += normal_(rng_) * clock_walk_stdev_ * dt;
        clock_bias_ += clock_bias_rate_ * dt;

        GTime t_now = t_ + start_time_;
        Vector3d p_ECEF = get_position_ecef();
        Vector3d v_ECEF = get_velocity_ecef();

        Vector3d z;
        int i;
        vector<Satellite>::iterator sat;
        for (i = 0, sat = satellites_.begin(); sat != satellites_.end(); sat++, i++)
        {
            sat->computeMeasurement(t_now, p_ECEF, v_ECEF, Vector2d{clock_bias_, clock_bias_rate_}, z);
            z(0) += normal_(rng_) * pseudorange_stdev_;
            z(1) += normal_(rng_) * pseudorange_rate_stdev_;
            z(2) += normal_(rng_) * carrier_phase_stdev_ + carrier_phase_integer_offsets_[i];
            for (std::vector<EstimatorBase*>::iterator it = est_.begin(); it != est_.end(); it++)
                (*it)->rawGnssCallback(t_now, z, raw_gnss_R_, *sat);
        }
    }
}


void Simulator::update_measurements()
{
  if (imu_enabled_)
    update_imu_meas();
  if (features_enabled_)
    update_feature_meas();
  if (alt_enabled_)
    update_alt_meas();
  if (pos_enabled_ || att_enabled_)
    update_mocap_meas();
  if (vo_enabled_)
    update_vo_meas();
  if (gnss_enabled_)
    update_gnss_meas();
  if (raw_gnss_enabled_)
    update_raw_gnss_meas();
}


Vector3d Simulator::get_vel() const
{
    Vector3d vel;
    vel = dyn_.get_state().v;
    return vel;
}


Xformd Simulator::get_pose() const
{
  return dyn_.get_state().X;
}


//void Simulator::get_features(const std::vector<int>& ids, ) const
//{
//  x = dyn_.get_state();
//  for (auto it = ids.begin(); it != ids.end(); it++)
//  {
//    int i = *it;
//    if (i < 0)
//    {
//      continue;
//      // This happens if the feature hasn't been added yet (because of the camera time offset)
//      // This usually happens when no features are in the camera's field of view
////      std::stringstream err;
////      err << "File: " << __FILE__ << ", Line: " << __LINE__;
////      err << "\nEKF feature not found in truth - cannot compare";
////      throw std::runtime_error(err.str());
//    }

//    int xZETA = 5*i;
//    int xRHO = 5*i + 4;
//    xf.block<4,1>(xZETA,0) = Quatd::from_two_unit_vectors(e_z, tracked_points_[i].zeta).elements();
//    x(xRHO,0) =1.0/ tracked_points_[i].depth;
//  }
//}


bool Simulator::update_feature(feature_t &feature) const
{
  if (feature.id > env_.get_points().size() || feature.id < 0)
    return false;
  
  // Calculate the bearing vector to the feature
  Vector3d pt = env_.get_points()[feature.id];
  feature.zeta = q_I_c_.rotp(pt - t_I_c_);
  feature.zeta /= feature.zeta.norm();
  feature.depth = (env_.get_points()[feature.id] - t_I_c_).norm();
  
  // we can reject anything behind the camera
  if (feature.zeta(2) < 0.0)
    return false;
  
  // See if the pixel is in the camera frame
  proj(feature.zeta, feature.pixel);
  if ((feature.pixel.array() < 0).any() || (feature.pixel.array() > image_size_.array()).any())
    return false; 
  else
    return true;
}

bool Simulator::get_previously_tracked_feature_in_frame(feature_t &feature)
{

  Vector3d ground_pt;
  env_.get_center_img_center_on_ground_plane(t_I_c_, q_I_c_, ground_pt);
  vector<Vector3d, aligned_allocator<Vector3d>> pts;
  vector<size_t> ids;
  if (env_.get_closest_points(ground_pt, NUM_FEATURES, 2.0, pts, ids))
  {
    for (int i = 0; i < pts.size(); i++)
    {
      if (is_feature_tracked(ids[i]))
        continue;
      // Calculate the bearing vector to the feature
      Vector3d pt = env_.get_points()[ids[i]];
      feature.zeta = q_I_c_.rotp(pt - t_I_c_);
      if (feature.zeta(2) < 0.0)
        continue;

      feature.zeta /= feature.zeta.norm();
      feature.depth = (pts[i] - t_I_c_).norm();
      proj(feature.zeta, feature.pixel);
      if ((feature.pixel.array() < 0).any() || (feature.pixel.array() > image_size_.array()).any())
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
  feature.id = env_.add_point(t_I_c_, q_I_c_, feature.zeta, feature.pixel, feature.depth);
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

void Simulator::proj(const Vector3d &zeta, Vector2d& pix) const
{
  double ezT_zeta = e_z.transpose() * zeta;  
  pix = cam_F_ * zeta / ezT_zeta + cam_center_;  
}


Vector4d Simulator::get_attitude()
{
  Vector3d noise;
  random_normal_vec(noise, attitude_noise_stdev_, normal_, rng_);
  Quatd q_I_m = dyn_.get_state().q * q_b_m_; //  q_I^m = q_I^b * q_b^m
  return (q_I_m + noise).elements();
}


Vector3d Simulator::get_position()
{
  Vector3d noise;
  random_normal_vec(noise, position_noise_stdev_, normal_, rng_);
  Vector3d I_p_b_I_ = dyn_.get_state().p; // p_{b/I}^I
  Vector3d I_p_m_I_ = I_p_b_I_ + dyn_.get_state().q.rota(p_b_m_); // p_{m/I}^I = p_{b/I}^I + R(q_I^b)^T (p_{m/b}^b)
  return I_p_m_I_;
}


Vector3d Simulator::get_acc()
{
  return get_true_imu().segment<3>(0);
}

Vector2d Simulator::get_pixel(const feature_t &feature)
{
  Vector2d pixel_noise;
  random_normal_vec(pixel_noise, pixel_noise_stdev_, normal_, rng_);
  return feature.pixel + pixel_noise;
}

double Simulator::get_depth(const feature_t &feature)
{
  return feature.depth + depth_noise_stdev_ * normal_(rng_);
}

Vector1d Simulator::get_altitude()
{
  Vector1d out;
  out << -1.0 * dyn_.get_state().p.z() + altimeter_noise_stdev_ * normal_(rng_);
  return out;
}

Matrix6d Simulator::get_imu_noise_covariance() const
{
    Vector6d cov = (Vector6d() << accel_noise_stdev_*accel_noise_stdev_,
                                  accel_noise_stdev_*accel_noise_stdev_,
                                  accel_noise_stdev_*accel_noise_stdev_,
                                  gyro_noise_stdev_*gyro_noise_stdev_,
                                  gyro_noise_stdev_*gyro_noise_stdev_,
                                  gyro_noise_stdev_*gyro_noise_stdev_).finished();
    if (cov.norm() < 1e-8)
        return Matrix6d::Identity() * 1e-8;
    else
        return cov.asDiagonal();
}

Matrix6d Simulator::get_mocap_noise_covariance() const
{
  Vector6d cov = (Vector6d() << position_noise_stdev_*position_noise_stdev_,
                                position_noise_stdev_*position_noise_stdev_,
                                position_noise_stdev_*position_noise_stdev_,
                                attitude_noise_stdev_*attitude_noise_stdev_,
                                attitude_noise_stdev_*attitude_noise_stdev_,
                                attitude_noise_stdev_*attitude_noise_stdev_).finished();
  if (cov.norm() < 1e-8)
      return Matrix6d::Identity() * 1e-8;
  else
      return cov.asDiagonal();
}

Vector3d Simulator::get_position_ecef() const
{
    return WSG84::ned2ecef(x_e2n_, dyn_.get_state().p);
}

Vector3d Simulator::get_velocity_ecef() const
{
    Vector3d v_NED = dyn_.get_global_pose().q().rota(dyn_.get_state().v);
    return x_e2n_.q().rota(v_NED);
}

}

