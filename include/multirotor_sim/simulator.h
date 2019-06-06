#pragma once

#include <random>
#include <algorithm>
#include <deque>
#include <fstream>
#include <functional>
#include <cstdint>
#include <memory>

#include <Eigen/Core>

#include "geometry/quat.h"
#include "geometry/cam.h"
#include "gnss_utils/wgs84.h"
#include "gnss_utils/satellite.h"

#include "multirotor_sim/utils.h"
#include "multirotor_sim/environment.h"
#include "multirotor_sim/state.h"
#include "multirotor_sim/dynamics.h"
#include "multirotor_sim/controller_base.h"
#include "multirotor_sim/controller.h"
#include "multirotor_sim/trajectory_base.h"
#include "multirotor_sim/estimator_base.h"


#ifdef MULTIROTOR_SIM_PRINT_DEBUG
#define DBG(msg, ...)\
  { \
    printf((msg), ##__VA_ARGS__); \
  }
#else
#define DBG(msg, ...) {}
#endif

using namespace quat;

namespace multirotor_sim
{
class Simulator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef struct
  {
    double t;
    VectorXd z;
    MatrixXd R;
    int feature_id;
    double depth;
  } measurement_t;
  
  Simulator(bool prog_indicator=false, uint64_t seed=0);
  ~Simulator();
  
  void load(std::string filename);
  void init_platform();
  void init_imu();
  void init_camera();
  void init_altimeter();
  void init_baro();
  void init_vo();
  void init_mocap();
  void init_gnss();
  void init_raw_gnss();

  bool run();
  Vector4d compute_low_level_control(const Vector4d& u); // {F[0-1], Wx(rad/s), Wy, Wz}

  void update_measurements();
  void update_imu_meas();
  void update_camera_meas();
  void update_alt_meas();
  void update_baro_meas();
  void update_mocap_meas();
  void update_vo_meas();
  void update_gnss_meas();
  void update_raw_gnss_meas();

  void use_custom_controller(ControllerBase* cont);
  void use_custom_trajectory(TrajectoryBase* traj);
  void register_estimator(EstimatorBase* est);

  const Vector6d& imu() const { return dyn_.imu_;}
  const State& state() const { return dyn_.get_state(); }
  State& state() { return dyn_.get_state(); }
  const Vector4d& input() const { return u_; }
  Vector4d& input() { return u_; }
  const State& commanded_state() const {return xc_;}
  Vector4d& reference_input() { return ur_; }
  const Vector4d& reference_input() const { return ur_; }
  State& commanded_state() {return xc_;}
  Vector3d get_position_ecef() const;
  Vector3d get_velocity_ecef() const;
  Vector3d get_gps_position_ned() const;
  Vector3d get_gps_position_ecef() const;
  Vector3d get_gps_velocity_ecef() const;

  Environment env_;
  Dynamics dyn_;
  ReferenceController ref_con_;
  ControllerBase* cont_;
  TrajectoryBase* traj_;
  typedef std::vector<EstimatorBase*> estVec;
  estVec est_;
  double t_, dt_, tmax_, t_round_off_;

  typedef struct
  {
    Vector3d zeta;
    Vector2d pixel;
    double depth;
    int id;
  } feature_t;
  
  /**
   * @brief update_feature
   * This function updates the zeta, pixel and depth fields
   * of the supplied feature object given the current
   * simulator state
   * @param feature
   * @return true if feature still in image, false otherwise
   */
  bool update_feature(feature_t &feature) const;
  
  /**
   * @brief get_feature_in_frame
   * If retrack is true first tries to find pixels previously tracked in the frame that are not
   * currently being tracked.  If retrack is false, or there are no previously observed landmarks,
   * or all previously observed landmarks are being tracked, creates a new, randomly selected landmark
   * in the camera frame from the environment and calculates its projection - global_id is automatically
   * incremented with each new landmark
   * @param feature - ouput new feature with projection loaded
   * @param retrack - flag of whether to rediscover old landmarks or always create new
   * @returns bool - true if suceeded, false if the ground plane is not in the camera FOV.
   */
  bool get_feature_in_frame(feature_t &feature, bool retrack);


  bool get_previously_tracked_feature_in_frame(feature_t &feature);
  bool create_new_feature_in_frame(feature_t &feature);
  bool is_feature_tracked(const int env_id) const;
  
  /**
   * @brief update_camera_pose
   * Updates the q_I_c_ and t_I_c_ objects (the camera pose wrt to the inertial frame).
   * given the current simulator state.  This needs to be called before update_feature()
   * after a call to run()
   */
  void update_camera_pose();



  // Progress indicator, updated by run()
  ProgressBar prog_;
  bool prog_indicator_;
  std::string log_filename_;
  std::string param_filename_;
  ofstream log_;

  Vector4d u_; // Command vector passed from controller to dynamics [F, Omega]
  Vector4d ur_; // Reference Command given by the trajectory
  State xc_; // Desired State

  // Random number Generation
  uint64_t seed_;
  default_random_engine rng_;
  uniform_real_distribution<double> uniform_;
  normal_distribution<double> normal_;

  // Multirotor Hardware
  double max_thrust_;
  Vector3d max_torque_;
  Vector3d kp_w_;
  Vector3d kd_w_;
  Vector3d w_err_prev_;

  // IMU
  bool imu_enabled_;
  Quatd q_b2u_;
  Vector3d p_b2u_;
  double imu_update_rate_;
  double last_imu_update_;
  Matrix6d imu_R_;
  Vector3d accel_bias_; // Memory for random walk
  double accel_noise_stdev_;
  double accel_walk_stdev_;
  Vector3d gyro_bias_; // Memory for random walk
  double gyro_noise_stdev_;
  double gyro_walk_stdev_;
  
  // Camera (Features)
  bool camera_enabled_;
  int num_features_;
  Xformd x_b2c_;
  Xformd x_I2c_;
  Matrix2d feat_R_;
  double pixel_noise_stdev_;
  double camera_update_rate_;
  double last_camera_update_;
  int next_feature_id_;
  double camera_time_offset_;
  double camera_transmission_noise_;
  double camera_transmission_time_;
  bool loop_closure_; // whether to re-use features if they show up in the frame again
  vector<feature_t, aligned_allocator<feature_t>> tracked_points_; // currently tracked features
  deque<std::pair<double,measurement_t>, aligned_allocator<std::pair<double,measurement_t>>> camera_measurements_buffer_; // container to hold measurements while waiting for delay
  Camera<double> cam_;
  ImageFeat img_;
  int image_id_;

  // Altimeter
  bool alt_enabled_;
  Matrix1d alt_R_;
  double altimeter_update_rate_;
  double last_altimeter_update_;
  double altimeter_noise_stdev_;

  // Barometer
  bool baro_enabled_;
  Matrix1d baro_R_;
  double baro_update_rate_;
  double last_baro_update_;
  double baro_noise_stdev_;
  double baro_bias_;
  double baro_bias_walk_stdev_;
  double alt0_;

  // Depth
  bool depth_enabled_;
  Matrix1d depth_R_;
  double depth_update_rate_;
  double last_depth_update_;
  double depth_noise_stdev_;

  // Visual Odometry
  bool vo_enabled_;
  xform::Xformd X_I2bk_; // Inertial to body keyframe pose
  Matrix6d vo_R_;
  double vo_delta_position_;
  double vo_delta_attitude_;
  double vo_translation_noise_stdev_;
  double vo_rotation_noise_stdev_;

  // Motion Capture
  bool mocap_enabled_;
  Quatd q_b2m_;
  Vector3d p_b2m_;
  Matrix6d mocap_R_;
  double mocap_update_rate_;
  double attitude_noise_stdev_;
  double position_noise_stdev_;
  double last_mocap_update_;
  double next_mocap_measurement_;
  double mocap_time_offset_;
  double mocap_transmission_noise_;
  double mocap_transmission_time_;
  deque<std::pair<double, measurement_t>, aligned_allocator<std::pair<double, measurement_t>>> mocap_measurement_buffer_; // container to hold measurements while waiting for delay

  // GNSS
  bool gnss_enabled_;
  Xformd X_e2n_; // transform from the ECEF frame to the Inertial (NED) frame
  Matrix6d gnss_R_;
  Vector3d p_b2g_; // position of gps antenna in body frame
  double gnss_update_rate_;
  double gnss_horizontal_position_stdev_;
  double gnss_vertical_position_stdev_;
  double gnss_velocity_stdev_;
  double last_gnss_update_;
  Vector3d gnss_position_noise_;

  // RAW GNSS
  bool raw_gnss_enabled_;
  Matrix3d raw_gnss_R_;
  double pseudorange_stdev_;
  double pseudorange_rate_stdev_;
  double carrier_phase_stdev_;
  double clock_walk_stdev_;
  double clock_init_stdev_;
  double clock_bias_;
  double clock_bias_rate_;

  Area multipath_area_;
  Area gps_denied_area_;
  double multipath_error_range_;
  double cycle_slip_prob_;
  std::string ephemeris_filename_;
  std::vector<double> multipath_offset_;
  double multipath_prob_;
  std::vector<int> carrier_phase_integer_offsets_;
  SatVec satellites_;
  double last_raw_gnss_update_;
  gnss_utils::GTime start_time_;
};
}
