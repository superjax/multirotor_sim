#pragma once

#include <random>
#include <algorithm>
#include <deque>
#include <fstream>
#include <functional>
#include <stdint.h>

#include <Eigen/Core>

#include "geometry/quat.h"

#include "multirotor_sim/utils.h"
#include "multirotor_sim/wsg84.h"
#include "multirotor_sim/environment.h"
#include "multirotor_sim/state.h"
#include "multirotor_sim/dynamics.h"
#include "multirotor_sim/controller_base.h"
#include "multirotor_sim/trajectory_base.h"
#include "multirotor_sim/estimator_base.h"


#ifndef NUM_FEATURES  // allows you to override the number of features in the build
#define NUM_FEATURES 12
#endif

#define MAX_X 17+NUM_FEATURES*5
#define MAX_DX 16+NUM_FEATURES*3

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
  typedef enum {
      ACC,
      ALT,
      ATT,
      POS,
      VEL,
      FEAT,
      VO,
      GNSS,
      TOTAL_MEAS
    } measurement_type_t;

  typedef struct
  {
    double t;
    measurement_type_t type;
    VectorXd z;
    MatrixXd R;
    int feature_id;
    double depth;
  } measurement_t;
  
  Simulator(ControllerBase& _cont, TrajectoryBase& _traj);
  ~Simulator();
  Simulator(ControllerBase& _cont, TrajectoryBase& _traj, bool prog_indicator);
  Simulator(ControllerBase& _cont, TrajectoryBase& _traj, bool prog_indicator, uint64_t seed);
  
  void load(std::string filename);
  void init_imu();
  void init_camera();
  void init_altimeter();
  void init_vo();
  void init_truth();
  void init_gps();  

  bool run();

  void update_measurements();
  void update_imu_meas();
  void update_feature_meas();
  void update_alt_meas();
  void update_mocap_meas();
  void update_vo_meas();
  void update_gnss_meas();

  void register_estimator(EstimatorBase* est);

  const Vector6d& get_true_imu() const { return dyn_.imu_;}
  Xformd get_pose() const;
  Vector3d get_vel() const;
  Matrix6d get_imu_noise_covariance() const;
  Matrix6d get_mocap_noise_covariance() const;
  inline Vector3d get_accel_bias() const { return accel_bias_; }
  inline Vector3d get_gyro_bias() const { return gyro_bias_; }

  void log_state();
  
  Environment env_;
  Dynamics dyn_;
  ControllerBase& cont_;
  TrajectoryBase& traj_;
  std::vector<EstimatorBase*> est_;
  double t_, dt_, tmax_;

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
   * @brief proj
   * Calculates the Projection of bearing vector zeta with
   * Simulator class Camera Intrinsic parameters
   * @param zeta - bearing vector (normalized)
   * @param pix - pixel location (in image frame)
   */
  void proj(const Vector3d& zeta, Vector2d& pix) const;
  
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


  /**
   * @brief get_previously_tracked_feature_in_frame
   * @param feature
   * @return true if success
   */
  bool get_previously_tracked_feature_in_frame(feature_t &feature);

  /**
   * @brief create_new_feature_in_frame
   * @param feature
   * @return true if success
   */
  bool create_new_feature_in_frame(feature_t &feature);

  
  /**
   * @brief is_feature_tracked
   * Returns true if the landmark referred to by env_id is already in the tracked_features_ list
   * @param env_id - the id of the point in the environment points array
   * @return true if tracked, false otherwise
   */
  bool is_feature_tracked(const int env_id) const;
  
  /**
   * @brief update_camera_pose
   * Updates the q_I_c_ and t_I_c_ objects (the camera pose wrt to the inertial frame).
   * given the current simulator state.  This needs to be called before update_feature()
   * after a call to run()
   */
  void update_camera_pose();
  
  
  /**
   * @brief get_attitude
   * @return current attitude, sampled from normal distribution centered at true mean
   * with covariance R_att_
   */
  Vector4d get_attitude();
  
  /**
   * @brief get_position
   * @return current position, sampled from normal distribution centered at true mean
   * with covariance R_pos_
   */
  Vector3d get_position();
  
  /**
   * @brief get_pixel
   * @param feature
   * @return current pixel measurement of feature sampled from normal distribution 
   * centered at true mean with covariance R_feat_
   */
  Vector2d get_pixel(const feature_t& feature);
  
  /**
   * @brief get_depth
   * @param feature
   * @return current feature depth of feature sampled from normal distribution 
   * centered at true mean with variance R_depth_
   */
  double get_depth(const feature_t& feature);
  
  /**
   * @brief get_altitude
   * @return current altitude, sampled from normal distribution centered at true mean
   * with variance R_alt_
   */
  Matrix<double, 1, 1> get_altitude();
  
  /**
   * @brief get_acc
   * @return current acceleration, sampled from normal distribution centered at true mean
   * with covariance R_acc_
   */
  Vector3d get_acc();

  // Progress indicator, updated by run()
  ProgressBar prog_;
  bool prog_indicator_;
  std::string log_filename_;
  std::string param_filename_;
  ofstream log_;

  // Command vector passed from controller to dynamics [F, Omega]
  Vector4d u_;

  // measurement vector
//  std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > meas_;
  
  // Random number Generation
  uint64_t seed_;
  default_random_engine rng_;
  uniform_real_distribution<double> uniform_;
  normal_distribution<double> normal_;

  // IMU
  Quatd q_b_u_;
  Vector3d p_b_u_;

  bool imu_enabled_;
  double imu_update_rate_;
  double last_imu_update_;
  Matrix6d imu_R_;
  bool use_accel_truth_;
  Vector3d accel_bias_; // Memory for random walk
  double accel_noise_stdev_; // Standard deviation of accelerometer noise
  double accel_walk_stdev_; // Strength of accelerometer random walk
  Vector3d accel_noise_;

  bool use_gyro_truth_;
  Vector3d gyro_bias_; // Memory for random walk
  double gyro_noise_stdev_; // Standard deviation of gyro noise
  double gyro_walk_stdev_; // Strength of gyro random walk
  Vector3d gyro_noise_;
  
  // Camera (Features)
  bool features_enabled_;
  Quatd q_b_c_;
  Vector3d p_b_c_;
  Matrix2d feat_R_;
  bool use_camera_truth_;
  double pixel_noise_stdev_;
  double camera_update_rate_;
  double last_camera_update_;
  int next_feature_id_;
  double camera_time_delay_;
  Vector2d pixel_noise_;
  bool loop_closure_; // whether to re-use features if they show up in the frame again
  vector<Vector3d> cam_; // Elements are landmarks in camera FOV given by a vector containing [pixel_x,pixel_y,label]
  vector<feature_t, aligned_allocator<feature_t>> tracked_points_; // currently tracked features
  deque<measurement_t, aligned_allocator<measurement_t>> camera_measurements_buffer_; // container to hold measurements while waiting for delay
  Quatd q_I_c_; // rotation from inertial frame to camera frame (updated by update_camera_pose())
  Vector3d t_I_c_; // translation from inertial frame to camera frame (updated by update_camera_pose())
  double feat_move_prob_; // probability of moving a feature (simulating bad data association)
  Matrix<double, 2, 3> cam_F_; // Camera intrinsics
  Vector2d cam_center_; // Camera intrinsics
  Vector2d image_size_; // Camera intrinsics

  // Altimeter
  bool alt_enabled_;
  Matrix<double, 1, 1> alt_R_;
  bool use_altimeter_truth_;
  double altimeter_update_rate_; // determines how often to generate an altitude measurement
  double last_altimeter_update_;
  double altimeter_noise_stdev_; // Standard deviation of altimeter noise
  double altimeter_noise_;

  // Depth
  bool depth_enabled_;
  bool use_depth_truth_;
  Matrix1d depth_R_;
  double depth_update_rate_; // determines how often to generate an altitude measurement
  double last_depth_update_;
  double depth_noise_stdev_; // Standard deviation of altimeter noise
  double depth_noise_;

  // Visual Odometry
  bool vo_enabled_;
  xform::Xformd T_i2bk_; // Inertial to body keyframe pose
  Matrix<double, 6, 6> vo_R_;
  bool use_vo_truth_;
  double vo_delta_position_;
  double vo_delta_attitude_;
  double vo_translation_noise_stdev_;
  double vo_rotation_noise_stdev_;

  // Motion Capture
  bool pos_enabled_;
  bool att_enabled_;
  Quatd q_b_m_;
  Vector3d p_b_m_;
  Matrix3d att_R_;
  Matrix3d pos_R_;
  bool use_attitude_truth_;
  bool use_position_truth_;
  double mocap_update_rate_; // determines how often to supply truth measurements
  double attitude_noise_stdev_;
  double position_noise_stdev_;
  double last_truth_update_;
  double next_truth_measurement_;
  double mocap_time_offset_;
  double mocap_transmission_noise_;
  double mocap_transmission_time_;
  deque<std::pair<double, measurement_t>, aligned_allocator<std::pair<double, measurement_t>>> mocap_measurement_buffer_; // container to hold measurements while waiting for delay

  // GPS
  bool gnss_enabled_;
  Xformd x_e2n_; // transform from the ECEF frame to the Inertial (NED) frame
  Matrix6d gps_R_;
  bool use_gps_truth_;
  double gps_update_rate_;
  double gps_horizontal_position_stdev_;
  double gps_vertical_position_stdev_;
  double gps_velocity_stdev_;
  double last_gps_update_;
  Vector3d gps_position_noise_;
};
}
