#pragma once

#include <random>
#include <algorithm>
#include <deque>

#include "Eigen/Core"

#include "utils.h"
#include "quat.h"
#include "environment.h"
#include "dynamics.h"
#include "controller.h"

#define NUM_FEATURES 12
#define MAX_X 17+NUM_FEATURES*5
#define MAX_DX 16+NUM_FEATURES*3

//#ifndef NDEBUG
//#define DBG(msg, ...)\
//  { \
//    printf((msg), ##__VA_ARGS__); \
//  }
//#else
#define DBG(msg, ...) {}
//#endif

using namespace quat;

class Simulator
{
public:
  typedef enum {
      ACC,
      ALT,
      ATT,
      POS,
      VEL,
      QZETA,
      FEAT,
      PIXEL_VEL,
      DEPTH,
      INV_DEPTH,
      TOTAL_MEAS
    } measurement_type_t;

  enum : int{
      xPOS = 0,
      xVEL = 3,
      xATT = 6,
      xB_A = 10,
      xB_G = 13,
      xMU = 16,
      xZ = 17
    };

  typedef Matrix<double, MAX_X, 1> xVector;
  typedef Matrix<double, MAX_DX, 1> dxVector;

  typedef struct
  {
    double t;
    measurement_type_t type;
    VectorXd z;
    MatrixXd R;
    int global_id;
    double depth;
    bool active;
  } measurement_t;
  
  Simulator();
  ~Simulator();
  Simulator(bool prog_indicator);
  Simulator(bool prog_indicator, uint64_t seed);
  
  /**
   * @brief load
   * loads parameters from file
   * @param filename
   */
  void load(std::string filename);
  
  /**
   * @brief run
   * Main driver of the simulation loop
   * @return true if not yet at final time, false if complete
   */
  bool run();
  
  /**
   * @brief get_measurements
   * returns a list of measurements which occurred since the last get_measurements call
   * these measurements all occur at the current time
   * @param meas
   */
  void get_measurements(std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > &meas);

  const Vector6d& get_imu_prev() const { return imu_prev_; }

  /**
   * @brief tracked_features
   * Returns a vector of the currently tracked features (global ids)
   * @param (modified) vector of currently tracked features (global_id)
   */
  void tracked_features(std::vector<int>& ids) const;

  /**
   * @brief get_pose
   * Returns the current (true) state of the multirotor
   * @returns current pose (position, attitude)
   */
  Xformd get_pose() const;

  /**
   * @brief get_truth
   * @return the full truth vector (as supplied by EKF class)
   */
  void get_truth(xVector& x, const std::vector<int> &tracked_features) const;

  inline Vector3d get_accel_bias() const { return accel_bias_; }
  inline Vector3d get_gyro_bias() const { return gyro_bias_; }
  
  Environment env_;
  dynamics::Dynamics dyn_;
  controller::Controller cont_;
  double t_, dt_, tmax_;

  Matrix3d att_R_;
  Matrix3d acc_R_;
  Matrix2d feat_R_;
  Matrix<double, 1, 1> alt_R_;
  Matrix<double, 1, 1> depth_R_;
  
  bool attitude_update_active_;
  bool depth_update_active_;
  bool feature_update_active_;
  bool drag_update_active_;
  bool altimeter_update_active_;
  
private:
  typedef struct
  {
    Vector3d zeta;
    Vector2d pixel;
    double depth;
    int env_id;
    int global_id;
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
   * @brief get_random_feature_in_frame
   * Finds a new, randomly selected pixel in the camera frame from the environment and calculates its 
   * projection - global_id is automatically incremented with each call
   * @returns new feature with projection loaded
   */
  bool get_random_feature_in_frame(feature_t &feature);
  
  /**
   * @brief is_feature_tracked
   * Returns true if the pixel referred to by env_id is already in the tracked_features_
   * object
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
  double get_depth(const feature_t& feature, bool override= false);
  
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

  /**
   * @brief global_to_local_feature_id
   * @param id a global id
   * @returns the local id (posiiton in the 'tracked_points' vector)
   */
  int global_to_local_feature_id(int id) const;

  // Progress indicator, updated by run()
  ProgressBar prog_;
  bool prog_indicator_;

  // Command vector passed from controller to dynamics
  dynamics::commandVector u_;
  
  /* === Sensors === */
  uint64_t seed_;
  default_random_engine generator_;
  uniform_real_distribution<double> uniform_;
  normal_distribution<double> normal_;

  // IMU
  Vector6d imu_; // Vector containing [accel;gyro]
  Vector6d imu_prev_; // Previous version for propagation
  double imu_update_rate_; // determines how often to generate an altitude measurement
  double last_imu_update_;

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
  
  // Camera
  bool use_camera_truth_;
  double pixel_noise_stdev_; // Standard deviation of camera pixel noise
  vector<Vector3d> cam_; // Elements are landmarks in camera FOV given by a vector containing [pixel_x,pixel_y,label]
  double camera_update_rate_;  // determines how often to supply a set of feature measurements
  double last_camera_update_;
  int next_global_feature_id_; 
  vector<feature_t, aligned_allocator<feature_t>> tracked_points_; // currently tracked features
  deque<measurement_t, aligned_allocator<measurement_t>> camera_measurements_buffer_; // container to hold measurements while waiting for delay
  double camera_time_delay_;
  Vector2d pixel_noise_;
  
  // Pose of camera in the body frame - assumed to be fixed
  Quatd q_b_c_;
  Vector3d t_b_c_;
  
  // Pose of camera in the inertial frame, updated by update_camera_pose()
  Quatd q_I_c_;
  Vector3d t_I_c_;
  double feat_move_prob_;
  
  // Camera intrinsics - assmed to be fixed
  Matrix<double, 2, 3> cam_F_;
  Vector2d cam_center_;
  Vector2d image_size_;

  // Altimeter
  bool use_altimeter_truth_;
  double altimeter_update_rate_; // determines how often to generate an altitude measurement
  double last_altimeter_update_;
  double altimeter_noise_stdev_; // Standard deviation of altimeter noise
  double altimeter_noise_;

  // Depth
  bool use_depth_truth_;
  bool init_depth_;
  double depth_update_rate_; // determines how often to generate an altitude measurement
  double last_depth_update_;
  double depth_noise_stdev_; // Standard deviation of altimeter noise
  double depth_noise_;

  // Attitude
  double attitude_update_rate_; // determines how often to supply truth measurements
  double last_attitude_update_;
};
